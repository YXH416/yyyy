"""ROUND-046 bounded console processing; no error-triggered transmit storm."""

from __future__ import annotations

import csv
import queue
import re
import threading
import time
from datetime import datetime
from pathlib import Path
import tkinter as tk
from tkinter import messagebox, ttk

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # Shown cleanly after Tk starts.
    serial = None
    list_ports = None


BAUD = 115200
PROTOCOL_ANGLE_LIMIT = 15.0
HEARTBEAT_MS = 200
STATUS_PERIOD_MS = 1000
MANUAL_START_TIMEOUT_MS = 1500
ANGLE_STEP_DEG = 1.0
KEY_REPEAT_DELAY_MS = 300
KEY_REPEAT_MS = 100
ZERO_READY_TOLERANCE_DEG = 0.30
BALL_LINE = re.compile(
    r"^ball:([-+0-9.eE]+),([-+0-9.eE]+),([-+0-9.eE]+),([-+0-9.eE]+)$"
)
FIELD = re.compile(r"([A-Za-z_][A-Za-z0-9_]*)=([^\s]+)")


class SerialLink:
    """One reader and one queued writer own the serial port."""

    def __init__(self, inbox: queue.Queue[tuple[str, object]]) -> None:
        self.inbox = inbox
        self.port: serial.Serial | None = None
        self.stop_event = threading.Event()
        self.reader: threading.Thread | None = None
        self.writer: threading.Thread | None = None
        self.tx_queue: queue.PriorityQueue[tuple[int, int, str, int]] = queue.PriorityQueue(128)
        self.queue_sequence = 0
        self.manual_sequence = 0
        self.sequence_lock = threading.Lock()

    @property
    def connected(self) -> bool:
        return self.port is not None and self.port.is_open

    def connect(self, name: str) -> None:
        self.close()
        self.stop_event.clear()
        self.tx_queue = queue.PriorityQueue(128)
        port = serial.Serial(name, BAUD, timeout=0.1, write_timeout=0.2)
        self.port = port
        self.reader = threading.Thread(target=self._read_loop, args=(port,), daemon=True)
        self.writer = threading.Thread(target=self._write_loop, args=(port,), daemon=True)
        self.reader.start()
        self.writer.start()

    def send(self, command: str, priority: int = 1) -> int | None:
        if not self.connected:
            return None
        command = command.rstrip("\r\n")
        is_manual = command.startswith("MANUAL,")
        with self.sequence_lock:
            self.queue_sequence += 1
            sequence = self.queue_sequence
            if is_manual:
                self.manual_sequence = (self.manual_sequence + 1) & 0xFFFFFFFF
                manual_sequence = self.manual_sequence
            if is_manual:
                priority = 0
                wire = f"{command},{manual_sequence}"
            else:
                wire = command
            # Assign sequence and enqueue atomically across GUI/heartbeat.
            try:
                self.tx_queue.put_nowait((priority, sequence, wire, time.perf_counter_ns()))
                return sequence
            except queue.Full:
                self.inbox.put(("tx_error", f"TX_QUEUE_FULL command={command}"))
                return None

    def close(self) -> None:
        self.stop_event.set()
        port, self.port = self.port, None
        reader, self.reader = self.reader, None
        writer, self.writer = self.writer, None
        if port is not None:
            try:
                port.close()
            except (serial.SerialException, OSError):
                pass
        if (reader is not None and reader.is_alive() and
                reader is not threading.current_thread()):
            reader.join(timeout=0.3)
        if (writer is not None and writer.is_alive() and
                writer is not threading.current_thread()):
            writer.join(timeout=0.3)

    def _read_loop(self, port: serial.Serial) -> None:
        buffer = bytearray()
        try:
            while not self.stop_event.is_set() and port.is_open:
                chunk = port.read(port.in_waiting or 1)
                if not chunk:
                    continue
                buffer.extend(chunk)
                while b"\n" in buffer:
                    raw, _, buffer = buffer.partition(b"\n")
                    line = raw.rstrip(b"\r").decode("utf-8", errors="replace")
                    if line:
                        self.inbox.put(("line", line))
        except (serial.SerialException, OSError) as exc:
            if not self.stop_event.is_set():
                self.inbox.put(("disconnect", str(exc)))

    def _write_loop(self, port: serial.Serial) -> None:
        try:
            while not self.stop_event.is_set() and port.is_open:
                try:
                    _priority, sequence, wire, queued_ns = self.tx_queue.get(timeout=0.1)
                except queue.Empty:
                    continue
                frame = (wire + "\n").encode("ascii")
                if port.write(frame) != len(frame):
                    raise OSError("Incomplete serial command write")
                self.inbox.put(("tx", (sequence, wire, queued_ns,
                                        time.perf_counter_ns())))
        except (serial.SerialException, OSError) as exc:
            if not self.stop_event.is_set():
                self.inbox.put(("disconnect", str(exc)))


class Recorder:
    def __init__(self) -> None:
        self.started_ns = 0
        self.folder: Path | None = None
        self.telemetry_file = None
        self.events_file = None
        self.telemetry = None
        self.events = None

    @property
    def active(self) -> bool:
        return self.telemetry_file is not None

    def start(self) -> Path:
        self.stop()
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.folder = Path(__file__).resolve().parent / "records" / stamp
        self.folder.mkdir(parents=True, exist_ok=False)
        self.telemetry_file = (self.folder / "telemetry.csv").open(
            "w", newline="", encoding="utf-8-sig"
        )
        self.events_file = (self.folder / "events.csv").open(
            "w", newline="", encoding="utf-8-sig"
        )
        self.telemetry = csv.writer(self.telemetry_file)
        self.events = csv.writer(self.events_file)
        self.telemetry.writerow([
            "time_ms", "ball_pos_mm", "ball_vel_mm_s", "motor_cmd_deg",
            "motor_real_deg", "vision_valid", "position_valid", "pwm_valid",
            "fault", "mode",
        ])
        self.events.writerow(["time_ms", "wall_time", "event", "detail"])
        self.started_ns = time.perf_counter_ns()
        self.event("RECORD_START", str(self.folder))
        return self.folder

    def elapsed_ms(self) -> int:
        return int((time.perf_counter_ns() - self.started_ns) / 1_000_000)

    def sample(self, values: list[object]) -> None:
        if self.active:
            self.telemetry.writerow([self.elapsed_ms(), *values])

    def event(self, event: str, detail: str = "") -> None:
        if self.active:
            self.events.writerow([
                self.elapsed_ms(), datetime.now().isoformat(timespec="milliseconds"),
                event, detail,
            ])
            self.events_file.flush()

    def stop(self) -> Path | None:
        folder = self.folder
        if self.events_file is not None:
            self.event("RECORD_STOP")
        for handle in (self.telemetry_file, self.events_file):
            if handle is not None:
                handle.flush()
                handle.close()
        self.telemetry_file = self.events_file = None
        self.telemetry = self.events = None
        return folder


class BallDebugger(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("平衡球键盘调试器 · ROUND-046")
        self.geometry("980x720")
        self.minsize(860, 650)
        self.inbox: queue.Queue[tuple[str, object]] = queue.Queue()
        self.link = SerialLink(self.inbox)
        self.recorder = Recorder()
        self.ball_pos = self.ball_vel = self.motor_cmd = self.motor_real = None
        self.vision_valid: bool | None = None
        self.position_valid: bool | None = None
        self.pwm_valid: bool | None = None
        self.fault = "UNKNOWN"
        self.feedback = "UNKNOWN"
        self.manual_active = False
        self.manual_state = "DISCONNECTED"
        self.manual_detail = "请先连接串口"
        self.manual_start_deadline = 0.0
        self.ignore_manual_started = False
        self.auto_enable_allowed = False
        self.auto_enable_after_zero = False
        self.zero_request_pending = False
        self.zero_deadline = 0.0
        self.last_rx_warning = 0.0
        self.rx_loss_count = 0
        self.firmware_command_limit = PROTOCOL_ANGLE_LIMIT
        self.mechanical_pos_limit = PROTOCOL_ANGLE_LIMIT
        self.mechanical_neg_limit = PROTOCOL_ANGLE_LIMIT
        self.mechanical_limit_confirmed = False
        self.current_manual_angle = 0.0
        self.pressed: set[str] = set()
        self.release_jobs: dict[str, str] = {}
        self.repeat_job: str | None = None
        self.repeat_key: str | None = None
        self.last_focus_ok = True
        self.heartbeat_stop = threading.Event()
        self.heartbeat_thread = threading.Thread(
            target=self._heartbeat_loop, daemon=True
        )
        self._build_ui()
        self._bind_keys()
        self.refresh_ports()
        self.protocol("WM_DELETE_WINDOW", self.close_window)
        self.after(20, self.process_inbox)
        self.after(STATUS_PERIOD_MS, self.poll_status)
        self.after(100, self.focus_guard)
        self.after(100, self.manual_state_guard)
        self.heartbeat_thread.start()
        if serial is None:
            self.after(100, lambda: messagebox.showerror(
                "缺少依赖", "未安装 pyserial。\n请运行：py -3 -m pip install pyserial"
            ))

    def _build_ui(self) -> None:
        style = ttk.Style(self)
        style.configure("Big.TLabel", font=("Microsoft YaHei UI", 22, "bold"))
        style.configure("State.TLabel", font=("Microsoft YaHei UI", 11, "bold"))

        top = ttk.Frame(self, padding=10)
        top.pack(fill="x")
        ttk.Label(top, text="串口：").pack(side="left")
        self.port_var = tk.StringVar(value="COM17")
        self.port_box = ttk.Combobox(top, textvariable=self.port_var, width=12)
        self.port_box.pack(side="left", padx=4)
        ttk.Button(top, text="刷新", command=self.refresh_ports).pack(side="left")
        self.connect_btn = ttk.Button(top, text="连接", command=self.toggle_connection)
        self.connect_btn.pack(side="left", padx=6)
        self.com_label = ttk.Label(top, text="未连接", style="State.TLabel")
        self.com_label.pack(side="left", padx=12)
        ttk.Button(top, text="回平衡零点", command=self.balance_zero).pack(side="right", padx=4)
        ttk.Button(top, text="清故障", command=lambda: self.send("FAULT,CLEAR")).pack(side="right", padx=4)

        dashboard = ttk.Frame(self, padding=(10, 0))
        dashboard.pack(fill="x")
        ball = ttk.LabelFrame(dashboard, text="小球状态", padding=10)
        motor = ttk.LabelFrame(dashboard, text="电机状态", padding=10)
        system = ttk.LabelFrame(dashboard, text="系统状态", padding=10)
        ball.pack(side="left", expand=True, fill="both", padx=(0, 5))
        motor.pack(side="left", expand=True, fill="both", padx=5)
        system.pack(side="left", expand=True, fill="both", padx=(5, 0))

        self.pos_label = ttk.Label(ball, text="-- mm", style="Big.TLabel")
        self.vel_label = ttk.Label(ball, text="-- mm/s", style="Big.TLabel")
        self.pos_label.pack(anchor="w")
        self.vel_label.pack(anchor="w")

        self.motor_cmd_label = ttk.Label(motor, text="--°", style="Big.TLabel")
        self.motor_real_label = ttk.Label(motor, text="--°", style="Big.TLabel")
        self.motor_error_label = ttk.Label(motor, text="角度误差 --°")
        self.motor_cmd_label.pack(anchor="w")
        self.motor_real_label.pack(anchor="w")
        self.motor_error_label.pack(anchor="w")

        self.vision_label = ttk.Label(system, text="视觉：未知", style="State.TLabel")
        self.position_valid_label = ttk.Label(system, text="毫米位置：未知", style="State.TLabel")
        self.pwm_label = ttk.Label(system, text="PWM：未知", style="State.TLabel")
        self.fault_label = ttk.Label(system, text="Fault：UNKNOWN", style="State.TLabel")
        self.mode_label = ttk.Label(system, text="状态：停止", style="State.TLabel")
        for widget in (self.vision_label, self.position_valid_label,
                       self.pwm_label, self.fault_label, self.mode_label):
            widget.pack(anchor="w", pady=3)

        scale_frame = ttk.LabelFrame(self, text="球位置尺（mm）", padding=8)
        scale_frame.pack(fill="x", padx=10, pady=8)
        self.scale_canvas = tk.Canvas(scale_frame, height=85, bg="white", highlightthickness=0)
        self.scale_canvas.pack(fill="x")
        self.scale_canvas.bind("<Configure>", lambda _e: self.draw_scale())

        controls = ttk.LabelFrame(self, text="键盘控制", padding=10)
        controls.pack(fill="x", padx=10)
        ttk.Label(controls, text="累加角度遥控：每次1°",
                  style="State.TLabel").grid(row=0, column=0, columnspan=3,
                                             sticky="w")
        self.manual_btn = ttk.Button(controls, text="确认启用遥控",
                                     command=self.confirm_enable)
        self.manual_btn.grid(row=0, column=4, padx=14)
        ttk.Button(controls, text="停止脉冲 (S)", command=self.stop_manual).grid(row=1, column=4, padx=14)
        ttk.Label(controls, text="轻按→/+1°，←/-1°；长按300 ms后每100 ms继续1°；松开保持当前目标").grid(
            row=2, column=0, columnspan=5, sticky="w", pady=(8, 0)
        )
        ttk.Label(controls, text="空格：回平衡零点    S：停止脉冲    Esc：退出").grid(
            row=3, column=0, columnspan=5, sticky="w"
        )
        self.manual_detail_label = ttk.Label(controls, text=self.manual_detail)
        self.manual_detail_label.grid(row=4, column=0, columnspan=5,
                                      sticky="w", pady=(4, 0))
        self.limit_label = ttk.Label(
            controls, text="协议±15.0°；机械限位+15.0°/-15.0°（未确认）"
        )
        self.limit_label.grid(row=5, column=0, columnspan=5, sticky="w")

        record = ttk.Frame(self, padding=(10, 8))
        record.pack(fill="x")
        self.record_btn = ttk.Button(record, text="开始记录", command=self.toggle_record)
        self.record_btn.pack(side="left")
        self.record_label = ttk.Label(record, text="未记录")
        self.record_label.pack(side="left", padx=10)

        log_frame = ttk.LabelFrame(self, text="串口事件", padding=5)
        log_frame.pack(fill="both", expand=True, padx=10, pady=(0, 10))
        self.log = tk.Text(log_frame, height=10, wrap="none", state="disabled",
                           font=("Consolas", 9))
        scroll = ttk.Scrollbar(log_frame, orient="vertical", command=self.log.yview)
        self.log.configure(yscrollcommand=scroll.set)
        self.log.pack(side="left", fill="both", expand=True)
        scroll.pack(side="right", fill="y")

    def _bind_keys(self) -> None:
        self.bind_all("<KeyPress>", self.key_down, add=True)
        self.bind_all("<KeyRelease>", self.key_up, add=True)

    def refresh_ports(self) -> None:
        if list_ports is None:
            return
        names = [p.device for p in list_ports.comports()]
        self.port_box["values"] = names
        if names and self.port_var.get() not in names:
            self.port_var.set(names[0])

    def toggle_connection(self) -> None:
        if self.link.connected:
            self.disconnect("USER")
            return
        if serial is None:
            messagebox.showerror("缺少依赖", "请先安装 pyserial。")
            return
        try:
            self.link.connect(self.port_var.get().strip())
        except (serial.SerialException, OSError, ValueError) as exc:
            messagebox.showerror("连接失败", str(exc))
            return
        self.connect_btn.configure(text="断开")
        self.com_label.configure(text=f"{self.port_var.get()}：已连接")
        self.append_log(f"[PC] CONNECT {self.port_var.get()} @ {BAUD}")
        self.auto_enable_allowed = True
        self.auto_enable_after_zero = False
        self.ignore_manual_started = False
        self.set_manual_state("CHECKING", "正在检查PWM反馈、故障和零点……")
        self.send("STREAM,ON", record=False, priority=5)
        self.send("STATUS", record=False, priority=5)

    def disconnect(self, reason: str) -> None:
        if self.manual_active and self.link.connected:
            self.send("MANUAL,ANGLE,0.000", priority=0)
            time.sleep(0.04)
        self.link.close()
        self.cancel_repeat()
        self.auto_enable_allowed = False
        self.auto_enable_after_zero = False
        self.set_manual_state("DISCONNECTED", f"串口已断开：{reason}")
        self.connect_btn.configure(text="连接")
        self.com_label.configure(text="未连接")
        self.mode_label.configure(text="状态：未连接")
        self.append_log(f"[PC] DISCONNECT reason={reason}")
        self.recorder.event("DISCONNECT", reason)

    def send(self, command: str, record: bool = True,
             priority: int = 1) -> bool:
        sequence = self.link.send(command, priority=priority)
        if sequence is None:
            if record:
                self.append_log("[PC] SEND_FAILED not_connected")
            return False
        return True

    def process_inbox(self) -> None:
        deadline = time.monotonic() + 0.008
        try:
            # Bound each GUI callback so errors cannot starve keys/timers.
            for _ in range(100):
                if time.monotonic() >= deadline:
                    break
                kind, payload = self.inbox.get_nowait()
                if kind == "line":
                    self.process_line(str(payload))
                elif kind == "tx":
                    sequence, wire, queued_ns, sent_ns = payload
                    delay_ms = (sent_ns - queued_ns) / 1_000_000
                    self.recorder.event(
                        "TX", f"seq={sequence} queue_ms={delay_ms:.1f} {wire}"
                    )
                    if not str(wire).startswith("MANUAL,HEARTBEAT"):
                        self.append_log(
                            f"[TX] seq={sequence} queue_ms={delay_ms:.1f} {wire}"
                        )
                elif kind == "tx_error":
                    self.append_log(f"[PC] {payload}")
                    self.recorder.event("TX_ERROR", str(payload))
                elif kind == "disconnect":
                    self.disconnect(str(payload))
        except queue.Empty:
            pass
        self.after(20, self.process_inbox)

    def process_line(self, line: str) -> None:
        match = BALL_LINE.match(line)
        if match:
            values = [float(x) for x in match.groups()]
            self.ball_pos, self.ball_vel, self.motor_cmd, self.motor_real = [
                None if x <= -9000 else x for x in values
            ]
            if self.ball_pos is None:
                self.position_valid = False
            else:
                self.position_valid = True
            self.update_dashboard()
            self.recorder.sample([
                self.ball_pos, self.ball_vel, self.motor_cmd, self.motor_real,
                self.vision_valid, self.position_valid, self.pwm_valid,
                self.fault, self.manual_state,
            ])
            return

        fields = dict(FIELD.findall(line))
        if line.startswith("[STATUS]"):
            self.vision_valid = fields.get("vision") == "1"
            self.position_valid = fields.get("pos_valid") == "1"
            self.pwm_valid = fields.get("pwm_valid") == "1"
            self.fault = fields.get("fault", self.fault)
            self.feedback = fields.get("fb", self.feedback)
            try:
                status_motor_deg = float(fields.get("motor_deg", "nan"))
                if status_motor_deg == status_motor_deg:
                    self.motor_real = status_motor_deg
                status_cmd_deg = float(fields.get("cmd_deg", "nan"))
                if status_cmd_deg == status_cmd_deg:
                    self.motor_cmd = status_cmd_deg
            except ValueError:
                pass
            try:
                self.firmware_command_limit = float(fields.get(
                    "manual_cmd_limit_deg", self.firmware_command_limit
                ))
                self.mechanical_pos_limit = float(fields.get(
                    "manual_mech_pos_deg", self.mechanical_pos_limit
                ))
                self.mechanical_neg_limit = float(fields.get(
                    "manual_mech_neg_deg", self.mechanical_neg_limit
                ))
            except ValueError:
                pass
            self.mechanical_limit_confirmed = (
                fields.get("manual_limit_confirmed") == "1"
            )
            if self.fault != "NONE":
                self.auto_enable_allowed = False
                self.auto_enable_after_zero = False
                self.set_manual_state("FAULT", f"固件故障：{self.fault}")
            elif (fields.get("manual") == "1" and not self.ignore_manual_started
                  and self.auto_enable_allowed and self.pwm_valid):
                if self.manual_state != "ACTIVE" and self.motor_cmd is not None:
                    self.current_manual_angle = max(
                        -PROTOCOL_ANGLE_LIMIT,
                        min(PROTOCOL_ANGLE_LIMIT, self.motor_cmd),
                    )
                self.set_manual_state("ACTIVE", "固件已确认手动模式")
            elif self.manual_state == "CHECKING":
                self.evaluate_connection_ready()
            elif fields.get("manual") == "0" and self.manual_state == "ACTIVE":
                self.auto_enable_allowed = False
                self.set_manual_state("STOPPED", "固件报告遥控已退，需要确认重新启用")
        elif line.startswith("[VISION]"):
            self.vision_valid = fields.get("fresh") == "1"
            if not self.vision_valid:
                self.position_valid = False
        elif line.startswith("[MOTOR_FAULT]"):
            if "fault" in fields:
                self.fault = fields["fault"]
            self.auto_enable_allowed = False
            self.auto_enable_after_zero = False
            self.set_manual_state("FAULT", line)
        elif line.startswith("[ERR]"):
            if "RX_LOST" in line:
                # Never respond per error: that forms a STATUS/error feedback
                # loop. The independent 1 Hz poll already queries MCU state.
                self.rx_loss_count += 1
                self.recorder.event("RX_LOST", line)
                now = time.monotonic()
                if now - self.last_rx_warning >= 1.0:
                    self.last_rx_warning = now
                    self.append_log(line)
                    self.manual_detail_label.configure(
                        text="串口接收错误：正在等待主控状态，检查TX→PA11及共地"
                    )
                return
            if "BAD_COMMAND_OR_RANGE" in line:
                self.recorder.event("RX", line)
                # A malformed frame is not an acknowledgement of START failure.
                return
            if "fault" in fields:
                self.fault = fields["fault"]
            if self.zero_request_pending and "BALANCE" in line:
                self.zero_request_pending = False
                self.auto_enable_allowed = False
                self.auto_enable_after_zero = False
                self.set_manual_state("STOPPED", f"回零失败：{line}")
            elif "MANUAL_MOTOR_NOT_AT_ZERO" in line:
                self.auto_enable_after_zero = True
                self.set_manual_state("NEEDS_ZERO", "需要回平衡零点")
            elif self.manual_state == "STARTING" or "MANUAL_" in line:
                self.auto_enable_allowed = False
                self.set_manual_state("STOPPED", line)
        elif line.startswith("[FAULT]") and fields.get("event") == "CLEARED":
            self.fault = fields.get("fault", "NONE")
            self.pwm_valid = True
            self.feedback = fields.get("fb", self.feedback)
        elif line.startswith("[MANUAL]"):
            event = fields.get("event")
            if (event == "STARTED" and not self.ignore_manual_started
                    and self.auto_enable_allowed):
                self.auto_enable_after_zero = False
                self.set_manual_state("ACTIVE", "遥控待命：可使用方向键")
            elif event in ("STOPPED", "SAFE_ZERO"):
                reason = fields.get("reason", event)
                if reason == "MANUAL_STOP" and self.zero_request_pending:
                    self.set_manual_state("ZEROING", "手动已退出，等待回零……")
                    self.update_dashboard()
                    self.append_log(line)
                    self.recorder.event("RX", line)
                    return
                self.ignore_manual_started = False
                state = ("FAULT" if any(token in reason for token in
                         ("FAULT", "PWM_LOST", "FEEDBACK")) else "STOPPED")
                self.auto_enable_allowed = False
                self.auto_enable_after_zero = False
                self.set_manual_state(state, f"遥控已退出：{reason}，需要确认重新启用")
        elif line.startswith("[BALANCE_ZERO]"):
            event = fields.get("event", "")
            if event == "STARTED":
                self.set_manual_state("ZEROING", "电机正在回平衡零点……")
            elif event == "REACHED":
                self.zero_request_pending = False
                if self.auto_enable_after_zero and self.auto_enable_allowed:
                    self.set_manual_state("STOPPED", "已到达平衡零点，正在启用遥控……")
                    self.after(50, self.start_manual)
                else:
                    self.set_manual_state("STOPPED", "已回零；需要确认重新启用")
        elif line.startswith("[STOP]"):
            reason = fields.get("reason", "STOP")
            if reason == "MANUAL_STOP" and self.zero_request_pending:
                self.set_manual_state("ZEROING", "已停止旧目标，等待回零……")
                self.update_dashboard()
                self.append_log(line)
                self.recorder.event("RX", line)
                return
            self.ignore_manual_started = False
            state = ("FAULT" if any(token in reason for token in
                     ("FAULT", "PWM_LOST", "FEEDBACK")) else "STOPPED")
            self.auto_enable_allowed = False
            self.auto_enable_after_zero = False
            self.set_manual_state(state, f"已停止：{reason}，需要确认重新启用")

        self.update_dashboard()
        self.append_log(line)
        self.recorder.event("RX", line)

    def update_dashboard(self) -> None:
        self.pos_label.configure(text=self.fmt(self.ball_pos, "mm"))
        self.vel_label.configure(text=self.fmt(self.ball_vel, "mm/s"))
        self.motor_cmd_label.configure(text=self.fmt(self.motor_cmd, "°"))
        self.motor_real_label.configure(text=self.fmt(self.motor_real, "°"))
        error = None
        if self.motor_cmd is not None and self.motor_real is not None:
            error = self.motor_cmd - self.motor_real
        self.motor_error_label.configure(text=f"角度误差 {self.fmt(error, '°')}")
        self.vision_label.configure(text=f"视觉：{self.flag(self.vision_valid)}")
        self.position_valid_label.configure(
            text=f"毫米位置：{self.flag(self.position_valid)}"
        )
        self.pwm_label.configure(text=f"PWM：{self.flag(self.pwm_valid)}  FB:{self.feedback}")
        self.fault_label.configure(text=f"Fault：{self.fault}")
        state_name = {
            "DISCONNECTED": "未连接", "CHECKING": "正在检查……",
            "NEEDS_ZERO": "需要回零", "ZEROING": "正在回零……",
            "STOPPED": "已停止", "STARTING": "正在启动……",
            "ACTIVE": "遥控待命", "FAULT": "故障",
        }.get(self.manual_state, self.manual_state)
        self.mode_label.configure(text=f"状态：{state_name}")
        confirmed = "已确认" if self.mechanical_limit_confirmed else "未确认"
        self.limit_label.configure(
            text=(f"协议±{self.firmware_command_limit:.1f}°；机械限位"
                  f"+{self.mechanical_pos_limit:.1f}°/"
                  f"-{self.mechanical_neg_limit:.1f}°（{confirmed}）")
        )
        self.draw_scale()

    @staticmethod
    def fmt(value: float | None, unit: str) -> str:
        return f"{value:+.2f} {unit}" if value is not None else f"-- {unit}"

    @staticmethod
    def flag(value: bool | None) -> str:
        return "有效" if value is True else ("无效" if value is False else "未知")

    def draw_scale(self) -> None:
        canvas = self.scale_canvas
        canvas.delete("all")
        width = max(canvas.winfo_width(), 100)
        left, right, y = 40, width - 40, 38
        canvas.create_line(left, y, right, y, width=3)
        for value in (-50, 0, 50):
            x = left + (value + 60) / 120 * (right - left)
            canvas.create_line(x, y - 10, x, y + 10, width=2)
            canvas.create_text(x, y + 25, text=str(value))
        if self.ball_pos is not None:
            position = max(-60.0, min(60.0, self.ball_pos))
            bx = left + (position + 60) / 120 * (right - left)
            canvas.create_oval(bx - 8, y - 8, bx + 8, y + 8,
                               fill="#d62728", outline="black")

    def set_manual_state(self, state: str, detail: str) -> None:
        changed = state != self.manual_state or detail != self.manual_detail
        self.manual_state = state
        self.manual_active = state == "ACTIVE"
        self.manual_detail = detail
        if not self.manual_active:
            self.current_manual_angle = 0.0
            self.pressed.clear()
            self.cancel_key_releases()
            self.cancel_repeat()
        if hasattr(self, "manual_btn"):
            if state == "NEEDS_ZERO":
                button_text, button_state = "回零并启用", "normal"
            elif state in ("STOPPED", "FAULT"):
                button_text, button_state = "确认重新启用", "normal"
            elif state == "ACTIVE":
                button_text, button_state = "遥控已启用", "disabled"
            elif state == "DISCONNECTED":
                button_text, button_state = "等待串口", "disabled"
            else:
                button_text, button_state = "正在处理……", "disabled"
            self.manual_btn.configure(text=button_text, state=button_state)
            self.manual_detail_label.configure(text=detail)
        if changed:
            self.recorder.event("MANUAL_STATE", f"{state} {detail}")
        self.update_dashboard()

    def evaluate_connection_ready(self) -> None:
        if self.fault != "NONE":
            self.auto_enable_allowed = False
            self.set_manual_state(
                "FAULT", f"固件故障：{self.fault}，不能启用遥控"
            )
            return
        if self.pwm_valid is not True:
            # A STATUS can arrive between two valid PWM captures.  Stay in the
            # automatic connection check so a later valid STATUS may proceed;
            # a real latched motor fault is handled by the branch above.
            self.set_manual_state("CHECKING", "等待有效PWM反馈……")
            return
        if self.motor_real is None:
            self.set_manual_state("CHECKING", "等待电机实际角……")
            return
        if abs(self.motor_real) <= ZERO_READY_TOLERANCE_DEG:
            self.start_manual()
        else:
            self.auto_enable_after_zero = True
            self.set_manual_state(
                "NEEDS_ZERO",
                f"当前{self.motor_real:+.2f}°，需先回平衡零点",
            )

    def confirm_enable(self) -> None:
        if not self.link.connected:
            messagebox.showwarning("未连接", "请先连接串口。")
            return
        if self.pwm_valid is not True or self.fault != "NONE":
            self.send("STATUS")
            messagebox.showwarning(
                "不能启用", "请先清除故障，并确认PWM有效。"
            )
            return
        self.auto_enable_allowed = True
        self.ignore_manual_started = False
        if self.motor_real is None:
            self.set_manual_state("CHECKING", "正在重新读取电机角度……")
            self.send("STATUS", priority=0)
        elif abs(self.motor_real) > ZERO_READY_TOLERANCE_DEG:
            self.balance_zero()
        else:
            self.start_manual()

    def start_manual(self) -> None:
        if not self.auto_enable_allowed:
            return
        if not self.link.connected:
            messagebox.showwarning("未连接", "请先连接串口。")
            return
        if self.pwm_valid is not True or self.fault != "NONE":
            self.auto_enable_allowed = False
            self.set_manual_state("FAULT", "需要PWM有效且Fault=NONE")
            return
        self.set_manual_state("STARTING", "正在等待固件确认……")
        self.ignore_manual_started = False
        self.manual_start_deadline = time.monotonic() + MANUAL_START_TIMEOUT_MS / 1000
        if not self.send("MANUAL,START"):
            self.set_manual_state("STOPPED", "启动命令发送失败")

    def stop_manual(self) -> None:
        self.pressed.clear()
        self.cancel_repeat()
        self.send("MANUAL,STOP")
        self.auto_enable_allowed = False
        self.auto_enable_after_zero = False
        self.zero_request_pending = False
        self.ignore_manual_started = True
        self.set_manual_state("STOPPED", "已停止脉冲，需要确认重新启用")
        self.recorder.event("KEY_STOP")

    def balance_zero(self) -> None:
        self.pressed.clear()
        self.cancel_repeat()
        if not self.link.connected:
            return
        if self.pwm_valid is not True or self.fault != "NONE":
            self.set_manual_state("FAULT", "回零被拒绝：需要PWM有效且Fault=NONE")
            return
        if self.manual_state in ("STARTING", "ACTIVE"):
            self.send("MANUAL,STOP")
        self.auto_enable_allowed = True
        self.auto_enable_after_zero = True
        self.zero_request_pending = True
        self.zero_deadline = time.monotonic() + 12.0
        self.ignore_manual_started = True
        self.set_manual_state("ZEROING", "正在回平衡零点……")
        self.send("BALANCE,ZERO")

    def command_manual_angle(self, value: float, event: str) -> None:
        value = max(-PROTOCOL_ANGLE_LIMIT, min(PROTOCOL_ANGLE_LIMIT, value))
        if self.manual_state != "ACTIVE":
            self.append_log(f"[PC] MANUAL_NOT_ACTIVE state={self.manual_state}")
            return
        if self.pwm_valid is not True or self.fault != "NONE":
            self.append_log("[PC] MOTION_BLOCKED feedback_or_fault")
            return
        directional_limit = (self.mechanical_pos_limit if value >= 0.0 else
                             self.mechanical_neg_limit)
        effective_limit = min(self.firmware_command_limit, directional_limit)
        if abs(value) > effective_limit + 0.0001:
            self.append_log(
                f"[PC] MOTION_BLOCKED configured_limit_deg={effective_limit:.1f}"
            )
            return
        if self.send(f"MANUAL,ANGLE,{value:.3f}"):
            self.current_manual_angle = value
            self.recorder.event(event, f"target_deg={value:.3f}")

    def step_angle(self, key: str, event: str) -> None:
        direction = 1.0 if key == "Right" else -1.0
        positive = min(PROTOCOL_ANGLE_LIMIT, self.firmware_command_limit,
                       self.mechanical_pos_limit)
        negative = min(PROTOCOL_ANGLE_LIMIT, self.firmware_command_limit,
                       self.mechanical_neg_limit)
        value = self.current_manual_angle + direction * ANGLE_STEP_DEG
        value = max(-negative, min(positive, value))
        if abs(value - self.current_manual_angle) < 0.0001:
            self.cancel_repeat()
            return
        self.command_manual_angle(value, event)

    def key_down(self, event: tk.Event) -> str | None:
        key = event.keysym
        lower = key.lower()
        if lower == "escape":
            self.close_window()
            return "break"
        if lower == "s":
            self.stop_manual()
            return "break"
        if key == "space":
            self.recorder.event("KEY_SPACE_ZERO")
            self.balance_zero()
            return "break"
        if key not in ("Left", "Right"):
            return None
        release_job = self.release_jobs.pop(key, None)
        if release_job is not None:
            self.after_cancel(release_job)
            return "break"
        if key in self.pressed:
            return "break"
        self.pressed.add(key)
        self.repeat_key = key
        self.cancel_repeat(clear_key=False)
        self.step_angle(key, f"KEY_{key.upper()}_DOWN")
        if key in self.pressed and self.manual_state == "ACTIVE":
            self.repeat_job = self.after(KEY_REPEAT_DELAY_MS, self.repeat_angle)
        return "break"

    def key_up(self, event: tk.Event) -> str | None:
        key = event.keysym
        if key not in ("Left", "Right"):
            return None
        if key not in self.pressed:
            return "break"
        if key not in self.release_jobs:
            self.release_jobs[key] = self.after(
                30, lambda released=key: self.commit_key_up(released)
            )
        return "break"

    def commit_key_up(self, key: str) -> None:
        self.release_jobs.pop(key, None)
        if key not in self.pressed:
            return
        self.pressed.discard(key)
        self.recorder.event(f"KEY_{key.upper()}_UP")
        if self.repeat_key == key:
            self.cancel_repeat()
            remaining = "Right" if "Right" in self.pressed else (
                "Left" if "Left" in self.pressed else None
            )
            if remaining is not None:
                self.repeat_key = remaining
                self.repeat_job = self.after(KEY_REPEAT_MS, self.repeat_angle)

    def cancel_key_releases(self) -> None:
        for job in self.release_jobs.values():
            self.after_cancel(job)
        self.release_jobs.clear()

    def repeat_angle(self) -> None:
        self.repeat_job = None
        key = self.repeat_key
        if (key not in self.pressed or self.manual_state != "ACTIVE"):
            self.repeat_key = None
            return
        self.step_angle(key, f"KEY_{key.upper()}_REPEAT")
        if self.repeat_key is not None and key in self.pressed:
            self.repeat_job = self.after(KEY_REPEAT_MS, self.repeat_angle)

    def cancel_repeat(self, clear_key: bool = True) -> None:
        if self.repeat_job is not None:
            self.after_cancel(self.repeat_job)
            self.repeat_job = None
        if clear_key:
            self.repeat_key = None

    def focus_guard(self) -> None:
        focus_ok = self.focus_displayof() is not None
        if self.last_focus_ok and not focus_ok and self.pressed:
            self.pressed.clear()
            self.cancel_key_releases()
            self.cancel_repeat()
            self.recorder.event(
                "WINDOW_FOCUS_LOST_HOLD",
                f"target_deg={self.current_manual_angle:.3f}",
            )
        self.last_focus_ok = focus_ok
        self.after(100, self.focus_guard)

    def _heartbeat_loop(self) -> None:
        period = HEARTBEAT_MS / 1000.0
        while not self.heartbeat_stop.wait(period):
            if self.link.connected and self.manual_state in ("STARTING", "ACTIVE"):
                self.link.send("MANUAL,HEARTBEAT", priority=0)

    def manual_state_guard(self) -> None:
        if (self.manual_state == "ZEROING" and self.zero_request_pending
                and time.monotonic() >= self.zero_deadline):
            self.stop_manual()
            self.set_manual_state("STOPPED", "回零确认超时：已请求停止，请检查串口和STATUS")
        if (self.manual_state == "STARTING" and
                time.monotonic() >= self.manual_start_deadline):
            self.auto_enable_allowed = False
            self.auto_enable_after_zero = False
            self.set_manual_state(
                "STOPPED", "启动超时：未收到MANUAL STARTED，需确认重新启用"
            )
        self.after(100, self.manual_state_guard)

    def poll_status(self) -> None:
        if self.link.connected:
            self.send("STATUS", record=False, priority=5)
        self.after(STATUS_PERIOD_MS, self.poll_status)

    def toggle_record(self) -> None:
        if self.recorder.active:
            folder = self.recorder.stop()
            self.record_btn.configure(text="开始记录")
            self.record_label.configure(text=f"已保存：{folder}")
        else:
            try:
                folder = self.recorder.start()
            except OSError as exc:
                messagebox.showerror("记录失败", str(exc))
                return
            self.record_btn.configure(text="停止并保存")
            self.record_label.configure(text=f"记录中：{folder.name}")

    def append_log(self, text: str) -> None:
        stamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.log.configure(state="normal")
        self.log.insert("end", f"[{stamp}] {text}\n")
        if int(self.log.index("end-1c").split(".")[0]) > 600:
            self.log.delete("1.0", "101.0")
        self.log.see("end")
        self.log.configure(state="disabled")

    def close_window(self) -> None:
        if self.manual_active and self.link.connected:
            self.send("MANUAL,ANGLE,0.000", priority=0)
            time.sleep(0.04)
        self.heartbeat_stop.set()
        if self.heartbeat_thread.is_alive():
            self.heartbeat_thread.join(timeout=0.3)
        self.recorder.stop()
        self.link.close()
        self.destroy()


if __name__ == "__main__":
    BallDebugger().mainloop()
