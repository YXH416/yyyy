"""ROUND-043 Windows keyboard debugger for the MSPM0 ball-balance rig."""

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
FINE_REPEAT_DELAY_MS = 350
FINE_REPEAT_MS = 120
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
                port.write((wire + "\n").encode("ascii"))
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
        self.title("平衡球键盘调试器 · ROUND-043")
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
        self.manual_state = "STOPPED"
        self.manual_detail = "尚未进入手动"
        self.manual_start_deadline = 0.0
        self.ignore_manual_started = False
        self.firmware_command_limit = PROTOCOL_ANGLE_LIMIT
        self.mechanical_pos_limit = PROTOCOL_ANGLE_LIMIT
        self.mechanical_neg_limit = PROTOCOL_ANGLE_LIMIT
        self.mechanical_limit_confirmed = False
        self.current_manual_angle = 0.0
        self.pressed: set[str] = set()
        self.release_jobs: dict[str, str] = {}
        self.fine_repeat_job: str | None = None
        self.fine_repeat_key: str | None = None
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
        self.target_pos_var = tk.DoubleVar(value=0.0)
        self.pos_label.pack(anchor="w")
        self.vel_label.pack(anchor="w")
        self.target_label = ttk.Label(ball, text="目标位置  +0 mm")
        self.target_label.pack(anchor="w")

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
        self.mode_var = tk.StringVar(value="direct")
        ttk.Radiobutton(controls, text="直接倾斜（默认）", variable=self.mode_var,
                        value="direct", command=self.mode_changed).grid(row=0, column=0, sticky="w")
        ttk.Radiobutton(controls, text="微调", variable=self.mode_var,
                        value="fine", command=self.mode_changed).grid(row=0, column=1, sticky="w")
        ttk.Label(controls, text="倾角幅值°").grid(row=1, column=0, sticky="e")
        self.amplitude_var = tk.StringVar(value="0.5")
        ttk.Combobox(controls, textvariable=self.amplitude_var, width=8,
                     values=("0.5", "1.0", "2.0", "3.0", "5.0", "10.0", "15.0")).grid(row=1, column=1, sticky="w", padx=5)
        ttk.Label(controls, text="微调步长°").grid(row=1, column=2, sticky="e")
        self.step_var = tk.StringVar(value="0.1")
        ttk.Combobox(controls, textvariable=self.step_var, width=8,
                     values=("0.1", "0.2", "0.5", "1.0")).grid(row=1, column=3, sticky="w", padx=5)
        ttk.Label(controls, text="目标位置").grid(row=2, column=0, sticky="e")
        targets = ttk.Frame(controls)
        targets.grid(row=2, column=1, columnspan=3, sticky="w")
        for value in (-50, 0, 50):
            ttk.Radiobutton(targets, text=f"{value:+d} mm", variable=self.target_pos_var,
                            value=float(value), command=self.target_changed).pack(side="left", padx=3)
        self.manual_btn = ttk.Button(controls, text="进入手动", command=self.toggle_manual)
        self.manual_btn.grid(row=0, column=4, padx=14)
        ttk.Button(controls, text="停止脉冲 (S)", command=self.stop_manual).grid(row=1, column=4, padx=14)
        ttk.Label(controls, text="按住→/←给倾角；松开回0°；空格回平衡零点；↑/↓调幅值或步长").grid(
            row=3, column=0, columnspan=5, sticky="w", pady=(8, 0)
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
        self.send("STREAM,ON", record=False, priority=5)
        self.send("STATUS", record=False, priority=5)

    def disconnect(self, reason: str) -> None:
        if self.manual_active and self.link.connected:
            self.send("MANUAL,ANGLE,0.000", priority=0)
            time.sleep(0.04)
        self.link.close()
        self.cancel_fine_repeat()
        self.set_manual_state("STOPPED", f"串口已断开：{reason}")
        self.connect_btn.configure(text="连接")
        self.com_label.configure(text="未连接")
        self.mode_label.configure(text="状态：停止")
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
        try:
            while True:
                kind, payload = self.inbox.get_nowait()
                if kind == "line":
                    self.process_line(str(payload))
                elif kind == "tx":
                    sequence, wire, queued_ns, sent_ns = payload
                    if not str(wire).startswith("MANUAL,HEARTBEAT"):
                        delay_ms = (sent_ns - queued_ns) / 1_000_000
                        self.append_log(
                            f"[TX] seq={sequence} queue_ms={delay_ms:.1f} {wire}"
                        )
                        self.recorder.event(
                            "TX", f"seq={sequence} queue_ms={delay_ms:.1f} {wire}"
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
            if fields.get("manual") == "1" and not self.ignore_manual_started:
                self.set_manual_state("ACTIVE", "固件已确认手动模式")
            elif fields.get("manual") == "0" and self.manual_state == "ACTIVE":
                self.set_manual_state("STOPPED", "固件报告手动模式已退出")
        elif line.startswith("[VISION]"):
            self.vision_valid = fields.get("fresh") == "1"
            if not self.vision_valid:
                self.position_valid = False
        elif line.startswith("[MOTOR_FAULT]"):
            if "fault" in fields:
                self.fault = fields["fault"]
            self.set_manual_state("FAULT", line)
        elif line.startswith("[ERR]"):
            if "fault" in fields:
                self.fault = fields["fault"]
            if self.manual_state == "STARTING" or "MANUAL_" in line:
                self.set_manual_state("STOPPED", line)
        elif line.startswith("[FAULT]") and fields.get("event") == "CLEARED":
            self.fault = fields.get("fault", "NONE")
            self.pwm_valid = True
            self.feedback = fields.get("fb", self.feedback)
        elif line.startswith("[MANUAL]"):
            event = fields.get("event")
            if event == "STARTED" and not self.ignore_manual_started:
                self.set_manual_state("ACTIVE", "手动已启动，可使用方向键")
            elif event in ("STOPPED", "SAFE_ZERO"):
                self.ignore_manual_started = False
                reason = fields.get("reason", event)
                state = ("FAULT" if any(token in reason for token in
                         ("FAULT", "PWM_LOST", "FEEDBACK")) else "STOPPED")
                self.set_manual_state(state, f"手动已退出：{reason}")
        elif line.startswith("[BALANCE_ZERO]"):
            event = fields.get("event", "")
            if event == "STARTED":
                self.set_manual_state("STOPPED", "电机正在回平衡零点……")
            elif event == "REACHED":
                self.set_manual_state("STOPPED", "已到达平衡零点")
        elif line.startswith("[STOP]"):
            self.ignore_manual_started = False
            reason = fields.get("reason", "STOP")
            state = ("FAULT" if any(token in reason for token in
                     ("FAULT", "PWM_LOST", "FEEDBACK")) else "STOPPED")
            self.set_manual_state(state, f"已停止：{reason}")

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
            "STOPPED": "停止", "STARTING": "正在启动……",
            "ACTIVE": "手动已启动", "FAULT": "故障",
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
        target = max(-60.0, min(60.0, self.target_pos_var.get()))
        tx = left + (target + 60) / 120 * (right - left)
        canvas.create_polygon(tx, y - 18, tx - 7, y - 30, tx + 7, y - 30,
                              fill="#1f77b4", outline="")
        if self.ball_pos is not None:
            position = max(-60.0, min(60.0, self.ball_pos))
            bx = left + (position + 60) / 120 * (right - left)
            canvas.create_oval(bx - 8, y - 8, bx + 8, y + 8,
                               fill="#d62728", outline="black")

    def target_changed(self) -> None:
        self.target_label.configure(text=f"目标位置  {self.target_pos_var.get():+.0f} mm")
        self.draw_scale()
        self.recorder.event("TARGET_DISPLAY", f"{self.target_pos_var.get():.1f}")

    def set_manual_state(self, state: str, detail: str) -> None:
        changed = state != self.manual_state or detail != self.manual_detail
        self.manual_state = state
        self.manual_active = state == "ACTIVE"
        self.manual_detail = detail
        if not self.manual_active:
            self.current_manual_angle = 0.0
            self.pressed.clear()
            self.cancel_key_releases()
            self.cancel_fine_repeat()
        if hasattr(self, "manual_btn"):
            self.manual_btn.configure(
                text="退出手动" if state in ("STARTING", "ACTIVE") else "进入手动"
            )
            self.manual_detail_label.configure(text=detail)
        if changed:
            self.recorder.event("MANUAL_STATE", f"{state} {detail}")
        self.update_dashboard()

    def toggle_manual(self) -> None:
        if self.manual_state in ("STARTING", "ACTIVE"):
            self.stop_manual()
        else:
            self.start_manual()

    def start_manual(self) -> None:
        if not self.link.connected:
            messagebox.showwarning("未连接", "请先连接串口。")
            return
        if self.pwm_valid is not True or self.fault != "NONE":
            self.send("STATUS")
            messagebox.showwarning("禁止运动", "需要 PWM 有效且 Fault=NONE。")
            return
        self.set_manual_state("STARTING", "正在等待固件确认……")
        self.ignore_manual_started = False
        self.manual_start_deadline = time.monotonic() + MANUAL_START_TIMEOUT_MS / 1000
        if not self.send("MANUAL,START"):
            self.set_manual_state("STOPPED", "启动命令发送失败")

    def stop_manual(self) -> None:
        self.pressed.clear()
        self.cancel_fine_repeat()
        if self.manual_state in ("STARTING", "ACTIVE"):
            self.send("MANUAL,STOP")
        self.ignore_manual_started = True
        self.set_manual_state("STOPPED", "用户停止脉冲")
        self.recorder.event("KEY_STOP")

    def balance_zero(self) -> None:
        self.pressed.clear()
        self.cancel_fine_repeat()
        if self.manual_state in ("STARTING", "ACTIVE"):
            self.send("MANUAL,STOP")
        self.ignore_manual_started = True
        self.set_manual_state("STOPPED", "正在回平衡零点")
        self.send("BALANCE,ZERO")

    def angle_value(self, variable: tk.StringVar, fallback: float) -> float:
        try:
            value = abs(float(variable.get()))
        except ValueError:
            value = fallback
        return max(0.01, min(PROTOCOL_ANGLE_LIMIT, value))

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
        if key in ("Up", "Down"):
            self.adjust_setting(1 if key == "Up" else -1)
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
        if self.mode_var.get() == "direct":
            amplitude = self.angle_value(self.amplitude_var, 0.5)
            value = amplitude if key == "Right" else -amplitude
        else:
            step = self.angle_value(self.step_var, 0.1)
            value = self.current_manual_angle + (step if key == "Right" else -step)
        self.command_manual_angle(value, f"KEY_{key.upper()}_DOWN")
        if self.mode_var.get() == "fine":
            self.fine_repeat_key = key
            self.cancel_fine_repeat(clear_key=False)
            self.fine_repeat_job = self.after(
                FINE_REPEAT_DELAY_MS, self.fine_repeat
            )
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
        if self.fine_repeat_key == key:
            self.cancel_fine_repeat()
        if self.mode_var.get() == "direct":
            if "Right" in self.pressed:
                value = self.angle_value(self.amplitude_var, 0.5)
            elif "Left" in self.pressed:
                value = -self.angle_value(self.amplitude_var, 0.5)
            else:
                value = 0.0
            self.command_manual_angle(value, "KEY_RELEASE_TARGET")

    def cancel_key_releases(self) -> None:
        for job in self.release_jobs.values():
            self.after_cancel(job)
        self.release_jobs.clear()

    def fine_repeat(self) -> None:
        self.fine_repeat_job = None
        key = self.fine_repeat_key
        if (self.mode_var.get() != "fine" or key not in self.pressed or
                self.manual_state != "ACTIVE"):
            self.fine_repeat_key = None
            return
        step = self.angle_value(self.step_var, 0.1)
        value = self.current_manual_angle + (step if key == "Right" else -step)
        self.command_manual_angle(value, f"KEY_{key.upper()}_REPEAT")
        self.fine_repeat_job = self.after(FINE_REPEAT_MS, self.fine_repeat)

    def cancel_fine_repeat(self, clear_key: bool = True) -> None:
        if self.fine_repeat_job is not None:
            self.after_cancel(self.fine_repeat_job)
            self.fine_repeat_job = None
        if clear_key:
            self.fine_repeat_key = None

    def adjust_setting(self, direction: int) -> None:
        if self.mode_var.get() == "direct":
            value = self.angle_value(self.amplitude_var, 0.5) + 0.1 * direction
            self.amplitude_var.set(
                f"{max(0.1, min(PROTOCOL_ANGLE_LIMIT, value)):.1f}"
            )
            self.recorder.event("AMPLITUDE_CHANGE", self.amplitude_var.get())
        else:
            value = self.angle_value(self.step_var, 0.1) + 0.05 * direction
            self.step_var.set(f"{max(0.1, min(1.0, value)):.2f}")
            self.recorder.event("STEP_CHANGE", self.step_var.get())

    def mode_changed(self) -> None:
        self.pressed.clear()
        self.cancel_key_releases()
        self.cancel_fine_repeat()
        if self.manual_active:
            self.command_manual_angle(0.0, "MODE_CHANGE_ZERO")
        self.recorder.event("MODE_CHANGE", self.mode_var.get())

    def focus_guard(self) -> None:
        focus_ok = self.focus_displayof() is not None
        if (self.last_focus_ok and not focus_ok and self.manual_active and
                (self.pressed or abs(self.current_manual_angle) > 0.0001)):
            self.pressed.clear()
            self.cancel_key_releases()
            self.command_manual_angle(0.0, "WINDOW_FOCUS_LOST_ZERO")
        self.last_focus_ok = focus_ok
        self.after(100, self.focus_guard)

    def _heartbeat_loop(self) -> None:
        period = HEARTBEAT_MS / 1000.0
        while not self.heartbeat_stop.wait(period):
            if self.link.connected and self.manual_state == "ACTIVE":
                self.link.send("MANUAL,HEARTBEAT", priority=0)

    def manual_state_guard(self) -> None:
        if (self.manual_state == "STARTING" and
                time.monotonic() >= self.manual_start_deadline):
            self.set_manual_state(
                "STOPPED", "启动超时：未收到MANUAL STARTED，请检查串口日志"
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
