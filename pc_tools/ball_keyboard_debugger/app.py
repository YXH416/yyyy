"""ROUND-042 Windows keyboard debugger for the MSPM0 ball-balance rig."""

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
FIRMWARE_ANGLE_LIMIT = 2.0
HEARTBEAT_MS = 200
STATUS_PERIOD_MS = 1000
BALL_LINE = re.compile(
    r"^ball:([-+0-9.eE]+),([-+0-9.eE]+),([-+0-9.eE]+),([-+0-9.eE]+)$"
)
FIELD = re.compile(r"([A-Za-z_][A-Za-z0-9_]*)=([^\s]+)")


class SerialLink:
    def __init__(self, inbox: queue.Queue[tuple[str, str]]) -> None:
        self.inbox = inbox
        self.port: serial.Serial | None = None
        self.stop_event = threading.Event()
        self.write_lock = threading.Lock()
        self.reader: threading.Thread | None = None

    @property
    def connected(self) -> bool:
        return self.port is not None and self.port.is_open

    def connect(self, name: str) -> None:
        self.close()
        self.stop_event.clear()
        self.port = serial.Serial(name, BAUD, timeout=0.1, write_timeout=0.2)
        self.reader = threading.Thread(target=self._read_loop, daemon=True)
        self.reader.start()

    def send(self, command: str) -> bool:
        if not self.connected:
            return False
        try:
            with self.write_lock:
                assert self.port is not None
                self.port.write((command.rstrip("\r\n") + "\n").encode("ascii"))
            return True
        except (serial.SerialException, OSError) as exc:
            self.inbox.put(("disconnect", str(exc)))
            return False

    def close(self) -> None:
        self.stop_event.set()
        port, self.port = self.port, None
        reader, self.reader = self.reader, None
        if port is not None:
            try:
                port.close()
            except (serial.SerialException, OSError):
                pass
        if (reader is not None and reader.is_alive() and
                reader is not threading.current_thread()):
            reader.join(timeout=0.3)

    def _read_loop(self) -> None:
        buffer = bytearray()
        try:
            while not self.stop_event.is_set() and self.connected:
                assert self.port is not None
                chunk = self.port.read(self.port.in_waiting or 1)
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
            "motor_real_deg", "vision_valid", "pwm_valid", "fault", "mode",
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
        self.title("平衡球键盘调试器 · ROUND-042")
        self.geometry("980x720")
        self.minsize(860, 650)
        self.inbox: queue.Queue[tuple[str, str]] = queue.Queue()
        self.link = SerialLink(self.inbox)
        self.recorder = Recorder()
        self.ball_pos = self.ball_vel = self.motor_cmd = self.motor_real = None
        self.vision_valid: bool | None = None
        self.pwm_valid: bool | None = None
        self.fault = "UNKNOWN"
        self.feedback = "UNKNOWN"
        self.manual_active = False
        self.current_manual_angle = 0.0
        self.pressed: set[str] = set()
        self.last_focus_ok = True
        self._build_ui()
        self._bind_keys()
        self.refresh_ports()
        self.protocol("WM_DELETE_WINDOW", self.close_window)
        self.after(20, self.process_inbox)
        self.after(HEARTBEAT_MS, self.heartbeat)
        self.after(STATUS_PERIOD_MS, self.poll_status)
        self.after(100, self.focus_guard)
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
        ttk.Button(top, text="回平衡零点", command=lambda: self.send("BALANCE,ZERO")).pack(side="right", padx=4)
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
        self.pwm_label = ttk.Label(system, text="PWM：未知", style="State.TLabel")
        self.fault_label = ttk.Label(system, text="Fault：UNKNOWN", style="State.TLabel")
        self.mode_label = ttk.Label(system, text="状态：停止", style="State.TLabel")
        for widget in (self.vision_label, self.pwm_label, self.fault_label, self.mode_label):
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
                     values=("0.5", "1.0", "1.5", "2.0")).grid(row=1, column=1, sticky="w", padx=5)
        ttk.Label(controls, text="微调步长°").grid(row=1, column=2, sticky="e")
        self.step_var = tk.StringVar(value="0.1")
        ttk.Entry(controls, textvariable=self.step_var, width=8).grid(row=1, column=3, sticky="w", padx=5)
        ttk.Label(controls, text="目标位置").grid(row=2, column=0, sticky="e")
        targets = ttk.Frame(controls)
        targets.grid(row=2, column=1, columnspan=3, sticky="w")
        for value in (-50, 0, 50):
            ttk.Radiobutton(targets, text=f"{value:+d} mm", variable=self.target_pos_var,
                            value=float(value), command=self.target_changed).pack(side="left", padx=3)
        self.manual_btn = ttk.Button(controls, text="进入手动", command=self.start_manual)
        self.manual_btn.grid(row=0, column=4, padx=14)
        ttk.Button(controls, text="停止脉冲 (S)", command=self.stop_manual).grid(row=1, column=4, padx=14)
        ttk.Label(controls, text="按住→/←给倾角；松开回0°；空格回0°；↑/↓调幅值或步长").grid(
            row=3, column=0, columnspan=5, sticky="w", pady=(8, 0)
        )

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
        self.send("STREAM,ON", record=False)
        self.send("STATUS", record=False)

    def disconnect(self, reason: str) -> None:
        if self.manual_active and self.link.connected:
            self.send("MANUAL,ANGLE,0.000")
        self.link.close()
        self.manual_active = False
        self.pressed.clear()
        self.connect_btn.configure(text="连接")
        self.com_label.configure(text="未连接")
        self.mode_label.configure(text="状态：停止")
        self.append_log(f"[PC] DISCONNECT reason={reason}")
        self.recorder.event("DISCONNECT", reason)

    def send(self, command: str, record: bool = True) -> bool:
        ok = self.link.send(command)
        if not ok:
            if record:
                self.append_log("[PC] SEND_FAILED not_connected")
            return False
        if record and command != "MANUAL,HEARTBEAT":
            self.recorder.event("COMMAND", command)
        return True

    def process_inbox(self) -> None:
        try:
            while True:
                kind, payload = self.inbox.get_nowait()
                if kind == "line":
                    self.process_line(payload)
                else:
                    self.disconnect(payload)
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
            self.update_dashboard()
            self.recorder.sample([
                self.ball_pos, self.ball_vel, self.motor_cmd, self.motor_real,
                self.vision_valid, self.pwm_valid, self.fault,
                "manual" if self.manual_active else "stopped",
            ])
            return

        fields = dict(FIELD.findall(line))
        if line.startswith("[STATUS]"):
            self.vision_valid = fields.get("vision") == "1"
            self.pwm_valid = fields.get("pwm_valid") == "1"
            self.fault = fields.get("fault", self.fault)
            self.feedback = fields.get("fb", self.feedback)
            self.manual_active = fields.get("manual") == "1"
        elif line.startswith("[VISION]"):
            self.vision_valid = fields.get("fresh") == "1"
        elif line.startswith("[MOTOR_FAULT]") or line.startswith("[ERR]"):
            if "fault" in fields:
                self.fault = fields["fault"]
        elif line.startswith("[FAULT]") and fields.get("event") == "CLEARED":
            self.fault = fields.get("fault", "NONE")
            self.pwm_valid = True
            self.feedback = fields.get("fb", self.feedback)
        elif line.startswith("[MANUAL]"):
            event = fields.get("event")
            if event == "STARTED":
                self.manual_active = True
            elif event in ("STOPPED", "SAFE_ZERO"):
                self.manual_active = False
                self.pressed.clear()
        elif line.startswith("[STOP]"):
            self.manual_active = False
            self.pressed.clear()

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
        self.pwm_label.configure(text=f"PWM：{self.flag(self.pwm_valid)}  FB:{self.feedback}")
        self.fault_label.configure(text=f"Fault：{self.fault}")
        self.mode_label.configure(text="状态：手动控制" if self.manual_active else "状态：停止")
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

    def start_manual(self) -> None:
        if not self.link.connected:
            messagebox.showwarning("未连接", "请先连接串口。")
            return
        if self.pwm_valid is not True or self.fault != "NONE":
            self.send("STATUS")
            messagebox.showwarning("禁止运动", "需要 PWM 有效且 Fault=NONE。")
            return
        self.send("MANUAL,START")

    def stop_manual(self) -> None:
        self.pressed.clear()
        self.send("MANUAL,STOP")
        self.recorder.event("KEY_STOP")

    def angle_value(self, variable: tk.StringVar, fallback: float) -> float:
        try:
            value = abs(float(variable.get()))
        except ValueError:
            value = fallback
        return max(0.01, min(FIRMWARE_ANGLE_LIMIT, value))

    def command_manual_angle(self, value: float, event: str) -> None:
        value = max(-FIRMWARE_ANGLE_LIMIT, min(FIRMWARE_ANGLE_LIMIT, value))
        if not self.manual_active:
            self.append_log("[PC] MANUAL_NOT_ACTIVE")
            return
        if self.pwm_valid is not True or self.fault != "NONE":
            self.append_log("[PC] MOTION_BLOCKED feedback_or_fault")
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
            self.pressed.clear()
            self.command_manual_angle(0.0, "KEY_SPACE_ZERO")
            return "break"
        if key in ("Up", "Down"):
            self.adjust_setting(1 if key == "Up" else -1)
            return "break"
        if key not in ("Left", "Right") or key in self.pressed:
            return None
        self.pressed.add(key)
        if self.mode_var.get() == "direct":
            amplitude = self.angle_value(self.amplitude_var, 0.5)
            value = amplitude if key == "Right" else -amplitude
        else:
            step = self.angle_value(self.step_var, 0.1)
            value = self.current_manual_angle + (step if key == "Right" else -step)
        self.command_manual_angle(value, f"KEY_{key.upper()}_DOWN")
        return "break"

    def key_up(self, event: tk.Event) -> str | None:
        key = event.keysym
        if key not in ("Left", "Right"):
            return None
        self.pressed.discard(key)
        self.recorder.event(f"KEY_{key.upper()}_UP")
        if self.mode_var.get() == "direct":
            if "Right" in self.pressed:
                value = self.angle_value(self.amplitude_var, 0.5)
            elif "Left" in self.pressed:
                value = -self.angle_value(self.amplitude_var, 0.5)
            else:
                value = 0.0
            self.command_manual_angle(value, "KEY_RELEASE_TARGET")
        return "break"

    def adjust_setting(self, direction: int) -> None:
        if self.mode_var.get() == "direct":
            value = self.angle_value(self.amplitude_var, 0.5) + 0.1 * direction
            self.amplitude_var.set(f"{max(0.1, min(2.0, value)):.1f}")
            self.recorder.event("AMPLITUDE_CHANGE", self.amplitude_var.get())
        else:
            value = self.angle_value(self.step_var, 0.1) + 0.05 * direction
            self.step_var.set(f"{max(0.05, min(0.5, value)):.2f}")
            self.recorder.event("STEP_CHANGE", self.step_var.get())

    def mode_changed(self) -> None:
        self.pressed.clear()
        if self.manual_active:
            self.command_manual_angle(0.0, "MODE_CHANGE_ZERO")
        self.recorder.event("MODE_CHANGE", self.mode_var.get())

    def focus_guard(self) -> None:
        focus_ok = self.focus_displayof() is not None
        if self.last_focus_ok and not focus_ok and self.pressed:
            self.pressed.clear()
            self.command_manual_angle(0.0, "WINDOW_FOCUS_LOST_ZERO")
        self.last_focus_ok = focus_ok
        self.after(100, self.focus_guard)

    def heartbeat(self) -> None:
        if self.link.connected and self.manual_active:
            self.send("MANUAL,HEARTBEAT", record=False)
        self.after(HEARTBEAT_MS, self.heartbeat)

    def poll_status(self) -> None:
        if self.link.connected:
            self.send("STATUS", record=False)
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
            self.send("MANUAL,ANGLE,0.000")
            time.sleep(0.03)
        self.recorder.stop()
        self.link.close()
        self.destroy()


if __name__ == "__main__":
    BallDebugger().mainloop()
