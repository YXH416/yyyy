"""ROUND-047 minimal keyboard remote.

Only three actions: +1 deg, -1 deg, return to balance zero.
No PID, no sweep, no CSV, no vision, no manual-mode state machine,
no heartbeat watchdog, no mechanical-limit confirmation, no stop button.
All serial writes stay serialized through one writer thread.
"""

from __future__ import annotations

import queue
import re
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # Shown cleanly after Tk starts.
    serial = None
    list_ports = None


BAUD = 115200
ANGLE_LIMIT_DEG = 15.0
ANGLE_STEP_DEG = 1.0
KEY_REPEAT_DELAY_MS = 300
KEY_REPEAT_MS = 100
STATUS_PERIOD_MS = 250
INVALID = -9999.0
FIELD = re.compile(r"([A-Za-z_][A-Za-z0-9_]*)=([^\s]+)")


class SerialLink:
    """One reader thread and one queued writer thread own the serial port."""

    def __init__(self, inbox: queue.Queue[tuple[str, object]]) -> None:
        self.inbox = inbox
        self.port: serial.Serial | None = None
        self.stop_event = threading.Event()
        self.reader: threading.Thread | None = None
        self.writer: threading.Thread | None = None
        self.tx_queue: queue.Queue[str] = queue.Queue()

    @property
    def connected(self) -> bool:
        return self.port is not None and self.port.is_open

    def connect(self, name: str) -> None:
        self.close()
        self.stop_event.clear()
        self.tx_queue = queue.Queue()
        port = serial.Serial(name, BAUD, timeout=0.1, write_timeout=0.2)
        self.port = port
        self.reader = threading.Thread(target=self._read_loop, args=(port,), daemon=True)
        self.writer = threading.Thread(target=self._write_loop, args=(port,), daemon=True)
        self.reader.start()
        self.writer.start()

    def send(self, command: str) -> bool:
        if not self.connected:
            return False
        command = command.rstrip("\r\n")
        try:
            self.tx_queue.put_nowait(command)
            return True
        except queue.Full:
            self.inbox.put(("tx_error", f"TX_QUEUE_FULL command={command}"))
            return False

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
                    command = self.tx_queue.get(timeout=0.1)
                except queue.Empty:
                    continue
                frame = (command + "\n").encode("ascii")
                if port.write(frame) != len(frame):
                    raise OSError("Incomplete serial command write")
        except (serial.SerialException, OSError) as exc:
            if not self.stop_event.is_set():
                self.inbox.put(("disconnect", str(exc)))


class BallRemote(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("平衡球遥控器 · ROUND-047")
        self.geometry("420x300")
        self.minsize(380, 260)
        self.inbox: queue.Queue[tuple[str, object]] = queue.Queue()
        self.link = SerialLink(self.inbox)
        self.target_angle = 0.0
        self.actual_angle: float | None = None
        self.pressed: set[str] = set()
        self.release_jobs: dict[str, str] = {}
        self.repeat_job: str | None = None
        self.repeat_key: str | None = None
        self.last_focus_ok = True
        self._build_ui()
        self._bind_keys()
        self.refresh_ports()
        self.protocol("WM_DELETE_WINDOW", self.close_window)
        self.after(20, self.process_inbox)
        self.after(STATUS_PERIOD_MS, self.poll_status)
        self.after(100, self.focus_guard)
        if serial is None:
            self.after(100, lambda: messagebox.showerror(
                "缺少依赖", "未安装 pyserial。\n请运行：py -3 -m pip install pyserial"
            ))

    def _build_ui(self) -> None:
        style = ttk.Style(self)
        style.configure("Big.TLabel", font=("Microsoft YaHei UI", 26, "bold"))
        style.configure("State.TLabel", font=("Microsoft YaHei UI", 11, "bold"))

        top = ttk.Frame(self, padding=12)
        top.pack(fill="x")
        ttk.Label(top, text="串口：").pack(side="left")
        self.port_var = tk.StringVar(value="COM17")
        self.port_box = ttk.Combobox(top, textvariable=self.port_var, width=12)
        self.port_box.pack(side="left", padx=4)
        ttk.Button(top, text="刷新", command=self.refresh_ports).pack(side="left")
        self.connect_btn = ttk.Button(top, text="连接", command=self.toggle_connection)
        self.connect_btn.pack(side="left", padx=6)
        self.com_label = ttk.Label(top, text="未连接", style="State.TLabel")
        self.com_label.pack(side="left", padx=10)

        body = ttk.Frame(self, padding=(16, 10))
        body.pack(fill="both", expand=True)
        ttk.Label(body, text="目标角").pack(anchor="w")
        self.target_label = ttk.Label(body, text="+0.0°", style="Big.TLabel")
        self.target_label.pack(anchor="w", pady=(0, 12))
        ttk.Label(body, text="实际角").pack(anchor="w")
        self.actual_label = ttk.Label(body, text="--°", style="Big.TLabel")
        self.actual_label.pack(anchor="w")

        hint = ttk.Frame(self, padding=(12, 0, 12, 12))
        hint.pack(fill="x")
        ttk.Label(
            hint,
            text="→ +1°　← -1°　长按300ms后每100ms连续加减　空格回平衡零点　Esc退出"
        ).pack(anchor="w")

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
        name = self.port_var.get().strip()
        if not name:
            messagebox.showwarning("未选择串口", "请先选择串口。")
            return
        try:
            self.link.connect(name)
        except (serial.SerialException, OSError, ValueError) as exc:
            messagebox.showerror("连接失败", str(exc))
            return
        self.connect_btn.configure(text="断开")
        self.com_label.configure(text=f"{name} · 已连接")
        self.target_angle = 0.0
        self.actual_angle = None
        self.send("STATUS")
        self.update_display()

    def disconnect(self, reason: str) -> None:
        self.pressed.clear()
        self.cancel_key_releases()
        self.cancel_repeat()
        self.link.close()
        self.connect_btn.configure(text="连接")
        self.com_label.configure(text="未连接")
        self.actual_angle = None
        self.update_display()

    def send(self, command: str) -> bool:
        return self.link.send(command)

    def send_relative(self, value: float) -> None:
        if not self.link.connected:
            return
        self.send(f"ANGLE_REL,{value:+.1f}")

    def balance_zero(self) -> None:
        self.pressed.clear()
        self.cancel_key_releases()
        self.cancel_repeat()
        self.target_angle = 0.0
        self.send("BALANCE,ZERO")
        self.update_display()

    def process_inbox(self) -> None:
        deadline = time.monotonic() + 0.01
        for _ in range(100):
            if time.monotonic() >= deadline:
                break
            try:
                kind, payload = self.inbox.get_nowait()
            except queue.Empty:
                break
            if kind == "line":
                self.process_line(str(payload))
            elif kind == "disconnect":
                self.disconnect(str(payload))
            elif kind == "tx_error":
                self.com_label.configure(text=str(payload))
        self.after(20, self.process_inbox)

    def process_line(self, line: str) -> None:
        if not line.startswith("[STATUS]"):
            return
        fields = dict(FIELD.findall(line))
        raw = fields.get("motor_deg")
        if raw is None:
            return
        try:
            value = float(raw)
        except ValueError:
            return
        self.actual_angle = None if value <= INVALID else value
        self.update_display()

    def poll_status(self) -> None:
        if self.link.connected:
            self.send("STATUS")
        self.after(STATUS_PERIOD_MS, self.poll_status)

    def update_display(self) -> None:
        self.target_label.configure(text=self.fmt(self.target_angle))
        self.actual_label.configure(text=self.fmt(self.actual_angle))

    @staticmethod
    def fmt(value: float | None) -> str:
        return f"{value:+.1f}°" if value is not None else "--°"

    def step_angle(self, key: str) -> None:
        direction = 1.0 if key == "Right" else -1.0
        value = max(-ANGLE_LIMIT_DEG,
                    min(ANGLE_LIMIT_DEG, self.target_angle + direction * ANGLE_STEP_DEG))
        if abs(value - self.target_angle) < 1e-9:
            self.cancel_repeat()
            return
        self.target_angle = value
        self.send_relative(value)
        self.update_display()

    def key_down(self, event: tk.Event) -> str | None:
        key = event.keysym
        if key == "Escape":
            self.close_window()
            return "break"
        if key == "space":
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
        self.step_angle(key)
        if self.repeat_key is not None:
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
        if key is None or key not in self.pressed:
            self.repeat_key = None
            return
        self.step_angle(key)
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
        self.last_focus_ok = focus_ok
        self.after(100, self.focus_guard)

    def close_window(self) -> None:
        self.cancel_repeat()
        self.cancel_key_releases()
        self.link.close()
        self.destroy()


if __name__ == "__main__":
    BallRemote().mainloop()
