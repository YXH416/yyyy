"""
K230 Camera Photo Capture Module (MOD-016)

A reusable photo capture module for the K230 board that follows the official
Yahboom/Canaan code patterns. It provides live preview on the LCD, key-triggered
photo capture, and saves images using the official `img.save()` method to the
standard `/data/snapshot/` directory with timestamp-based folder naming.

Usage as standalone user_program:
    Copy this file as user_program.py, or update user_program.py to call
    CameraCapture().main().

Usage as imported module:
    from camera_capture import CameraCapture
    cam = CameraCapture()
    cam.start()        # init sensor, display, media
    cam.capture()      # take one photo programmatically
    cam.run_forever()  # live preview + key-triggered capture loop
    cam.stop()         # cleanup

Integration:
    import camera_capture
    camera_capture.main()   # equivalent to standalone entry

Dependencies (K230 MicroPython):
    - media.sensor    (Sensor, CAM_CHN_ID_1)
    - media.display   (Display, ST7701)
    - media.media     (MediaManager)
    - image           (Image)
    - uos             (directory operations)
    - machine         (Pin for key input)
    - time            (sleep, ticks)

"""

import uos
import time
import gc
from media.sensor import *
from media.display import *
from media.media import *
import image

# ── Optional key module ─────────────────────────────────────────────────────
try:
    from ybUtils.YbKey import YbKey
    _KEY_AVAILABLE = True
except ImportError:
    _KEY_AVAILABLE = False

# ── Optional: use machine.Pin directly as fallback ──────────────────────────
try:
    from machine import Pin
    _PIN_AVAILABLE = True
except ImportError:
    _PIN_AVAILABLE = False


# ═══════════════════════════════════════════════════════════════════════════════
# Configuration
# ═══════════════════════════════════════════════════════════════════════════════

# Image resolution (must match the Yahboom ST7701 LCD)
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# Pixel format: RGB565 is the standard for K230 LCD display
PIXEL_FORMAT = Sensor.RGB565

# Sensor channel for capture & display
CAM_CHANNEL = CAM_CHN_ID_1

# Official photo storage base path (K230 convention)
SAVE_BASE_PATH = "/data/snapshot/"

# JPEG quality when saving (if supported by the firmware)
# K230's img.save() typically uses the extension to decide format;
# .jpg extension triggers JPEG compression.
JPEG_QUALITY = 85

# Key debounce delay in milliseconds
KEY_DEBOUNCE_MS = 300

# OSD font scale and colors
OSD_FONT_SIZE = 30
OSD_COLOR_TITLE = (255, 255, 0)     # yellow
OSD_COLOR_INFO = (0, 255, 0)        # green
OSD_COLOR_FLASH = (0, 255, 255)     # cyan (flash feedback when photo saved)


# ═══════════════════════════════════════════════════════════════════════════════
# Utility: recursive directory creation (official K230 pattern)
# ═══════════════════════════════════════════════════════════════════════════════

def ensure_dir(directory):
    """
    Recursively create a directory, following the official K230 snapshot pattern.

    Reference: k230源码/02.Basic/19.snapshot.py
    """
    if not directory or directory == '/':
        return

    directory = directory.rstrip('/')

    try:
        uos.stat(directory)
        return  # already exists
    except OSError:
        pass  # need to create

    # Create parent first
    if '/' in directory:
        parent = directory[:directory.rindex('/')]
        if parent and parent != directory:
            ensure_dir(parent)

    try:
        uos.mkdir(directory)
        print("[Camera] created dir: %s" % directory)
    except OSError:
        # Possible race with concurrent creation — check again
        try:
            uos.stat(directory)
        except Exception:
            print("[Camera] WARNING: failed to create dir: %s" % directory)


# ═══════════════════════════════════════════════════════════════════════════════
# CameraCapture class
# ═══════════════════════════════════════════════════════════════════════════════

class CameraCapture:
    """
    K230 camera photo capture controller.

    Owns the sensor, display, and media lifecycle. Photos are saved via the
    official K230 `img.save()` method to:

        /data/snapshot/<session_timestamp>/<counter>.jpg

    where ``session_timestamp`` is derived from ``time.ticks_us() % 10000``
    (consistent with the Yahboom snapshot.py example).
    """

    def __init__(
        self,
        width=FRAME_WIDTH,
        height=FRAME_HEIGHT,
        save_path=SAVE_BASE_PATH,
        jpeg_quality=JPEG_QUALITY,
        enable_display=True,
        display_type=None,
        to_ide=True,
        hmirror=False,
        vflip=False,
    ):
        """
        Parameters
        ----------
        width, height : int
            Frame size (default 640x480 to match ST7701 LCD).
        save_path : str
            Base directory for saved photos.
        jpeg_quality : int
            Quality hint for JPEG saving (1-100).
        enable_display : bool
            When False, skip LCD init (headless capture mode).
        display_type : int or None
            Display type; defaults to Display.ST7701 when None.
        to_ide : bool
            Route display output to CanMV IDE when True.
        hmirror : bool
            Horizontal mirror.
        vflip : bool
            Vertical flip.
        """
        self.width = width
        self.height = height
        self.save_path = save_path.rstrip('/') + '/'
        self.jpeg_quality = jpeg_quality
        self.enable_display = enable_display
        self.display_type = (
            display_type if display_type is not None else Display.ST7701
        )
        self.to_ide = to_ide
        self.hmirror = hmirror
        self.vflip = vflip

        # Internal state
        self.sensor = None
        self._running = False
        self._key = None
        self._session_folder = None
        self._photo_count = 0
        self._last_key_ms = 0
        self._flash_timer = 0  # ticks_ms when flash started, 0 = no flash

    # ── Public API ──────────────────────────────────────────────────────────

    def start(self):
        """
        Initialise sensor, display, and media manager.

        Must be called before capture() or run_forever().
        """
        if self._running:
            return

        print("[Camera] initialising sensor (%dx%d) ..." % (self.width, self.height))

        # ── Sensor ──────────────────────────────────────────────────────
        self.sensor = Sensor()
        self.sensor.reset()

        # Apply optional mirror/flip before setting framesize
        if self.hmirror:
            self.sensor.set_hmirror(True)
        if self.vflip:
            self.sensor.set_vflip(True)

        self.sensor.set_framesize(
            width=self.width, height=self.height, chn=CAM_CHANNEL
        )
        self.sensor.set_pixformat(PIXEL_FORMAT, chn=CAM_CHANNEL)

        # ── Display ─────────────────────────────────────────────────────
        if self.enable_display:
            Display.init(
                self.display_type,
                width=self.width,
                height=self.height,
                to_ide=self.to_ide,
            )

        # ── Media ───────────────────────────────────────────────────────
        MediaManager.init()

        # ── Start sensor stream ─────────────────────────────────────────
        self.sensor.run()

        # ── Prepare key input ───────────────────────────────────────────
        if _KEY_AVAILABLE:
            self._key = YbKey()

        # ── Generate session folder name (official convention) ─────────
        self._session_folder = str(time.ticks_us() % 10000)
        ensure_dir(self.save_path + self._session_folder)

        self._running = True
        self._photo_count = 0

        print(
            "[Camera] ready — saving to %s%s/"
            % (self.save_path, self._session_folder)
        )
        print("[Camera] press the onboard key to capture a photo")

    def stop(self):
        """
        Stop the sensor, deinit display and release media buffers.

        Idempotent — safe to call multiple times.
        """
        if not self._running:
            return

        print("[Camera] stopping ...")

        self._running = False

        # Stop sensor
        if isinstance(self.sensor, Sensor):
            try:
                self.sensor.stop()
            except Exception as e:
                print("[Camera] sensor.stop error: %s" % e)
        self.sensor = None

        # Deinit display
        if self.enable_display:
            try:
                Display.deinit()
            except Exception as e:
                print("[Camera] Display.deinit error: %s" % e)

        # Allow sleep
        try:
            uos.exitpoint(uos.EXITPOINT_ENABLE_SLEEP)
        except Exception:
            pass
        time.sleep_ms(100)

        # Release media
        try:
            MediaManager.deinit()
        except Exception as e:
            print("[Camera] MediaManager.deinit error: %s" % e)

        gc.collect()
        print("[Camera] stopped — captured %d photos this session" % self._photo_count)

    def capture(self):
        """
        Capture and save one photo immediately.

        Returns
        -------
        str or None
            The full save path on success, None on failure.
        """
        if not self._running or self.sensor is None:
            print("[Camera] ERROR: not started — call start() first")
            return None

        # Grab a frame from the sensor
        img = self.sensor.snapshot(chn=CAM_CHANNEL)
        if img is None:
            print("[Camera] ERROR: snapshot returned None")
            return None

        self._photo_count += 1
        filename = "%d.jpg" % self._photo_count
        save_path = self.save_path + self._session_folder + "/" + filename

        try:
            # ── Official K230 save method ───────────────────────────────
            # JPEG format is inferred from the .jpg extension.
            img.save(save_path)
            print("[Camera] photo saved: %s" % save_path)
            self._flash_timer = time.ticks_ms()
            return save_path
        except Exception as e:
            print("[Camera] ERROR saving photo: %s" % e)
            return None

    def _is_key_pressed(self):
        """Detect key press with debounce. Returns True on rising edge."""
        now = time.ticks_ms()

        # Use YbKey if available, else fall back to Pin polling
        if self._key is not None:
            raw = self._key.is_pressed()
        elif _PIN_AVAILABLE:
            # Yahboom K230 typically uses a key on a specific GPIO;
            # adjust the pin number to match your board.
            try:
                key_pin = Pin(34, Pin.IN, Pin.PULL_UP)
                raw = 1 if key_pin.value() == 0 else 0
            except Exception:
                return False
        else:
            return False

        if raw == 1:
            if time.ticks_diff(now, self._last_key_ms) > KEY_DEBOUNCE_MS:
                self._last_key_ms = now
                return True
        return False

    def _draw_osd(self, img):
        """Draw status overlay on the preview image."""
        # Title line (top-left)
        img.draw_string_advanced(
            10, 10, OSD_FONT_SIZE,
            "K230 Camera",
            color=OSD_COLOR_TITLE,
        )

        # Save directory info
        img.draw_string_advanced(
            10, 48, 22,
            "Dir: %s" % self._session_folder,
            color=OSD_COLOR_INFO,
        )

        # Photo count
        img.draw_string_advanced(
            10, 80, 22,
            "Photos: %d" % self._photo_count,
            color=OSD_COLOR_INFO,
        )

        # Key hint (bottom)
        img.draw_string_advanced(
            10, self.height - 70, 24,
            "Press KEY to capture",
            color=OSD_COLOR_TITLE,
        )

        # Flash feedback when a photo was just taken
        if self._flash_timer:
            elapsed = time.ticks_diff(time.ticks_ms(), self._flash_timer)
            if elapsed < 500:
                img.draw_string_advanced(
                    self.width // 2 - 80, self.height // 2 - 16,
                    40, "SAVED!",
                    color=OSD_COLOR_FLASH,
                )
            else:
                self._flash_timer = 0  # clear flash

    def run_forever(self):
        """
        Main loop: live preview on LCD + key-triggered capture.

        Blocks until KeyboardInterrupt (IDE stop) or exception.
        Call start() before this, or use main() for the all-in-one entry.
        """
        if not self._running:
            self.start()

        fps_clock = time.clock()
        last_status = False

        print("[Camera] entering preview loop — Ctrl+C to stop")
        try:
            while True:
                fps_clock.tick()

                # IDE exit point check
                uos.exitpoint()

                # ── Grab preview frame ──────────────────────────────────
                img = self.sensor.snapshot(chn=CAM_CHANNEL)
                if img is None:
                    time.sleep_ms(10)
                    continue

                # ── Draw OSD ────────────────────────────────────────────
                self._draw_osd(img)

                # ── Show on LCD ─────────────────────────────────────────
                if self.enable_display:
                    Display.show_image(img)

                # ── Key handling (rising-edge detect) ───────────────────
                key_now = self._is_key_pressed()
                if key_now and not last_status:
                    self.capture()
                last_status = key_now

                # ── Periodic GC and FPS print ───────────────────────────
                if self._photo_count > 0 and self._photo_count % 10 == 0:
                    gc.collect()

                time.sleep_ms(5)

        except KeyboardInterrupt:
            print("[Camera] user stopped")
        except BaseException as e:
            print("[Camera] exception: %s" % e)
        finally:
            self.stop()


# ═══════════════════════════════════════════════════════════════════════════════
# Standalone entry point — compatible with main.py → user_program.main()
# ═══════════════════════════════════════════════════════════════════════════════

def main():
    """
    Entry point compatible with the K230 launcher pattern.

    main.py does::

        from user_program import main
        main()

    so this function can be used as the user_program entry, or called
    directly when this file is placed as /sdcard/user_program.py.
    """
    uos.exitpoint(uos.EXITPOINT_ENABLE)

    cam = CameraCapture(
        width=640,
        height=480,
        enable_display=True,
        hmirror=False,
        vflip=False,
    )
    cam.run_forever()


# ── Direct execution support ─────────────────────────────────────────────────
if __name__ == "__main__":
    main()


