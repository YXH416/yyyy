# K230 rectangle-center user program with reusable threshold runtime.
import time, os, math
from math import sqrt
from media.sensor import *
from media.display import *
from media.media import *
import cv_lite
from time import ticks_ms
from machine import Pin
from ybUtils.YbUart import YbUart
from yb_ascii_protocol import send_center
from threshold_runtime import ThresholdRuntime

CHANGE_NAME = "MOD_011_LAB_GATED_DUAL_CHANNEL"
UART_SEND_INTERVAL_MS = 50

INC_KEY = Pin(32, Pin.IN, Pin.PULL_UP)   # 增大
DEC_KEY = Pin(33, Pin.IN, Pin.PULL_UP)   # 减小

# The verified Yahboom EXPORT transport uses IO9/IO10 at 115200 baud.
uart = YbUart(baudrate=115200)

DISPLAY_WIDTH = ALIGN_UP(640, 16)
DISPLAY_HEIGHT = 480
PROCESS_WIDTH = 320
PROCESS_HEIGHT = 240
DISPLAY_SCALE_X = DISPLAY_WIDTH // PROCESS_WIDTH
DISPLAY_SCALE_Y = DISPLAY_HEIGHT // PROCESS_HEIGHT
lst=[[1,2],[1,2],[1,2],[1,2]]
sensor = None
image_shape = [PROCESS_HEIGHT, PROCESS_WIDTH]

LAB_X_STRIDE = 4
LAB_Y_STRIDE = 2
LAB_AREA_THRESHOLD = 200
LAB_PIXELS_THRESHOLD = 100
LAB_MERGE_MARGIN = 10
LAB_RECT_MARGIN = 8

# -------------------------------
# 可调参数（建议调试时调整）
# -------------------------------
canny_thresh1       = 50
canny_thresh2       = 150
approx_epsilon      = 0.04
area_min_ratio      = 0.001
max_angle_cos       = 0.5
gaussian_blur_size  = 5
length_threshold    = 120
last = 0

def split_to_2d(arr, cols=1):
    return [arr[i:i + cols] for i in range(0, len(arr), cols)]

def get_vertices(rect):
    x, y, w, h = rect
    return [(x, y), (x + w, y), (x + w, y + h), (x, y + h)]

def find_max(arr):
    max_size=0
    max_blob=None
    for s in range (len(arr)):
        if arr[s][2]*arr[s][3] > max_size:
            max_blob = arr[s]
            max_size = arr[s][2]*arr[s][3]
    return max_blob

def rect_matches_lab(rect, blobs):
    """Keep rectangle candidates whose center is inside a LAB blob box."""
    center_x = rect[0] + rect[2] // 2
    center_y = rect[1] + rect[3] // 2
    for blob in blobs:
        left = blob[0] - LAB_RECT_MARGIN
        top = blob[1] - LAB_RECT_MARGIN
        right = blob[0] + blob[2] + LAB_RECT_MARGIN
        bottom = blob[1] + blob[3] + LAB_RECT_MARGIN
        if left <= center_x <= right and top <= center_y <= bottom:
            return True
    return False

def display_x(x):
    return int(x * DISPLAY_SCALE_X)

def display_y(y):
    return int(y * DISPLAY_SCALE_Y)

def display_point(point):
    return display_x(point[0]), display_y(point[1])

def draw_quad(target, corners, color, thickness=3):
    for index in range(4):
        start = display_point(corners[index])
        end = display_point(corners[(index + 1) % 4])
        target.draw_line(
            start[0], start[1], end[0], end[1],
            color=color, thickness=thickness
        )
        target.draw_circle(
            start[0], start[1], 4,
            color=(0, 0, 255), fill=True, thickness=3
        )

def draw_lab_box(target, blob):
    target.draw_rectangle(
        display_x(blob[0]),
        display_y(blob[1]),
        display_x(blob[2]),
        display_y(blob[3]),
        color=(0, 255, 255),
        thickness=2,
    )

def are_segments_parallel(theta1, theta2, tolerance=30):
    angle_difference = abs(theta1 - theta2)
    if(angle_difference>180): angle_difference = angle_difference - 180
    return math.isclose(angle_difference, 0, abs_tol=tolerance) or math.isclose(angle_difference, 180, abs_tol=tolerance)

def are_segments_vertical(theta1, theta2, tolerance=30):
    angle_difference = abs(theta1 - theta2)
    if(angle_difference>180): angle_difference = angle_difference - 180
    return math.isclose(angle_difference, 90, abs_tol=tolerance)

def find_intersection(x1, y1, x2, y2, x3, y3, x4, y4):
    def calculate_determinant(A, B):
        return A[0] * B[1] - A[1] * B[0]

    AB = (x2 - x1, y2 - y1)
    AC = (x3 - x1, y3 - y1)
    CD = (x4 - x3, y4 - y3)
    det = calculate_determinant(AB, CD)
    if det == 0: return None
    t = calculate_determinant(AC, CD) / det
    return int(x1 + t * AB[0]), int(y1 + t * AB[1])

def camera_init():
    global sensor
    # Yahboom 的多通道例程使用独立的显示通道和处理通道。
    sensor = Sensor(id=2, width=1280, height=960, fps=90)
    sensor.reset()
    sensor.set_framesize(
        width=PROCESS_WIDTH,
        height=PROCESS_HEIGHT,
        chn=CAM_CHN_ID_0,
    )
    sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_0)
    sensor.set_framesize(
        width=DISPLAY_WIDTH,
        height=DISPLAY_HEIGHT,
        chn=CAM_CHN_ID_1,
    )
    sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_1)
    Display.init(
        Display.ST7701,
        width=DISPLAY_WIDTH,
        height=DISPLAY_HEIGHT,
        to_ide=True,
        quality=50,
    )
    print(
        "#VISION display=640x480_RGB565 process=320x240_RGB565 "
        "lab_gate=find_blobs cv=cv_lite mod=MOD-011"
    )
    MediaManager.init()
    sensor.run()

def camera_deinit():
    global sensor
    sensor.stop()
    Display.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    MediaManager.deinit()

def capture_picture(threshold_runtime):
    fps = time.clock()
    S_THRESHOLD = 2000
    THRESHOLD_MIN = 500
    THRESHOLD_MAX = 10000
    THRESHOLD_STEP = 500
    last_key_time = 0
    DEBOUNCE_DELAY = 200
    last_uart_send_time = 0

    while True:
        fps.tick()
        rect_flag=0
        current_time = ticks_ms()

        try:
            os.exitpoint()
            global sensor
            display_img = sensor.snapshot(chn=CAM_CHN_ID_1)

            threshold_runtime.poll_touch()
            if threshold_runtime.tuning:
                threshold_runtime.show_tuner(display_img)
                del display_img
                continue

            # The native RGB565 process frame is used for the official
            # OpenMV LAB find_blobs API. Only LAB-matching frames are
            # converted to small RGB888 images for cv_lite.
            process_img = sensor.snapshot(chn=CAM_CHN_ID_0)
            active_threshold = threshold_runtime.threshold()
            active_invert = threshold_runtime.inverted()
            lab_blobs = process_img.find_blobs(
                [active_threshold],
                invert=active_invert,
                x_stride=LAB_X_STRIDE,
                y_stride=LAB_Y_STRIDE,
                area_threshold=LAB_AREA_THRESHOLD,
                pixels_threshold=LAB_PIXELS_THRESHOLD,
                merge=True,
                margin=LAB_MERGE_MARGIN,
            )

            rects = []
            if lab_blobs:
                detect_img = process_img.to_rgb888()
                img_np = detect_img.to_numpy_ref()
                raw_rects = cv_lite.rgb888_find_rectangles_with_corners(
                    image_shape, img_np,
                    canny_thresh1, canny_thresh2,
                    approx_epsilon, area_min_ratio,
                    max_angle_cos, gaussian_blur_size
                )
                rects = [
                    rect for rect in raw_rects
                    if rect_matches_lab(rect, lab_blobs)
                ]
                del img_np
                del detect_img

                lab_target = find_max(lab_blobs)
                if lab_target is not None:
                    draw_lab_box(display_img, lab_target)

            inc_key_raw = INC_KEY.value()
            dec_key_raw = DEC_KEY.value()
            inc_key_pressed = (inc_key_raw == 0) and (time.ticks_diff(current_time, last_key_time) > DEBOUNCE_DELAY)
            dec_key_pressed = (dec_key_raw == 0) and (time.ticks_diff(current_time, last_key_time) > DEBOUNCE_DELAY)

            if inc_key_pressed:
                S_THRESHOLD = min(THRESHOLD_MAX, S_THRESHOLD + THRESHOLD_STEP)
                last_key_time = current_time
            if dec_key_pressed:
                S_THRESHOLD = max(THRESHOLD_MIN, S_THRESHOLD - THRESHOLD_STEP)
                last_key_time = current_time

            if(rects):
                max_rects = find_max(rects)
                c=[[1,2],[1,2],[1,2],[1,2]]
                for i in range(4):
                    c[i][0] = max_rects[2*i+4]
                    c[i][1] = max_rects[2*i+5]
                draw_quad(display_img, c, color=(255, 0, 0))

                len1 = sqrt(pow(c[0][0]-c[1][0],2)+pow(c[0][1]-c[1][1],2))
                len2 = sqrt(pow(c[2][0]-c[3][0],2)+pow(c[2][1]-c[3][1],2))
                len3 = sqrt(pow(c[0][0]-c[3][0],2)+pow(c[0][1]-c[3][1],2))
                len4 = sqrt(pow(c[1][0]-c[2][0],2)+pow(c[1][1]-c[2][1],2))

                S = max_rects[2]*max_rects[3]
                err1 = (abs(len1-len2))
                err2 = (abs(len3-len4))

                if(S>S_THRESHOLD and err1<length_threshold and err2<length_threshold and len1>30 and len2>30 and len3>30 and len4>30):
                    theta1 = math.atan2(c[0][1]-c[1][1], c[0][0]-c[1][0])
                    theta2 = math.atan2(c[2][1]-c[3][1], c[2][0]-c[3][0])
                    theta3 = math.atan2(c[0][1]-c[3][1], c[0][0]-c[3][0])
                    theta4 = math.atan2(c[1][1]-c[2][1], c[1][0]-c[2][0])

                    theta1_degrees = math.degrees(theta1)
                    theta2_degrees = math.degrees(theta2)
                    theta3_degrees = math.degrees(theta3)
                    theta4_degrees = math.degrees(theta4)

                    is_line1_line2_parallel = are_segments_parallel(theta1_degrees, theta2_degrees)
                    is_line3_line4_parallel = are_segments_parallel(theta3_degrees, theta4_degrees)
                    is_line1_line3_vertical = are_segments_vertical(theta1_degrees, theta3_degrees)

                    if (is_line1_line3_vertical and is_line1_line2_parallel and is_line3_line4_parallel):
                        rect_flag=1
                        for s in range (4):
                            lst[s][0]=c[s][0]
                            lst[s][1]=c[s][1]
                    else: rect_flag=0
                else: rect_flag=0
            else: rect_flag=0

            if rect_flag==1:
                intersection = find_intersection(lst[0][0], lst[0][1], lst[2][0], lst[2][1], lst[1][0], lst[1][1], lst[3][0], lst[3][1])
                if intersection is None:
                    rect_flag = 0

            if rect_flag==1:
                display_center = display_point(intersection)
                display_img.draw_circle(
                    display_center[0], display_center[1], 3,
                    color=(255, 0, 0), thickness=5
                )

                print("(cx=%s,cy=%s)" % display_center)
                display_img.draw_string_advanced(
                    0, 0, 30,
                    "(x=%s,y=%s)" % display_center,
                    color=(255, 255, 0),
                    scale=3,
                )
                draw_quad(display_img, c, color=(0, 255, 0))

                # Send 640x480 display coordinates to preserve the original
                # external coordinate system while processing at 320x240.
                if time.ticks_diff(current_time, last_uart_send_time) >= UART_SEND_INTERVAL_MS:
                    packet = send_center(
                        uart, display_center[0], display_center[1])
                    last_uart_send_time = current_time
                    print("#UART_CENTER packet=%s" % packet)
                display_img.draw_string_advanced(
                    300, 0, 30, "fps:%s" % int(fps.fps()),
                    color=(0, 255, 0)
                )
            else:
                display_img.draw_string_advanced(
                    20, 50, 30, "fps:%s" % int(fps.fps()),
                    color=(0, 255, 0)
                )
                display_img.draw_string_advanced(
                    10, 0, 30, "S_THRESHOLD:{}".format(S_THRESHOLD),
                    color=(255, 0, 0)
                )

            display_img.draw_string_advanced(
                10, 88, 20,
                "LAB:{} {}".format(
                    active_threshold,
                    "INV" if active_invert else "NORMAL",
                ),
                color=(0, 255, 255),
            )
            threshold_runtime.show_normal(display_img)
            del process_img
            del display_img

        except KeyboardInterrupt as e:
            print("user stop: ", e)
            break
        except BaseException as e:
            print(f"Exception {e}")
            break

def main():
    os.exitpoint(os.EXITPOINT_ENABLE)
    camera_is_init = False
    threshold_runtime = None
    print("#CFG,event=TARGET_CENTER,change=%s,baud=115200,"
          "uart=EXPORT_IO9_IO10,frame=$len,16,cx,cy,#" % CHANGE_NAME)
    try:
        camera_init()
        camera_is_init = True
        threshold_runtime = ThresholdRuntime(
            DISPLAY_WIDTH, DISPLAY_HEIGHT
        )
        capture_picture(threshold_runtime)
    except Exception as e:
        print(f"Exception {e}")
    finally:
        if threshold_runtime is not None:
            threshold_runtime.close()
        if camera_is_init:
            camera_deinit()

if __name__ == "__main__":
    main()


