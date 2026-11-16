# config.py - OpenMV H7 Plus, firmware 5.x
# Restore the confirmed QR/LCD settings. Other thresholds from existing demos.

UART_ID = 3
UART_BAUD = 115200

LCD_WIDTH = 128
LCD_HEIGHT = 160
LCD_BACKLIGHT = 100

QR_CONFIRM_FRAMES = 3
QR_RECT_HOLD_MS = 600

# 1=red 2=yellow 3=blue 4=green 5=black 6=lightblue
MATERIAL_THRESHOLDS = {
    1: (15, 100, 25, 100, 10, 90),
    2: (55, 100, -40, 25, 35, 110),
    3: (10, 100, 0, 80, -128, -30),
    4: (20, 100, -90, -20, 10, 70),
    5: (0, 25, -20, 20, -20, 20),
    6: (50, 100, -30, 10, -80, -20),
}
MATERIAL_MIN_PIXELS = 200
MATERIAL_MIN_DENSITY = 0.35
MATERIAL_STILL_FRAMES = 8
MATERIAL_POS_TOL_PX = 15
MATERIAL_SIZE_TOL_PCT = 8
MATERIAL_STABLE_FRAMES = 3
MATERIAL_DIAMETER_MM = {1: 40.0, 2: 40.0, 3: 40.0,
                        4: 40.0, 5: 40.0, 6: 40.0}

# Distance is NOT emitted as nonzero until calibration is explicitly enabled.
DISTANCE_CALIBRATED = False
FOCAL_PX = 0.0
RING_DIAMETER_MM = 0.0
PRINCIPAL_X_PX = 160.0
PRINCIPAL_Y_PX = 120.0

# Black printed concentric rings, NOT colored rings.
RING_DETECT_INTERVAL = 2
RING_MAX_RADIUS = 65
RING_MIN_RADIUS = 9
RING_BLACK_THRESHOLD = -1   # -1 = automatic
RING_RADIAL_MIN_FRACTION = 0.41
RING_MIN_HITS = 3
RING_OCR_WINDOW = 5
RING_OCR_VOTES = 3
RING_OCR_MIN_SCORE = 0.74
RING_OCR_MIN_GAP = 0.05
RING_TEXT_ROTATION = None   # None: auto 0/90/180/270; or set 90 etc.
RING_TARGET_X_PX = 160
RING_TARGET_Y_PX = 120

ACK_TIMEOUT_MS = 1500
MAX_RESENDS = 3
STATUS_PERIOD_MS = 500

# Edge demo thresholds: near = lower image row, NOT physical millimeters.
EDGE_STOP_ROW = 190
EDGE_CONFIRM = 3
EDGE_RELEASE = 3
EDGE_DARK_MAX = 150
EDGE_CONTRAST = 18

# Blue area, same LAB as demo_edge_blue.
BLUE_THRESHOLD = (10, 100, 0, 80, -128, -30)
BLUE_MIN_PIXELS = 200
BLUE_MIN_DENSITY = 0.10

# Only remove a display group on STM32's #GROUP_DONE,n message.
# K/B remain SINGLE ITEM acknowledgements, never group completion.

# #MODE,EDGE / #MODE,BLUE are auxiliary modes. They do not drive motors.

# ====== New protocol and intrinsic calibration ======
# Optical frame: X right, Y down, Z forward. Do NOT publish valid mm until calibrated.
# Measure actual top-face diameter of your printed material, in millimeters.
# 0 means unknown. Color is NOT guaranteed to determine shape; set each real value.
TOP_FACE_DIAMETER_MM = {1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0, 5: 0.0, 6: 0.0}
FX_PX = 0.0
FY_PX = 0.0
CX_PX = 160.0
CY_PX = 120.0
# Physical diameter of the SAME rim represented by RingFinder track radius;
# 0 disables metric ring distances; calibrate the actual tracked feature.
RING_TRACKED_DIAMETER_MM = 0.0
MAT_REPORT_PERIOD_MS = 120
DISK_REACQUIRE_MS = 0  # 0: remember trusted disk until changing mode or RESET; require rim each frame
# Optical correction: the pinhole estimate assumes measured circular face is near frontal.
# Calibrate and correct lens distortion/tilt in real installation.

# ===== STM32 command-driven single-shot vision jobs =====
JOB_TOL_X_PX = 8
JOB_TOL_Y_PX = 8
JOB_ARRIVED_FRAMES = 4          # consecutive *fresh* observed frames
JOB_POS_PERIOD_MS = 100         # position stream while target visible
JOB_LOST_PERIOD_MS = 500        # rate-limited no-target notification
JOB_TIMEOUT_MS = 20000         # per task; can be tuned for a slow turntable
JOB_QR_TIMEOUT_MS = 12000
JOB_IDLE_SLEEP_MS = 10

# ===== 固定物料区：新流程 JOB40/41/42（实际装车需标定） =====
# 区域确认：不检测整圈圆盘外缘，只确认三种颜色、120°几何、中心附近亮区。
AREA_CONFIRM_FRAMES = 3
AREA_TRIANGLE_RECHECK_FRAMES = 4  # 首次确认后每4帧复核三角布局，降低CPU占用
AREA_LIGHT_CHECK = True
AREA_LIGHT_MIN = 120          # grayscale 0..255；根据实拍校准
AREA_LIGHT_HITS = 3          # 5 个采样点至少亮几个
AREA_GEOMETRY_MAX_SHIFT_PX = 36
AREA_GEOMETRY_MAX_RADIUS_DELTA_PX = 22
# 三角形倒置：画面上方两件，下方一件。抓取位必须在下方。
BOTTOM_MIN_Y_GAP_PX = 14
BOTTOM_MIN_OFFSET_RADIUS = 0.16
BOTTOM_X_TOL_RADIUS = 0.52
BOTTOM_Y_GAP_2_PX = 10
# 机械爪对准位置是 *物块中心应落在的图像坐标*，必须实机标定！
GRIP_TARGET_X_PX = 160
GRIP_TARGET_Y_PX = 195     # 仅示意默认值，不保证是真正抓取点
GRIP_TARGET_CALIBRATED = False  # 未标定前拒绝 JOB41/JOB42，防止误夹
# 静止判断使用跟踪结果随时间的像素位移，须先停稳底盘。
TURNTABLE_STILL_MS = 650
TURNTABLE_STILL_DISP_PX = 5
TURNTABLE_STILL_MIN_HITS = 3
AREA_STATUS_PERIOD_MS = 550
AREA_JOB_TIMEOUT_MS = 20000
ALIGN_JOB_TIMEOUT_MS = 35000
ALIGN_STEP_TIMEOUT_MS = 2500
PICK_JOB_TIMEOUT_MS = 55000
