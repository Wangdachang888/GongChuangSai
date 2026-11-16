# utils.py - OpenMV 5.x helpers: QR, LCD, UART, materials, rings, edge and blue.
import image
import display
import math
import time
from machine import UART
import config as C


def val(obj, name, default=0):
    v = getattr(obj, name, None)
    return default if v is None else (v() if callable(v) else v)


def normalize_code(raw):
    if isinstance(raw, bytes):
        try:
            raw = raw.decode('ascii')
        except Exception:
            return None
    if not isinstance(raw, str):
        return None
    return raw.strip().replace(' ', '').replace('\r', '').replace('\n', '')


def parse_code(raw):
    code = normalize_code(raw)
    if code is None:
        return None
    groups = code.split('+')
    if len(groups) != 4:
        return None
    for index, group in enumerate(groups):
        if len(group) != 3 or not group.isdigit():
            return None
        chars = '123456' if index % 2 == 0 else '123'
        if any(c not in chars for c in group) or len(set(group)) != 3:
            return None
    return groups


class TaskBoard:
    def __init__(self):
        self.groups = None
        self.completed = 0

    def locked(self):
        return self.groups is not None

    def code(self):
        return '+'.join(self.groups) if self.locked() else ''

    def remaining(self):
        return self.groups[self.completed:] if self.locked() else []

    def remaining_text(self):
        return '+'.join(self.remaining())

    def accept(self, raw):
        if self.locked():
            return False
        groups = parse_code(raw)
        if groups is None:
            return False
        self.groups = groups
        self.completed = 0
        return True

    def finish_group(self, num):
        if not self.locked() or num != self.completed + 1 or num > 4:
            return False
        self.completed = num
        return True

    def reset(self):
        self.groups = None
        self.completed = 0


class TaskLCD:
    # SAME LCD configuration that passed standalone and QR demo testing.
    def __init__(self):
        self.lcd = display.SPIDisplay(width=C.LCD_WIDTH, height=C.LCD_HEIGHT)
        self.lcd.backlight(C.LCD_BACKLIGHT)
        self.canvas = image.Image(C.LCD_WIDTH, C.LCD_HEIGHT, image.RGB565)
        self.signature = None
        print('LCD INIT OK')

    def update(self, board):
        signature = (board.code(), board.completed)
        if signature == self.signature:
            return
        self.canvas.clear()
        if not board.locked():
            self.canvas.draw_string((4, 55), 'SCAN QR',
                                    color=(255, 255, 255), scale=2)
        elif board.completed >= 4:
            self.canvas.draw_string((14, 45), 'DONE',
                                    color=(0, 255, 0), scale=3)
            self.canvas.draw_string((30, 95), '4/4',
                                    color=(255, 255, 255), scale=2)
        else:
            parts = board.remaining()
            for i, part in enumerate(parts):
                line = part + ('+' if i < len(parts) - 1 else '')
                self.canvas.draw_string((5, 6 + 38 * i), line,
                                        color=(255, 255, 255), scale=3)
        self.lcd.write(self.canvas)
        self.signature = signature
        print('LCD REFRESH:', board.remaining_text() or
              ('DONE' if board.locked() else 'SCAN QR'))


class VisionUART:
    # Nonblocking: legacy single-byte K/B/R; framed #...\n commands.
    def __init__(self):
        self.uart = UART(C.UART_ID, C.UART_BAUD, timeout=0)
        self.rx = bytearray()
        print('UART%d READY %d' % (C.UART_ID, C.UART_BAUD))

    def send(self, msg):
        self.uart.write(msg.encode('ascii'))
        print('TX:', msg.strip())

    def sequence(self, board):
        self.send('#SEQ,' + ','.join(ch for g in board.groups for ch in g) + '\n')

    def poll(self):
        out = []
        n = self.uart.any()
        if not n:
            return out
        data = self.uart.read(n)
        if not data:
            return out
        for b in data:
            if not self.rx and b in (75, 66, 82):  # K/B/R as single bytes
                out.append(chr(b))
            elif b in (10, 13):
                if self.rx:
                    try:
                        out.append(self.rx.decode('ascii').strip())
                    except Exception:
                        pass
                    self.rx = bytearray()
            elif len(self.rx) < 80:
                self.rx.append(b)
            else:
                self.rx = bytearray()
        return [e for e in out if e]


# Material detection: same LAB thresholds and selection rule as demo_material.
def material(img, label):
    thr = C.MATERIAL_THRESHOLDS.get(label)
    if thr is None:
        return None
    blobs = img.find_blobs([thr], pixels_threshold=C.MATERIAL_MIN_PIXELS,
                           area_threshold=C.MATERIAL_MIN_PIXELS, merge=True)
    best = None
    for b in blobs:
        w, h = val(b, 'w'), val(b, 'h')
        if w <= 0 or h <= 0:
            continue
        if val(b, 'pixels') / float(w * h) < C.MATERIAL_MIN_DENSITY:
            continue
        if best is None or val(b, 'pixels') > val(best, 'pixels'):
            best = b
    return best


def material_still(history):
    if len(history) < C.MATERIAL_STILL_FRAMES:
        return False
    sizes = [s[0] for s in history]
    xs = [s[1] for s in history]
    ys = [s[2] for s in history]
    smallest = min(sizes)
    return (smallest > 0 and
            100.0 * (max(sizes) - smallest) / smallest <= C.MATERIAL_SIZE_TOL_PCT
            and max(xs) - min(xs) <= C.MATERIAL_POS_TOL_PX
            and max(ys) - min(ys) <= C.MATERIAL_POS_TOL_PX)


def material_distance(label, pixels):
    if not C.DISTANCE_CALIBRATED or C.FOCAL_PX <= 0 or pixels <= 0:
        return 0
    return int(round(C.FOCAL_PX * C.MATERIAL_DIAMETER_MM[label] / pixels))


# Black ring candidates use connected dark shapes plus radial multi-peak checks.
N = 24
UNIT_CIRCLE = [(math.cos(2*math.pi*i/N), math.sin(2*math.pi*i/N))
               for i in range(N)]


def black_threshold(gray):
    if C.RING_BLACK_THRESHOLD >= 0:
        return C.RING_BLACK_THRESHOLD
    st = gray.get_statistics()
    low = val(st, 'min')
    mid = val(st, 'median')
    return max(45, min(195, int(0.18 * low + 0.82 * mid)))


def radial_candidate(gray, x, y, r0, threshold):
    w, h = gray.width(), gray.height()
    x, y = int(round(x)), int(round(y))
    r0 = max(6.0, float(r0))
    lo = max(6, int(0.43 * r0))
    hi = min(C.RING_MAX_RADIUS, int(1.85*r0) + 3,
             x-2, y-2, w-3-x, h-3-y)
    if hi - lo < 5:
        return None
    peaks = []
    limit = int(N * C.RING_RADIAL_MIN_FRACTION + 0.999)
    for r in range(lo, hi+1):
        dark = 0
        for ca, sa in UNIT_CIRCLE:
            if gray.get_pixel((int(round(x+r*ca)), int(round(y+r*sa)))) < threshold:
                dark += 1
        if dark >= limit:
            peaks.append((r, dark))
    if not peaks:
        return None
    order = sorted(peaks, key=lambda v: v[1], reverse=True)
    distinct = []
    for r, hits in order:
        if all(abs(r - v[0]) >= 3 for v in distinct):
            distinct.append((r, hits))
        if len(distinct) == 6:
            break
    if len(distinct) < 2:
        return None
    rr = sorted(p[0] for p in distinct)
    if rr[-1] - rr[0] < max(3, int(rr[-1] * .12)):
        return None
    outer = max(r for r, dark in peaks)
    if not (C.RING_MIN_RADIUS <= outer <= C.RING_MAX_RADIUS):
        return None
    return {'x': float(x), 'y': float(y), 'r': float(outer),
            'score': len(distinct)*10 + sum(v[1] for v in distinct)/N}


def circle_candidates(gray):
    threshold = black_threshold(gray)
    blobs = gray.find_blobs([(0, threshold)], x_stride=1, y_stride=1,
                             pixels_threshold=25, area_threshold=90, merge=False)
    seeds = []
    for b in blobs:
        x, y, w, h = val(b, 'rect', (0, 0, 0, 0))
        if not (17 <= w <= 2*C.RING_MAX_RADIUS+6 and
                17 <= h <= 2*C.RING_MAX_RADIUS+6):
            continue
        aspect = w / float(h)
        fill = val(b, 'pixels') / float(w*h)
        if .72 <= aspect <= 1.28 and .045 <= fill <= .64:
            seeds.append((x+w/2.0, y+h/2.0, (w+h)/4.0, w*h))
    seeds.sort(key=lambda v: v[3], reverse=True)
    candidates = []
    for x, y, r, area in seeds[:18]:
        cand = radial_candidate(gray, x, y, r, threshold)
        if cand is None:
            continue
        for k, old in enumerate(candidates):
            dx = cand['x'] - old['x']
            dy = cand['y'] - old['y']
            if dx*dx + dy*dy < max(7, .36*max(cand['r'], old['r']))**2:
                if cand['score'] > old['score']:
                    candidates[k] = cand
                break
        else:
            candidates.append(cand)
    candidates.sort(key=lambda v: v['score'], reverse=True)
    selected = []
    for c in candidates:
        if all((c['x']-v['x'])**2+(c['y']-v['y'])**2 >
               (.72*(c['r']+v['r']))**2 for v in selected):
            selected.append(c)
        if len(selected) == 3:
            break
    return selected, threshold


# OCR templates: geometric recognition only, not color. Rotation can be fixed.
FONT = {
    0: ('01110','10001','10011','10101','11001','10001','01110'),
    1: ('00100','01100','00100','00100','00100','00100','01110'),
    2: ('01110','10001','00001','00010','00100','01000','11111'),
    3: ('11110','00001','00001','01110','00001','00001','11110'),
    4: ('00010','00110','01010','10010','11111','00010','00010'),
    5: ('11111','10000','10000','11110','00001','00001','11110'),
    6: ('00110','01000','10000','11110','10001','10001','01110'),
    7: ('11111','00001','00010','00100','01000','01000','01000'),
    8: ('01110','10001','10001','01110','10001','10001','01110'),
    9: ('01110','10001','10001','01111','00001','00010','11100')
}
FONT_EXTRA = {
    1: (('00010','00010','00010','00010','11111','11111','11111'),),
    2: (('01110','11011','00001','00011','01100','11000','11111'),),
    3: (('01110','10001','00011','00110','00001','10001','01110'),)
}


def text_bitmap(gray, box, threshold, rotation):
    x, y, w, h = box
    rows = []
    for j in range(7):
        row = ''
        for i in range(5):
            dark = 0
            for oy in (.25, .75):
                for ox in (.25, .75):
                    u, v = (i+ox)/5.0, (j+oy)/7.0
                    if rotation == 0:
                        a, b = u, v
                    elif rotation == 90:
                        a, b = v, 1-u
                    elif rotation == 180:
                        a, b = 1-u, 1-v
                    else:
                        a, b = 1-v, u
                    px = min(gray.width()-1, int(x+a*w))
                    py = min(gray.height()-1, int(y+b*h))
                    if gray.get_pixel((px, py)) < threshold:
                        dark += 1
            row += '1' if dark >= 2 else '0'
        rows.append(row)
    return rows


def read_ring_digit(gray, x, y, radius):
    x, y = int(round(x)), int(round(y))
    half = int(radius * .50) + 1
    x0, y0 = max(0, x-half), max(0, y-half)
    x1, y1 = min(gray.width(), x+half+1), min(gray.height(), y+half+1)
    w, h = x1-x0, y1-y0
    if w < 15 or h < 15:
        return None
    st = gray.get_statistics(roi=(x0, y0, w, h))
    thr = max(35, min(200, int(.20*val(st, 'min')+.80*val(st, 'median'))))
    blobs = gray.find_blobs([(0, thr)], roi=(x0, y0, w, h),
                             x_stride=1, y_stride=1, pixels_threshold=6,
                             area_threshold=10, merge=False)
    chosen, best_rank = None, -100000
    for b in blobs:
        bx, by, bw, bh = val(b, 'rect', (0, 0, 0, 0))
        if bw < 2 or bh < 6 or bw > .84*w or bh > .94*h:
            continue
        if bx <= x0 or by <= y0 or bx+bw >= x1 or by+bh >= y1:
            continue
        delta = abs(bx+bw/2.0-x) + abs(by+bh/2.0-y)
        if delta > .46*(w+h):
            continue
        rank = val(b, 'pixels')-2.7*delta
        if rank > best_rank:
            chosen, best_rank = (bx, by, bw, bh), rank
    if chosen is None:
        return None
    angles = (C.RING_TEXT_ROTATION,) if C.RING_TEXT_ROTATION in (0,90,180,270) \
             else (0,90,180,270)
    ranking = []
    for rotation in angles:
        bits = text_bitmap(gray, chosen, thr, rotation)
        for digit in range(10):
            variants = (FONT[digit],) + FONT_EXTRA.get(digit, ())
            errors = min(sum(bits[j][i] != font[j][i]
                             for j in range(7) for i in range(5))
                         for font in variants)
            ranking.append((1-errors/35.0, digit))
    ranking.sort(reverse=True)
    best_score, best_digit = ranking[0]
    runner = max((s for s,d in ranking if d != best_digit), default=0.0)
    if best_score < C.RING_OCR_MIN_SCORE or best_score-runner < C.RING_OCR_MIN_GAP:
        return None
    if C.RING_TEXT_ROTATION is None and best_digit in (6,9):
        return None  # ambiguous if orientation is unknown
    return best_digit


class Kalman1D:
    def __init__(self, z, R=8.0, Q=3200.0):
        self.x, self.v = float(z), 0.0
        self.p00, self.p01, self.p11 = 30.0, 0.0, 400.0
        self.R, self.Q = R, Q

    def predict(self, dt):
        dt = max(.001, min(.5, dt))
        a = self.p00 + 2*dt*self.p01 + dt*dt*self.p11
        b = self.p01 + dt*self.p11
        self.p00 = a + self.Q*dt**4/4
        self.p01 = b + self.Q*dt**3/2
        self.p11 += self.Q*dt*dt
        self.x += self.v*dt

    def distance(self, z):
        return (z-self.x)**2 / max(.001, self.p00+self.R)

    def correct(self, z):
        den = self.p00 + self.R
        err = z-self.x
        kx, kv = self.p00/den, self.p01/den
        old01 = self.p01
        self.x += kx*err
        self.v += kv*err
        self.p00 = max(.01, (1-kx)*self.p00)
        self.p01 = (1-kx)*old01
        self.p11 = max(.01, self.p11-kv*old01)


class RingTrack:
    def __init__(self, c, now):
        self.x, self.y = Kalman1D(c['x']), Kalman1D(c['y'])
        self.r = Kalman1D(c['r'], R=6.0, Q=900.0)
        self.hits, self.miss = 1, 0
        self.last_predict, self.last_measured = now, now
        self.votes, self.last_ocr = [], now
        self.score = c['score']
        self.matched = True

    def predict(self, now):
        dt = time.ticks_diff(now, self.last_predict)/1000.0
        for f in (self.x, self.y, self.r):
            f.predict(dt)
        self.last_predict = now

    def gate(self, c):
        dx, dy = c['x']-self.x.x, c['y']-self.y.x
        if dx*dx+dy*dy > max(14, .60*self.r.x)**2:
            return None
        if abs(c['r']-self.r.x) > max(8, .38*self.r.x):
            return None
        d = (self.x.distance(c['x'])+self.y.distance(c['y'])+
             self.r.distance(c['r']))
        return d if d < 16 else None

    def update(self, c, now):
        self.x.correct(c['x']); self.y.correct(c['y']); self.r.correct(c['r'])
        self.score = c['score']
        self.hits += 1
        self.miss = 0
        self.matched = True
        self.last_measured = now

    def voted_digit(self, now):
        if self.hits < C.RING_MIN_HITS or self.miss or \
           time.ticks_diff(now, self.last_measured) > 700 or \
           time.ticks_diff(now, self.last_ocr) > 1200:
            return None
        ranking = sorted(((self.votes.count(i), i) for i in range(10)),
                         reverse=True)
        return ranking[0][1] if (ranking[0][0] >= C.RING_OCR_VOTES and
                                ranking[0][0] - ranking[1][0] >= 2) else None


class RingFinder:
    def __init__(self):
        self.tracks = []
        self.cycle = 0

    def reset(self):
        self.tracks = []
        self.cycle = 0

    def process(self, img):
        self.cycle += 1
        now = time.ticks_ms()
        if self.cycle % C.RING_DETECT_INTERVAL:
            return
        gray = img.to_grayscale(copy=True)
        try:
            candidates, threshold = circle_candidates(gray)
            for t in self.tracks:
                t.predict(now)
                t.matched = False
            pairs = []
            for i, t in enumerate(self.tracks):
                for j, c in enumerate(candidates):
                    distance = t.gate(c)
                    if distance is not None:
                        pairs.append((distance, i, j))
            pairs.sort()
            used_t, used_c = set(), set()
            for dist, i, j in pairs:
                if i not in used_t and j not in used_c:
                    self.tracks[i].update(candidates[j], now)
                    used_t.add(i); used_c.add(j)
            for i, t in enumerate(self.tracks):
                if i not in used_t:
                    t.miss += 1
                    t.hits = 0
                    t.votes = []
            self.tracks = [t for t in self.tracks if t.miss <= 2]
            for j, c in enumerate(candidates):
                if j not in used_c and len(self.tracks) < 3:
                    self.tracks.append(RingTrack(c, now))
            if (self.cycle // C.RING_DETECT_INTERVAL) % 2 == 0:
                for t in self.tracks:
                    if t.matched and t.hits >= 2:
                        digit = read_ring_digit(gray, t.x.x, t.y.x, t.r.x)
                        t.votes.append(digit)
                        if len(t.votes) > C.RING_OCR_WINDOW:
                            t.votes.pop(0)
                        if digit is not None:
                            t.last_ocr = now
        finally:
            del gray

    def confirmed(self, label):
        now = time.ticks_ms()
        matches = [t for t in self.tracks if t.voted_digit(now) == label]
        return matches[0] if len(matches) == 1 else None

    def draw(self, img):
        now = time.ticks_ms()
        for t in self.tracks:
            if t.hits >= C.RING_MIN_HITS and t.miss == 0 and \
               time.ticks_diff(now, t.last_measured) < 700:
                cx, cy = int(round(t.x.x)), int(round(t.y.x))
                r = max(1, int(round(t.r.x)))
                img.draw_cross((cx, cy), color=(255,0,0), size=10, thickness=2)
                img.draw_circle((cx, cy, r), color=(255,0,0), thickness=1)
                d = t.voted_digit(now)
                if d is not None:
                    img.draw_string((max(0,cx-5), max(0,cy-r-15)),
                                    str(d), color=(255,0,0), scale=2)


def ring_distance(t):
    if not C.DISTANCE_CALIBRATED or C.FOCAL_PX <= 0 or \
       C.RING_DIAMETER_MM <= 0 or t.r.x <= 0:
        return 0
    z = C.FOCAL_PX*C.RING_DIAMETER_MM/(2*t.r.x)
    dx = (t.x.x-C.PRINCIPAL_X_PX)/C.FOCAL_PX
    dy = (t.y.x-C.PRINCIPAL_Y_PX)/C.FOCAL_PX
    return int(round(z*math.sqrt(1+dx*dx+dy*dy)))


# Auxiliary blue-outline mode; cropped/corner regions remain valid observations.
def detect_blue(img):
    blobs = img.find_blobs([C.BLUE_THRESHOLD],
                           pixels_threshold=C.BLUE_MIN_PIXELS,
                           area_threshold=C.BLUE_MIN_PIXELS, merge=True)
    best = None
    for b in blobs:
        w, h = val(b,'w'), val(b,'h')
        if w < 3 or h < 3 or max(w,h) < 20:
            continue
        pixels = val(b,'pixels')
        if pixels/float(w*h) < C.BLUE_MIN_DENSITY:
            continue
        if best is None or pixels > val(best,'pixels'):
            best = b
    if best is None:
        return None
    x, y, w, h = val(best, 'rect')
    mode = 1 if w >= 3*h else (2 if h >= 3*w else 3)
    return (x,y,w,h,val(best,'cx'),val(best,'cy'),mode)


# Auxiliary black line: validate a THIN dark stripe on brighter background.
# STOP_ROW is an image-row trigger; does not pretend to be metric distance.
def _dark_stripe(img, x, y):
    x, y = int(x), int(y)
    if x < 0 or x >= img.width() or y < 10 or y >= img.height()-10:
        return False
    dark = min(img.get_pixel((x, y+d)) for d in (-2,-1,0,1,2))
    top = img.get_pixel((x,y-7))
    bottom = img.get_pixel((x,y+7))
    if dark > C.EDGE_DARK_MAX or top-dark < C.EDGE_CONTRAST or \
       bottom-dark < C.EDGE_CONTRAST:
        return False
    level = min(top,bottom)-C.EDGE_CONTRAST//2
    width = sum(img.get_pixel((x,y+d)) <= level for d in range(-6,7))
    return width <= 9


def detect_edge(img):
    w, h = img.width(), img.height()
    top = int(h*.33)
    # Native line candidates from the original edge demo, then dark-stripe tests.
    lines = img.find_lines(roi=(0,top,w,h-top), threshold=600,
                           x_stride=1, y_stride=1, theta_margin=10,
                           rho_margin=8)
    best, quality = None, -1
    for l in lines:
        x1, y1, x2, y2 = (val(l,'x1'),val(l,'y1'),val(l,'x2'),val(l,'y2'))
        dx, dy = x2-x1, y2-y1
        if abs(dx) < .55*w or abs(dy) > .45*abs(dx):
            continue
        hits = 0
        for k in range(1,10):
            xx = k*w//10
            if xx < min(x1,x2) or xx > max(x1,x2):
                continue
            yy = int(round(y1+(xx-x1)*dy/float(dx)))
            if yy >= top+8 and yy < h-8 and _dark_stripe(img,xx,yy):
                hits += 1
        if hits >= 6 and hits*1000+val(l,'magnitude') > quality:
            quality = hits*1000+val(l,'magnitude')
            row = int(round(y1+(w//2-x1)*dy/float(dx)))
            best = (x1,y1,x2,y2,row)
    if best is not None:
        return best

    # Original demo's second strategy: scan several columns and fit
    # a nearly horizontal thin stripe when Hough does not return a line.
    columns = []
    for x in range(w//2-60, w//2+61, 12):
        points = []
        for y in range(max(top+10, 10), h-10):
            if _dark_stripe(img, x, y):
                points.append(y)
        columns.append((x, points))
    best_support = 0
    best = None
    for seed_y in range(top+10, h-10, 3):
        matches = []
        for x, rows in columns:
            nearby = [yy for yy in rows if abs(yy-seed_y) <= 6]
            if nearby:
                yy = min(nearby, key=lambda z:abs(z-seed_y))
                matches.append((x, yy))
        if len(matches) < 8 or matches[-1][0]-matches[0][0] < 85:
            continue
        n = len(matches)
        mean_x = sum(x for x,y in matches)/float(n)
        mean_y = sum(y for x,y in matches)/float(n)
        denom = sum((x-mean_x)**2 for x,y in matches)
        if denom <= 0:
            continue
        slope = sum((x-mean_x)*(y-mean_y) for x,y in matches)/denom
        if abs(slope) > .22:
            continue
        if max(abs(y-mean_y-slope*(x-mean_x)) for x,y in matches) > 3:
            continue
        if n > best_support:
            x1, x2 = matches[0][0], matches[-1][0]
            y1 = int(round(mean_y+slope*(x1-mean_x)))
            y2 = int(round(mean_y+slope*(x2-mean_x)))
            best = (x1,y1,x2,y2,int(round(mean_y+slope*(w/2-mean_x))))
            best_support = n
    return best


# ====== ORIGINAL demo_material.py white-disk gate and multi-target Kalman tracker ======
# The detection thresholds/geometry are copied without tuning.
# 六种比赛物料的LAB颜色阈值
# 需要根据实际光照进一步标定
THRESHOLDS = [
    (15, 100, 25, 100, 10, 90),       # 1 红
    (55, 100, -40, 25, 35, 110),     # 2 黄
    (10, 100, 0, 80, -128, -30),     # 3 蓝
    (20, 100, -90, -20, 10, 70),     # 4 绿
    (0, 25, -20, 20, -20, 20),       # 5 黑
    (50, 100, -30, 10, -80, -20)    # 6 浅蓝
]

COLORS = [
    (255, 0, 0),
    (255, 255, 0),
    (0, 80, 255),
    (0, 255, 0),
    (255, 255, 255),
    (0, 255, 255)
]

# ==================== 圆盘验证参数 ====================

# 规则：白色转盘直径300mm
# 三物料中心分布圆直径200mm
#
# 以下像素范围依据当前未装车时的画面设置
# 正式安装后可能需要重新标定

DISK_R_MIN = 95
DISK_R_MAX = 165

DISK_SCAN_EVERY = 4
DISK_CONFIRM_SCANS = 5
DISK_LOST_FRAMES = 3

# 圆盘内侧亮度和内外对比度
RIM_IN_BRIGHT_MIN = 150
RIM_CONTRAST_MIN = 42

# 外缘必须有多处有效采样点
RIM_MIN_HITS = 8
RIM_MIN_SECTORS = 2
RIM_WHITE_FRACTION = 0.72

# 严格模式：最初锁定圆盘前，
# 必须同时观察到三个不同颜色的物料
REQUIRE_THREE_MATERIALS = True

# ==================== 物料参数 ====================

BLOB_PIXELS_MIN = 65
BLOB_SIDE_MIN = 13
BLOB_SIDE_MAX = 105

# 顶部圆面霍夫检测
MATERIAL_HOUGH_THRESHOLD = 1200
CAP_R_MIN = 7
CAP_R_MAX = 31

# ==================== 跟踪参数 ====================

MOVE_PX_S = 8.0
STILL_PX_S = 3.0

MOVE_MS = 150
STILL_MS = 550

TRACK_MISSES = 4

SEND_MS = 90
DEBUG_MS = 1100

TWOPI = 2.0 * math.pi

# 圆盘外缘48点采样
RIM_ANGLES = [
    (
        math.cos(TWOPI * k / 48),
        math.sin(TWOPI * k / 48)
    )
    for k in range(48)
]

# 圆盘内部亮度采样
CORE_ANGLES = [
    (
        math.cos(TWOPI * k / 12),
        math.sin(TWOPI * k / 12)
    )
    for k in range(12)
]


# ==================== 工具函数 ====================

def a(obj, name, default=0):
    v = getattr(obj, name, None)

    if v is None:
        return default

    return v() if callable(v) else v


def inside_image(x, y, w=320, h=240):
    return 0 <= x < w and 0 <= y < h


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


# ==================== 颜色区域初筛 ====================

def color_blobs(img):

    # 这里只提取场景特征。
    # 没有圆盘确认时，绝不把色块作为物料结果发送。

    result = []

    blobs = img.find_blobs(
        THRESHOLDS,
        x_stride=2,
        y_stride=2,
        pixels_threshold=BLOB_PIXELS_MIN,
        area_threshold=100,
        merge=False
    )

    for b in blobs:

        code = a(b, 'code')

        # 同时匹配多种颜色的区域不确认
        if code <= 0 or code & (code - 1):
            continue

        label = 1

        while code > 1:
            code >>= 1
            label += 1

        x, y, w, h = a(
            b, 'rect', (0, 0, 0, 0)
        )

        if (
            w < BLOB_SIDE_MIN
            or h < BLOB_SIDE_MIN
            or w > BLOB_SIDE_MAX
            or h > BLOB_SIDE_MAX
        ):
            continue

        ratio = min(w, h) / float(max(w, h))
        pixels = a(b, 'pixels')

        if ratio < 0.36 or pixels < BLOB_PIXELS_MIN:
            continue

        density = pixels / float(max(1, w * h))

        if density < 0.20 or density > 0.99:
            continue

        result.append((
            label,
            float(a(b, 'cx')),
            float(a(b, 'cy')),
            x, y, w, h, pixels
        ))

    return result


# ==================== 三物料布局检验 ====================

def triangle_hint(blobs):

    # 按比赛几何结构寻找三个物料：
    # 三个不同颜色
    # 围绕同一个中心
    # 相邻方向间隔约120度
    #
    # 这只负责推测圆盘位置。
    # 最终必须通过真实外缘亮度验证。

    best = None

    for i in range(len(blobs)):
        for j in range(i + 1, len(blobs)):
            for k in range(j + 1, len(blobs)):

                bs = (
                    blobs[i],
                    blobs[j],
                    blobs[k]
                )

                labels = (
                    bs[0][0],
                    bs[1][0],
                    bs[2][0]
                )

                if len(set(labels)) != 3:
                    continue

                cx = sum(b[1] for b in bs) / 3.0
                cy = sum(b[2] for b in bs) / 3.0

                ds = [
                    math.sqrt(
                        (b[1] - cx) ** 2
                        + (b[2] - cy) ** 2
                    )
                    for b in bs
                ]

                if min(ds) < 25:
                    continue

                if max(ds) > min(ds) * 1.55:
                    continue

                angles = sorted([
                    math.atan2(
                        b[2] - cy,
                        b[1] - cx
                    ) % TWOPI
                    for b in bs
                ])

                gaps = [
                    angles[1] - angles[0],
                    angles[2] - angles[1],
                    angles[0] + TWOPI - angles[2]
                ]

                # 容许透视和检测误差
                if any(
                    abs(g - TWOPI / 3) > 0.65
                    for g in gaps
                ):
                    continue

                # 200/300 = 0.667
                # 考虑斜视和圆台色块重心偏移
                r = sum(ds) / 3.0 / 0.76

                if not DISK_R_MIN <= r <= DISK_R_MAX:
                    continue

                score = sum(b[7] for b in bs)

                if best is None or score > best[0]:
                    best = (
                        score,
                        cx,
                        cy,
                        r
                    )

    return best


# ==================== 圆盘外缘验证 ====================

def disk_rim_score(gray, cx, cy, r):

    # 真正检查圆周两侧的亮度差。
    #
    # 圆盘内侧采样半径：0.91r
    # 圆盘外侧采样半径：1.07r
    #
    # 白墙没有这种持续的圆周明暗跃迁，
    # 因此不能仅凭颜色通过。

    hits = 0
    sectors = 0

    for k, (ca, sa) in enumerate(RIM_ANGLES):

        xi = int(round(cx + 0.91 * r * ca))
        yi = int(round(cy + 0.91 * r * sa))

        xo = int(round(cx + 1.07 * r * ca))
        yo = int(round(cy + 1.07 * r * sa))

        if not (
            inside_image(xi, yi)
            and inside_image(xo, yo)
        ):
            # 圆周被画面裁切时，
            # 这些点不能当作验证成功
            continue

        vin = gray.get_pixel((xi, yi))
        vout = gray.get_pixel((xo, yo))

        if (
            vin >= RIM_IN_BRIGHT_MIN
            and vin - vout >= RIM_CONTRAST_MIN
        ):
            hits += 1

            # 分成8个方向区域
            sectors |= 1 << (k // 6)

    if hits < RIM_MIN_HITS:
        return -1

    active_sectors = 0

    while sectors:
        active_sectors += sectors & 1
        sectors >>= 1

    if active_sectors < RIM_MIN_SECTORS:
        return -1

    return hits


# ==================== 圆盘内部验证 ====================

def disk_core_ok(gray, cx, cy, r):

    total = 0
    white = 0

    for fraction in (0.20, 0.42, 0.60):

        for ca, sa in CORE_ANGLES:

            x = int(round(
                cx + r * fraction * ca
            ))

            y = int(round(
                cy + r * fraction * sa
            ))

            if inside_image(x, y):

                total += 1

                if gray.get_pixel(
                    (x, y)
                ) >= RIM_IN_BRIGHT_MIN:

                    white += 1

    return (
        total >= 23
        and white >= total * RIM_WHITE_FRACTION
    )


# ==================== 检测大圆盘 ====================

def detect_disk(gray, blobs):

    hint = triangle_hint(blobs)

    if hint is None:
        return None

    _, cx, cy, r0 = hint

    best = None

    # 小范围局部搜索，提高圆盘定位精度
    # 不进行每帧全图大半径霍夫搜索

    for dx in (-6, 0, 6):
        for dy in (-6, 0, 6):
            for dr in (
                -18, -12, -6,
                0, 6, 12, 18
            ):

                r = r0 + dr

                if (
                    r < DISK_R_MIN
                    or r > DISK_R_MAX
                ):
                    continue

                xx = cx + dx
                yy = cy + dy

                score = disk_rim_score(
                    gray,
                    xx,
                    yy,
                    r
                )

                if score < 0:
                    continue

                if (
                    best is None
                    or score > best[0]
                ):
                    best = (
                        score,
                        xx,
                        yy,
                        r
                    )

    if best is None:
        return None

    _, cx, cy, r = best

    # 必须同时满足圆盘内部亮度条件
    if not disk_core_ok(
        gray,
        cx,
        cy,
        r
    ):
        return None

    return (cx, cy, r)


# ==================== 圆盘状态门控 ====================

class DiskGate:

    def __init__(self):

        self.disk = None
        self.pending = None

        self.hits = 0
        self.misses = 0

        self.locked = False

    def reset(self):
        self.__init__()

    def update(self, gray, blobs, frame):

        # 已经锁定圆盘
        if self.locked:

            cx, cy, r = self.disk

            # 每帧继续确认圆盘是否存在
            valid = (
                disk_rim_score(
                    gray, cx, cy, r
                ) >= 0
                and disk_core_ok(
                    gray, cx, cy, r
                )
            )

            if valid:
                self.misses = 0
                return self.disk

            self.misses += 1

            if self.misses >= DISK_LOST_FRAMES:
                self.reset()

            # 验证失败的帧不能识别物料
            return None

        # 未锁定时降低检测频率
        if frame % DISK_SCAN_EVERY != 0:
            return None

        cand = detect_disk(
            gray,
            blobs
        )

        if cand is None:

            self.hits = 0
            self.pending = None

            return None

        if self.pending is not None:

            dx = cand[0] - self.pending[0]
            dy = cand[1] - self.pending[1]

            similar = (
                dx * dx + dy * dy < 12 * 12
                and abs(
                    cand[2] - self.pending[2]
                ) < 15
            )

            if similar:

                self.hits += 1

                self.pending = tuple(
                    0.7 * a + 0.3 * b
                    for a, b in zip(
                        self.pending,
                        cand
                    )
                )

            else:

                self.hits = 1
                self.pending = cand

        else:

            self.hits = 1
            self.pending = cand

        # 连续五次检测成功，才确认圆盘
        if self.hits >= DISK_CONFIRM_SCANS:

            self.disk = self.pending
            self.locked = True
            self.misses = 0

            print(
                'DISK LOCKED',
                self.disk
            )

            return self.disk

        return None


# ==================== 顶部圆面识别 ====================

def material_candidates(img, gray, disk):

    cx0, cy0, disk_r = disk

    out = []

    # 只在已确认圆盘后使用颜色候选
    blobs = color_blobs(img)

    for label, cx, cy, x, y, w, h, pixels in blobs:

        d = math.sqrt(
            (cx - cx0) ** 2
            + (cy - cy0) ** 2
        )

        # 物料中心应位于转盘内部规定的分布区域
        if not (
            0.39 * disk_r
            <= d
            <= 0.96 * disk_r
        ):
            continue

        # 仅对候选物料的小区域使用霍夫检测
        pad = 4

        x0 = max(0, x - pad)
        y0 = max(0, y - pad)

        x1 = min(
            img.width(),
            x + w + pad
        )

        y1 = min(
            img.height(),
            y + h + pad
        )

        if x1 - x0 < 18 or y1 - y0 < 18:
            continue

        circles = gray.find_circles(
            roi=(
                x0,
                y0,
                x1 - x0,
                y1 - y0
            ),
            threshold=MATERIAL_HOUGH_THRESHOLD,
            x_stride=2,
            y_stride=2,
            x_margin=5,
            y_margin=5,
            r_margin=3,
            r_min=CAP_R_MIN,
            r_max=min(
                CAP_R_MAX,
                (min(w, h) + 6) // 2
            ),
            r_step=2
        )

        best = None

        for c in circles:

            xc = a(c, 'x')
            yc = a(c, 'y')
            rr = a(c, 'r')

            if not (
                x - 2 <= xc <= x + w + 2
                and y - 2 <= yc <= y + h + 2
            ):
                continue

            diameter_ratio = (
                2.0 * rr / float(min(w, h))
            )

            if not (
                0.20 <= diameter_ratio <= 1.18
            ):
                continue

            radial_d = math.sqrt(
                (xc - cx0) ** 2
                + (yc - cy0) ** 2
            )

            if not (
                0.35 * disk_r
                <= radial_d
                <= 1.00 * disk_r
            ):
                continue

            score = a(
                c, 'magnitude', 0
            )

            if (
                best is None
                or score > best[0]
            ):
                best = (
                    score,
                    xc,
                    yc,
                    rr
                )

        if best is None:
            continue

        # quality仅为检测强度参考值，不是概率
        quality = min(
            99,
            int(best[0] / 40.0)
        )

        out.append((
            label,
            best[1],
            best[2],
            best[3],
            quality
        ))

    # 去除同一物料的重复检测
    result = []

    for v in out:

        separate = all(
            (v[1] - q[1]) ** 2
            + (v[2] - q[2]) ** 2
            > 13 ** 2
            for q in result
        )

        if separate:
            result.append(v)

    return result


# ==================== 卡尔曼滤波 ====================

class KF:

    # 状态向量：[位置, 速度]

    def __init__(self, pos):

        self.x = float(pos)
        self.v = 0.0

        self.p00 = 30.0
        self.p01 = 0.0
        self.p11 = 500.0

    def predict(self, dt):

        dt = clamp(
            dt, 0.001, 0.3
        )

        p00 = self.p00
        p01 = self.p01
        p11 = self.p11

        self.x += self.v * dt

        self.p00 = (
            p00
            + 2 * dt * p01
            + dt * dt * p11
            + 225 * dt ** 4
        )

        self.p01 = (
            p01
            + dt * p11
            + 450 * dt ** 3
        )

        self.p11 = (
            p11
            + 900 * dt * dt
        )

    def correct(self, pos):

        e = pos - self.x
        var = self.p00 + 9.0

        k0 = self.p00 / var
        k1 = self.p01 / var

        old01 = self.p01

        self.x += k0 * e
        self.v += k1 * e

        self.p00 = max(
            0.01,
            self.p00 * (1 - k0)
        )

        self.p01 = (
            self.p01 * (1 - k0)
        )

        self.p11 = max(
            0.01,
            self.p11 - k1 * old01
        )


# ==================== 单个物料跟踪 ====================

class Track:

    def __init__(self, det, tid, now):

        self.label = det[0]
        self.id = tid

        self.kx = KF(det[1])
        self.ky = KF(det[2])

        self.r = float(det[3])
        self.quality = det[4]

        self.last = now
        self.last_sent = now

        self.hits = 1
        self.misses = 0

        self.seen = True

        # U 未确认
        # M 运动
        # S 静止
        self.state = 'U'

        self.pending = None
        self.pending_at = now

    def predict(self, now):

        dt = time.ticks_diff(
            now,
            self.last
        ) / 1000.0

        self.kx.predict(dt)
        self.ky.predict(dt)

        self.last = now
        self.seen = False

    def update(self, det, now):

        self.kx.correct(det[1])
        self.ky.correct(det[2])

        self.r = (
            0.7 * self.r
            + 0.3 * det[3]
        )

        self.quality = det[4]

        self.hits += 1
        self.misses = 0
        self.seen = True

        speed = math.sqrt(
            self.kx.v ** 2
            + self.ky.v ** 2
        )

        if speed >= MOVE_PX_S:
            proposed = 'M'

        elif speed <= STILL_PX_S:
            proposed = 'S'

        else:
            proposed = None

        if proposed is None:

            self.pending = None

        elif self.pending != proposed:

            self.pending = proposed
            self.pending_at = now

        else:

            duration = (
                MOVE_MS
                if proposed == 'M'
                else STILL_MS
            )

            if time.ticks_diff(
                now,
                self.pending_at
            ) >= duration:

                self.state = proposed


# ==================== 多物料跟踪 ====================

class MaterialTracker:

    def __init__(self):

        self.items = []
        self.next_id = 1

    def clear(self):
        self.items = []

    def update(self, detections, now):

        for t in self.items:
            t.predict(now)

        pairs = []

        for i, t in enumerate(self.items):

            for j, d in enumerate(detections):

                if t.label != d[0]:
                    continue

                dist2 = (
                    (t.kx.x - d[1]) ** 2
                    + (t.ky.x - d[2]) ** 2
                )

                gate = max(
                    22,
                    2.0 * t.r
                )

                if dist2 < gate ** 2:
                    pairs.append(
                        (dist2, i, j)
                    )

        pairs.sort()

        used_i = set()
        used_j = set()

        for _, i, j in pairs:

            if (
                i not in used_i
                and j not in used_j
            ):

                self.items[i].update(
                    detections[j],
                    now
                )

                used_i.add(i)
                used_j.add(j)

        for i, t in enumerate(self.items):

            if i not in used_i:

                t.misses += 1
                t.state = 'U'

        lost = [
            (t.label, t.id)
            for t in self.items
            if t.misses > TRACK_MISSES
        ]

        self.items = [
            t for t in self.items
            if t.misses <= TRACK_MISSES
        ]

        for j, d in enumerate(detections):

            if (
                j not in used_j
                and len(self.items) < 3
            ):

                self.items.append(
                    Track(
                        d,
                        self.next_id,
                        now
                    )
                )

                self.next_id += 1

        return lost



# ====== Strict framed ASCII protocol: @SEQ,LEN,BODY*XOR\n ======
# Checksum over ASCII bytes BETWEEN @ and *; LEN counts BODY ASCII bytes.
class PacketSerial:
    def __init__(self):
        self.uart = UART(C.UART_ID, C.UART_BAUD, timeout=0)
        self.rx = bytearray()
        self.seq = 0
        self.bad = 0
        self.rx_seq = 0
        self.rx_seq_valid = False

    def send(self, *parts):
        body = ','.join(str(p) for p in parts)
        if len(body) > 128 or any(ord(ch) < 32 or ord(ch) > 126 or ch in '@*' for ch in body):
            return False
        header = '%d,%d,%s' % (self.seq, len(body), body)
        crc = 0
        for ch in header:
            crc ^= ord(ch)
        self.uart.write(('@%s*%02X\n' % (header, crc)).encode('ascii'))
        self.seq = (self.seq + 1) % 65536
        return True

    def poll(self):
        result = []
        n = self.uart.any()
        if not n:
            return result
        data = self.uart.read(n)
        if not data:
            return result
        for b in data:
            if b == 64:  # '@' begins a new frame even after noise.
                self.rx = bytearray((b,))
            elif b == 10:
                if self.rx:
                    p = self._decode(self.rx)
                    if p is not None and self._accept_rx_seq(p[0]):
                        result.append(p[1])
                    else:
                        self.bad += 1
                self.rx = bytearray()
            elif b == 13:
                continue
            elif self.rx:
                if len(self.rx) < 180:
                    self.rx.append(b)
                else:
                    self.rx = bytearray()
                    self.bad += 1
        return result

    def _decode(self, raw):
        try:
            s = raw.decode('ascii')
            if not s.startswith('@') or s.count('*') != 1:
                return None
            content, chk = s[1:].split('*')
            if len(chk) != 2:
                return None
            crc = 0
            for ch in content:
                crc ^= ord(ch)
            if int(chk, 16) != crc:
                return None
            seq, ln, body = content.split(',', 2)
            if not seq.isdigit() or not ln.isdigit() or len(body) != int(ln) or len(body) > 128:
                return None
            if int(seq) > 65535:
                return None
            return (int(seq), body.split(','))
        except Exception:
            return None

    def _accept_rx_seq(self, sequence):
        if not self.rx_seq_valid:
            self.rx_seq = sequence
            self.rx_seq_valid = True
            return True
        distance = (sequence - self.rx_seq) % 65536
        if distance == 0 or distance >= 32768:
            return False
        self.rx_seq = sequence
        return True

    def reset_rx_sequence(self):
        self.rx_seq = 0
        self.rx_seq_valid = False


def metric_from_circle(px, py, diameter_px, known_diameter_mm):
    # 3D pinhole estimate; 3D X/Y/Z in optical frame, radial distance D.
    if (C.FX_PX <= 0 or C.FY_PX <= 0 or known_diameter_mm <= 0
            or diameter_px <= 0):
        return (0, 0, 0, 0, 0)
    # Use fx for width of face (frontal circle approximation).
    z = C.FX_PX * known_diameter_mm / float(diameter_px)
    x = (px - C.CX_PX) * z / C.FX_PX
    y = (py - C.CY_PX) * z / C.FY_PX
    d = math.sqrt(x*x + y*y + z*z)
    return (int(round(x)), int(round(y)), int(round(z)), int(round(d)), 1)


def material_metric(t):
    return metric_from_circle(t.kx.x, t.ky.x, 2.0*t.r,
                              C.TOP_FACE_DIAMETER_MM.get(t.label, 0))


def ring_metric(t):
    return metric_from_circle(t.x.x, t.y.x, 2.0*t.r.x,
                              C.RING_TRACKED_DIAMETER_MM)


def local_verified_disk(gray, prev):
    # Existing brightness/rim checks remain the FINAL criterion. No markers.
    if prev is None:
        return None
    x, y, r = prev
    if disk_rim_score(gray, x, y, r) >= 0 and disk_core_ok(gray, x, y, r):
        return prev
    best = None
    for dx in (-7, 0, 7):
        for dy in (-7, 0, 7):
            for dr in (-5, 0, 5):
                xx, yy, rr = x+dx, y+dy, r+dr
                if not DISK_R_MIN <= rr <= DISK_R_MAX:
                    continue
                score = disk_rim_score(gray, xx, yy, rr)
                if score >= 0 and (best is None or score > best[0]):
                    best = (score, xx, yy, rr)
    if best is None or not disk_core_ok(gray, best[1], best[2], best[3]):
        return None
    return best[1:]


class PersistentDiskGate(DiskGate):
    # Initial lock STILL requires 3 different colors and 5 strict disk scans.
    # After successful initial lock the trusted geometry can survive a brief
    # missing frame while checking the exact same physical rim. Supports 2/1
    # remaining pieces, without inventing virtual materials. By default the
    # trusted location survives until MODE/RESET but never validates WITHOUT
    # a fresh rim + core check on a current image.
    def __init__(self):
        super().__init__()
        self.trusted_disk = None
        self.last_valid_ms = 0

    def update(self, gray, blobs, frame):
        now = time.ticks_ms()
        if self.trusted_disk is not None:
            # Do not search an unrelated disk after losing the old one.
            if (C.DISK_REACQUIRE_MS == 0 or
                    time.ticks_diff(now, self.last_valid_ms) <= C.DISK_REACQUIRE_MS):
                found = local_verified_disk(gray, self.trusted_disk)
                if found is not None:
                    self.trusted_disk = found
                    self.disk = found
                    self.locked = True
                    self.misses = 0
                    self.last_valid_ms = now
                    return found
                self.locked = False
                self.disk = None
                return None
            # Expired: a new strict 3-material initial lock is required.
            self.trusted_disk = None
            self.disk = None
            self.locked = False
            self.hits = 0
            self.pending = None
        disk = super().update(gray, blobs, frame)
        if disk is not None and self.locked:
            self.trusted_disk = disk
            self.last_valid_ms = now
        return disk


# ===== MaterialArea: merged intact from material_workflow.py =====
"""固定物料区调度适配层（不更改 utils.py 原始 LAB / 霍夫圆 / 卡尔曼算法）。

先用 3 色物块的 120° 三角形和中心亮度确认物料区。
几何圆仅作为原物料顶部圆面算法的 ROI 参数，不验证完整盘缘。
首次确认后跨任务保留该区域，允许 2 / 1 个物料继续识别。
"""
import math
import time


def same_geometry(a, b, shift, radius_delta):
    if a is None or b is None:
        return False
    return ((a[0]-b[0])**2 + (a[1]-b[1])**2 <= shift**2
            and abs(a[2]-b[2]) <= radius_delta)


def bright_disk_core(gray, geometry, config):
    """只抽样内部 5 点，无圆周扫描；浅色背景下可能误认，因此需配合三角形。"""
    if not config.AREA_LIGHT_CHECK:
        return True
    cx, cy, r = geometry
    offset = int(round(r * 0.28))
    pts = [(int(cx), int(cy)),
           (int(cx)-offset, int(cy)), (int(cx)+offset, int(cy)),
           (int(cx), int(cy)-offset), (int(cx), int(cy)+offset)]
    hits = 0
    for x, y in pts:
        if 0 <= x < gray.width() and 0 <= y < gray.height():
            try:
                if gray.get_pixel(x, y) >= config.AREA_LIGHT_MIN:
                    hits += 1
            except Exception:
                return False
    return hits >= config.AREA_LIGHT_HITS


def bottom_singleton(tracks, geometry, config, excluded=()):
    """从当前帧已测得的物块里选下方单独那件；不以画面中心最近为准。"""
    if geometry is None:
        return None
    cx, cy, r = geometry
    good = [t for t in tracks if t.seen and t.hits >= 3 and t.label not in excluded]
    if not good:
        return None
    good.sort(key=lambda t: t.ky.x, reverse=True)
    bottom = good[0]
    x, y = bottom.kx.x, bottom.ky.x
    # 下方抓取槽：避免只剩1件时把转盘上方物块当作可抓取物。
    if y < cy + config.BOTTOM_MIN_OFFSET_RADIUS * r:
        return None
    if abs(x-cx) > config.BOTTOM_X_TOL_RADIUS * r:
        return None
    if len(good) >= 3:
        # 下方必须明显比上方两块更低，且大致在它们之间。
        if y-good[1].ky.x < config.BOTTOM_MIN_Y_GAP_PX:
            return None
        upper_x_mid = (good[1].kx.x + good[2].kx.x)/2.0
        if abs(x-upper_x_mid) > config.BOTTOM_X_TOL_RADIUS*r:
            return None
    elif len(good) == 2 and y-good[1].ky.x < config.BOTTOM_Y_GAP_2_PX:
        return None
    return bottom


class MotionWindow:
    """多帧基于物料中心位移的静止判定；不等同于直接检测转台电机。"""
    def __init__(self, config):
        self.c = config
        self.anchor = {}
        self.start = None
        self.still = False

    def reset(self):
        self.anchor = {}
        self.start = None
        self.still = False

    def update(self, tracks, now):
        visible = {t.id: (t.kx.x, t.ky.x) for t in tracks
                   if t.seen and t.hits >= self.c.TURNTABLE_STILL_MIN_HITS}
        if not visible:
            self.reset()
            return False
        if self.start is None or not (set(visible) & set(self.anchor)):
            self.anchor = visible
            self.start = now
            self.still = False
            return False
        tol2 = self.c.TURNTABLE_STILL_DISP_PX**2
        moved = any((x-self.anchor[tid][0])**2 + (y-self.anchor[tid][1])**2 > tol2
                    for tid, (x,y) in visible.items() if tid in self.anchor)
        if moved:
            self.anchor = visible
            self.start = now
            self.still = False
            return False
        for tid, p in visible.items():
            if tid not in self.anchor:
                self.anchor[tid] = p
        self.still = time.ticks_diff(now, self.start) >= self.c.TURNTABLE_STILL_MS
        return self.still


class MaterialArea:
    def __init__(self, u, config):
        self.u = u
        self.c = config
        self.tracker = u.MaterialTracker()
        self.motion = MotionWindow(config)
        self.geometry = None
        self.pending_geometry = None
        self.confirm_count = 0
        self.confirmed = False
        self.picked_labels = set()
        self.frame_count = 0

    def reset(self):
        self.tracker.clear()
        self.motion.reset()
        self.geometry = None
        self.pending_geometry = None
        self.confirm_count = 0
        self.confirmed = False
        self.picked_labels = set()
        self.frame_count = 0

    def update(self, img, gray, now):
        self.frame_count += 1
        check_layout = (not self.confirmed or
                        self.frame_count % self.c.AREA_TRIANGLE_RECHECK_FRAMES == 0)
        blobs = self.u.color_blobs(img) if check_layout else []
        hint = self.u.triangle_hint(blobs) if len(blobs) >= 3 else None
        proposed = hint[1:4] if hint is not None else None
        good = proposed is not None and bright_disk_core(gray, proposed, self.c)
        if not self.confirmed:
            if good:
                if same_geometry(self.pending_geometry, proposed,
                                 self.c.AREA_GEOMETRY_MAX_SHIFT_PX,
                                 self.c.AREA_GEOMETRY_MAX_RADIUS_DELTA_PX):
                    self.confirm_count += 1
                else:
                    self.confirm_count = 1
                self.pending_geometry = proposed
                if self.confirm_count >= self.c.AREA_CONFIRM_FRAMES:
                    self.confirmed = True
                    self.geometry = proposed
            else:
                self.confirm_count = 0
                self.pending_geometry = None
                self.motion.reset()
                self.tracker.clear()
                return []
        elif good and same_geometry(self.geometry, proposed,
                                    self.c.AREA_GEOMETRY_MAX_SHIFT_PX,
                                    self.c.AREA_GEOMETRY_MAX_RADIUS_DELTA_PX):
            # 小范围缓慢更新几何中心；绝不跟随任意跳变的误识别区域。
            cx, cy, r = self.geometry
            self.geometry = (cx*.8+proposed[0]*.2,
                             cy*.8+proposed[1]*.2, r*.8+proposed[2]*.2)
        if not self.confirmed:
            return []
        # 原源码顶部圆面霍夫检测仍完整运行。传入三角估计的几何半径，
        # 不依赖 disk_rim_score()/detect_disk()/PersistentDiskGate()。
        detections = self.u.material_candidates(img, gray, self.geometry)
        self.tracker.update(detections, now)
        self.motion.update(self.tracker.items, now)
        return self.tracker.items

    def bottom(self):
        return bottom_singleton(self.tracker.items, self.geometry, self.c,
                                self.picked_labels)

    def gripper_error(self, track):
        return (int(round(track.kx.x - self.c.GRIP_TARGET_X_PX)),
                int(round(track.ky.x - self.c.GRIP_TARGET_Y_PX)))
