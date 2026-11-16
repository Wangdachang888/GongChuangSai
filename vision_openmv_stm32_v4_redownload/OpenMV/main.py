# Command-driven visual jobs for OpenMV H7 Plus, firmware 5.x.
# Original recognition algorithms are kept in utils.py; area scheduler is appended there.
# STM32 is responsible for controlling/braking the robot; this code is vision only.
import csi
import time
import gc
import config as C
import utils as U


# ===== 单次任务模型：原 vision_job.py 已合并至本文件 =====
# Pure-Python job state; no camera or UART dependency. Compatible with MicroPython.
class Job:
    def __init__(self, kind, target, tol_x, tol_y, now, timeout_ms, required_frames):
        self.kind = kind
        self.target = target
        self.tol_x = tol_x
        self.tol_y = tol_y
        self.started = now
        self.timeout_ms = timeout_ms
        self.required_frames = required_frames
        self.streak = 0
        self.track_id = None
        self.last_pos = now
        self.last_lost = now
        self.last_measure_stamp = None
        self.align_session = 0
        self.align_step = 0
        self.align_wait_done = False

    def observed(self, dx, dy, identity=None):
        # Do not accumulate samples from different objects of the same color.
        if identity is not None:
            if self.track_id is not None and self.track_id != identity:
                self.streak = 0
            self.track_id = identity
        if abs(dx) <= self.tol_x and abs(dy) <= self.tol_y:
            self.streak += 1
        else:
            self.streak = 0
        return self.streak >= self.required_frames

    def missing(self):
        self.streak = 0
        self.track_id = None


def decode_job(parts, now, config):
    # JOB,id[,tolerance_x_px,tolerance_y_px]
    if len(parts) not in (2, 4):
        raise ValueError('JOB_FORMAT')
    code = int(parts[1])
    if code == 1:
        kind, target = 'QR', 0
    elif 11 <= code <= 16:
        kind, target = 'MATERIAL', code - 10
    elif 21 <= code <= 23:
        kind, target = 'RING', code - 20
    elif code == 40:
        kind, target = 'AREA', 0
    elif code == 41:
        kind, target = 'MATERIAL_ALIGN', 0
    elif code == 42:
        kind, target = 'MATERIAL_PICK', 0
    elif code == 31:
        kind, target = 'EDGE', 0
    elif code == 32:
        kind, target = 'BLUE', 0
    else:
        raise ValueError('JOB_CODE')
    tol_x = config.JOB_TOL_X_PX
    tol_y = config.JOB_TOL_Y_PX
    if len(parts) == 4:
        tol_x, tol_y = int(parts[2]), int(parts[3])
    if not (1 <= tol_x <= 100 and 1 <= tol_y <= 100):
        raise ValueError('JOB_TOL')
    if kind == 'QR':
        timeout = config.JOB_QR_TIMEOUT_MS
    elif kind == 'AREA':
        timeout = config.AREA_JOB_TIMEOUT_MS
    elif kind == 'MATERIAL_ALIGN':
        timeout = config.ALIGN_JOB_TIMEOUT_MS
    elif kind == 'MATERIAL_PICK':
        timeout = config.PICK_JOB_TIMEOUT_MS
    else:
        timeout = config.JOB_TIMEOUT_MS
    return code, Job(kind, target, tol_x, tol_y, now, timeout, config.JOB_ARRIVED_FRAMES)




def camera_create():
    cam = csi.CSI()
    cam.reset()
    cam.pixformat(csi.GRAYSCALE)
    cam.framesize(csi.QVGA)
    cam.snapshot(time=1500)
    return cam


def camera_mode(cam, mode):
    # Preserve the latest source camera settings.
    color = mode in ('MATERIAL', 'BLUE', 'RING', 'RING_ALIGN', 'AREA', 'MATERIAL_ALIGN', 'MATERIAL_PICK')
    cam.pixformat(csi.RGB565 if color else csi.GRAYSCALE)
    cam.framesize(csi.QVGA)
    cam.snapshot(time=350)
    if color:
        cam.auto_gain(False)
        cam.auto_whitebal(False)
    else:
        cam.auto_gain(True)


def choose_material(tracker, target):
    # Preserve original white-disk gate + top-face candidates + multi-Kalman.
    # Only consider objects SEEN in the current frame and >=3 hits.
    candidates = [t for t in tracker.items
                  if t.seen and t.hits >= 3 and t.label == target]
    if not candidates:
        return None
    def ranking(t):
        x, y, z, distance, valid = U.material_metric(t)
        if valid:
            return (0, distance)
        return (1, (t.kx.x-C.CX_PX)**2+(t.ky.x-C.CY_PX)**2)
    return min(candidates, key=ranking)


def choose_ring(rings, target, now):
    # Must be measured in this frame, not an old Kalman prediction.
    matches = [t for t in rings.tracks
               if t.hits >= C.RING_MIN_HITS and t.miss == 0
               and time.ticks_diff(now, t.last_measured) < 700
               and t.voted_digit(now) == target]
    return matches[0] if len(matches) == 1 else None


def main():
    cam = camera_create()
    serial = U.PacketSerial()  # Original @SEQ,LEN,BODY*XOR\n protocol
    board = U.TaskBoard()
    try:
        lcd = U.TaskLCD()
        lcd.update(board)
    except Exception as exc:
        lcd = None
        print('LCD unavailable:', exc)

    job = None
    active_camera_mode = 'QR'
    disk_gate = U.PersistentDiskGate()
    material_tracker = U.MaterialTracker()  # legacy JOB11~16 only
    area = U.MaterialArea(U, C)  # new JOB40/41/42
    pick_indices = [0, 0]     # QR material groups 1/3; advance ONLY on PICK_OK
    pick_pending = None        # (batch_index, color, index) after PICK_READY
    rings = U.RingFinder()
    disk_locked = False
    frame = 0
    qr_last = ''
    qr_streak = 0
    last_wait_report = 0

    def idle():
        # Stop processing; keep the verified disk lock across consecutive
        # material jobs, as later pickups can leave fewer than 3 objects.
        nonlocal job
        job = None
        serial.send('IDLE')

    def wait_report(now, *parts):
        nonlocal last_wait_report
        if time.ticks_diff(now, last_wait_report) >= C.AREA_STATUS_PERIOD_MS:
            serial.send('WAIT', *parts)
            last_wait_report = now

    serial.send('READY', 'IDLE')
    while True:
        now = time.ticks_ms()
        # Commands are polled at every iteration, including while waiting.
        for packet in serial.poll():
            command = packet[0]
            try:
                if command == 'PING' and len(packet) == 1:
                    serial.send('PONG')
                elif command == 'STATUS' and len(packet) == 1:
                    serial.send('STATUS', 'IDLE' if job is None else job.kind,
                                0 if job is None else job.target,
                                int(area.confirmed if job is None or job.kind in ('AREA','MATERIAL_ALIGN','MATERIAL_PICK') else disk_locked),
                                int(board.locked()), board.completed)
                elif command == 'CANCEL' and len(packet) == 1:
                    if job is not None:
                        serial.send('CANCELLED', job.kind, job.target)
                        idle()
                    elif pick_pending is not None:
                        pick_pending = None
                        serial.send('CANCELLED', 'PICK')
                        serial.send('IDLE')
                    else:
                        serial.send('ACK', 'IDLE')
                elif command == 'RESET' and len(packet) == 1:
                    job = None
                    board.reset()
                    if lcd:
                        lcd.update(board)
                    disk_gate = U.PersistentDiskGate()
                    material_tracker.clear()
                    area.reset()
                    pick_indices = [0, 0]
                    pick_pending = None
                    rings.reset()
                    disk_locked = False
                    qr_last, qr_streak = '', 0
                    serial.reset_rx_sequence()
                    serial.send('ACK', 'RESET')
                    serial.send('IDLE')
                elif command == 'PICK_STATUS' and len(packet) == 1:
                    batch = 0 if board.completed < 2 else 1
                    serial.send('PICK_STATUS', batch+1, pick_indices[batch]+1,
                                0 if pick_pending is None else pick_pending[1])
                elif command == 'DONE' and len(packet) == 2:
                    # Original LCD group completion interface (not a vision job).
                    group = int(packet[1])
                    required_batch = 0 if group <= 2 else 1
                    if (group in (1, 2, 3, 4) and pick_indices[required_batch] < 3):
                        serial.send('ERR', 'PICKS_INCOMPLETE')
                        continue
                    if board.finish_group(group):
                        if lcd:
                            lcd.update(board)
                        if group == 2:  # second loading of three objects
                            area.reset()
                            pick_pending = None
                        serial.send('ACK', 'DONE', group)
                    else:
                        serial.send('ERR', 'DONE_ORDER')
                elif command == 'PICK_OK' and len(packet) == 1:
                    if job is not None or pick_pending is None:
                        serial.send('ERR', 'NO_PICK_PENDING')
                        continue
                    batch, color, index = pick_pending
                    if pick_indices[batch] != index:
                        serial.send('ERR', 'PICK_ORDER')
                        continue
                    pick_indices[batch] += 1
                    area.picked_labels.add(color)
                    pick_pending = None
                    serial.send('ACK', 'PICK_OK', color, batch+1, index+1)
                elif command == 'PICK_FAIL' and len(packet) == 1:
                    if job is not None or pick_pending is None:
                        serial.send('ERR', 'NO_PICK_PENDING')
                    else:
                        pick_pending = None
                        serial.send('ACK', 'PICK_FAIL')
                elif command == 'AREA_RESET' and len(packet) == 1:
                    if job is not None or pick_pending is not None:
                        serial.send('ERR', 'BUSY')
                    else:
                        area.reset()
                        serial.send('ACK', 'AREA_RESET')
                elif command == 'STEP_DONE' and len(packet) == 3:
                    if (job is None or job.kind != 'RING_ALIGN' or
                            not job.align_wait_done):
                        serial.send('ERR', 'NO_ALIGN_STEP')
                        continue
                    step = int(packet[1])
                    moved = int(packet[2])
                    if step != job.align_step or moved not in (0, 1):
                        serial.send('ERR', 'ALIGN_STEP_ORDER')
                        continue
                    job.align_wait_done = False
                    job.align_step += 1
                    job.last_measure_stamp = None
                    serial.send('ACK', 'STEP_DONE', step)
                elif command == 'ARRIVED' and len(packet) == 7:
                    if job is None or job.kind != 'RING_ALIGN':
                        serial.send('ERR', 'NO_ALIGN_JOB')
                        continue
                    target = int(packet[2])
                    session = int(packet[5])
                    if (target != job.target or session != job.align_session or
                            int(packet[6]) < 1):
                        serial.send('ERR', 'ALIGN_SESSION')
                        continue
                    serial.send('ACK', 'ARRIVED', session)
                    idle()
                elif command == 'ALIGN' and len(packet) == 3:
                    if job is not None or pick_pending is not None:
                        serial.send('ERR', 'BUSY')
                        continue
                    target = int(packet[1])
                    session = int(packet[2])
                    if target not in (1, 2, 3) or not (1 <= session <= 65535):
                        serial.send('ERR', 'ALIGN_FORMAT')
                        continue
                    new_job = Job('RING_ALIGN', target, C.JOB_TOL_X_PX,
                                  C.JOB_TOL_Y_PX, now,
                                  C.ALIGN_JOB_TIMEOUT_MS, 1)
                    new_job.align_session = session
                    new_job.align_step = 0
                    new_job.align_wait_done = False
                    if active_camera_mode != 'RING_ALIGN':
                        camera_mode(cam, 'RING_ALIGN')
                        active_camera_mode = 'RING_ALIGN'
                    rings.reset()
                    frame = 0
                    job = new_job
                    serial.send('ACK', 'ALIGN', target, session)
                elif command == 'JOB':
                    if job is not None:
                        serial.send('ERR', 'BUSY')
                        continue
                    if pick_pending is not None:
                        serial.send('ERR', 'PICK_PENDING')
                        continue
                    code, new_job = decode_job(packet, now, C)
                    if new_job.kind in ('MATERIAL_ALIGN', 'MATERIAL_PICK'):
                        if not C.GRIP_TARGET_CALIBRATED:
                            serial.send('ERR', 'GRIP_CALIBRATION_REQUIRED')
                            continue
                        if not area.confirmed:
                            serial.send('ERR', 'AREA_REQUIRED')
                            continue
                    if new_job.kind == 'MATERIAL_PICK':
                        if not board.locked():
                            serial.send('ERR', 'QR_REQUIRED')
                            continue
                        batch = 0 if board.completed < 2 else 1
                        index = pick_indices[batch]
                        if index >= 3:
                            serial.send('ERR', 'BATCH_DONE')
                            continue
                        new_job.target = int(board.groups[batch*2][index])
                        new_job.batch = batch
                        new_job.index = index
                    if new_job.kind in ('MATERIAL_ALIGN', 'MATERIAL_PICK'):
                        # Standstill confirmation starts FRESH for every job;
                        # idle time must not count as proof of stopped platter.
                        area.motion.reset()
                    if new_job.kind == 'AREA':
                        # Explicit re-localization requires 3 colours, not only old geometry.
                        area.reset()
                    new_job.bottom_track_id = None
                    last_wait_report = time.ticks_add(now, -C.AREA_STATUS_PERIOD_MS)

                    if active_camera_mode != new_job.kind:
                        # Existing legacy MATERIAL jobs still use their strict disk gate.
                        if active_camera_mode == 'MATERIAL' or new_job.kind == 'MATERIAL':
                            disk_gate = U.PersistentDiskGate()
                            disk_locked = False
                        camera_mode(cam, new_job.kind)
                        active_camera_mode = new_job.kind
                    if new_job.kind == 'MATERIAL':
                        material_tracker.clear()
                    elif new_job.kind == 'RING':
                        rings.reset()
                    elif new_job.kind == 'QR':
                        # A QR job scans fresh; previously displayed task code is
                        # retained until RESET, not overwritten by another scan.
                        qr_last, qr_streak = '', 0
                    frame = 0
                    job = new_job
                    serial.send('ACK', 'JOB', code)
                    if new_job.kind == 'MATERIAL_PICK':
                        serial.send('EXPECT', 'COLOR', new_job.target, new_job.batch+1, new_job.index+1)
                else:
                    serial.send('ERR', 'BAD_COMMAND')
            except (ValueError, TypeError) as exc:
                serial.send('ERR', str(exc).replace(',', '_'))

        if job is None:
            time.sleep_ms(C.JOB_IDLE_SLEEP_MS)
            continue
        now = time.ticks_ms()
        if time.ticks_diff(now, job.started) >= job.timeout_ms:
            serial.send('TIMEOUT', job.kind, job.target)
            idle()
            continue

        img = cam.snapshot()
        now = time.ticks_ms()
        frame += 1
        observed = None  # (dx,dy,px,py,identity); only real current-frame sightings

        if job.kind == 'QR':
            found = None
            for qr in img.find_qrcodes():
                code = U.normalize_code(U.val(qr, 'payload', ''))
                if U.parse_code(code) is not None:
                    found = code
                    break
            if found is None:
                qr_streak = 0
            elif found == qr_last:
                qr_streak += 1
            else:
                qr_last, qr_streak = found, 1
            if found and qr_streak >= C.QR_CONFIRM_FRAMES:
                if not board.locked():
                    board.accept(found)
                    if lcd:
                        lcd.update(board)
                if board.code() != found:
                    serial.send('ERR', 'QR_CONFLICT')
                    idle()
                    continue
                serial.send('QR', found)
                serial.send('DONE', 'QR')
                idle()
                continue

        elif job.kind in ('AREA', 'MATERIAL_ALIGN', 'MATERIAL_PICK'):
            # The original material detection & Kalman tracker remain unchanged.
            # Only their entry condition is changed: coarse material-zone geometry
            # replaces strict full-rim disk gating.
            gray = img.to_grayscale(copy=True)
            try:
                area.update(img, gray, now)
            finally:
                del gray
            if job.kind == 'AREA':
                if area.confirmed:
                    cx0, cy0, r0 = area.geometry
                    serial.send('AREA_OK', int(cx0), int(cy0), int(r0))
                    serial.send('DONE', 'AREA')
                    idle()
                    continue
                wait_report(now, 'AREA_SEARCH')
                continue

            bottom = area.bottom()
            if not area.motion.still:
                # When the chassis is moving this also becomes non-still;
                # position feedback may continue, but NEVER report arrival.
                if job.kind == 'MATERIAL_ALIGN' and job.bottom_track_id is None:
                    wait_report(now, 'ROTATING')
                elif job.kind == 'MATERIAL_PICK':
                    wait_report(now, 'ROTATING')
                job.missing()
            if bottom is None:
                job.missing()
                wait_report(now, 'BOTTOM_NOT_FOUND')
            elif job.kind == 'MATERIAL_ALIGN':
                if job.bottom_track_id is None:
                    if not area.motion.still:
                        continue
                    job.bottom_track_id = bottom.id
                if bottom.id != job.bottom_track_id:
                    job.bottom_track_id = None
                    job.missing()
                    wait_report(now, 'BOTTOM_CHANGED')
                    continue
                px, py = int(round(bottom.kx.x)), int(round(bottom.ky.x))
                dx, dy = area.gripper_error(bottom)
                # The material target field 0 = any colour, fixed lower pickup slot.
                observed = (dx, dy, px, py, bottom.id)
            elif job.kind == 'MATERIAL_PICK':
                if not area.motion.still:
                    # Do not command grasp while the turntable appears moving.
                    continue
                if bottom.label != job.target:
                    job.missing()
                    wait_report(now, 'COLOR', job.target, bottom.label)
                    continue
                px, py = int(round(bottom.kx.x)), int(round(bottom.ky.x))
                dx, dy = area.gripper_error(bottom)
                observed = (dx, dy, px, py, bottom.id)

        elif job.kind == 'MATERIAL':
            blobs = U.color_blobs(img)
            gray = img.to_grayscale(copy=True)
            disk = disk_gate.update(gray, blobs, frame)
            disk_locked = disk is not None
            if disk is not None:
                detections = U.material_candidates(img, gray, disk)
                material_tracker.update(detections, now)
                t = choose_material(material_tracker, job.target)
                if t is not None:
                    px, py = int(round(t.kx.x)), int(round(t.ky.x))
                    observed = (px-int(C.CX_PX), py-int(C.CY_PX), px, py, t.id)
            else:
                material_tracker.clear()
            del gray

        elif job.kind == 'RING_ALIGN':
            rings.process(img)
            ring = choose_ring(rings, job.target, now)
            if (ring is not None and not job.align_wait_done and
                    ring.last_measured != job.last_measure_stamp):
                job.last_measure_stamp = ring.last_measured
                px = int(round(ring.x.x))
                py = int(round(ring.y.x))
                dx = px - int(C.RING_TARGET_X_PX)
                dy = py - int(C.RING_TARGET_Y_PX)
                serial.send('POS', 'RING', job.target, dx, dy, px, py,
                            job.align_session, job.align_step)
                job.align_wait_done = True
                job.last_pos = now
            elif (ring is None and
                  time.ticks_diff(now, job.last_lost) >= C.JOB_LOST_PERIOD_MS):
                serial.send('LOST', 'RING', job.target)
                job.last_lost = now
            continue

        elif job.kind == 'RING':
            rings.process(img)
            t = choose_ring(rings, job.target, now)
            if t is not None and t.last_measured != job.last_measure_stamp:
                job.last_measure_stamp = t.last_measured
                px, py = int(round(t.x.x)), int(round(t.y.x))
                observed = (px-int(C.RING_TARGET_X_PX), py-int(C.RING_TARGET_Y_PX), px, py, None)

        elif job.kind == 'EDGE':
            line = U.detect_edge(img)
            if line is not None:
                x1, y1, x2, y2, row = line[:5]
                px = int(round((x1+x2)/2))
                py = int(round(row))
                observed = (px-int(C.CX_PX), py-int(C.EDGE_STOP_ROW), px, py, None)

        elif job.kind == 'BLUE':
            region = U.detect_blue(img)
            if region is not None:
                x, y, w, h, px, py, kind = region
                observed = (px-int(C.CX_PX), py-int(C.CY_PX), px, py, None)

        if observed is None:
            if job.kind == 'RING' and rings.cycle % C.RING_DETECT_INTERVAL:
                continue  # Detector is intentionally skipped on this frame.
            job.missing()
            if time.ticks_diff(now, job.last_lost) >= C.JOB_LOST_PERIOD_MS:
                report_kind = 'MATERIAL' if job.kind in ('MATERIAL_ALIGN', 'MATERIAL_PICK') else job.kind
                serial.send('LOST', report_kind, job.target)
                job.last_lost = now
        else:
            dx, dy, px, py, identity = observed
            # Stability must be rechecked at the final pose, otherwise a rotating
            # turntable could briefly pass through the tolerance window.
            if job.kind in ('MATERIAL_ALIGN', 'MATERIAL_PICK') and not area.motion.still:
                job.missing()
                arrived = False
            else:
                arrived = job.observed(dx, dy, identity)
            if time.ticks_diff(now, job.last_pos) >= C.JOB_POS_PERIOD_MS:
                report_kind = 'MATERIAL' if job.kind in ('MATERIAL_ALIGN', 'MATERIAL_PICK') else job.kind
                serial.send('POS', report_kind, job.target, dx, dy, px, py)
                job.last_pos = now
            if arrived:
                # One ARRIVED, one IDLE; never sends another POS until next JOB.
                if job.kind == 'MATERIAL_PICK':
                    pick_pending = (job.batch, job.target, job.index)
                    serial.send('ARRIVED', 'MATERIAL', job.target, dx, dy)
                    serial.send('PICK_READY', job.target, job.batch+1, job.index+1)
                else:
                    report_kind = 'MATERIAL' if job.kind == 'MATERIAL_ALIGN' else job.kind
                    serial.send('ARRIVED', report_kind, job.target, dx, dy)
                idle()
                continue
        if frame % 24 == 0:
            gc.collect()


if __name__ == '__main__':
    main()
