"""Host-only synthetic checks for the Motor.c M/T estimator contract.
This does not emulate STM32 registers or claim hardware validation.
"""
from dataclasses import dataclass

ENTER, EXIT, TIMEOUT = 2, 4, 0.120

@dataclass
class Est:
    mode: str = "M"
    speed: float = 0.0
    last_edge: float | None = None
    prev_edge: float | None = None
    direction: int = 1

    def update(self, count: int, now: float, edge_times=()):
        for t, direction in edge_times:
            self.prev_edge, self.last_edge = self.last_edge, t
            self.direction = direction
        n = abs(count)
        if self.mode == "M" and n <= ENTER:
            self.mode = "T"
        elif self.mode == "T" and n >= EXIT:
            self.mode = "M"
        if self.mode == "T" and self.last_edge is not None and self.prev_edge is not None:
            period = self.last_edge - self.prev_edge
            if period > 0 and now - self.last_edge <= TIMEOUT:
                raw = 4.0 / period * self.direction
            elif now - self.last_edge > TIMEOUT:
                raw = 0.0
            else:
                raw = count / 0.005
        else:
            raw = count / 0.005
        self.speed = raw
        return raw

def signed_delta(now, old, bits):
    mask = (1 << bits) - 1
    d = (now - old) & mask
    if d & (1 << (bits - 1)):
        d -= 1 << bits
    return d

def test_counter_wraps():
    assert signed_delta(0x0001, 0xFFFF, 16) == 2
    assert signed_delta(0xFFFF, 0x0001, 16) == -2
    assert signed_delta(0x00000001, 0xFFFFFFFF, 32) == 2
    assert signed_delta(0xFFFFFFFF, 0x00000001, 32) == -2

def test_mt_switch_and_continuity():
    e = Est()
    assert e.update(0, 0.005) == 0
    assert e.mode == "T"
    assert e.update(1, 0.040, ((0.038, 1),)) == 200.0
    assert e.update(1, 0.080, ((0.078, 1),)) == 100.0
    assert e.mode == "T"
    e.update(4, 0.085, ((0.083, 1),))
    assert e.mode == "M"
    assert e.update(-4, 0.090, ((0.088, -1),)) == -800.0

def test_direction_and_timeout():
    e = Est()
    e.update(1, 0.040, ((0.020, -1),))
    e.update(1, 0.080, ((0.060, -1),))
    assert e.speed < 0
    assert e.update(0, 0.200) == 0.0

def test_bad_timestamps_fall_back_to_m():
    e = Est()
    e.update(1, 0.010, ((0.020, 1),))
    # Repeated timestamp is rejected; M count remains the safe estimate.
    assert e.update(1, 0.015, ((0.020, 1),)) == 200.0

if __name__ == "__main__":
    test_counter_wraps()
    test_mt_switch_and_continuity()
    test_direction_and_timeout()
    test_bad_timestamps_fall_back_to_m()
    print("PASS: synthetic M/T estimator checks (host-only; hardware NOT_RUN)")
