"""Updraft - steer a bird through a winding canyon with a TF-Luna lidar + ultrasonic sensor.

Hold your hand above the sensors: raise it to fly higher, lower it to dive.
Reads the "Luna (cm): N  Sonar (cm): N" lines printed by the Arduino sketch
and combines them into one hand distance.

Usage:  python updraft.py [--port COM3] [--sensors both|luna|sonar] [--low 5] [--high 60]
"""
import argparse
import math
import os
import random
import re
import threading
import time
from collections import deque

import pygame

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None

W, H = 960, 540
FPS = 60
BIRD_X = 220
BIRD_R = 14
STEP = 16  # canyon wall segment width in px
DRIFT_SPEED = 90  # px/s the bird moves toward out-of-range readings
WARN_LOOKAHEAD = 100  # px ahead of the bird checked for crash warnings
WARN_BORDER = 10  # thickness of the red warning border in px
MEDIAN_WINDOW = 5  # sensor readings combined to reject stray echoes
TARGET_SMOOTHING = 0.12  # seconds; higher = smoother but laggier bird
DEADBAND_CM = 1.1  # hand movements smaller than this are treated as sensor noise
COUNTDOWN = 5  # seconds before the canyon starts moving
LUNA_MIN_CM = 20  # TF-Luna readings closer than this are ignored
AGREE_CM = 8  # sensors within this many cm are averaged together
LUNA_WEIGHT = 0.7  # share of the TF-Luna in that average (it's the more precise one)
BEST_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "best.txt")

LINE_RE = re.compile(r"Distance \(cm\):\s*(-?\d+)")
DUAL_RE = re.compile(r"Luna \(cm\):\s*(-?\d+)\s+Sonar \(cm\):\s*(-?\d+)")

SKY_TOP = (24, 32, 72)
SKY_BOTTOM = (236, 140, 96)
ROCK = (58, 40, 52)
ROCK_EDGE = (120, 78, 82)
SEED = (255, 226, 120)
STORM = (52, 56, 74)
TEXT = (245, 240, 230)
WARN = (235, 60, 60)


def recent_median(history):
    """Median of the valid readings in history, or None if too many recent reads failed."""
    valid = sorted(v for v in history if v is not None)
    if len(valid) * 2 <= len(history):
        return None
    return valid[len(valid) // 2]


def fuse(luna, sonar):
    """Combine the two sensors into one hand distance (cm), or None if neither sees it."""
    if luna is not None and luna < LUNA_MIN_CM:
        luna = None  # the TF-Luna is unreliable this close
    if luna is None or sonar is None:
        return sonar if luna is None else luna
    if abs(luna - sonar) <= AGREE_CM:
        return round(LUNA_WEIGHT * luna + (1 - LUNA_WEIGHT) * sonar)
    # disagreeing usually means the lidar's narrow beam missed the hand and hit something
    # behind it, so trust the nearer reading
    return min(luna, sonar)


class SensorReader(threading.Thread):
    """Reads both sensors' distance lines from the Arduino in the background."""

    def __init__(self, port, sensors="both", baud=115200):
        super().__init__(daemon=True)
        self.port = port
        self.sensors = sensors  # "both", "luna" or "sonar"
        self.baud = baud
        self.distance = None  # fused hand distance
        self.luna = None  # per-sensor medians, so single bad readings are ignored
        self.sonar = None
        self.luna_history = deque(maxlen=MEDIAN_WINDOW)
        self.sonar_history = deque(maxlen=MEDIAN_WINDOW)
        self.last_update = 0.0
        self.status = "connecting..."
        self.stop_event = threading.Event()

    def add_readings(self, luna, sonar):
        if self.sensors == "sonar":
            luna = None
        elif self.sensors == "luna":
            sonar = None
        self.luna_history.append(luna if luna is not None and luna >= 0 else None)
        self.sonar_history.append(sonar if sonar is not None and sonar >= 0 else None)
        self.luna = recent_median(self.luna_history)
        self.sonar = recent_median(self.sonar_history)
        fused = fuse(self.luna, self.sonar)
        if fused is not None:
            self.distance = fused
            self.last_update = time.monotonic()
            self.status = f"sensors on {self.port}"
        else:
            self.status = "no sensor sees your hand - check wiring"

    def run(self):
        while not self.stop_event.is_set():
            try:
                with serial.Serial(self.port, self.baud, timeout=1) as ser:
                    self.status = f"waiting for data on {self.port}"
                    while not self.stop_event.is_set():
                        line = ser.readline().decode(errors="ignore")
                        dual = DUAL_RE.search(line)
                        single = LINE_RE.search(line)
                        if dual:
                            self.add_readings(int(dual.group(1)), int(dual.group(2)))
                        elif single:
                            # older single-sensor sketch
                            self.add_readings(None, int(single.group(1)))
                        elif "Error" in line:
                            self.status = "sensor read error - check wiring"
            except (serial.SerialException, OSError) as exc:
                self.status = f"can't open {self.port}: {exc}"[:70]
                time.sleep(1)


def find_arduino_port():
    if serial is None:
        return None
    for p in list_ports.comports():
        if p.vid == 0x2341 or "Arduino" in (p.description or ""):
            return p.device
    return None


class Canyon:
    """Scrolling canyon made of evenly spaced (x, top, bottom) segments."""

    def __init__(self):
        self.segs = deque()
        self.center = H / 2
        self.gap = 340.0
        self.vel = 0.0
        x = 0.0
        while x < W + STEP * 2:
            self.segs.append(self._make(x, 0))
            x += STEP

    def _make(self, x, difficulty):
        target_gap = max(150.0, 340.0 - difficulty * 9)
        self.gap += (target_gap - self.gap) * 0.05
        wiggle = min(2.2, 0.5 + difficulty * 0.09)
        self.vel = (self.vel + random.uniform(-wiggle, wiggle)) * 0.93
        self.center += self.vel
        half = self.gap / 2
        lo, hi = 24 + half, H - 24 - half
        if self.center < lo:
            self.center, self.vel = lo, abs(self.vel)
        elif self.center > hi:
            self.center, self.vel = hi, -abs(self.vel)
        return [x, self.center - half, self.center + half]

    def update(self, dx, difficulty):
        for seg in self.segs:
            seg[0] -= dx
        while self.segs[1][0] < 0:
            self.segs.popleft()
        while self.segs[-1][0] < W + STEP:
            self.segs.append(self._make(self.segs[-1][0] + STEP, difficulty))

    def walls_at(self, x):
        i = int((x - self.segs[0][0]) // STEP)
        i = max(0, min(i, len(self.segs) - 2))
        a, b = self.segs[i], self.segs[i + 1]
        t = (x - a[0]) / STEP
        return a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t

    def draw(self, surf):
        first, last = self.segs[0][0], self.segs[-1][0]
        top = [(s[0], s[1]) for s in self.segs]
        bottom = [(s[0], s[2]) for s in self.segs]
        pygame.draw.polygon(surf, ROCK, [(first, 0)] + top + [(last, 0)])
        pygame.draw.polygon(surf, ROCK, [(first, H)] + bottom + [(last, H)])
        pygame.draw.lines(surf, ROCK_EDGE, False, top, 4)
        pygame.draw.lines(surf, ROCK_EDGE, False, bottom, 4)


def make_sky():
    sky = pygame.Surface((W, H))
    for y in range(H):
        t = y / H
        color = [int(SKY_TOP[i] + (SKY_BOTTOM[i] - SKY_TOP[i]) * t) for i in range(3)]
        pygame.draw.line(sky, color, (0, y), (W, y))
    return sky


def draw_hills(surf, offset):
    pts = [(0, H)]
    for x in range(0, W + 20, 20):
        wx = x + offset
        y = H * 0.62 + math.sin(wx * 0.006) * 40 + math.sin(wx * 0.017) * 14
        pts.append((x, y))
    pts.append((W, H))
    pygame.draw.polygon(surf, (150, 90, 100), pts)


def draw_bird(surf, y, tilt, t, dead=False):
    body = pygame.Surface((60, 60), pygame.SRCALPHA)
    flap = math.sin(t * 14) * 9 if not dead else 0
    pygame.draw.ellipse(body, (90, 200, 220), (14, 21, 32, 20))
    pygame.draw.polygon(body, (60, 150, 190), [(26, 30), (36, 30), (30, 18 - flap)])
    pygame.draw.polygon(body, (255, 170, 60), [(45, 29), (54, 32), (45, 35)])
    pygame.draw.circle(body, (255, 255, 255), (40, 28), 4)
    pygame.draw.circle(body, (20, 20, 30), (41, 28), 2)
    pygame.draw.polygon(body, (60, 150, 190), [(15, 29), (6, 24), (8, 34)])
    rotated = pygame.transform.rotate(body, tilt)
    surf.blit(rotated, rotated.get_rect(center=(BIRD_X, int(y))))


def draw_storm(surf, storm, t):
    x, y, r = storm["x"], storm["y"], storm["r"]
    for dx, dy, k in ((-0.6, 0.1, 0.7), (0.6, 0.15, 0.65), (0, -0.2, 0.85), (0, 0.2, 0.8)):
        pygame.draw.circle(surf, STORM, (int(x + dx * r), int(y + dy * r)), int(r * k))
    if int(t * 6 + storm["phase"]) % 5 == 0:
        bolt = [(x - 4, y + r * 0.5), (x + 6, y + r * 0.9), (x - 2, y + r * 1.0), (x + 4, y + r * 1.4)]
        pygame.draw.lines(surf, (255, 250, 170), False, bolt, 3)


def load_best():
    try:
        with open(BEST_FILE) as f:
            return int(f.read().strip() or 0)
    except (OSError, ValueError):
        return 0


def save_best(best):
    try:
        with open(BEST_FILE, "w") as f:
            f.write(str(best))
    except OSError:
        pass


class Game:
    def __init__(self, args):
        pygame.init()
        pygame.display.set_caption("Updraft")
        self.screen = pygame.display.set_mode((W, H))
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont("segoeui", 22)
        self.big = pygame.font.SysFont("segoeui", 64, bold=True)
        self.sky = make_sky()
        self.low, self.high = args.low, args.high
        self.best = load_best()

        port = args.port or find_arduino_port()
        self.sensor = None
        if serial is None:
            self.no_sensor_reason = "pyserial not installed (pip install pyserial)"
        elif port is None:
            self.no_sensor_reason = "no Arduino found - pass --port COMx"
        else:
            self.sensor = SensorReader(port, args.sensors)
            self.sensor.start()
        self.use_mouse = self.sensor is None
        self.state = "title"
        self.reset()

    def reset(self):
        self.canyon = Canyon()
        self.bird_y = H / 2
        self.bird_vy = 0.0
        self.seeds = []
        self.storms = []
        self.particles = []
        self.elapsed = 0.0
        self.travel = 0.0
        self.seed_score = 0
        self.seed_timer = 1.0
        self.storm_timer = 6.0
        self.hill_offset = 0.0
        self.target = self.bird_y
        self.countdown = 0.0
        self.danger_top = False
        self.danger_bottom = False

    @property
    def score(self):
        return int(self.travel / 40) + self.seed_score

    def target_y(self):
        """Screen y the bird should head toward, and whether the reading is out of range."""
        if self.use_mouse:
            return pygame.mouse.get_pos()[1], False
        if self.sensor is None or self.sensor.distance is None:
            return self.bird_y, False
        d = self.sensor.distance
        span = max(1, self.high - self.low)
        t = max(0.0, min(1.0, (d - self.low) / span))
        margin = 30
        return H - margin - t * (H - 2 * margin), not self.low <= d <= self.high

    def handle_key(self, key):
        if key == pygame.K_ESCAPE:
            return False
        if key == pygame.K_m and self.sensor is not None:
            self.use_mouse = not self.use_mouse
        if self.sensor is not None and self.sensor.distance is not None:
            if key == pygame.K_l:
                self.low = self.sensor.distance
            elif key == pygame.K_h:
                self.high = self.sensor.distance
        if key == pygame.K_SPACE and self.state in ("title", "over"):
            self.reset()
            self.state = "countdown"
            self.countdown = COUNTDOWN
        return True

    def update_danger(self):
        """Would the bird hit a wall if it reached the current target over the next stretch?"""
        ahead = [self.canyon.walls_at(x) for x in range(BIRD_X, BIRD_X + WARN_LOOKAHEAD, STEP)]
        self.danger_top = self.target - BIRD_R + 3 < max(a[0] for a in ahead)
        self.danger_bottom = self.target + BIRD_R - 3 > min(a[1] for a in ahead)

    def update(self, dt):
        raw_target, out_of_range = self.target_y()
        # ignore the sensor's ~1 cm wobble, then smooth what's left before the bird follows it
        deadband = 0 if self.use_mouse else DEADBAND_CM * (H - 60) / max(1, self.high - self.low)
        if abs(raw_target - self.target) > deadband:
            self.target += (raw_target - self.target) * min(1.0, dt / TARGET_SMOOTHING)
        if out_of_range:
            # readings past the calibrated range are often stray echoes, so drift there slowly
            step = max(-DRIFT_SPEED * dt, min(DRIFT_SPEED * dt, self.target - self.bird_y))
            new_y = self.bird_y + step
        else:
            new_y = self.bird_y + (self.target - self.bird_y) * min(1.0, dt * 10)
        self.bird_vy = (new_y - self.bird_y) / max(dt, 1e-6)
        self.bird_y = new_y

        if self.state == "countdown":
            # canyon holds still so the player can line up with the gap
            self.update_danger()
            self.countdown -= dt
            if self.countdown <= 0:
                self.state = "play"
            return

        if self.state != "play":
            self.hill_offset += 30 * dt
            return

        self.elapsed += dt
        difficulty = self.elapsed / 8
        speed = min(420, 180 + difficulty * 14)
        dx = speed * dt
        self.travel += dx
        self.hill_offset += dx * 0.3
        self.canyon.update(dx, difficulty)

        self.seed_timer -= dt
        if self.seed_timer <= 0:
            self.seed_timer = random.uniform(0.8, 1.6)
            top, bottom = self.canyon.walls_at(W + 20)
            self.seeds.append({"x": W + 20, "y": random.uniform(top + 30, bottom - 30)})

        self.storm_timer -= dt
        if self.storm_timer <= 0:
            self.storm_timer = max(1.4, random.uniform(2.5, 4.0) - difficulty * 0.1)
            top, bottom = self.canyon.walls_at(W + 60)
            r = random.uniform(22, 30)
            if bottom - top > 2 * r + 4 * BIRD_R + 40:
                y = random.uniform(top + r + 10, bottom - r - 10)
                self.storms.append({"x": W + 60, "y": y, "r": r, "phase": random.random() * 5})

        for obj in self.seeds + self.storms:
            obj["x"] -= dx
        self.seeds = [s for s in self.seeds if s["x"] > -20]
        self.storms = [s for s in self.storms if s["x"] > -80]

        for s in self.seeds[:]:
            if math.hypot(s["x"] - BIRD_X, s["y"] - self.bird_y) < BIRD_R + 10:
                self.seeds.remove(s)
                self.seed_score += 10
                for _ in range(12):
                    a = random.uniform(0, math.tau)
                    v = random.uniform(60, 180)
                    self.particles.append([s["x"], s["y"], math.cos(a) * v, math.sin(a) * v, 0.5])

        for p in self.particles:
            p[0] += (p[2] - speed) * dt
            p[1] += p[3] * dt
            p[4] -= dt
        self.particles = [p for p in self.particles if p[4] > 0]

        self.update_danger()

        top, bottom = self.canyon.walls_at(BIRD_X)
        hit_wall = self.bird_y - BIRD_R + 3 < top or self.bird_y + BIRD_R - 3 > bottom
        hit_storm = any(
            math.hypot(s["x"] - BIRD_X, s["y"] - self.bird_y) < s["r"] + BIRD_R - 6 for s in self.storms
        )
        if hit_wall or hit_storm:
            self.state = "over"
            if self.score > self.best:
                self.best = self.score
                save_best(self.best)

    def text(self, msg, pos, font=None, center=False):
        img = (font or self.font).render(msg, True, TEXT)
        rect = img.get_rect(center=pos) if center else img.get_rect(topleft=pos)
        self.screen.blit(img, rect)

    def draw_warnings(self, t):
        """Ghost marker at the sensor's target, plus a red border on the side you'd crash into."""
        danger = self.danger_top or self.danger_bottom
        ghost = WARN if danger else TEXT
        pygame.draw.circle(self.screen, ghost, (BIRD_X, int(self.target)), BIRD_R + 4, 2)

        pulse = 0.5 + 0.5 * math.sin(t * 12)
        overlay = pygame.Surface((W, WARN_BORDER), pygame.SRCALPHA)
        overlay.fill((*WARN, int(110 + 110 * pulse)))
        if self.danger_top:
            self.screen.blit(overlay, (0, 0))
            self.text("TOO HIGH", (W // 2, WARN_BORDER + 16), center=True)
        if self.danger_bottom:
            self.screen.blit(overlay, (0, H - WARN_BORDER))
            self.text("TOO LOW", (W // 2, H - WARN_BORDER - 16), center=True)

    def draw(self, t):
        self.screen.blit(self.sky, (0, 0))
        draw_hills(self.screen, self.hill_offset)
        if self.state != "title":
            self.canyon.draw(self.screen)
            for s in self.seeds:
                glow = 9 + math.sin(t * 8 + s["x"] * 0.05) * 2
                pygame.draw.circle(self.screen, (255, 200, 90), (int(s["x"]), int(s["y"])), int(glow + 4), 2)
                pygame.draw.circle(self.screen, SEED, (int(s["x"]), int(s["y"])), int(glow))
            for s in self.storms:
                draw_storm(self.screen, s, t)
            for p in self.particles:
                pygame.draw.circle(self.screen, SEED, (int(p[0]), int(p[1])), 3)

        if self.state in ("play", "countdown"):
            self.draw_warnings(t)

        tilt = max(-35, min(35, -self.bird_vy * 0.05))
        draw_bird(self.screen, self.bird_y, tilt, t, dead=self.state == "over")

        if self.state == "title":
            self.text("UPDRAFT", (W // 2, 120), self.big, center=True)
            self.text("Raise your hand over the sensor to climb, lower it to dive.", (W // 2, 200), center=True)
            self.text("Grab seeds, dodge storm clouds, don't touch the canyon walls.", (W // 2, 230), center=True)
            self.text("Calibrate: hold hand low + press L, hold hand high + press H", (W // 2, 290), center=True)
            self.text("SPACE start    M toggle mouse    ESC quit", (W // 2, 330), center=True)
        elif self.state == "countdown":
            self.text(str(math.ceil(self.countdown)), (W // 2, H // 2 - 40), self.big, center=True)
            self.text("Get ready - line the bird up with the gap", (W // 2, H // 2 + 20), center=True)
        elif self.state == "play":
            self.text(f"Score {self.score}", (16, 12))
            self.text(f"Best {self.best}", (16, 40))
        else:
            self.text("Crashed!", (W // 2, 180), self.big, center=True)
            self.text(f"Score {self.score}    Best {self.best}", (W // 2, 250), center=True)
            self.text("SPACE to fly again", (W // 2, 290), center=True)

        if self.use_mouse:
            status = "mouse control" + ("" if self.sensor else f" ({self.no_sensor_reason})")
        else:
            d = self.sensor.distance
            stale = time.monotonic() - self.sensor.last_update > 0.5
            reading = "--" if d is None else f"{d} cm"
            luna, sonar = self.sensor.luna, self.sensor.sonar
            luna = "off" if self.sensor.sensors == "sonar" else "--" if luna is None else luna
            sonar = "off" if self.sensor.sensors == "luna" else "--" if sonar is None else sonar
            reading += f" (luna {luna}, sonar {sonar})"
            status = f"{self.sensor.status}  |  {reading}{' (stale)' if stale else ''}  |  range {self.low}-{self.high} cm"
        self.text(status, (16, H - 32))

    def run(self):
        running = True
        start = time.monotonic()
        while running:
            dt = min(self.clock.tick(FPS) / 1000, 0.05)
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    running = self.handle_key(event.key)
            self.update(dt)
            self.draw(time.monotonic() - start)
            pygame.display.flip()
        if self.sensor:
            self.sensor.stop_event.set()
        pygame.quit()


def main():
    parser = argparse.ArgumentParser(description="Distance-sensor-controlled canyon flyer")
    parser.add_argument("--port", help="serial port, e.g. COM3 (auto-detected if omitted)")
    parser.add_argument("--sensors", choices=("both", "luna", "sonar"), default="both", help="which sensors to use")
    parser.add_argument("--low", type=int, help="distance in cm for the bottom of the screen")
    parser.add_argument("--high", type=int, default=60, help="distance in cm for the top of the screen")
    args = parser.parse_args()
    if args.low is None:
        # the TF-Luna can't see closer than LUNA_MIN_CM, so start the range there when it's alone
        args.low = LUNA_MIN_CM if args.sensors == "luna" else 5
    Game(args).run()


if __name__ == "__main__":
    main()
