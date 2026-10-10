"""A fake robot that produces realistic-looking telemetry, so the whole pipeline
can be built and tested before the real ESP32 is streaming.

It is NOT a physics simulation. It just shapes each signal the way a stuck leg
roughly would:
  - encoder: the stuck leg stops following its command
  - current: the stuck leg's motor strains toward stall current
  - IMU:     the body tilts and twists toward the stuck corner
  - ToF:     the stuck corner of the body sags
plus noise, terrain bumps, and per-trial variation (speed, battery, surface)
so the problem is not trivially easy.
"""
import math
import random

from common import LEGS

# Leg corner positions on the body: +x = front, +y = left.
CORNER = {"FL": (1, 1), "FR": (1, -1), "RL": (-1, 1), "RR": (-1, -1)}
# Trot gait: diagonal pairs (FL+RR, FR+RL) move together, half a cycle apart.
PHASE_OFFSET = {"FL": 0.0, "FR": 0.5, "RL": 0.5, "RR": 0.0}
G = 9.81


class FakeRobot:
    def __init__(self, seed=None):
        self.rng = random.Random(seed)
        r = self.rng
        # Per-trial variation: makes trials differ the way real sessions would.
        self.freq = r.uniform(1.0, 2.2)          # gait cycles per second
        self.amp = r.uniform(20, 35)             # joint swing amplitude, deg
        self.battery = r.uniform(0.8, 1.15)      # scales motor current
        self.rough = r.uniform(0.3, 1.5)         # surface roughness (noise level)
        self.t = 0.0
        self.phase = 0.0
        self.enc = {leg: 0.0 for leg in LEGS}
        self.stuck = {}                          # jammed legs: {"FL": strength}; 1 = hard jam, <1 = partial snag
        self.detached = set()                    # legs that have come off: motor off, angle frozen
        self.last_cmd = {leg: 0.0 for leg in LEGS}
        self.tilt = [0.0, 0.0]                   # roll, pitch (rad), low-pass filtered
        self.bump = 0.0
        self.terrain = [0.0, 0.0]                # slow ground slope under the robot (roll, pitch)
        self.turn = 0.0                          # deliberate turning rate, deg/s
        self.turn_target = 0.0

    def set_stuck(self, leg):
        """Exactly this one leg jammed (None = no leg). Used by logger.py's single-leg labels."""
        if leg is None:
            if self.stuck:                       # same random draws as before, so datasets reproduce
                self.rng.choice([1.0, 0.8, 0.5, 0.3])
            self.stuck = {}
        elif list(self.stuck) != [leg]:
            # Some snags are hard jams, some are partial (harder to detect).
            self.stuck = {leg: self.rng.choice([1.0, 0.8, 0.5, 0.3])}

    def add_stuck(self, leg, strength=1.0):
        """Jam one more leg, keeping any already jammed."""
        self.stuck[leg] = strength

    def free(self, leg=None):
        """Free one leg, or every leg if leg is None."""
        if leg is None:
            self.stuck = {}
        else:
            self.stuck.pop(leg, None)

    def detach(self, leg):
        self.detached.add(leg)
        self.stuck.pop(leg, None)

    def attach(self, leg):
        self.detached.discard(leg)

    @property
    def stuck_leg(self):
        return next(iter(self.stuck), None)

    def step(self, dt):
        r = self.rng
        self.t += dt
        self.phase = (self.phase + self.freq * dt) % 1.0

        # Occasional terrain bump: a short current + IMU spike on a random leg.
        # These are "hard negatives" that look a little like a stuck leg.
        bump_leg = None
        if r.random() < 0.02 * self.rough:
            self.bump = 1.0
            bump_leg = r.choice(LEGS)
        self.bump *= 0.85

        # Uneven ground: slope drifts slowly. Turning: the operator steers sometimes.
        for k in range(2):
            self.terrain[k] += r.gauss(0, 0.004 * self.rough)
            self.terrain[k] = max(-0.05, min(0.05, self.terrain[k]))
        if r.random() < 0.01:
            self.turn_target = r.choice([0, 0, 0, -20, 20, -40, 40])
        self.turn += (self.turn_target - self.turn) * min(1.0, 3 * dt)

        cmd, enc, cur, tof = [], [], [], []
        sag = {leg: 0.0 for leg in LEGS}
        for leg in LEGS:
            if leg in self.detached:             # no motor load, leg and command frozen where it came off
                cmd.append(round(self.last_cmd[leg], 2))
                enc.append(round(self.enc[leg], 2))
                cur.append(round(abs(r.gauss(0, 5)), 1))
                continue
            ph = 2 * math.pi * (self.phase + PHASE_OFFSET[leg])
            c = self.amp * math.sin(ph)
            self.last_cmd[leg] = c
            dc = self.amp * 2 * math.pi * self.freq * math.cos(ph)  # deg/s
            stuck = leg in self.stuck
            if stuck:
                strength = self.stuck[leg]
                # Encoder barely moves toward the command.
                follow = 1.0 - strength
                self.enc[leg] += (c - self.enc[leg]) * min(1.0, 8 * dt) * follow
                self.enc[leg] += r.gauss(0, 0.4)
                load = 350 + 250 * strength
                current = load * self.battery + r.gauss(0, 40)
                sag[leg] = -7 * strength
            else:
                # Normal tracking: first-order lag behind the command.
                self.enc[leg] += (c - self.enc[leg]) * min(1.0, 15 * dt)
                self.enc[leg] += r.gauss(0, 0.6 * self.rough)
                stance = 1.0 if math.sin(ph) < 0 else 0.0   # foot on ground half the cycle
                current = (120 + 1.2 * abs(dc) + 90 * stance) * self.battery
                current += r.gauss(0, 25 * self.rough)
            if leg == bump_leg or (self.bump > 0.3 and r.random() < 0.15):
                current += 220 * self.bump
            cmd.append(round(c, 2))
            enc.append(round(self.enc[leg], 2))
            cur.append(round(max(0.0, current), 1))

        # Body tilt: each stuck corner pulls that corner down and twists the body.
        target_roll = target_pitch = yaw_rate = 0.0
        for stuck_leg, s in self.stuck.items():
            sx, sy = CORNER[stuck_leg]
            target_pitch += -0.06 * sx * s    # nose down if a front leg is stuck
            target_roll += 0.06 * sy * s      # roll toward the stuck side
            yaw_rate += 15 * sy * s           # deg/s, other legs push the body around it
        self.tilt[0] += (target_roll - self.tilt[0]) * min(1.0, 4 * dt)
        self.tilt[1] += (target_pitch - self.tilt[1]) * min(1.0, 4 * dt)
        roll = self.tilt[0] + self.terrain[0]
        pitch = self.tilt[1] + self.terrain[1]

        bounce = 0.6 * math.sin(4 * math.pi * self.phase)   # two footfalls per cycle
        n = 0.25 * self.rough
        ax = -G * math.sin(pitch) + r.gauss(0, n) + 1.5 * self.bump * r.gauss(0, 1)
        ay = G * math.sin(roll) + r.gauss(0, n)
        az = G * math.cos(roll) * math.cos(pitch) + bounce + r.gauss(0, n)
        gx = r.gauss(0, 3 * self.rough) + 40 * (target_roll - self.tilt[0])
        gy = r.gauss(0, 3 * self.rough) + 40 * (target_pitch - self.tilt[1])
        gz = yaw_rate + self.turn + r.gauss(0, 4 * self.rough)

        for leg in LEGS:
            sx, sy = CORNER[leg]
            # Corner height from body tilt + gait bob + stuck sag + sensor noise.
            # pitch < 0 = nose down -> front corners lower; roll > 0 = left side lower.
            h = 80.0 + 150 * (sx * math.sin(pitch) - sy * math.sin(roll))
            h += 3 * math.sin(2 * math.pi * (self.phase + PHASE_OFFSET[leg]))
            h += sag[leg] + r.gauss(0, 2.5 * self.rough)
            if r.random() < 0.01:                   # ToF glitch: occasional bad reading
                h += r.choice([-1, 1]) * r.uniform(15, 40)
            tof.append(round(h, 1))

        return {
            "t": int(self.t * 1000),
            "imu": [round(v, 3) for v in (ax, ay, az, gx, gy, gz)],
            "tof": tof, "cur": cur, "enc": enc, "cmd": cmd,
            "ph": round(self.phase, 3),
        }
