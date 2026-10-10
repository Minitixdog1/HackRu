"""Shared constants and helpers for the stuck-leg pipeline.

One row of the CSV log = one telemetry sample from the ESP32.
Legs are ordered FL, FR, RL, RR everywhere (front-left, front-right, rear-left, rear-right).
"""

LEGS = ["FL", "FR", "RL", "RR"]

# How fast the dog turns at full turn (gait "turn" = +-1), in degrees per second.
# "Turn left 90 degrees" holds a full turn for 90 / TURN_RATE_DEG_S seconds, then
# goes straight again. Measure it on the real robot (time a 360) and update this.
TURN_RATE_DEG_S = 45.0

# Which legs may be off at the same time. One leg can always come off; a second
# only if it is diagonal to the first (FL+RR or FR+RL), so the robot keeps one
# front and one back leg on opposite sides and can still balance.
DIAGONAL_PAIRS = [("FL", "RR"), ("FR", "RL")]


def diagonal_partner(i):
    """Index of the leg diagonal to leg i (FL<->RR, FR<->RL)."""
    return 3 - i


def detach_allowed(attached, i):
    """True if leg i may come off, given which legs are attached (list of 4 bools)."""
    off = {j for j in range(4) if not attached[j]} | {i}
    return off <= {i, diagonal_partner(i)}


def detach_refusal(attached, i):
    """Plain-English reason leg i can't come off, or None if it can."""
    if detach_allowed(attached, i):
        return None
    off = [LEGS[j] for j in range(4) if not attached[j]]
    return (f"can't drop {LEGS[i]}: {', '.join(off)} already off, and only diagonal pairs "
            f"(FL+RR or FR+RL) can be off together")


# Class labels. 0 = nothing stuck, 1..4 = that leg is stuck.
CLASS_NAMES = ["normal"] + [f"stuck_{leg}" for leg in LEGS]

IMU_COLS = ["ax", "ay", "az", "gx", "gy", "gz"]   # m/s^2 and deg/s
TOF_COLS = [f"tof_{leg}" for leg in LEGS]          # mm, downward ToF per corner
CUR_COLS = [f"cur_{leg}" for leg in LEGS]          # mA, motor current per leg
ENC_COLS = [f"enc_{leg}" for leg in LEGS]          # deg, measured joint angle
CMD_COLS = [f"cmd_{leg}" for leg in LEGS]          # deg, commanded joint angle

JAM_COLS = [f"jam_{leg}" for leg in LEGS]          # 1 = this leg is stuck (the labels the model learns)
ATT_COLS = [f"att_{leg}" for leg in LEGS]          # 1 = this leg is attached (from the robot)

CSV_COLS = (["trial", "t_ms"] + IMU_COLS + TOF_COLS + CUR_COLS + ENC_COLS
            + CMD_COLS + ["phase", "label"] + JAM_COLS + ATT_COLS)


def telemetry_to_row(msg, trial, jams):
    """Flatten one telemetry message from the ESP32 into a CSV row (dict).

    jams: which legs are stuck right now, as a set of leg names ({"FL", "RR"}), a list
    of 4 flags, or (older code) a single label 0-4.

    Expected message (JSON):
      {"t": 10342, "imu": [ax, ay, az, gx, gy, gz], "tof": [4], "cur": [4],
       "enc": [4], "cmd": [4], "ph": 0.37, "att": [4]}
    """
    flags = jam_flags(jams)
    label = flags.index(1) + 1 if any(flags) else 0     # first stuck leg, kept for older scripts
    row = {"trial": trial, "t_ms": msg["t"], "phase": msg.get("ph", 0.0), "label": label}
    row.update(zip(JAM_COLS, flags))
    row.update(zip(ATT_COLS, [int(a) for a in msg.get("att", [1, 1, 1, 1])]))
    row.update(zip(IMU_COLS, msg["imu"]))
    row.update(zip(TOF_COLS, msg["tof"]))
    row.update(zip(CUR_COLS, msg["cur"]))
    row.update(zip(ENC_COLS, msg["enc"]))
    row.update(zip(CMD_COLS, msg["cmd"]))
    return row


def jam_flags(jams):
    """Any way of saying which legs are stuck -> [0/1, 0/1, 0/1, 0/1] in LEGS order."""
    if jams is None:
        return [0, 0, 0, 0]
    if isinstance(jams, (int,)) and not isinstance(jams, bool):
        return [int(jams == i + 1) for i in range(4)]
    jams = list(jams)
    if len(jams) == 4 and all(isinstance(j, (int, bool)) for j in jams) and not any(isinstance(j, str) for j in jams):
        return [int(bool(j)) for j in jams]
    return [int(leg in jams) for leg in LEGS]
