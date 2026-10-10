"""Shared constants and helpers for the stuck-leg pipeline.

One row of the CSV log = one telemetry sample from the ESP32.
Legs are ordered FL, FR, RL, RR everywhere (front-left, front-right, rear-left, rear-right).
"""

LEGS = ["FL", "FR", "RL", "RR"]

# Class labels. 0 = nothing stuck, 1..4 = that leg is stuck.
CLASS_NAMES = ["normal"] + [f"stuck_{leg}" for leg in LEGS]

IMU_COLS = ["ax", "ay", "az", "gx", "gy", "gz"]   # m/s^2 and deg/s
TOF_COLS = [f"tof_{leg}" for leg in LEGS]          # mm, downward ToF per corner
CUR_COLS = [f"cur_{leg}" for leg in LEGS]          # mA, motor current per leg
ENC_COLS = [f"enc_{leg}" for leg in LEGS]          # deg, measured joint angle
CMD_COLS = [f"cmd_{leg}" for leg in LEGS]          # deg, commanded joint angle

CSV_COLS = (["trial", "t_ms"] + IMU_COLS + TOF_COLS + CUR_COLS + ENC_COLS
            + CMD_COLS + ["phase", "label"])


def telemetry_to_row(msg, trial, label):
    """Flatten one telemetry message from the ESP32 into a CSV row (dict).

    Expected message (JSON):
      {"t": 10342, "imu": [ax, ay, az, gx, gy, gz], "tof": [4], "cur": [4],
       "enc": [4], "cmd": [4], "ph": 0.37}
    """
    row = {"trial": trial, "t_ms": msg["t"], "phase": msg.get("ph", 0.0), "label": label}
    row.update(zip(IMU_COLS, msg["imu"]))
    row.update(zip(TOF_COLS, msg["tof"]))
    row.update(zip(CUR_COLS, msg["cur"]))
    row.update(zip(ENC_COLS, msg["enc"]))
    row.update(zip(CMD_COLS, msg["cmd"]))
    return row
