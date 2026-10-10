# Robot Dog: Stuck-Leg Detection

Detects when a leg of the robot dog is stuck (and which one) from IMU, ToF, motor current and encoder data, and decides when to try freeing it or detach it.

| File | What it does |
| --- | --- |
| `common.py` | Leg order, column names, telemetry-to-CSV conversion |
| `fake_robot.py` | Fake telemetry with stuck legs, for testing before the ESP32 streams |
| `logger.py` | Logs telemetry to CSV and labels it with keypresses (1-4 stuck leg, 0 free, q quit) |
| `features.py` | Cuts logs into 0.5 s windows and computes features |
| `metrics.py` | False alarm, missed, and wrong-leg rates |
| `baseline.py` | Rule-based detector (tracking error + motor current) |
| `train.py` | Random forest, split by trial, compared against the baseline |
| `decide.py` | Live detector + release state machine (unstick, resume, detach) |

## Quick start

```bash
python -m venv .venv
source .venv/bin/activate          # Windows: .venv\Scripts\activate
pip install -r requirements.txt

python logger.py --source fake --auto --trials 40 --duration 60 --fast
python baseline.py data/
python train.py data/
python decide.py models/stuck_rf.joblib data/fake_011.csv
```

Real robot: `python logger.py --source serial --port COM5` or `python logger.py --source udp --port 4210`.

## Telemetry message (ESP32 → PC, ~50 Hz)

```json
{"t": 10342, "imu": [ax, ay, az, gx, gy, gz], "tof": [FL, FR, RL, RR], "cur": [FL, FR, RL, RR], "enc": [FL, FR, RL, RR], "cmd": [FL, FR, RL, RR], "ph": 0.37}
```

The PC sends `{"cmd":"hb"}` every 100 ms; the ESP32 should stop all motors after ~300 ms without one.
