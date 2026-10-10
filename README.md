# HackRU Robot Dog

A four-legged robot dog whose legs detach and reattach. An ESP32 runs the motors and sensors and streams everything to a laptop over Wi-Fi. The laptop detects stuck legs with a small ML model, frees or drops them on its own, and takes plain-English commands through a local Qwen model. A 2D dashboard shows it all live.

```
you: "walk forward for 3 seconds"
  -> dashboard.py (or commander.py) + Qwen on the laptop
  -> live loop: heartbeat + stuck-leg ML, 50 times a second
  -> Wi-Fi (UDP)
  -> ESP32: gait, motors, latch servos, sensors
```

| Folder | What's in it |
| --- | --- |
| [`esp32-firmware/`](esp32-firmware/) | PlatformIO project for the ESP32: Wi-Fi, telemetry, gait, unstick/detach, and a fake-sensor mode for testing with a bare board. Wiring and pins are in its README. |
| [`arduino-esp32c3/`](arduino-esp32c3/) | The same robot code for our **ESP32-C3**, as an **Arduino IDE** sketch: no encoders, open-loop N20 motors, and a mode where the real motors walk while simulated sensors feed the ML. |
| [`stuck-leg/`](stuck-leg/) | Laptop code: Wi-Fi link, stuck-leg ML (logging, training, live detection), the Qwen commander, and the 2D dashboard. |
| [`Onshapefile/`](Onshapefile/) | CAD: STEP exports of the hub, top platform, leg arms, leg spokes and full assembly, with previews and a part-by-part explanation. |

## Quick start

**1. Set up the laptop** (Python 3.10+):

```bash
cd stuck-leg
python -m venv .venv
.venv\Scripts\activate             # macOS/Linux: source .venv/bin/activate
pip install -r requirements.txt
python logger.py --source fake --auto --trials 60 --duration 60 --fast   # fake training data
python train.py data/                                                    # the stuck-leg model
```

**2. Try it with no hardware:**

```bash
python dashboard.py --demo
```

A window opens with the dog. Click **Walk**, then **Jam a leg: FR**, and watch the model catch it, try to free it, and detach it. With Qwen running in Ollama, type commands in the box instead.

**3. Use the ESP32.** Open `esp32-firmware/` in VS Code with PlatformIO, put your phone hotspot's name and password in `include/config.h`, and click Upload. It starts in fake-sensor mode, so no wiring is needed yet. With the laptop on the same hotspot:

```bash
python robot_link.py      # finds the ESP32
python dashboard.py       # same window, now driving the ESP32
```

**Using the ESP32-C3 with the Arduino IDE?** Open `arduino-esp32c3/RobotDogC3/RobotDogC3.ino` instead and follow [`arduino-esp32c3/README.md`](arduino-esp32c3/README.md). Train the model with `--no-enc`, since that robot has no encoders.

Details: [`esp32-firmware/README.md`](esp32-firmware/README.md) and [`stuck-leg/README.md`](stuck-leg/README.md).
