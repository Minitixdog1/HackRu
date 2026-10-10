# HackRU Robot Dog

A four-legged robot dog whose legs detach and reattach. An ESP32 runs the motors and sensors and streams everything to a laptop over Wi-Fi. The laptop detects stuck legs with a small ML model, frees or drops them on its own, and takes plain-English commands through a local Qwen model.

```
you: "walk forward for 3 seconds"
  -> commander.py + Qwen (on a laptop, or a friend's computer)
  -> live loop: heartbeat + stuck-leg ML, 50 times a second
  -> Wi-Fi (UDP)
  -> ESP32: gait, motors, latch servos, sensors
```

| Folder | What's in it |
| --- | --- |
| [`esp32-firmware/`](esp32-firmware/) | PlatformIO project for the ESP32: Wi-Fi, telemetry, gait, unstick/detach, and a fake-sensor mode for testing with a bare board. Wiring and pins are in its README. |
| [`stuck-leg/`](stuck-leg/) | Laptop code: Wi-Fi link, stuck-leg ML (logging, training, live detection), and the Qwen commander. |

## Quick start

1. **Flash the ESP32.** Open `esp32-firmware/` in VS Code with PlatformIO, put your phone hotspot's name and password in `include/config.h`, and click Upload. It starts in fake-sensor mode, so no wiring is needed yet.
2. **Set up the laptop** (Python 3.10+), on the same hotspot as the ESP32:

   ```bash
   cd stuck-leg
   python -m venv .venv
   .venv\Scripts\activate             # macOS/Linux: source .venv/bin/activate
   pip install -r requirements.txt
   python logger.py --source fake --auto --trials 40 --duration 60 --fast   # fake training data
   python train.py data/                                                    # stuck-leg model
   python robot_link.py                                                     # finds the ESP32
   ```

3. **Run it** with either:

   ```bash
   python live.py                                     # keyboard control + live stuck-leg display
   python commander.py --llm-url http://FRIEND_IP:11434   # talk to it through Qwen
   ```

Details: [`esp32-firmware/README.md`](esp32-firmware/README.md) and [`stuck-leg/README.md`](stuck-leg/README.md).
