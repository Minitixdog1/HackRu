# ESP32 Firmware: Robot Dog over Wi-Fi

The ESP32 reads the sensors, runs the motors, and streams everything to the laptop over Wi-Fi. The laptop runs the ML (`../stuck-leg/live.py`) and sends commands back: walk, stop, unstick a leg, detach a leg.

```
ESP32  --(sensor data, 50/s, UDP)-->  laptop: live.py runs the model
ESP32  <--(heartbeat + commands)----  laptop
```

If the laptop goes quiet for 300 ms (script crashed, Wi-Fi dropped), the ESP32 turns the motors off.

## 1. Test with a bare ESP32 first (no wiring)

`config.h` ships with `SIM_SENSORS 1`: the ESP32 makes up realistic sensor data, and you can jam a leg from the laptop. This tests Wi-Fi and the ML end to end before any hardware is wired.

1. Install VS Code and the **PlatformIO** extension.
2. Open this `esp32-firmware` folder in VS Code.
3. In `include/config.h`, set `WIFI_SSID` and `WIFI_PASS` to a **phone hotspot**. Campus Wi-Fi usually blocks laptop-to-device traffic. Connect the laptop to the same hotspot.
4. Plug in the ESP32 and click **Upload** (the → arrow in the bottom bar), then **Serial Monitor** (the plug icon). You should see `Connected. Robot IP: ...`.
5. On the laptop, in the `stuck-leg` folder:

```bash
python robot_link.py              # should print "found robot" and ~50 messages/second

# once: make fake training data and a model (data/ and models/ aren't in git)
python logger.py --source fake --auto --trials 40 --duration 60 --fast
python train.py data/

python live.py                    # robot starts "walking"; press 2 to jam FR and watch it react
```

If the ESP32 can't join the hotspot within 15 s, it makes its own network called **robotdog** (password `robotdog123`). Join it from the laptop and run the same commands; the robot is at `192.168.4.1`.

## 2. Real hardware

Set `SIM_SENSORS 0` in `config.h`, and set the `HAS_...` flags to match what's actually connected (a missing part with its flag on just reads as zeros).

### Parts this assumes

| Part | Job | I2C address |
| --- | --- | --- |
| ESP32 DevKit (WROOM-32) | Brain, Wi-Fi | |
| MPU6050 | IMU (accelerometer + gyro) | 0x68 |
| 4× VL53L0X | Downward distance at each corner | 0x29 at power-up, set to 0x30–0x33 at boot |
| 4× INA219 | Current of each motor | 0x40, 0x41, 0x44, 0x45 |
| PCA9685 | 4 latch servos + the 4 ToF XSHUT lines | 0x60 (close the A5 jumper) |
| 4× N20 with hall encoders | Leg joints | |
| 2× DRV8833 (or TB6612) | Motor drivers, IN1/IN2 mode | |
| 4× hobby servos | Leg latches | |

### Wiring

| Signal | FL | FR | RL | RR |
| --- | --- | --- | --- | --- |
| Motor IN1 | 16 | 18 | 23 | 26 |
| Motor IN2 | 17 | 19 | 25 | 27 |
| Encoder A | 34 | 36 | 32 | 4 |
| Encoder B | 35 | 39 | 33 | 13 |
| Latch servo (PCA9685 channel) | 0 | 1 | 2 | 3 |
| ToF XSHUT (PCA9685 channel) | 4 | 5 | 6 | 7 |

- **I2C bus:** SDA = GPIO 21, SCL = GPIO 22. The IMU, all INA219s, all VL53L0Xs, and the PCA9685 share it.
- **Encoders:** power them from 3.3 V. GPIO 34–39 have no internal pull-ups, so if the encoder outputs are open-collector, add 10 kΩ from each signal to 3.3 V.
- **Current sensors:** put each INA219 in series with one motor wire. The firmware reports the absolute value, so direction doesn't matter.
- **Power:** motors and servos from the battery (through a regulator for the servos, typically 5–6 V), never from the ESP32's 3.3 V pin. **All grounds connected together.**
- **No PCA9685?** Set `HAS_PCA9685 0`. Servos then go on GPIO 5, 14, 2, 15 (`SERVO_PIN`). There aren't enough pins left for four XSHUT lines, so either use a TCA9548A I2C multiplexer or a single ToF (it gets copied into all four slots).

### First power-up checklist

1. Set the legs to their neutral position before powering on: that position becomes 0°. (Or send `{"cmd":"zero"}` later.)
2. Open the serial monitor: every sensor prints `ok` or `NOT FOUND`.
3. Prop the robot up so the legs swing freely, then run `python live.py --watch-only`.
4. If a leg drives the wrong way or runs away, flip its sign in `MOTOR_DIR` or `ENC_DIR`.
5. If the legs oscillate or buzz, lower `KP`; if they lag far behind the command, raise it.
6. Set `ENC_COUNTS_PER_REV` from your motor's spec: magnet pulses per motor turn × 4 × gear ratio.

### Train on real data

The model from step 1 is trained on fake data. Once real sensors work:

```bash
python logger.py --source udp       # press g to walk, 1-4 when you jam a leg, 0 when free, q to save
python train.py data/               # after deleting the fake trials from data/
python live.py
```

## Protocol

All messages are JSON over UDP port 4210.

| Direction | Message | Meaning |
| --- | --- | --- |
| PC → robot | `{"cmd":"hello"}` | Discovery; the robot replies `{"hello":"robotdog","sim":1,"ap":0}` |
| PC → robot | `{"cmd":"hb"}` | Heartbeat every 100 ms; silence for 300 ms stops the motors |
| PC → robot | `{"cmd":"gait","on":1,"freq":1.5,"amp":25,"turn":0}` | Start/stop walking; `freq`, `amp`, `turn` optional (`turn` -1 = left … 1 = right) |
| PC → robot | `{"cmd":"stop"}` | Motors off |
| PC → robot | `{"cmd":"unstick","leg":"FR"}` | Wiggle that leg for 1.5 s, then resume walking |
| PC → robot | `{"cmd":"detach","leg":"FR"}` | Motor off and latch servo releases |
| PC → robot | `{"cmd":"attach","leg":"FR"}` | Latch locks, leg rejoins the gait |
| PC → robot | `{"cmd":"zero"}` | Current leg angles become 0° |
| PC → robot | `{"cmd":"sim_jam","leg":"FR"}` | Sim mode only: jam a leg (`null` frees it) |
| robot → PC | `{"t":..,"imu":[6],"tof":[4],"cur":[4],"enc":[4],"cmd":[4],"ph":..,"gait":..,"att":[4],"sim":..,"jam":..}` | Sensor data, 50 times a second |

## Troubleshooting

| Symptom | Fix |
| --- | --- |
| `robot_link.py` says no answer | Laptop and ESP32 on the same network? Allow Python through the Windows firewall when asked. Try `--robot-ip` with the IP from the serial monitor. |
| Works on hotspot, not campus Wi-Fi | Campus networks block device-to-device traffic. Use the hotspot or the robot's own `robotdog` network. |
| Motors stop after a moment | Heartbeats aren't arriving. Something on the laptop is blocking `live.py`'s loop, or Wi-Fi is dropping packets. |
| Rate shows far below 50 Hz | Move closer, or switch to the robot's own network. |
