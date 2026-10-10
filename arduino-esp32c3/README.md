# Robot Dog: ESP32-C3 sketch (Arduino IDE)

This is the robot's code for an **ESP32-C3**, written for the **Arduino IDE**. It does the same job as
[`esp32-firmware/`](../esp32-firmware/), which targets a regular ESP32 with PlatformIO, but it is built
for our real robot: an ESP32-C3, N20 motors **without encoders**, and an IMU.

The board does not run the ML. It streams sensor numbers to the laptop over Wi-Fi and drives the motors.
Everything in [`stuck-leg/`](../stuck-leg/) (logger, training, live detection, dashboard, Qwen commander)
talks to it unchanged.

```
laptop (stuck-leg/)                                     ESP32-C3 (this sketch)
  dashboard.py / live.py / commander.py                   gait + open-loop N20 motors
  stuck-leg ML model, 50x a second      <-- Wi-Fi UDP -->  sensors: simulated or real
  sends: walk, turn, unstick, detach, heartbeat           sends: imu, current, ToF, angles, 50x a second
```

## "Simulation runs while it walks"

There are two switches in `config.h`:

| `SIM_SENSORS` | `MOTORS_ENABLED` | What happens |
|---|---|---|
| 1 | 1 | **Default.** The real motors walk, while the numbers sent to the laptop come from the built-in simulator. You can jam a leg from the dashboard and watch the ML catch it, unstick it and drop it, all while the dog actually moves. |
| 1 | 0 | Bare board on a desk: no motors move, but everything else works. Good for testing Wi-Fi and the ML. |
| 0 | 1 | Real robot: real motors, plus the real sensors marked `HAS_...` in `config.h`. |

The simulator is the same model as `stuck-leg/fake_robot.py`, translated to C++, so a model trained on
fake data recognises the simulated jams.

## No encoders: what that changes

The N20s run **open loop**: the board sets a speed and nothing measures how far the leg actually turned.

- Each telemetry message sends `"enc"` equal to the command, plus `"enc_ok":0` so the laptop knows
  these are not real readings.
- The simulator does the same by default (`SIM_HAS_ENCODERS 0`), so training data from it looks like
  the real robot.
- The model therefore has to spot a stuck leg from **motor current** (a stalled N20 draws a lot more),
  the **IMU** (the body tilts and turns toward the stuck corner) and **ToF**.
- On fake data that works well. The no-encoder model scored 95% exact match on held-out trials,
  compared with 84% for a hand-written current rule.

If you start live detection with a model trained *with* encoders, the dashboard warns you to retrain.

## Set up the Arduino IDE (once)

1. Install **Arduino IDE 2**.
2. **Add the ESP32 boards.** Open **Tools → Board → Boards Manager**, search **esp32** and install
   **"esp32 by Espressif Systems"**.
   - The 3.x versions are what we tested (3.0.7). The 2.x versions work too.
   - If it isn't listed, add this URL under **File → Preferences → Additional boards manager URLs**:
     `https://espressif.github.io/arduino-esp32/package_esp32_index.json`
3. **Install the libraries.** Open **Sketch → Include Library → Manage Libraries**.
   - **ArduinoJson** by Benoit Blanchon, version 7. This is the only one you need in simulation mode.
   - Only needed for real sensors (`SIM_SENSORS 0`):
     - **Adafruit MPU6050**. Say yes when it asks to install its dependencies.
     - **Adafruit INA219**
     - **VL53L0X** by Pololu

## Upload

1. Open `arduino-esp32c3/RobotDogC3/RobotDogC3.ino`. `config.h` shows up as a second tab.
2. In `config.h`, set `WIFI_SSID` and `WIFI_PASS` to your phone hotspot.
3. In the **Tools** menu:
   - **Board:** *ESP32C3 Dev Module*. This works for almost every C3 board. If your board is listed by
     name (for example *XIAO_ESP32C3*), you can pick that instead.
   - **USB CDC On Boot: Enabled.** Boards with no separate USB chip (Super Mini, XIAO) need this, or the
     Serial Monitor stays blank.
   - **Port:** the board's port.
4. Click **Upload**. If it can't connect, hold **BOOT**, tap **RESET**, release **BOOT**, then upload again.
5. Open the **Serial Monitor** at 115200 baud. It prints the robot's IP address, or that it made its
   own `robotdog` network.

## Run it with the ML

Do this on the laptop, from the `stuck-leg` folder, connected to the same hotspot:

```bash
# 1. A model that, like this robot, has no encoders (about a minute)
python logger.py --source fake --auto --trials 60 --duration 60 --fast --no-enc --out data/noenc
python train.py data/noenc --out models/stuck_rf.joblib

# 2. Find the board, then drive it
python robot_link.py
python dashboard.py
```

In the dashboard:
1. Click **Walk**. The real motors spin.
2. Click **Jam a leg: FR**. The simulator jams it.
3. The model spots the jam and sends **unstick**, and the real FR motor rocks back and forth for 1.5 s.
4. If the leg stays stuck, the model **detaches** it, and that motor stops.

Qwen commands ("walk forward", "turn left 90 degrees") work the same way.

You can try the same laptop flow with no board at all: `python dashboard.py --demo --no-enc`.

**When the real sensors are wired:** set `SIM_SENSORS 0` and upload. Then record real data with
`python logger.py --source udp`, pressing 1–4 while you hold a leg, and retrain. A model trained on fake
data is a starting point, not a replacement for real logs.

## Wiring (defaults in `config.h`)

The C3 has only **6 PWM channels**, so each motor uses one PWM pin and one plain on/off direction pin.
That fits four motors on a DRV8833 driver.

| Leg | DRV8833 IN1 (PWM) | DRV8833 IN2 (direction) |
|-----|------|------|
| FL  | GPIO 4 | GPIO 5 |
| FR  | GPIO 6 | GPIO 7 |
| RL  | GPIO 0 | GPIO 1 |
| RR  | GPIO 3 | GPIO 10 |

| Other | Pins |
|---|---|
| I2C (IMU, INA219s, ToF) | SDA = GPIO 8, SCL = GPIO 9 |
| Power | Connect the C3 GND to the driver and battery GND. Motors run from the battery through the DRV8833, never from the C3's 3.3 V pin. |

**How one PWM pin drives a motor both ways.** A DRV8833 input pair does:
- 0/0 = coast
- 1/0 = forward
- 0/1 = reverse
- 1/1 = brake

The sketch uses those states like this:
- **Forward:** direction pin LOW and PWM at duty *d*, so the motor alternates between forward and coast.
- **Reverse:** direction pin HIGH and PWM at *(full − d)*, so the motor alternates between reverse and
  brake. That still gives *d* worth of reverse drive.

**Pins to avoid on the C3:**
- **GPIO 2:** a boot pin.
- **GPIO 11–17:** connected to the flash chip.
- **GPIO 18/19:** USB on Super Mini and XIAO boards.
- **GPIO 20/21:** the USB chip on DevKitM-1 boards.

GPIO 8/9 are boot pins too, but I2C lines rest HIGH, which is what boot needs, so they are fine for I2C.

## How the legs move (`LEG_STYLE`)

- **`LEG_STYLE_WHEG`** (default): each leg spins continuously, like the rotating spoke
  (part 05 in `Onshapefile`).
  - Walking speed scales with the gait speed (`WHEG_DUTY`).
  - Turning speeds up one side and slows the other.
  - Unstick rocks the stuck motor back and forth at full power while the other legs pause.
- **`LEG_STYLE_SWING`**: each leg swings back and forth following the trot pattern. Without encoders the
  swing slowly drifts, which is why wheg is the default.

The laptop still sees the trot pattern as leg angles (`"cmd"`) in both styles. The dashboard animation
and the simulator use it.

## What has been tested

- Compiled with **esp32 core 3.0.7, board ESP32C3 Dev Module**, in three setups: the default, real
  sensors, and swing style with simulated encoders.
- The sketch logic was run on a laptop (Wi-Fi and pins swapped for stand-ins), with the real
  `stuck-leg` live loop and a no-encoder model connected to it. It checked out:
  - walking: all motors forward
  - turning right: left legs faster
  - a simulated FR jam was detected (probability 0.89), the FR motor alone rocked in reverse and forward,
    the leg came free, and walking resumed
  - detaching FL and then RL was refused, because they aren't a diagonal pair
  - all motors stopped within 0.3 s of the laptop going away
- **Not yet run on a physical C3.** Check the leg directions (`MOTOR_FLIP`) and `WHEG_DUTY` on the
  first power-up, with the robot held off the ground.

## Troubleshooting

| Problem | Fix |
|---|---|
| Error "This sketch is for the ESP32-C3" | Tools → Board → pick an ESP32-C3 board |
| Serial Monitor is blank | Tools → USB CDC On Boot → Enabled, upload again, then press RESET |
| Never joins the hotspot (dots forever, then `robotdog` network) | Hotspot must be 2.4 GHz (turn on "maximize compatibility" on iPhone). Super Mini boards: set `WIFI_LOW_TX_POWER 1` |
| A leg spins backwards | Set that leg's entry in `MOTOR_FLIP` to -1 |
| Motors hum but don't turn | Raise `WHEG_DUTY` (or `MIN_DUTY`); check the battery |
| Motors stop after a moment | Normal safety: they stop 0.3 s after the laptop's last heartbeat. Keep `dashboard.py` or `live.py` running |
| Dashboard warns "trained with encoders" | Retrain with `--no-enc` (see "Run it with the ML") |

## Using a regular Arduino (Uno / Nano)?

An Uno or Nano has no Wi-Fi, so this sketch can't run on it. It would need a version that talks to the
laptop over the USB cable instead. That's doable (`logger.py` already has a `--source serial` mode), so
ask if that's your setup. If "Arduino" just means the **Arduino IDE** on the ESP32-C3, this sketch is
the right one.
