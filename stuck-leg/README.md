# Robot Dog: Laptop Code

Everything that runs on the laptop: the Wi-Fi link to the ESP32, stuck-leg detection (ML), the Qwen commander that turns plain-English commands into robot actions, and a live 2D dashboard.

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
| `robot_link.py` | Wi-Fi link to the ESP32: discovery, heartbeat, telemetry, commands |
| `runner.py` | The live loop: heartbeat, telemetry, stuck-leg model, automatic unstick/detach |
| `live.py` | Runs the live loop with a keyboard UI and a live stuck-probability display |
| `commander.py` | Plain-English control through a local Qwen model (Ollama or any OpenAI-compatible server) |
| `dashboard.py` | Live 2D window: the dog from above, leg states, event log, Qwen chat box, and buttons |
| `fake_esp32.py` | A pretend ESP32 on the laptop (same protocol) for testing with no hardware |

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

## Live on the ESP32 over Wi-Fi

Flash `../esp32-firmware` first (see its README; it can fake the sensors so a bare ESP32 works). Then:

```bash
python robot_link.py        # finds the robot, prints messages/second
python live.py              # walks, runs the model, unsticks or detaches legs on its own
python live.py --watch-only # predictions only, never sends unstick/detach
python logger.py --source udp   # collect real labeled data (g = walk, 1-4 = stuck leg, 0 = free)
```

Add `--robot-ip 192.168.4.1` (or the IP on the serial monitor) if discovery doesn't find it.

## The dashboard

```bash
python dashboard.py --demo                     # no hardware: runs a pretend ESP32 on this laptop
python dashboard.py                            # the real ESP32 on the same Wi-Fi
python dashboard.py --llm-url http://FRIEND_IP:11434   # Qwen on another computer
python dashboard.py --no-llm                   # buttons only
```

- **Left:** the dog seen from above, with its path. Position is estimated from the commands, since the robot has no position sensor.
- **Leg colors:** green = fine, yellow = maybe stuck, red = stuck, orange = being freed, gray = detached. The legs swing using the real encoder angles, so a stuck leg visibly freezes.
- **Turning:** Left/Right (and Qwen's turn action) rotate by an angle, then carry on straight. How fast the robot turns is set by `TURN_RATE_DEG_S` in `common.py`. Time a full 360° on the real robot and update it so 90° really means 90°.
- **Right:** each leg's stuck probability against the release threshold (dotted line), a log of everything that happened, Walk/Turn/Stop/Reattach buttons, and a box for typing commands to Qwen.
- **Jam a leg:** these buttons appear when the robot has simulated sensors (the ESP32's fake-sensor mode, or `--demo`). Use them to show the stuck-leg model working.

Only one program can talk to the robot at a time, so run the dashboard *instead of* `live.py` or `commander.py`, not alongside them.

## Talk to it with Qwen

`commander.py` sends what you type to a Qwen model, which picks actions (walk, turn, stop, unstick, detach, attach, status). The live loop keeps running underneath, so stuck-leg detection still works while you talk to it.

```bash
python commander.py                                       # Ollama on this laptop, model qwen3.5:4b
python commander.py --llm-url http://FRIEND_IP:11434      # Ollama on another computer
python commander.py --llm-url http://FRIEND_IP:1234/v1 --llm-model MODEL_NAME   # LM Studio / llama.cpp / vLLM
python commander.py --no-robot                            # test the LLM alone; actions are only printed
```

If Qwen runs on another computer, that computer has to accept outside connections:
- **Ollama:** set the environment variable `OLLAMA_HOST=0.0.0.0`, restart Ollama, and allow port 11434 through its firewall.
- **LM Studio:** turn on "Serve on Local Network" in the server settings (port 1234).
- **llama.cpp:** start `llama-server` with `--host 0.0.0.0 --jinja` (`--jinja` is needed for tool calling).

Thinking mode is switched off automatically, because Qwen3.5 answers much faster without it.

## Telemetry message (ESP32 → PC, ~50 Hz)

```json
{"t": 10342, "imu": [ax, ay, az, gx, gy, gz], "tof": [FL, FR, RL, RR], "cur": [FL, FR, RL, RR], "enc": [FL, FR, RL, RR], "cmd": [FL, FR, RL, RR], "ph": 0.37}
```

The firmware also adds `gait`, `att`, `sim`, and `jam`. The PC sends `{"cmd":"hb"}` every 100 ms; the ESP32 stops all motors after 300 ms without one. The full command list is in `../esp32-firmware/README.md`.
