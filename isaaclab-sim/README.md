# Isaac Lab Sim Dog, Driven by Qwen

Type a command, Qwen picks the action, and a robot dog walks in Isaac Lab. It uses the exact same `commander.py` as the real robot: `sim_dog.py` makes the simulated dog answer on the same Wi-Fi protocol as the ESP32, so the laptop code can't tell the difference.

```
you: "walk forward for 3 seconds, curve left"
  -> commander.py + Qwen (Ollama on your laptop)
  -> UDP 127.0.0.1:4210
  -> sim_dog.py: Isaac Lab, Unitree Go2 walking policy
```

The simulated dog is a **Unitree Go2** (Isaac Lab's built-in quadruped), not a model of your robot. Making a sim of your own robot needs its URDF; see the end of this page.

## What your laptop needs

Written for an HP Omen 15 with an RTX 5050 (8 GB) on Windows 11.

| Need | Why |
| --- | --- |
| **25 GB+ free disk** | Isaac Lab installs PyTorch with CUDA plus the Newton physics engine, about 10 GB or more. With 11 GB free it will likely run out. Clear space first. |
| **NVIDIA driver 581.42 or newer** | Isaac Lab 3.0 uses CUDA 13. Update through the NVIDIA app. |
| **Windows long paths enabled** | Isaac Lab's file paths are long (step 1). |
| **Qwen stopped while training** | Training and Qwen both want the 8 GB of GPU memory. Run `ollama stop qwen3.5:4b` before training; the demo itself (one dog) fits alongside Qwen. |

We use the **Newton** physics engine without Isaac Sim. It's the lighter option in Isaac Lab 3.0, and the Go2 walking task uses Newton by default. Full Isaac Sim recommends 16 GB of GPU memory, which this laptop doesn't have.

## 1. Install Isaac Lab 3.0

In an **administrator** PowerShell, enable long paths once:

```powershell
New-ItemProperty -Path "HKLM:\SYSTEM\CurrentControlSet\Control\FileSystem" -Name LongPathsEnabled -Value 1 -PropertyType DWORD -Force
```

Then in a normal PowerShell:

```powershell
powershell -ExecutionPolicy ByPass -c "irm https://astral.sh/uv/install.ps1 | iex"
cd $HOME
git clone -b release/3.0.0 --depth 1 https://github.com/isaac-sim/IsaacLab.git
cd IsaacLab
```

## 2. Get a walking policy

Try Isaac Lab's published Go2 policy first (it may not be published for every setup). This also checks the install, since the first run downloads everything:

```powershell
uv run isaaclab play --rl_library rsl_rl --task Isaac-Velocity-Flat-UnitreeGo2 --checkpoint pretrained --viz newton
```

If a window opens with a Go2 walking around, skip to step 3 and add `--checkpoint pretrained` there too. If it says no checkpoint is published, train one:

```powershell
ollama stop qwen3.5:4b
uv run isaaclab train --rl_library rsl_rl --task Isaac-Velocity-Flat-UnitreeGo2 --num_envs 2048
```

- It trains for 300 iterations and prints each one's time, so after a minute you'll know how long it takes.
- If it runs out of GPU memory, use `--num_envs 1024`.
- The result is saved under `IsaacLab\logs\rsl_rl\`. Watch it with `uv run isaaclab play --rl_library rsl_rl --task Isaac-Velocity-Flat-UnitreeGo2 --viz newton`.

## 3. Run the sim dog and talk to it

**Terminal 1** (in the `IsaacLab` folder) starts the simulated dog. Change the path to wherever you cloned `hackru`:

```powershell
uv run python $HOME\hackru\isaaclab-sim\sim_dog.py --task Isaac-Velocity-Flat-UnitreeGo2 --viz newton
```

**Terminal 2** (in the `hackru\stuck-leg` folder, with Ollama running Qwen) starts the commander:

```powershell
python commander.py --robot-ip 127.0.0.1
```

Then type: `walk forward for 3 seconds`, `turn right slowly`, `walk and curve left`, `stop`, `drop the front right leg`, `put it back`.

| Command to the robot | What the sim dog does |
| --- | --- |
| walk (slow / normal / fast) | Walks forward at 0.3 / 0.6 / 1.0 m/s |
| turn -1 … 1 | Turns left (negative) or right (positive), up to 1 rad/s |
| stop, or commander closed | Stands still (it also stops if the heartbeat is lost for 300 ms, like the ESP32) |
| detach / attach a leg | That leg freezes in its default pose / rejoins. The policy wasn't trained for this, so expect stumbling, which shows why a detachable-leg robot needs its own policy. |
| unstick | Acknowledged; nothing gets stuck in the sim |

`sim_dog.py` takes the normal Isaac Lab play options too (`--checkpoint`, `--viz`, `--num_envs`) plus `--port` (default 4210).

## Notes

- **Isaac Lab 3.0 is an Early Access release.** `sim_dog.py` reuses parts of Isaac Lab's own play script. If an update renames something, it will fail at startup with an import error. Send me the error.
- **Stuck-leg detection is off for the sim dog.** It needs motor current and encoder data the sim doesn't send. `commander.py` turns it off automatically when it connects to the sim.
- **ANYmal works too:** `--task Isaac-Velocity-Flat-AnymalD`. Use a task you have a checkpoint for.

## Simulating your own robot (next step)

To put *your* dog in Isaac Lab, or train a policy for it, export a URDF from the Onshape model with [`onshape-to-robot`](https://github.com/Rhoban/onshape-to-robot) (needs a free Onshape API key). Then copy the Go2 task config for your robot and train it the same way as step 2. Its inputs and outputs shrink to match your 4 motors.
