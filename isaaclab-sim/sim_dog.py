"""A robot dog in Isaac Lab that speaks the same Wi-Fi protocol as the ESP32.

commander.py, robot_link.py and live.py can't tell the difference: run this,
then point them at 127.0.0.1 and Qwen drives the simulated dog.

Run it from your Isaac Lab checkout (so `uv run` uses Isaac Lab's environment):

  uv run python PATH\\TO\\hackru\\isaaclab-sim\\sim_dog.py --task Isaac-Velocity-Flat-UnitreeGo2 --viz newton

It loads the newest checkpoint you trained for that task (logs/rsl_rl/...), or
pass --checkpoint pretrained to try Isaac Lab's published one. Then, in another
terminal:

  python commander.py --robot-ip 127.0.0.1

How commands map onto the simulated dog:
  gait on/off + freq   -> forward speed (slow 0.3, normal 0.6, fast 1.0 m/s)
  gait turn (-1..1)    -> turning rate (-1 = left, +1 = right, up to 1 rad/s)
  stop / heartbeat lost -> stand still
  detach / attach      -> that leg freezes in its default pose / rejoins
  unstick, zero        -> acknowledged (no stuck legs in the sim)
"""
from __future__ import annotations

import json
import math
import socket
import sys
import time

LEGS = ["FL", "FR", "RL", "RR"]
LEG_PREFIXES = {"FL": ("FL_", "LF_"), "FR": ("FR_", "RF_"), "RL": ("RL_", "LH_"), "RR": ("RR_", "RH_")}
FREQ_TO_SPEED = [(1.25, 0.3), (1.75, 0.6), (99.0, 1.0)]   # gait freq (Hz) -> forward speed (m/s)
MAX_YAW_RATE = 1.0                                          # rad/s at turn = +-1
HEARTBEAT_TIMEOUT_S = 0.3
PC_IDLE_S = 2.0


# ====================================================================== protocol (no Isaac Lab needed)
class UdpDogServer:
    """The ESP32's side of the protocol: discovery, heartbeat watchdog, commands, telemetry."""

    def __init__(self, port=4210, log=print):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", port))
        self.sock.setblocking(False)
        self.log = log
        self.pc = None
        self.last_pc = 0.0
        self.walking = False
        self.speed = 0.6
        self.turn = 0.0
        self.attached = [True] * 4
        log(f"[sim_dog] listening on UDP port {port} (same protocol as the ESP32)")

    # ---------------------------------------------------------------- commands
    def poll(self):
        """Handle every packet waiting on the socket, then apply the heartbeat watchdog."""
        while True:
            try:
                data, addr = self.sock.recvfrom(2048)
            except (BlockingIOError, ConnectionResetError):
                break
            try:
                msg = json.loads(data)
            except ValueError:
                continue
            if self.pc != addr:
                self.log(f"[sim_dog] PC at {addr[0]}:{addr[1]}")
            self.pc, self.last_pc = addr, time.monotonic()
            self.handle(msg)
        if self.walking and time.monotonic() - self.last_pc > HEARTBEAT_TIMEOUT_S:
            self.walking = False
            self.log("[sim_dog] no heartbeat from PC: standing still")

    def handle(self, msg):
        cmd = msg.get("cmd", "")
        leg = LEGS.index(msg["leg"]) if msg.get("leg") in LEGS else -1
        if cmd == "hello":
            self.send({"hello": "robotdog-sim", "sim": 1, "isaac": 1, "ap": 0})
        elif cmd == "gait":
            self.walking = bool(msg.get("on", 1))
            if "freq" in msg:
                self.speed = next(v for f, v in FREQ_TO_SPEED if float(msg["freq"]) < f)
            self.turn = max(-1.0, min(1.0, float(msg.get("turn", 0) or 0)))
            self.log(f"[sim_dog] {'walk' if self.walking else 'stand'} speed={self.speed} m/s turn={self.turn:+.2f}")
        elif cmd == "stop":
            self.walking = False
            self.log("[sim_dog] stop")
        elif cmd == "detach" and leg >= 0:
            self.attached[leg] = False
            self.log(f"[sim_dog] detach {LEGS[leg]} (leg frozen in default pose)")
        elif cmd == "attach" and leg >= 0:
            self.attached[leg] = True
            self.log(f"[sim_dog] attach {LEGS[leg]}")
        elif cmd in ("unstick", "resume", "zero", "sim_jam"):
            pass   # nothing gets stuck in the sim

    def velocity_command(self):
        """(forward m/s, sideways m/s, yaw rad/s) for the policy. Positive yaw = turn left."""
        if not self.walking:
            return 0.0, 0.0, 0.0
        return self.speed, 0.0, -self.turn * MAX_YAW_RATE

    # ---------------------------------------------------------------- telemetry
    def send(self, obj):
        if self.pc:
            self.sock.sendto(json.dumps(obj).encode(), self.pc)

    def send_telemetry(self, t_ms, ang_vel_b, gravity_b, leg_angles_deg, leg_targets_deg, phase):
        if not self.pc or time.monotonic() - self.last_pc > PC_IDLE_S:
            return
        g = 9.81
        self.send({
            "t": int(t_ms),
            # accelerometer reads -gravity in the body frame when standing still; gyro in deg/s
            "imu": [round(-g * gravity_b[0], 3), round(-g * gravity_b[1], 3), round(-g * gravity_b[2], 3),
                    round(math.degrees(ang_vel_b[0]), 2), round(math.degrees(ang_vel_b[1]), 2),
                    round(math.degrees(ang_vel_b[2]), 2)],
            "tof": [0, 0, 0, 0], "cur": [0, 0, 0, 0],
            "enc": [round(a, 2) for a in leg_angles_deg],
            "cmd": [round(a, 2) for a in leg_targets_deg],
            "ph": round(phase, 3), "gait": int(self.walking),
            "att": [int(a) for a in self.attached], "sim": 1, "jam": -1,
        })


# ====================================================================== Isaac Lab side
def _split_args(argv):
    port = 4210
    rest = []
    it = iter(argv)
    for a in it:
        if a == "--port":
            port = int(next(it))
        elif a.startswith("--port="):
            port = int(a.split("=", 1)[1])
        else:
            rest.append(a)
    if not any(a == "--num_envs" or a.startswith("--num_envs=") for a in rest):
        rest += ["--num_envs", "1"]            # one dog
    if not any(a == "--real-time" for a in rest):
        rest.append("--real-time")             # play at real speed so commands feel live
    return port, rest


def run(argv):
    port, argv = _split_args(argv)
    # Isaac Lab imports are deferred so the protocol class above works without it.
    import contextlib

    import torch
    from isaaclab.app import launch_simulation
    from isaaclab_rl.entrypoints.backends import play_rsl_rl as play
    from isaaclab_rl.entrypoints.common import apply_env_overrides, close_env, create_isaaclab_env
    from isaaclab_rl.rsl_rl import (
        RslRlVecEnvWrapper,
        check_rsl_rl_version,
        create_rsl_rl_runner,
        handle_deprecated_rsl_rl_cfg,
    )
    from isaaclab_tasks.utils import resolve_task_config

    args_cli = play._parse_args(argv)
    if not args_cli.task:
        raise SystemExit("pass --task, e.g. --task Isaac-Velocity-Flat-UnitreeGo2")
    installed_version = check_rsl_rl_version()
    env_cfg, agent_cfg = resolve_task_config(args_cli.task, args_cli.agent, play_mode=True)

    # Commands come from the PC, not from random resampling.
    vel = env_cfg.commands.base_velocity
    vel.heading_command = False
    vel.rel_heading_envs = 0.0
    vel.rel_standing_envs = 0.0
    vel.resampling_time_range = (1.0e9, 1.0e9)
    env_cfg.episode_length_s = 1.0e5           # no timeout resets mid-demo (falls still reset)

    server = UdpDogServer(port)
    with launch_simulation(env_cfg, args_cli), contextlib.ExitStack() as cleanup:
        import os
        agent_cfg = play.cli_args.update_rsl_rl_cfg(agent_cfg, args_cli)
        agent_cfg = handle_deprecated_rsl_rl_cfg(agent_cfg, installed_version)
        apply_env_overrides(args_cli, env_cfg)
        env_cfg.seed = agent_cfg.seed
        log_root_path = os.path.abspath(os.path.join("logs", "rsl_rl", agent_cfg.experiment_name))
        resume_path = play._resolve_checkpoint(args_cli, agent_cfg, env_cfg, log_root_path)
        if resume_path is None:
            raise SystemExit("no checkpoint found: train first (see isaaclab-sim/README.md) or pass --checkpoint")
        print(f"[sim_dog] policy: {resume_path}")

        env = create_isaaclab_env(args_cli.task, env_cfg, args_cli, convert_marl_to_single_agent=False)
        cleanup.callback(lambda: close_env(env))
        env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)
        runner = create_rsl_rl_runner(env, agent_cfg)
        runner.load(resume_path)
        policy = runner.get_inference_policy(device=env.unwrapped.device)

        base = env.unwrapped
        cmd_term = base.command_manager.get_term("base_velocity")
        robot = base.scene["robot"]
        names = list(robot.joint_names)
        leg_joints = [[j for j, n in enumerate(names) if n.startswith(LEG_PREFIXES[leg])] for leg in LEGS]
        first_joint = [idx[0] if idx else 0 for idx in leg_joints]
        print(f"[sim_dog] joints per leg: " + ", ".join(f"{LEGS[i]}={len(j)}" for i, j in enumerate(leg_joints)))
        dt = base.step_dt
        obs = env.get_observations()
        t0 = time.monotonic()
        phase = 0.0
        print("[sim_dog] running. Point commander.py at --robot-ip 127.0.0.1 (Ctrl+C to quit)")

        with contextlib.suppress(KeyboardInterrupt), torch.inference_mode():
            while True:
                start = time.time()
                server.poll()
                vx, vy, wz = server.velocity_command()
                cmd_term.vel_command_b[:, 0] = vx
                cmd_term.vel_command_b[:, 1] = vy
                cmd_term.vel_command_b[:, 2] = wz

                actions = policy(obs)
                for i in range(4):
                    if not server.attached[i] and leg_joints[i]:
                        actions[:, leg_joints[i]] = 0.0   # zero action = default joint pose
                obs, _, dones, _ = env.step(actions)
                policy.reset(dones)

                data = robot.data
                ang = data.root_ang_vel_b.torch[0].tolist()
                grav = data.projected_gravity_b.torch[0].tolist()
                q = data.joint_pos.torch[0]
                q_def = data.default_joint_pos.torch[0] if hasattr(data, "default_joint_pos") else q * 0
                angles = [math.degrees(float(q[j])) for j in first_joint]
                targets = [math.degrees(float(q_def[j])) for j in first_joint]
                if server.walking:
                    phase = (phase + 1.5 * dt) % 1.0
                server.send_telemetry((time.monotonic() - t0) * 1000, ang, grav, angles, targets, phase)

                sleep = dt - (time.time() - start)
                if args_cli.real_time and sleep > 0:
                    time.sleep(sleep)


if __name__ == "__main__":
    run(sys.argv[1:])
