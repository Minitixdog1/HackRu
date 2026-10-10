"""Live 2D dashboard for the robot dog: see it walk, see legs get stuck, talk to it through Qwen.

  python dashboard.py --demo                         # no hardware: a pretend ESP32 on this laptop
  python dashboard.py                                # real ESP32 on the same Wi-Fi (found automatically)
  python dashboard.py --robot-ip 192.168.4.1         # skip discovery
  python dashboard.py --llm-url http://FRIEND_IP:11434   # Qwen on another computer
  python dashboard.py --no-llm                       # buttons only

Left: the dog seen from above. It moves as you command it (position is estimated
from the commands). Each leg is colored by what the stuck-leg model thinks:
green = fine, yellow = maybe stuck, red = stuck, orange = being freed, gray = detached.
Right: leg status, everything that happened, and a box to type commands to Qwen.
"""
import argparse
import math
import os
import queue
import threading
import time
import tkinter as tk

from common import LEGS

# ------------------------------------------------------------------ look
BG = "#0f1419"
PANEL = "#171d24"
GRID = "#1f2832"
TEXT = "#e6e9ec"
MUTED = "#8b95a1"
BODY = "#3a4654"
COLORS = {"ok": "#3fb950", "standing": "#2f6f3e", "maybe": "#d29922", "stuck": "#f85149",
          "freeing": "#ff8c3a", "detached": "#4b5563"}
LABELS = {"ok": "OK", "standing": "standing", "maybe": "maybe stuck", "stuck": "STUCK",
          "freeing": "freeing...", "detached": "detached"}
LOG_COLORS = {"you": "#58a6ff", "qwen": "#d2a8ff", "action": MUTED, "auto": "#ff8c3a", "info": TEXT, "warn": "#f85149"}
FONT = ("Segoe UI", 10)
FONT_B = ("Segoe UI", 11, "bold")
MONO = ("Consolas", 9)

PX_PER_M = 220          # world scale for motion and the grid
BODY_L, BODY_W = 130, 74
LEG_R = 15
SWING_PX_PER_DEG = 0.8
CORNERS = {"FL": (1, 1), "FR": (1, -1), "RL": (-1, 1), "RR": (-1, -1)}   # (+front, +left)


def speed_from_freq(freq):
    return 0.17 * freq                         # m/s, rough figure for a small robot dog


class Dashboard:
    def __init__(self, root, runner, commander=None, llm_note=""):
        self.root, self.runner, self.commander = root, runner, commander
        self.events = queue.Queue()
        self.busy = False
        self.x = self.y = 0.0
        self.heading = math.pi / 2             # facing up the screen
        self.trail = [(0.0, 0.0)]
        self.last_t = time.monotonic()
        self.sim = runner.sim
        runner.on_event = lambda text: self.events.put(("auto", text))

        root.title("Robot Dog")
        root.configure(bg=BG)
        root.geometry("1180x720")
        root.minsize(980, 620)

        # left: the world
        self.canvas = tk.Canvas(root, bg=BG, highlightthickness=0)
        self.canvas.pack(side="left", fill="both", expand=True)

        # right: panel
        panel = tk.Frame(root, bg=PANEL, width=420)
        panel.pack(side="right", fill="y")
        panel.pack_propagate(False)

        tk.Label(panel, text="Robot Dog", font=("Segoe UI", 16, "bold"), fg=TEXT, bg=PANEL).pack(anchor="w", padx=16, pady=(14, 0))
        self.conn = tk.Label(panel, text="", font=FONT, fg=MUTED, bg=PANEL, justify="left")
        self.conn.pack(anchor="w", padx=16)
        self.state = tk.Label(panel, text="", font=FONT_B, fg=TEXT, bg=PANEL, justify="left")
        self.state.pack(anchor="w", padx=16, pady=(8, 4))

        self.legs = tk.Canvas(panel, bg=PANEL, height=4 * 30 + 6, highlightthickness=0)
        self.legs.pack(fill="x", padx=16)

        buttons = tk.Frame(panel, bg=PANEL)
        buttons.pack(fill="x", padx=16, pady=(8, 0))
        for text, cmd in [("Walk", self.walk), ("◀ Left", lambda: self.walk(-0.6)), ("Right ▶", lambda: self.walk(0.6)),
                          ("Stop", self.stop), ("Reattach all", self.reattach)]:
            self._button(buttons, text, cmd).pack(side="left", padx=(0, 6))
        if self.sim:
            jam = tk.Frame(panel, bg=PANEL)
            jam.pack(fill="x", padx=16, pady=(6, 0))
            tk.Label(jam, text="Jam a leg:", font=FONT, fg=MUTED, bg=PANEL).pack(side="left", padx=(0, 6))
            for leg in LEGS:
                self._button(jam, leg, lambda l=leg: self.jam(l)).pack(side="left", padx=(0, 4))
            self._button(jam, "Free", lambda: self.jam(None)).pack(side="left", padx=(6, 0))

        tk.Label(panel, text="What happened", font=FONT_B, fg=TEXT, bg=PANEL).pack(anchor="w", padx=16, pady=(12, 2))
        self.log = tk.Text(panel, bg=BG, fg=TEXT, font=MONO, wrap="word", relief="flat", height=10,
                           padx=8, pady=6, state="disabled")
        self.log.pack(fill="both", expand=True, padx=16)
        for tag, color in LOG_COLORS.items():
            self.log.tag_configure(tag, foreground=color)

        ask = tk.Frame(panel, bg=PANEL)
        ask.pack(fill="x", padx=16, pady=12)
        self.entry = tk.Entry(ask, font=("Segoe UI", 11), bg=BG, fg=TEXT, insertbackground=TEXT, relief="flat")
        self.entry.pack(side="left", fill="x", expand=True, ipady=6)
        self.entry.bind("<Return>", lambda e: self.ask())
        self.send_btn = self._button(ask, "Ask Qwen", self.ask)
        self.send_btn.pack(side="left", padx=(8, 0))
        if commander is None:
            self.entry.insert(0, "Qwen not connected: use the buttons")
            self.entry.configure(state="disabled")
            self.send_btn.configure(state="disabled")

        self.write("info", "Connected to " + ("the pretend ESP32 (simulated sensors)" if runner.link.info.get("fake")
                   else "the ESP32 (simulated sensors)" if self.sim else "the ESP32"))
        if not runner.bundle:
            self.write("warn", "No stuck-leg model loaded: run  python train.py data/  first. Legs stay gray-green.")
        if llm_note:
            self.write("warn" if commander is None else "info", llm_note)
        self.write("info", "Try: \"walk forward for 3 seconds\", \"turn right\", or press Jam FR and watch.")
        self.tick()

    # ------------------------------------------------------------ widgets
    def _button(self, parent, text, cmd):
        return tk.Button(parent, text=text, command=cmd, font=FONT, bg="#24303c", fg=TEXT, activebackground="#2f3d4b",
                         activeforeground=TEXT, relief="flat", padx=10, pady=4, cursor="hand2")

    def write(self, tag, text):
        stamp = time.strftime("%H:%M:%S")
        self.log.configure(state="normal")
        self.log.insert("end", f"{stamp}  {text}\n", tag)
        lines = int(self.log.index("end-1c").split(".")[0])
        if lines > 300:
            self.log.delete("1.0", f"{lines - 300}.0")
        self.log.see("end")
        self.log.configure(state="disabled")

    # ------------------------------------------------------------ actions
    def walk(self, turn=0.0):
        self.runner.send({"cmd": "gait", "on": 1, "freq": 1.5, "turn": turn})
        self.write("action", "walk" + (f" (turn {turn:+.1f})" if turn else ""))

    def stop(self):
        self.runner.send({"cmd": "gait", "on": 0})
        self.write("action", "stop")

    def reattach(self):
        for leg in LEGS:
            self.runner.send({"cmd": "attach", "leg": leg})
        self.runner.reset_controller()
        self.write("action", "reattach all legs")

    def jam(self, leg):
        self.runner.send({"cmd": "sim_jam", "leg": leg})
        self.write("action", f"jammed {leg} (simulated)" if leg else "freed the jammed leg")

    def ask(self):
        text = self.entry.get().strip()
        if not text or self.busy or self.commander is None:
            return
        self.entry.delete(0, "end")
        self.write("you", f"you: {text}")
        self.busy = True
        self.send_btn.configure(text="thinking...", state="disabled")

        def work():
            t0 = time.monotonic()
            try:
                reply = self.commander.ask(text)
                self.events.put(("qwen", f"qwen: {reply}  ({time.monotonic() - t0:.1f} s)"))
            except Exception as e:
                self.events.put(("warn", f"Qwen error: {e}"))
            self.events.put(("done", ""))
        threading.Thread(target=work, daemon=True).start()

    # ------------------------------------------------------------ the loop
    def tick(self):
        while True:
            try:
                tag, text = self.events.get_nowait()
            except queue.Empty:
                break
            if tag == "done":
                self.busy = False
                self.send_btn.configure(text="Ask Qwen", state="normal")
            else:
                self.write(tag, text.strip())
        s = self.runner.snapshot()
        states = self.leg_states(s)
        self.move(s)
        self.draw_world(s, states)
        self.draw_panel(s, states)
        self.root.after(50, self.tick)

    def leg_states(self, s):
        probs = s.get("stuck_probability") or {}
        out = {}
        for leg in LEGS:
            if leg in s["detached"]:
                out[leg] = "detached"
            elif s["acting_on_leg"] == leg:
                out[leg] = "stuck" if s["release_logic_state"] == "HOLD" else "freeing"
            elif leg in probs and probs[leg] >= s["threshold"]:
                out[leg] = "stuck"
            elif leg in probs and probs[leg] >= 0.5:
                out[leg] = "maybe"
            else:
                out[leg] = "ok" if s["walking"] else "standing"
        return out

    def move(self, s):
        now = time.monotonic()
        dt, self.last_t = min(now - self.last_t, 0.2), now
        g = s["gait_command"]
        if not (s["walking"] and g["on"]) or s["release_logic_state"] == "UNSTICK":
            return
        legs_on = len(s["attached"])
        v = speed_from_freq(g["freq"]) * {4: 1.0, 3: 0.6, 2: 0.3}.get(legs_on, 0.0)
        self.heading += -g["turn"] * 0.8 * dt          # turn > 0 = right = clockwise
        self.x += v * math.cos(self.heading) * dt
        self.y += v * math.sin(self.heading) * dt
        lx, ly = self.trail[-1]
        if math.hypot(self.x - lx, self.y - ly) > 0.02:
            self.trail.append((self.x, self.y))
            self.trail = self.trail[-800:]

    def draw_world(self, s, states):
        c = self.canvas
        c.delete("all")
        w, h = c.winfo_width(), c.winfo_height()
        cx, cy = w / 2, h / 2

        def to_screen(wx, wy):
            return cx + (wx - self.x) * PX_PER_M, cy - (wy - self.y) * PX_PER_M

        # grid that scrolls under the dog (every 0.5 m)
        step = 0.5 * PX_PER_M
        gx = (cx - self.x * PX_PER_M) % step
        while gx < w:
            c.create_line(gx, 0, gx, h, fill=GRID)
            gx += step
        gy = (cy + self.y * PX_PER_M) % step
        while gy < h:
            c.create_line(0, gy, w, gy, fill=GRID)
            gy += step

        if len(self.trail) > 1:
            pts = [p for wx, wy in self.trail for p in to_screen(wx, wy)] + list(to_screen(self.x, self.y))
            c.create_line(*pts, fill="#2d6cdf", width=3, smooth=True)

        def body(bx, by):                               # body px (forward, left) -> screen
            dx = bx * math.cos(self.heading) - by * math.sin(self.heading)
            dy = bx * math.sin(self.heading) + by * math.cos(self.heading)
            return cx + dx, cy - dy

        angles = s.get("leg_angles", [0, 0, 0, 0])
        for i, leg in enumerate(LEGS):                  # legs first, under the body
            fx, fy = CORNERS[leg]
            swing = max(-30, min(30, angles[i] * SWING_PX_PER_DEG)) if states[leg] != "detached" else 0
            hip = body(fx * (BODY_L / 2 - 14), fy * (BODY_W / 2))
            foot = body(fx * (BODY_L / 2 - 14) + swing, fy * (BODY_W / 2 + 22))
            col = COLORS[states[leg]]
            if states[leg] == "detached":
                c.create_oval(foot[0] - LEG_R, foot[1] - LEG_R, foot[0] + LEG_R, foot[1] + LEG_R,
                              outline=col, width=2, dash=(4, 3))
            else:
                c.create_line(*hip, *foot, fill=col, width=8, capstyle="round")
                c.create_oval(foot[0] - LEG_R, foot[1] - LEG_R, foot[0] + LEG_R, foot[1] + LEG_R, fill=col, outline="")
            lab = body(fx * (BODY_L / 2 - 14) + swing, fy * (BODY_W / 2 + 58))
            jammed = s.get("sim_jammed_leg") == leg
            c.create_text(*lab, text=leg + ("\n(jammed)" if jammed else ""), fill=TEXT, font=FONT_B, justify="center")

        corners = [body(BODY_L / 2, BODY_W / 2), body(BODY_L / 2, -BODY_W / 2),
                   body(-BODY_L / 2, -BODY_W / 2), body(-BODY_L / 2, BODY_W / 2)]
        c.create_polygon(*[p for xy in corners for p in xy], fill=BODY, outline="#5b6b7c", width=2)
        nose = [body(BODY_L / 2 + 22, 0), body(BODY_L / 2 - 2, 14), body(BODY_L / 2 - 2, -14)]
        c.create_polygon(*[p for xy in nose for p in xy], fill="#8fb3ff", outline="")

        status = "WALKING" if s["walking"] else "STOPPED"
        c.create_text(16, 16, anchor="nw", text=status, fill=TEXT, font=("Segoe UI", 14, "bold"))
        c.create_text(16, 40, anchor="nw", fill=MUTED, font=FONT,
                      text=f"{len(s['attached'])} legs attached   ·   position estimated from commands")
        y = h - 24
        x = 16
        for key in ("ok", "maybe", "stuck", "freeing", "detached"):
            c.create_oval(x, y - 6, x + 12, y + 6, fill=COLORS[key], outline="")
            c.create_text(x + 18, y, anchor="w", text=LABELS[key], fill=MUTED, font=FONT)
            x += 30 + 8 * len(LABELS[key])

    def draw_panel(self, s, states):
        link = self.runner.link
        where = link.robot[0] if link.robot else "?"
        kind = "pretend ESP32" if link.info.get("fake") else ("ESP32, simulated sensors" if self.sim else "ESP32")
        self.conn.configure(text=f"{kind} at {where}   ·   {s['messages_per_second']} msg/s" +
                            ("" if s["connected"] else "   ·   waiting for data..."))
        logic = s["release_logic_state"]
        acting = f" on {s['acting_on_leg']}" if s["acting_on_leg"] else ""
        self.state.configure(text=f"{'Walking' if s['walking'] else 'Stopped'}   ·   stuck-leg logic: {logic}{acting}")
        c = self.legs
        c.delete("all")
        probs = s.get("stuck_probability") or {}
        width = max(c.winfo_width(), 380)
        for i, leg in enumerate(LEGS):
            y = 6 + i * 30
            st = states[leg]
            c.create_oval(0, y + 4, 14, y + 18, fill=COLORS[st], outline="")
            c.create_text(24, y + 11, anchor="w", text=leg, fill=TEXT, font=FONT_B)
            bx0, bx1 = 60, width - 130
            c.create_rectangle(bx0, y + 5, bx1, y + 17, fill=BG, outline="")
            p = probs.get(leg)
            if p is not None:
                c.create_rectangle(bx0, y + 5, bx0 + (bx1 - bx0) * p, y + 17, fill=COLORS[st], outline="")
                thr_x = bx0 + (bx1 - bx0) * s["threshold"]
                c.create_line(thr_x, y + 2, thr_x, y + 20, fill=MUTED, dash=(2, 2))
            txt = LABELS[st] + (f"  {p:.0%}" if p is not None and st != "detached" else "")
            c.create_text(bx1 + 10, y + 11, anchor="w", text=txt, fill=COLORS[st] if st != "standing" else MUTED, font=FONT)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--demo", action="store_true", help="no hardware: run a pretend ESP32 on this laptop")
    ap.add_argument("--robot-ip", default=None)
    ap.add_argument("--model", default="models/stuck_rf.joblib", help="stuck-leg model")
    ap.add_argument("--watch-only", action="store_true", help="stuck-leg model never unsticks/detaches on its own")
    ap.add_argument("--llm-url", default=os.environ.get("LLM_URL", "http://localhost:11434"))
    ap.add_argument("--llm-model", default=os.environ.get("LLM_MODEL", "qwen3.5:4b"))
    ap.add_argument("--no-llm", action="store_true", help="buttons only, no Qwen")
    args = ap.parse_args()

    from robot_link import RobotLink
    from runner import LiveRunner

    fake = None
    if args.demo:
        from fake_esp32 import FakeEsp32
        fake = FakeEsp32(port=4210, host="127.0.0.1", log=lambda t: None).start()
        args.robot_ip = "127.0.0.1"

    bundle = None
    if os.path.exists(args.model):
        import joblib
        bundle = joblib.load(args.model)

    link = RobotLink(args.robot_ip)
    print("looking for the robot...")
    link.connect()
    runner = LiveRunner(link, bundle, watch_only=args.watch_only)
    runner.start()

    commander, note = None, ""
    if not args.no_llm:
        from commander import Commander, LLMClient, RobotActions
        try:
            llm = LLMClient(args.llm_url, args.llm_model)
            available = llm.models()
            note = f"Qwen ready ({args.llm_model} at {llm.url})"
            if available and args.llm_model not in available:
                note = (f"Model '{args.llm_model}' isn't on {llm.url}. It has: {', '.join(available)}. "
                        "Restart with --llm-model NAME")
            commander = Commander(llm, RobotActions(runner, log=lambda t: None), log=lambda t: None)
        except Exception as e:
            note = f"Qwen not reachable at {args.llm_url}: {e}. Buttons still work."

    root = tk.Tk()
    dash = Dashboard(root, runner, commander, note)
    if commander:                                      # show Qwen's tool calls in the log
        commander.log = lambda t: dash.events.put(("action", t.strip()))
        commander.actions.log = lambda t: dash.events.put(("action", t.strip()))

    def close():
        runner.close()
        if fake:
            fake.stop()
        root.destroy()
    root.protocol("WM_DELETE_WINDOW", close)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        close()


if __name__ == "__main__":
    main()
