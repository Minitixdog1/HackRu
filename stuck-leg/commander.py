"""Talk to the robot dog in plain English. A local Qwen model picks the actions.

The LLM decides WHAT to do (walk, turn, stop, detach a leg...). The live loop
(runner.py) does the fast work underneath: heartbeat, stuck-leg detection, and
unstick/detach on its own, 50 times a second.

  python commander.py                                   # Qwen via Ollama on this laptop
  python commander.py --llm-url http://192.168.1.42:11434   # Qwen on a friend's computer
  python commander.py --llm-url http://localhost:8080/v1    # llama.cpp / LM Studio / vLLM server
  python commander.py --no-robot                        # test the LLM only; actions are printed

Then type things like:  "walk forward for 3 seconds"  "turn left slowly"  "stop"
                        "how are the legs?"  "drop the front right leg"
Type q to quit (motors stop).
"""
import argparse
import json
import os
import re
import time
import urllib.error
import urllib.request

from common import LEGS

SPEEDS = {"slow": 1.0, "normal": 1.5, "fast": 2.0}   # gait cycles per second
MAX_WALK_S = 30

SYSTEM_PROMPT = """You control a small four-legged robot dog through tools. Its legs are FL (front left),
FR (front right), RL (rear left) and RR (rear right); each leg has one motor and can be detached
and reattached. The robot already detects stuck legs by itself and tries to free them, so you do
not need to manage that unless asked.

Rules:
- Use tools to act. Keep replies to one or two short sentences.
- For "walk forward N seconds", call walk with seconds=N. For "walk" with no duration, call walk
  without seconds (it keeps walking until stopped).
- turn is between -1 and 1: negative turns left, positive turns right, 0 is straight.
- Only call detach_leg if the user explicitly asks to remove, drop or detach a leg.
- If you are unsure what the robot is doing, call get_status first.
- If the user asks for something the robot can't do (jump, fetch, talk), say so briefly."""

LEG_PARAM = {"type": "string", "enum": LEGS, "description": "FL, FR, RL or RR"}
TOOLS = [
    {"type": "function", "function": {
        "name": "walk",
        "description": "Start walking. Optionally walk for a number of seconds and then stop.",
        "parameters": {"type": "object", "properties": {
            "seconds": {"type": "number", "description": f"how long to walk, up to {MAX_WALK_S}; omit to keep walking"},
            "speed": {"type": "string", "enum": list(SPEEDS), "description": "default normal"},
            "turn": {"type": "number", "description": "-1 (hard left) to 1 (hard right); 0 = straight"},
        }}}},
    {"type": "function", "function": {
        "name": "stop", "description": "Stop walking (motors off).",
        "parameters": {"type": "object", "properties": {}}}},
    {"type": "function", "function": {
        "name": "unstick_leg", "description": "Wiggle one leg to free it if it is caught.",
        "parameters": {"type": "object", "properties": {"leg": LEG_PARAM}, "required": ["leg"]}}},
    {"type": "function", "function": {
        "name": "detach_leg", "description": "Release one leg from the body. Only when the user asks.",
        "parameters": {"type": "object", "properties": {"leg": LEG_PARAM}, "required": ["leg"]}}},
    {"type": "function", "function": {
        "name": "attach_leg", "description": "Lock a leg back onto the body after it was put back in place.",
        "parameters": {"type": "object", "properties": {"leg": LEG_PARAM}, "required": ["leg"]}}},
    {"type": "function", "function": {
        "name": "get_status",
        "description": "Get the robot's state: walking or not, attached legs, which legs look stuck, recent events.",
        "parameters": {"type": "object", "properties": {}}}},
]


# ====================================================================== LLM client
class LLMClient:
    """Chat with tools against Ollama's own API or any OpenAI-compatible server."""

    def __init__(self, url, model, api="auto", timeout=120):
        url = url.rstrip("/")
        if api == "auto":
            api = "openai" if url.endswith("/v1") else ("ollama" if self._is_ollama(url) else "openai")
        if api == "openai" and not url.endswith("/v1"):
            url += "/v1"
        self.url, self.model, self.api, self.timeout = url, model, api, timeout
        self.no_think = True        # Qwen3.5 thinks by default; slow and unneeded for commands

    @staticmethod
    def _is_ollama(url):
        try:
            with urllib.request.urlopen(url + "/api/tags", timeout=3) as r:
                return "models" in json.loads(r.read())
        except Exception:
            return False

    def _post(self, path, body):
        req = urllib.request.Request(self.url + path, data=json.dumps(body).encode(),
                                     headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as r:
                return json.loads(r.read())
        except urllib.error.HTTPError as e:
            raise RuntimeError(f"HTTP {e.code}: {e.read().decode(errors='replace')[:300]}") from None

    def models(self):
        try:
            if self.api == "ollama":
                with urllib.request.urlopen(self.url + "/api/tags", timeout=5) as r:
                    return [m["name"] for m in json.loads(r.read()).get("models", [])]
            with urllib.request.urlopen(self.url + "/models", timeout=5) as r:
                return [m["id"] for m in json.loads(r.read()).get("data", [])]
        except Exception as e:
            raise RuntimeError(f"can't reach the LLM server at {self.url}: {e}") from None

    def chat(self, messages, tools):
        """Returns (assistant_message_to_keep, text, [(call_id, name, args_dict), ...])."""
        if self.api == "ollama":
            body = {"model": self.model, "messages": messages, "tools": tools, "stream": False,
                    "options": {"temperature": 0.2}}
            if self.no_think:
                body["think"] = False
            try:
                data = self._post("/api/chat", body)
            except RuntimeError as e:
                if "think" not in str(e):
                    raise
                self.no_think = False            # model has no thinking switch; ask again without it
                body.pop("think")
                data = self._post("/api/chat", body)
            msg = data["message"]
            calls = [(None, c["function"]["name"], _args(c["function"].get("arguments")))
                     for c in msg.get("tool_calls") or []]
            keep = {"role": "assistant", "content": msg.get("content", ""), "tool_calls": msg.get("tool_calls") or []}
        else:
            body = {"model": self.model, "messages": messages, "tools": tools, "tool_choice": "auto",
                    "temperature": 0.2}
            if self.no_think:
                body["chat_template_kwargs"] = {"enable_thinking": False}   # llama.cpp / vLLM / SGLang
            try:
                data = self._post("/chat/completions", body)
            except RuntimeError:
                if "chat_template_kwargs" not in body:
                    raise
                self.no_think = False            # server rejects the extra field; ask again without it
                body.pop("chat_template_kwargs")
                data = self._post("/chat/completions", body)
            msg = data["choices"][0]["message"]
            calls = [(c.get("id"), c["function"]["name"], _args(c["function"].get("arguments")))
                     for c in msg.get("tool_calls") or []]
            keep = {"role": "assistant", "content": msg.get("content") or ""}
            if msg.get("tool_calls"):
                keep["tool_calls"] = msg["tool_calls"]
        text = re.sub(r"<think>.*?</think>", "", keep.get("content") or "", flags=re.S).strip()
        return keep, text, calls

    def tool_result(self, call_id, name, result):
        content = json.dumps(result)
        if self.api == "ollama":
            return {"role": "tool", "content": content, "tool_name": name}
        return {"role": "tool", "tool_call_id": call_id or name, "content": content}


def _args(a):
    if isinstance(a, dict):
        return a
    try:
        return json.loads(a or "{}")
    except ValueError:
        return {}


# ====================================================================== robot actions
class RobotActions:
    """What each tool does. With runner=None (--no-robot) actions are only printed."""

    def __init__(self, runner=None):
        self.runner = runner

    def _send(self, cmd):
        print(f"   -> robot: {json.dumps(cmd)}")
        if self.runner:
            self.runner.send(cmd)

    def status(self):
        if self.runner:
            return self.runner.snapshot()
        return {"connected": False, "note": "--no-robot: no robot attached", "walking": False,
                "attached": LEGS, "detached": [], "looks_stuck": []}

    def run(self, name, args):
        if name == "walk":
            speed = args.get("speed", "normal")
            freq = SPEEDS.get(speed, SPEEDS["normal"])
            turn = max(-1.0, min(1.0, float(args.get("turn", 0) or 0)))
            seconds = args.get("seconds")
            self._send({"cmd": "gait", "on": 1, "freq": freq, "turn": turn})
            if seconds:
                seconds = max(0.0, min(float(seconds), MAX_WALK_S))
                time.sleep(seconds)                 # the runner thread keeps the heartbeat going meanwhile
                self._send({"cmd": "gait", "on": 0})
                return {"ok": True, "walked_seconds": seconds, "status": self.status()}
            return {"ok": True, "walking": True, "speed": speed, "turn": turn}
        if name == "stop":
            self._send({"cmd": "gait", "on": 0})
            return {"ok": True, "walking": False}
        if name in ("unstick_leg", "detach_leg", "attach_leg"):
            leg = str(args.get("leg", "")).upper()
            if leg not in LEGS:
                return {"ok": False, "error": f"leg must be one of {LEGS}"}
            self._send({"cmd": name.split("_")[0], "leg": leg})
            if name == "attach_leg" and self.runner:
                self.runner.reset_controller()
            return {"ok": True, "leg": leg}
        if name == "get_status":
            return self.status()
        return {"ok": False, "error": f"unknown tool {name}"}


# ====================================================================== conversation
class Commander:
    def __init__(self, llm, actions, max_history=12):
        self.llm, self.actions, self.max_history = llm, actions, max_history
        self.history = []

    def ask(self, text):
        s = self.actions.status()
        brief = (f"[robot now: walking={'yes' if s.get('walking') else 'no'}, "
                 f"attached={','.join(s.get('attached', [])) or 'none'}, "
                 f"looks stuck={','.join(s.get('looks_stuck', [])) or 'none'}]")
        self.history.append({"role": "user", "content": f"{brief}\n{text}"})
        self.history = self.history[-self.max_history:]
        while self.history and self.history[0]["role"] != "user":   # never start mid tool exchange
            self.history.pop(0)
        turn_msgs = []
        for _ in range(5):                              # at most 5 rounds of tool calls per request
            messages = [{"role": "system", "content": SYSTEM_PROMPT}] + self.history + turn_msgs
            keep, reply, calls = self.llm.chat(messages, TOOLS)
            turn_msgs.append(keep)
            if not calls:
                break
            for call_id, name, args in calls:
                print(f"   [{name}({json.dumps(args)})]")
                turn_msgs.append(self.llm.tool_result(call_id, name, self.actions.run(name, args)))
        self.history += turn_msgs
        return reply or "(done)"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--llm-url", default=os.environ.get("LLM_URL", "http://localhost:11434"),
                    help="Ollama (http://HOST:11434) or OpenAI-compatible server (http://HOST:PORT/v1)")
    ap.add_argument("--llm-model", default=os.environ.get("LLM_MODEL", "qwen3.5:4b"))
    ap.add_argument("--api", choices=["auto", "ollama", "openai"], default="auto")
    ap.add_argument("--no-robot", action="store_true", help="LLM only: print actions instead of sending them")
    ap.add_argument("--robot-ip", default=None)
    ap.add_argument("--model", default="models/stuck_rf.joblib", help="stuck-leg model (optional)")
    ap.add_argument("--watch-only", action="store_true", help="stuck-leg model never acts on its own")
    args = ap.parse_args()

    llm = LLMClient(args.llm_url, args.llm_model, args.api)
    try:
        available = llm.models()
    except RuntimeError as e:
        raise SystemExit(
            f"{e}\nCheck: is Ollama (or your LLM server) running? If it's on another computer, use its IP\n"
            "in --llm-url, and on that computer set OLLAMA_HOST=0.0.0.0 and restart Ollama so it accepts\n"
            "connections from other machines (and allow port 11434 through its firewall).")
    print(f"LLM server: {llm.url} ({llm.api} API)")
    if available and args.llm_model not in available:
        print(f"!! model '{args.llm_model}' isn't on that server. It has: {', '.join(available)}")
        print("   pass the right one with --llm-model")

    runner = None
    if not args.no_robot:
        from robot_link import RobotLink
        from runner import LiveRunner
        bundle = None
        if os.path.exists(args.model):
            import joblib
            bundle = joblib.load(args.model)
        else:
            print(f"(no stuck-leg model at {args.model}: running without automatic stuck detection)")
        link = RobotLink(args.robot_ip)
        print("looking for the robot...")
        info = link.connect()
        print(f"connected to {link.robot[0]} ({'SIMULATED sensors' if info.get('sim') else 'real sensors'})")
        runner = LiveRunner(link, bundle, watch_only=args.watch_only,
                            on_event=lambda text: print(f"\n   !! {text}"))
        runner.start()

    commander = Commander(llm, RobotActions(runner))
    print("Ready. Type a command (q to quit).")
    try:
        while True:
            try:
                text = input("\nyou> ").strip()
            except EOFError:
                break
            if text.lower() in ("q", "quit", "exit"):
                break
            if not text:
                continue
            t0 = time.monotonic()
            try:
                reply = commander.ask(text)
            except Exception as e:
                print(f"dog> (LLM error: {e})")
                continue
            print(f"dog> {reply}   ({time.monotonic() - t0:.1f} s)")
    except KeyboardInterrupt:
        pass
    finally:
        if runner:
            runner.close()
        print("\nmotors stopped, bye")


if __name__ == "__main__":
    main()
