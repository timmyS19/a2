import queue
import sys
import threading
import time

import config
from firmata import Board
from light_node import LightNode

HELP = """commands:
  raw | pct                     read the sensor now
  set <0-100>                   manual LED brightness %
  mode off|manual|follow|pid    behaviour of update()
  sp <0-100>                    setpoint %
  kp|ki|kd <value>              PID gains
  gamma <0.5-5>                 actuator curve exponent
  n <1-64>                      samples averaged per reading
  cal | cal dark | cal bright   show / capture anchors
  tele on|off                   1 Hz telemetry line
  status                        everything at once
  quit                          LED off and exit"""


def clamp(value, low, high):
    return max(low, min(high, value))


def telemetry_line(node, t0):
    return (f"t={time.monotonic() - t0:.1f} raw={node.raw} pct={node.pct:.1f} "
            f"sp={node.setpoint:.1f} duty={node.brightness():.1f} mode={node.mode}")


def status_lines(node, board, state):
    pid = node.pid
    return [
        f"mode={node.mode} raw={node.raw} pct={node.pct:.1f} "
        f"duty={node.brightness():.1f}% duty_u16={node.pwm.duty_u16()}",
        f"anchors dark={node.dark} bright={node.bright} n={node.n} "
        f"adc_bits={node.adc.bits} pwm_bits={node.pwm.bits} firmata={board.version}",
        f"pid sp={node.setpoint:.1f} kp={pid.kp:.3f} ki={pid.ki:.3f} kd={pid.kd:.3f} "
        f"e={pid.error:.2f} P={pid.p:.2f} I={pid.i:.2f} D={pid.d:.2f} "
        f"u={node.u:.1f} gamma={node.gamma:.2f}",
        f"loop period_ms={config.PERIOD * 1000:.0f} last_dt_ms={state['dt'] * 1000:.1f}",
    ]


def capture_anchor(node):
    node.set_mode("off")
    time.sleep(0.5)
    return node.read_raw()


def handle(line, node, board, state):
    words = line.strip().lower().split()
    if not words:
        return
    print("> " + " ".join(words))
    cmd = words[0]
    arg = words[1] if len(words) > 1 else ""
    try:
        if cmd in ("help", "?"):
            print(HELP)
        elif cmd == "raw":
            print(node.read_raw())
        elif cmd == "pct":
            print(f"{node.read_pct():.1f}")
        elif cmd == "set":
            node.set_mode("manual")
            node.set_brightness(float(arg))
            print(f"brightness={node.brightness():.1f}% duty_u16={node.pwm.duty_u16()}")
        elif cmd == "mode":
            node.set_mode(arg)
            print(f"mode={node.mode}")
        elif cmd == "sp":
            node.setpoint = clamp(float(arg), 0.0, 100.0)
            print(f"sp={node.setpoint:.1f}")
        elif cmd in ("kp", "ki", "kd"):
            setattr(node.pid, cmd, float(arg))
            print(f"{cmd}={float(arg):.3f}")
        elif cmd == "gamma":
            node.gamma = clamp(float(arg), 0.5, 5.0)
            print(f"gamma={node.gamma:.2f}")
        elif cmd == "n":
            node.n = int(clamp(int(arg), 1, 64))
            print(f"n={node.n}")
        elif cmd == "cal":
            if arg == "dark":
                node.dark = capture_anchor(node)
            elif arg == "bright":
                node.bright = capture_anchor(node)
            print(f"DARK={node.dark} BRIGHT={node.bright}")
        elif cmd == "tele":
            state["tele"] = arg != "off"
            state["next_tele"] = time.monotonic() + config.TELEMETRY_PERIOD
            print("tele=" + ("on" if state["tele"] else "off"))
        elif cmd == "status":
            print("\n".join(status_lines(node, board, state)))
        elif cmd in ("quit", "exit", "q"):
            state["running"] = False
        else:
            print("? unknown command, type help")
    except ValueError:
        print("? bad value, type help")
    sys.stdout.flush()


def read_input(lines):
    for line in sys.stdin:
        lines.put(line)
    lines.put("quit")


def main():
    board = Board(config.PORT, config.BAUD, config.SAMPLING_MS)
    node = LightNode(board, config.LDR_CHANNEL, config.LED_PIN, config.DARK, config.BRIGHT, config.N)
    node.setpoint = config.SETPOINT
    node.gamma = config.GAMMA
    node.pid.kp, node.pid.ki, node.pid.kd = config.KP, config.KI, config.KD
    node.set_mode(config.START_MODE)
    lines = queue.Queue()
    threading.Thread(target=read_input, args=(lines,), daemon=True).start()
    t0 = time.monotonic()
    state = {"running": True, "tele": True, "dt": config.PERIOD,
             "next_tele": t0 + config.TELEMETRY_PERIOD}
    print(f"A2 light node ready on {config.PORT} (firmata {board.version}) - type help", flush=True)
    last = t0
    next_tick = t0
    try:
        while state["running"]:
            while not lines.empty():
                handle(lines.get(), node, board, state)
            now = time.monotonic()
            if now >= next_tick:
                state["dt"] = now - last
                last = now
                node.update(state["dt"])
                next_tick += config.PERIOD
                if next_tick < now:
                    next_tick = now + config.PERIOD
            if state["tele"] and now >= state["next_tele"]:
                print(telemetry_line(node, t0), flush=True)
                state["next_tele"] += config.TELEMETRY_PERIOD
            time.sleep(0.002)
    except KeyboardInterrupt:
        pass
    finally:
        node.set_brightness(0)
        time.sleep(0.05)
        board.close()


if __name__ == "__main__":
    main()
