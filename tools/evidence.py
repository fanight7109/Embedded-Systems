#!/usr/bin/env python3
"""IS05 test-plan evidence tool (PostTrib1 subsystem test plan, INT-09..15).

Works with an EVIDENCE=1 image (make NET=lwip MQTT=1 EVIDENCE=1). The car
prints "[EV] key=value" lines on USB serial and answers "ev" commands over
MQTT (command code 6). Field tables come from dashboard.py.

Laptop side (needs paho-mqtt):
    python3 tools/evidence.py cmd   --broker IP print [1]   # USB report (1 = raw)
    python3 tools/evidence.py cmd   --broker IP reset | fill | get t2_lat_max
    python3 tools/evidence.py cmd   --broker IP inject 5    # F05 obstacle fault
    python3 tools/evidence.py cmd   --broker IP stress 10 [--unprotected]
    python3 tools/evidence.py wifi  --broker IP --log logs/DATE_INT-09_laptop.txt
    python3 tools/evidence.py stops --broker IP --trigger estop --fill --trials 10
    python3 tools/evidence.py hump  --broker IP --ref-mm 25 --tol 5 --runs 10
    python3 tools/evidence.py hump  --broker IP --flat     # false triggers

Offline (no car):
    python3 tools/evidence.py parse logs/DATE_INT-xx.txt [--csv OUTDIR]
        Summarises every [EV] record in a saved USB console log with
        PASS/FAIL per test, and (--csv) writes timing, raw and log tables.

Owner: Integration (all buddies).
"""

import argparse
import collections
import csv
import os
import re
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import dashboard as dash  # noqa: E402  (field tables and decoders)

EV_CMD = 6
ACTIONS = {"get": 0, "print": 1, "reset": 2, "fill": 3, "inject": 4,
           "stress": 5}
# EV_GET_* indices (app_program/evidence/evidence.h) and their units.
GET = {"stop_src": (0, ""), "src_to_flag": (1, "10us"),
       "flag_to_pwm0": (2, "10us"), "src_to_pwm0": (3, "10us"),
       "pwm0_to_still": (4, "ms"), "fill": (5, ""), "sys_state": (6, ""),
       "t2_lat_max": (7, "10us"), "t2_dev_max": (8, "10us"),
       "t2_late": (9, ""), "t3_lat_max": (10, "10us"),
       "t3_dev_max": (11, "10us"), "t3_late": (12, ""),
       "link_drops": (13, ""), "offline": (14, "100ms"),
       "up_to_tlm": (15, "ms"), "stress_state": (16, ""),
       "stress_err_tt": (17, ""), "stress_err_ti": (18, ""),
       "stress_kops": (19, ""), "t2_kcycles": (20, ""), "faults": (21, ""),
       "trials": (22, ""), "latched": (23, "")}
SRC = ["NONE", "ESTOP_MQTT", "INJECT", "FAULT", "STOP_MQTT", "STOP_BUTTON",
       "OBSTACLE"]
STOP_LIMIT_US = 20000          # NFR9, APP_EV_STOP_LIMIT_US
ROUTINE_LIMIT_US = 320000      # APP_STOP_RAMP_MS + 2 T2 cycles
RESUME_LIMIT_MS = 10000        # NFR5
JITTER_LIMIT_US = 2000         # NFR1
FACE_MIN_MM = 100              # INT-12 stop margin
REACQUIRE_MS = 5000            # INT-12 / F07


# --------------------------------------------------------------- MQTT
class Car:
    """Send commands and wait for their acks."""

    def __init__(self, broker, on_frame=None):
        import paho.mqtt.client as mqtt
        self.acks = {}
        self.cond = threading.Condition()
        self.on_frame = on_frame
        self.seq = int(time.time()) % 256
        self.client = mqtt.Client(client_id=f"is05-evidence-{os.getpid()}")
        self.client.on_message = self._message
        self.client.connect(broker, 1883, 10)
        self.client.subscribe("is05/car/#", qos=1)
        self.client.loop_start()
        time.sleep(0.5)

    def _message(self, _client, _user, msg):
        leaf = msg.topic.rsplit("/", 1)[-1]
        if leaf == "ack" and len(msg.payload) == 4:
            ack, _ = dash.unpack(msg.payload, dash.ACK)
            ack["value"] = (ack["val_hi"] << 10) | ack["val"]
            with self.cond:
                self.acks[ack["seq"]] = ack
                self.cond.notify_all()
        if self.on_frame:
            self.on_frame(leaf, msg.payload, time.time())

    def send(self, cmd, key=0, val=0, wait=2.0):
        self.seq = (self.seq + 1) % 256
        seq = self.seq
        frame = dash.pack({"ver": 1, "seq": seq, "cmd": cmd, "key": key,
                           "val": val},
                          [("ver", 4), ("seq", 8), ("cmd", 3), ("key", 3),
                           ("val", 10), ("pad", 4)])
        self.client.publish("is05/car/cmd", frame, qos=1)
        deadline = time.time() + wait
        with self.cond:
            while seq not in self.acks and time.time() < deadline:
                self.cond.wait(deadline - time.time())
            return self.acks.pop(seq, None)

    def ev(self, action, val=0):
        return self.send(EV_CMD, ACTIONS[action], val)

    def get(self, name):
        ack = self.ev("get", GET[name][0])
        return None if ack is None or not ack["ok"] else ack["value"]

    def close(self):
        self.client.loop_stop()
        self.client.disconnect()


class Watch:
    """Latest telemetry state, for waiting on READY / STOPPED."""

    def __init__(self):
        self.state = None
        self.tlm = None
        self.events = []
        self.tlm_times = []

    def __call__(self, leaf, data, now):
        if leaf == "telemetry" and len(data) == 22:
            self.tlm = dash.decode_telemetry(data)
            self.state = dash.SYSTEM[self.tlm["state"]["system"]] \
                if self.tlm["state"]["system"] < 9 else "?"
            self.tlm_times.append(now)
        elif leaf == "event":
            self.events.append((now, dash.decode_event(data)))

    def wait_state(self, names, timeout):
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.state in names:
                return True
            time.sleep(0.05)
        return False


def us(value, unit):
    """Ack value back to microseconds / ms text."""
    if value is None:
        return "no-ack"
    if value >= 0xFFF:
        return "sat/none"
    if unit == "10us":
        return f"{value * 10} us"
    if unit == "100ms":
        return f"{value * 100} ms"
    return f"{value} {unit}".strip()


def cmd_main(args):
    car = Car(args.broker)
    action = args.action
    val = 0
    if action == "get":
        name = args.value
        print(f"{name} = {us(car.get(name), GET[name][1])}")
        car.close()
        return 0
    if action in ("print", "inject"):
        val = int(args.value or 0)
    elif action == "stress":
        val = (int(args.value or 10) & 0xFF) | (0x200 if args.unprotected
                                                 else 0)
    ack = car.ev(action, val)
    print("ack:", "none (is the image built with EVIDENCE=1?)" if ack is None
          else f"ok={ack['ok']} err={dash.ACK_ERR[ack['err']]} "
               f"value={ack['value']}")
    car.close()
    return 0


# ------------------------------------------------------------- INT-09
def wifi_main(args):
    watch = Watch()
    log = open(args.log, "a") if args.log else None
    marks = {}

    def note(text):
        line = f"{time.strftime('%H:%M:%S')} {text}"
        print(line)
        if log:
            log.write(line + "\n")
            log.flush()

    last = [None]

    def on_frame(leaf, data, now):
        watch(leaf, data, now)
        if leaf == "telemetry" and len(data) == 22:
            if last[0] is not None and now - last[0] > 0.5:
                note(f"telemetry gap {now - last[0]:.2f} s ended")
            if "on" in marks and "resume" not in marks and now > marks["on"]:
                marks["resume"] = now
                note(f"TELEMETRY RESUMED {now - marks['on']:.2f} s after "
                     f"hotspot ON")
            last[0] = now

    car = Car(args.broker, on_frame)
    note("INT-09: start line following (GP20). Press Enter when the hotspot "
         "goes OFF, Enter again when it is back ON, then 'q' + Enter after "
         "telemetry resumes and the car is stopped (GP21).")
    step = 0
    while True:
        text = sys.stdin.readline().strip().lower()
        if text == "q":
            break
        if step == 0:
            marks["off"] = time.time()
            note("MARK hotspot OFF")
        elif step == 1:
            marks["on"] = time.time()
            note(f"MARK hotspot ON ({marks['on'] - marks['off']:.1f} s off)")
        step += 1
    times = watch.tlm_times
    gaps = [b - a for a, b in zip(times, times[1:])]
    resume = marks.get("resume", 0) - marks.get("on", 0) \
        if "resume" in marks else None
    note(f"telemetry frames {len(times)}, largest gap "
         f"{max(gaps) if gaps else 0:.2f} s")
    if resume is None:
        note("NFR5 resume: no telemetry after the ON mark -> FAIL")
    else:
        note(f"NFR5 resume {resume:.2f} s (limit {RESUME_LIMIT_MS / 1000} s)"
             f" -> {'PASS' if resume * 1000 <= RESUME_LIMIT_MS else 'FAIL'}")
    for name in ("t2_lat_max", "t2_dev_max", "t2_late", "t3_lat_max",
                 "t3_dev_max", "t3_late", "link_drops", "offline",
                 "up_to_tlm", "faults"):
        note(f"car {name} = {us(car.get(name), GET[name][1])}")
    note("Now plug in USB and run: evidence.py cmd --broker IP print 1 "
         "(save the console log, then evidence.py parse it)")
    car.close()
    return 0


# ------------------------------------------------------------- INT-10
def stops_main(args):
    watch = Watch()
    car = Car(args.broker, watch)
    rows = []
    for trial in range(1, args.trials + 1):
        input(f"\nTrial {trial}/{args.trials}: "
              + ("reset the Pico (power cycle), " if trial > 1 and
                 args.trigger in ("estop", "inject") else "")
              + "car on the floor or wheels up, press Enter when READY...")
        if not watch.wait_state(("READY", "STOPPED"), 30):
            print("  car not READY/STOPPED (state", watch.state, ") - skipped")
            continue
        car.send(0, 0, args.drive)                     # drive N mm
        time.sleep(args.lead)
        if args.fill:
            ack = car.ev("fill")
            print("  fill armed:", ack is not None and ack["ok"])
        if args.trigger == "estop":
            car.send(2)
        elif args.trigger == "inject":
            car.ev("inject", 5)
        elif args.trigger == "stop":
            car.send(1)
        else:
            input("  press GP21 (STOP) on the car now, then Enter")
        time.sleep(1.5)
        row = {name: car.get(name) for name in
               ("stop_src", "src_to_flag", "flag_to_pwm0", "src_to_pwm0",
                "pwm0_to_still", "fill", "sys_state", "trials")}
        if args.trigger in ("estop", "inject"):
            ack = car.send(0)                           # start: must refuse
            row["start_refused"] = (ack is not None and not ack["ok"] and
                                    dash.ACK_ERR[ack["err"]] == "BAD_STATE")
            time.sleep(4.5)
            row["latched"] = car.get("latched")
            limit = STOP_LIMIT_US
        else:
            ack = car.send(0, 0, args.drive)            # resumes
            row["resumed"] = watch.wait_state(("BARCODE_ACTION",
                                               "LINE_FOLLOW"), 3)
            car.send(1)
            watch.wait_state(("STOPPED",), 5)
            limit = ROUTINE_LIMIT_US
        f2p = row["flag_to_pwm0"]
        row["verdict"] = "PASS" if (f2p is not None and f2p < 0xFFF and
                                    f2p * 10 <= limit and
                                    row.get("start_refused", True) and
                                    row.get("latched", 1) == 1 and
                                    row.get("resumed", True)) else "FAIL"
        rows.append(row)
        print("  " + ", ".join(
            f"{k}={us(v, GET[k][1]) if k in GET else v}"
            for k, v in row.items()))
    if rows:
        worst = max((r["flag_to_pwm0"] or 0) for r in rows) * 10
        worst_src = max((r["src_to_pwm0"] or 0) for r in rows) * 10
        print(f"\n{len(rows)} trials, trigger={args.trigger}: worst "
              f"flag->PWM0 {worst} us, worst source->PWM0 {worst_src} us, "
              f"{sum(r['verdict'] == 'PASS' for r in rows)} PASS")
        if args.csv:
            with open(args.csv, "a", newline="") as handle:
                writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
                if handle.tell() == 0:
                    writer.writeheader()
                writer.writerows(rows)
    car.close()
    return 0


# ------------------------------------------------------------- INT-13
def hump_main(args):
    watch = Watch()
    car = Car(args.broker, watch)
    seen = 0
    heights = []
    print("Listening for HUMP events" + (" (flat ground: every event is a "
                                         "false trigger)" if args.flat else
                                         f", reference {args.ref_mm} mm")
          + ". Ctrl-C to finish.")
    try:
        while args.flat or len(heights) < args.runs:
            time.sleep(0.2)
            for _, ev in watch.events[seen:]:
                if ev["name"] == "HUMP":
                    heights.append(ev["h"])
                    err = ev["h"] - args.ref_mm
                    print(f"  hump #{len(heights)}: {ev['h']} mm "
                          + ("(FALSE TRIGGER)" if args.flat else
                             f"error {err:+d} mm -> "
                             f"{'PASS' if abs(err) <= args.tol else 'FAIL'}"))
                elif ev["name"] == "MISSION_END":
                    print(f"  MISSION_END peak={ev['peak']} mm n={ev['n']}")
            seen = len(watch.events)
    except KeyboardInterrupt:
        pass
    tlm_max = watch.tlm["hump_max"] if watch.tlm else None
    if args.flat:
        print(f"\nflat ground: {len(heights)} hump events -> "
              f"{'PASS' if not heights else 'FAIL'}")
    elif heights:
        errors = [h - args.ref_mm for h in heights]
        mean = sum(errors) / len(errors)
        worst = max(errors, key=abs)
        print(f"\n{len(heights)} humps, reference {args.ref_mm} mm: mean "
              f"error {mean:+.1f} mm, worst {worst:+d} mm, tolerance "
              f"+/-{args.tol} mm -> "
              f"{'PASS' if abs(worst) <= args.tol else 'FAIL'}")
        print(f"highest event {max(heights)} mm, telemetry hump_max "
              f"{tlm_max} mm -> "
              f"{'PASS' if tlm_max == max(heights) else 'CHECK'}")
    car.close()
    return 0


# ---------------------------------------------------------------- parse
LINE = re.compile(r"\[EV\] (\w+)(.*)")
PAIR = re.compile(r"(\w+)=(\S+)")


def parse_log(path):
    records = collections.defaultdict(list)
    seen = set()
    with open(path, errors="replace") as handle:
        for raw in handle:
            match = LINE.search(raw)
            if not match:
                continue
            kind, rest = match.groups()
            fields = dict(PAIR.findall(rest))
            if kind == "HIST":
                fields["counts"] = rest.split("bin_us=")[1].split()[1:]
            key = (kind, fields.get("loop"), fields.get("k"), fields.get("n"),
                   fields.get("s"), fields.get("angle"), fields.get("duty"))
            if kind in ("STATE", "NAV", "MAN", "LINK", "RAW", "SEC") and \
                    key in seen:
                continue                     # repeated by a later "print"
            seen.add(key)
            records[kind].append(fields)
    return records


def num(fields, name, default=0):
    try:
        return int(fields.get(name, default), 0)
    except ValueError:
        return default


def parse_main(args):
    rec = parse_log(args.log)
    out = []

    def say(text=""):
        out.append(text)
        print(text)

    # INT-09
    timing = {t["loop"]: t for t in rec["TIMING"] if "nom_us" in t}
    if timing or rec["LINKSUM"]:
        say("INT-09 control-loop timing during the WiFi outage")
        for loop in ("T2", "T3", "T4"):
            t = timing.get(loop)
            if t:
                say(f"  {loop}: n={t['n']} period {t['per_min']}..."
                    f"{t['per_max']} us (nominal {t['nom_us']}), max |dev| "
                    f"{t['dev_max']} us, latency max {t['lat_max']} us mean "
                    f"{t['lat_mean']} us, exec max {t['exec_max']} us, "
                    f">2 ms: lat {t['lat_gt_limit']} dev {t['dev_gt_limit']}"
                    f" overrun {t['overrun']} -> {t['verdict']}")
        for link in rec["LINKSUM"][-1:]:
            say(f"  link: drops {link['drops']}, offline {link['offline_ms']}"
                f" ms, WiFi up -> telemetry {link['up_to_tlm_ms']} ms -> "
                f"{link['verdict']} (laptop: evidence.py wifi gives the "
                f"hotspot-ON -> telemetry time)")
    # INT-10
    if rec["STOP"]:
        say("\nINT-10 stop latency per trigger")
        groups = collections.defaultdict(list)
        for stop in rec["STOP"]:
            groups[stop["src"]].append(stop)
        for src, items in groups.items():
            f2p = [num(s, "flag_to_pwm0_us", -1) for s in items]
            s2p = [num(s, "src_to_pwm0_us", -1) for s in items]
            fills = sorted({s["fill"] for s in items})
            say(f"  {src}: {len(items)} trials, flag->PWM0 max {max(f2p)} us,"
                f" source->PWM0 max {max(s2p)} us, fill {fills}, "
                f"{sum(s['verdict'] == 'PASS' for s in items)} PASS")
        latches = rec["LATCH"]
        if latches:
            say(f"  latched 5 s after a safety stop: "
                f"{sum(x['verdict'] == 'PASS' for x in latches)}/"
                f"{len(latches)}")
    # INT-11
    turns = [m for m in rec["MAN"] if m["type"] == "TURN"]
    barcodes = [n for n in rec["NAV"] if n["ev"] == "BARCODE"]
    if turns or barcodes:
        say("\nINT-11 barcode reads and turns (encoder-measured heading)")
        names = ["LEFT", "RIGHT", "STRAIGHT", "U_TURN"]
        for bc in barcodes:
            say(f"  t={bc['t_ms']} ms BARCODE {names[num(bc, 'a')]} read "
                f"{'reversed' if num(bc, 'b') else 'forward'} at odo "
                f"{bc['odo']} mm")
        for m in turns:
            target = num(m, "param")
            if m["end"] == "CUT":
                say(f"  t={m['t_ms']} ms TURN {m['dir']} {target} deg ended "
                    f"early after {num(m, 'heading_x10') / 10.0:+.1f} deg "
                    f"(search step stopped by LINE_FOUND): not judged")
                continue
            got = num(m, "late_x10" if m.get("late_ok") == "1"
                      else "heading_x10") / 10.0
            sign = 1 if m["dir"] == "R" else -1
            err = abs(got) - target
            tol = 10 if target >= 180 else 5
            say(f"  t={m['t_ms']} ms TURN {m['dir']} {target} deg: "
                f"{got:+.1f} deg after the stop ({m['end']}), error "
                f"{err:+.1f} deg -> "
                f"{'PASS' if abs(err) <= tol and got * sign > 0 else 'FAIL'}"
                f" (encoder estimate; confirm on video)")
    # INT-12
    obstacles = [n for n in rec["NAV"] if n["ev"] == "OBSTACLE"]
    if obstacles:
        say("\nINT-12 obstacle stop, profile and rejoin")
        states = rec["STATE"]
        navs = rec["NAV"]
        for obs in obstacles:
            t0 = num(obs, "t_ms")
            later = [s for s in states if num(s, "t_ms") >= t0]
            coarse = next((s for s in later if s["sys"] == "OBSTACLE" and
                           s["phase"] == "5"), None)
            follow = next((s for s in later if s["sys"] == "LINE_FOLLOW"),
                          None)
            steps = next((s for s in later if s["sys"] == "OBSTACLE" and
                          s["phase"] == "7"), None)
            prof = next((n for n in navs if n["ev"] == "PROFILE" and
                         num(n, "t_ms") >= t0), None)
            dec = next((n for n in navs if n["ev"] == "DECISION" and
                        num(n, "t_ms") >= t0), None)
            stop_mm = num(coarse, "odo") - num(obs, "odo") if coarse else None
            face = num(obs, "val") - stop_mm if stop_mm is not None else None
            say(f"  detected at {obs['val']} mm (odo {obs['odo']}), stopped "
                f"after {stop_mm} mm -> face ~{face} mm "
                f"({'PASS' if face is not None and face >= FACE_MIN_MM else 'FAIL'}"
                f" vs {FACE_MIN_MM} mm; coarse-scan d_min "
                f"{prof['val'] if prof else '?'} mm)")
            if dec:
                decisions = ["CONTINUE", "BYPASS_LEFT", "BYPASS_RIGHT",
                             "REVERSE", "STOP"]
                say(f"    decision {decisions[num(dec, 'a')]} "
                    f"(deviation {dec['val']} mm)")
            if follow and steps:
                rejoin = num(follow, "t_ms") - num(steps, "t_ms")
                say(f"    bypass start -> LINE_FOLLOW {rejoin} ms -> "
                    f"{'PASS' if rejoin <= REACQUIRE_MS else 'FAIL'}")
            elif follow:
                say(f"    obstacle -> LINE_FOLLOW "
                    f"{num(follow, 't_ms') - t0} ms (no bypass steps)")
            else:
                say("    never returned to LINE_FOLLOW -> FAIL")
    # INT-13
    humps = [n for n in rec["NAV"] if n["ev"] == "HUMP"]
    if humps:
        say("\nINT-13 hump events (car side)")
        for hump in humps:
            say(f"  t={hump['t_ms']} ms height {hump['val']} mm (count "
                f"{hump['a']}) at odo {hump['odo']} mm")
    # INT-14
    if rec["BENCH"] or rec["SERVO"]:
        say("\nINT-14 motor PWM / servo")
        for duty in rec["BENCHDUTY"]:
            say(f"  duty {duty['duty']}: {duty['angles']} angles, "
                f"{duty['bad']} bad, pulse error max {duty['hi_err_max_us']}"
                f" us, period {duty['per_min']}..{duty['per_max']} us")
        for end in rec["BENCH"]:
            if "verdict" in end:
                say(f"  slice register changes {end['slice_changes']}, "
                    f"glitches {end['glitches']} -> {end['verdict']}")
        for change in rec["PWMCHANGE"]:
            say(f"  CHANGED: {change}")
    # INT-15
    if rec["STRESSEND"]:
        say("\nINT-15 packed shared data")
        for end in rec["STRESSEND"]:
            say(f"  {end['mode']} {end['t_s']} s: ops A {end['ops_a']} B "
                f"{end['ops_b']} ISR {end['ops_isr']}, errors TT "
                f"{end['err_tt']} TI {end['err_ti']}, lock fails "
                f"{end['lock_fail']} -> {end['verdict']}")
    if not out:
        print("no [EV] records found (was the image built with EVIDENCE=1?)")
    if args.csv:
        os.makedirs(args.csv, exist_ok=True)
        for kind in ("STATE", "NAV", "MAN", "LINK", "SEC", "STOP", "SERVO",
                     "PWMREG", "STRESS", "STRESSEND"):
            if rec[kind]:
                names = sorted({k for r in rec[kind] for k in r})
                with open(os.path.join(args.csv, f"{kind.lower()}.csv"), "w",
                          newline="") as handle:
                    writer = csv.DictWriter(handle, fieldnames=names)
                    writer.writeheader()
                    writer.writerows(rec[kind])
        raws = collections.defaultdict(list)
        for raw in rec["RAW"]:
            raws[raw["loop"]].append(raw)
        for loop, items in raws.items():
            items.sort(key=lambda r: num(r, "k"))
            with open(os.path.join(args.csv, f"raw_{loop}.csv"), "w",
                      newline="") as handle:
                writer = csv.writer(handle)
                writer.writerow(["k", "t_us", "period_us", "latency_us"])
                prev = None
                for item in items:
                    t = num(item, "t_us")
                    period = (t - prev) & 0xFFFFFFFF if prev is not None \
                        else ""
                    writer.writerow([item["k"], t, period, item["lat_us"]])
                    prev = t
        print(f"\nCSV tables written to {args.csv}")
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = parser.add_subparsers(dest="mode", required=True)

    p = sub.add_parser("cmd", help="send one ev command")
    p.add_argument("--broker", default="127.0.0.1")
    p.add_argument("action", choices=sorted(ACTIONS))
    p.add_argument("value", nargs="?")
    p.add_argument("--unprotected", action="store_true",
                   help="stress: negative control without locks")

    p = sub.add_parser("wifi", help="INT-09 hotspot outage, laptop side")
    p.add_argument("--broker", default="127.0.0.1")
    p.add_argument("--log")

    p = sub.add_parser("stops", help="INT-10 stop-latency trials")
    p.add_argument("--broker", default="127.0.0.1")
    p.add_argument("--trigger", choices=["estop", "inject", "stop",
                                         "button"], required=True)
    p.add_argument("--trials", type=int, default=10)
    p.add_argument("--drive", type=int, default=800,
                   help="test-drive length in mm before the trigger")
    p.add_argument("--lead", type=float, default=1.0,
                   help="seconds of driving before the trigger")
    p.add_argument("--fill", action="store_true",
                   help="fill mbf_motion first (estop / inject only)")
    p.add_argument("--csv")

    p = sub.add_parser("hump", help="INT-13 known-hump runs")
    p.add_argument("--broker", default="127.0.0.1")
    p.add_argument("--ref-mm", type=int, default=0)
    p.add_argument("--tol", type=int, default=5)
    p.add_argument("--runs", type=int, default=10)
    p.add_argument("--flat", action="store_true")

    p = sub.add_parser("parse", help="summarise a saved USB console log")
    p.add_argument("log")
    p.add_argument("--csv")

    args = parser.parse_args()
    if args.mode == "cmd" and args.action == "get" and \
            args.value not in GET:
        parser.error(f"get needs one of: {', '.join(GET)}")
    return {"cmd": cmd_main, "wifi": wifi_main, "stops": stops_main,
            "hump": hump_main, "parse": parse_main}[args.mode](args)


if __name__ == "__main__":
    sys.exit(main())
