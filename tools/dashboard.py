#!/usr/bin/env python3
"""IS05 laptop dashboard: decode binary MQTT frames and send commands.

Field tables mirror app_program/communications/mqtt_fmt.h exactly (Design Review 1.4).

Usage:
    pip install paho-mqtt
    python3 dashboard.py --broker 192.168.1.10            # live view
    python3 dashboard.py --broker IP --send start         # start mission
    python3 dashboard.py --broker IP --send stop | estop
    python3 dashboard.py --broker IP --send drive 600     # barcode test:
                                     # straight 600 mm (1..1023) at
                                     # base_speed, line ignored
    python3 dashboard.py --broker IP --send set kp 2.5    # READY, driving or STOPPED
    python3 dashboard.py --broker IP --send set base_speed 200  # also STOPPED; slews
                                     # 10 mm/s per 100 ms while driving;
                                     # obstacle_mm / hump_pitch_deg: READY only
    python3 dashboard.py --broker IP --send set kp 1.0 --send set ki 0.05
    python3 dashboard.py --broker IP --send get base_speed
    python3 dashboard.py --broker IP --send get ir_cal_l   # IR contrast
    python3 dashboard.py --broker IP --send diag bc_raw    # barcode AO now
    python3 dashboard.py --broker IP --send diag bc_white --send diag bc_black
                                     # barcode range in the last CALIBRATE
    python3 dashboard.py --broker IP --watch bc_raw        # every 0.5 s
    python3 dashboard.py --broker IP --send diag bc_trace  # barcode trace now
                                     # (also sent after every barcode read;
                                     # dashboard_web.py saves it as a CSV)
    python3 dashboard.py --verify testing/host/build/vectors.txt  # A10 check

Owner: Buddy 1 (JONATHAN TAN JIANG MING).
"""

import argparse
import os
import sys
import time

TELEMETRY = [("ver", 4), ("t", 19), ("st", 24), ("spd_l", 10),
             ("spd_r", 10), ("enc_l", 16), ("enc_r", 16), ("dist", 16),
             ("ir", 3), ("err", 7), ("bc", 2), ("bc_dir", 1),
             ("hump_cur", 7), ("hump_max", 7), ("obs_d", 11),
             ("obs_brg", 5), ("flt", 14), ("drop", 4)]
HEARTBEAT = [("ver", 4), ("st", 4), ("t", 19), ("crit", 1), ("rcn", 4)]
EVENT_HEAD = [("ver", 4), ("t", 19), ("ev", 3)]
EVENT_DATA = {
    0: ("BARCODE", [("cmd", 2), ("dir", 1), ("sym", 7), ("nocmd", 1)]),
    1: ("JUNCTION", [("dist", 16)]),
    2: ("HUMP", [("h", 7), ("n", 4)]),
    3: ("OBSTACLE", [("d", 11)]),
    4: ("PROFILE", [("brg", 5), ("d_min", 11), ("width", 8),
                    ("clr_l", 8), ("clr_r", 8)]),
    5: ("FAULT", [("id", 4), ("flt", 14)]),
    6: ("MISSION_END", [("peak", 7), ("n", 4), ("dist", 16), ("rej", 4)]),
}
ACK = [("ver", 4), ("seq", 8), ("ok", 1), ("err", 3), ("key", 3),
       ("val", 10), ("val_hi", 2), ("pad", 1)]
# Barcode trace frame (is05/car/bctrace): header, then 11 samples of
# blackness:12 | black:1 (slicer colour), oldest first, bin_um apart.
TRACE_HEAD = [("ver", 4), ("id", 8), ("chunk", 8), ("chunks", 8),
              ("bin_um", 12)]
TRACE_PER_FRAME = 11
TRACE_BYTES = 23
TRACE_BINS = 1024                # APP_BC_TRACE_BINS in app_cfg.h
STATE_WORD = [("system", 4), ("motion", 3), ("mode", 2), ("line", 3),
              ("barcode", 2), ("obstacle", 4), ("imu", 3), ("comms", 3)]

SYSTEM = ["INIT", "CALIBRATE", "READY", "LINE_FOLLOW", "BARCODE_ACTION",
          "LINE_SEARCH", "OBSTACLE", "STOPPED", "SAFE_STOP"]
COMMS = ["INIT", "WIFI_JOIN", "WIFI_WAIT", "MQTT_CONN", "MQTT_WAIT",
         "ONLINE", "BACKOFF"]
BARCODE = ["LEFT", "RIGHT", "STRAIGHT", "U_TURN"]
ACK_ERR = ["NONE", "BAD_LEN", "BAD_VER", "UNKNOWN_CMD", "BAD_STATE",
           "OUT_OF_RANGE", "DUP_SEQ"]
CMDS = {"start": 0, "stop": 1, "estop": 2, "set": 3, "get": 4, "diag": 5,
        "drive": 0}              # drive N = start with val N (test drive)
DRIVE_MAX_MM = 1023              # 10-bit val
KEYS = {"base_speed": (0, lambda v: int(v) - 100),
        "kp": (1, lambda v: round(float(v) * 100)),
        "ki": (2, lambda v: round(float(v) * 100)),
        "kd": (3, lambda v: round(float(v) * 100)),
        "obstacle_mm": (4, lambda v: int(v) - 150),
        "hump_pitch_deg": (5, lambda v: int(v) - 2),
        # Read-only: IR contrast from the last CALIBRATE, ADC counts / 4.
        "ir_cal_l": (6, None),
        "ir_cal_c": (7, None)}
IR_CAL_KEYS = {6: "ir_cal_l", 7: "ir_cal_c"}
# Read-only DIAG keys (mqtt_fmt.h MQTT_DIAG_*): barcode sensor on GP28 AO,
# ADC counts 0..4095, higher = blacker.
DIAG_KEYS = {"bc_raw": (0, "barcode AO now"),
             "bc_white": (1, "barcode lowest (white) in last CALIBRATE"),
             "bc_black": (2, "barcode highest (black) in last CALIBRATE"),
             "bc_black_on": (3, "barcode white->black threshold"),
             "bc_white_on": (4, "barcode black->white threshold"),
             "bc_trace": (5, "barcode trace requested (bins)")}
SENT = {}                        # seq -> (command, key name), for acks
IR_MIN_CONTRAST = 300            # APP_IR_MIN_CONTRAST in app_cfg.h
FAULTS = ["F01 encoder", "F02 manoeuvre timeout", "F03 line search",
          "F04 self-test", "F05 ultrasonic", "F06 no bypass",
          "F07 reacquire", "F08 collision", "F09 IMU", "F10 network",
          "F11 mbf full", "F12 deadline", "F13 ESTOP", "F14 cmd timeout"]


def unpack(data, table, start=0):
    """Read MSB-first fields; returns (dict, next bit position)."""
    value = int.from_bytes(data, "big")
    total = len(data) * 8
    pos = start
    out = {}
    for name, width in table:
        shift = total - pos - width
        out[name] = (value >> shift) & ((1 << width) - 1)
        pos += width
    return out, pos


def pack(fields, table):
    value = 0
    bits = 0
    for name, width in table:
        value = (value << width) | (fields.get(name, 0) & ((1 << width) - 1))
        bits += width
    pad = (-bits) % 8
    return (value << pad).to_bytes((bits + pad) // 8, "big")


def decode_telemetry(data):
    tlm, _ = unpack(data, TELEMETRY)
    state, _ = unpack(tlm["st"].to_bytes(3, "big"), STATE_WORD)
    tlm["state"] = state
    tlm["spd_l"] -= 512
    tlm["spd_r"] -= 512
    tlm["err_mm"] = (tlm.pop("err") - 64) / 2.0
    return tlm


def decode_event(data):
    head, pos = unpack(data, EVENT_HEAD)
    name, table = EVENT_DATA.get(head["ev"], ("UNKNOWN", []))
    if name == "BARCODE" and len(data) * 8 < pos + 11:
        table = table[:2]        # Older firmware: cmd and dir only.
    body, _ = unpack(data, table, pos)
    body.update(head)
    body["name"] = name
    if name == "BARCODE":
        body["code"] = barcode_label(body)
    return body


def barcode_label(ev):
    """Symbol read (e.g. "*Z* no command"), or the command name."""
    sym = ev.get("sym", 0)
    cmd = BARCODE[ev["cmd"]] if ev.get("cmd", 0) < len(BARCODE) else "?"
    if not 32 < sym < 127:
        return cmd               # Bench injection / older firmware.
    if ev.get("nocmd"):
        return f"*{chr(sym)}* no command"
    return f"*{chr(sym)}* {cmd}"


def decode_trace(data):
    """One bctrace frame: header fields and samples [(blackness, black)]."""
    head, pos = unpack(data, TRACE_HEAD)
    body, _ = unpack(data, [(f"s{i}", 13) for i in range(TRACE_PER_FRAME)],
                     pos)
    head["samples"] = [(body[f"s{i}"] & 0xFFF, body[f"s{i}"] >> 12)
                       for i in range(TRACE_PER_FRAME)]
    return head


def fault_names(mask):
    return [FAULTS[bit] for bit in range(14) if mask & (1 << bit)]


def show(topic, data):
    leaf = topic.rsplit("/", 1)[-1]
    if leaf == "telemetry" and len(data) == 22:
        tlm = decode_telemetry(data)
        st = tlm["state"]
        print(f"[{tlm['t'] * 10:>8} ms] {SYSTEM[st['system']]:<14} "
              f"comms={COMMS[st['comms']] if st['comms'] < 7 else '?'} "
              f"v=({tlm['spd_l']},{tlm['spd_r']}) mm/s "
              f"dist={tlm['dist']} mm enc=({tlm['enc_l']},{tlm['enc_r']}) "
              f"err={tlm['err_mm']:+.1f} mm "
              f"ir={tlm['ir']:03b} bc={BARCODE[tlm['bc']]} "
              f"hump={tlm['hump_cur']}/{tlm['hump_max']} mm "
              f"obs={'none' if tlm['obs_d'] == 2047 else tlm['obs_d']} "
              f"faults={fault_names(tlm['flt'])} drop={tlm['drop']}")
    elif leaf == "heartbeat" and len(data) == 4:
        hb, _ = unpack(data, HEARTBEAT)
        print(f"  heartbeat {SYSTEM[hb['st']] if hb['st'] < 9 else '?'} "
              f"crit={hb['crit']} reconnects={hb['rcn']}")
    elif leaf == "event":
        ev = decode_event(data)
        if ev["name"] == "FAULT":
            ev["faults"] = fault_names(ev["flt"])
        print(f"  EVENT {ev}")
    elif leaf == "ack" and len(data) == 4:
        ack, _ = unpack(data, ACK)
        value = (ack["val_hi"] << 10) | ack["val"]
        sent = SENT.get(ack["seq"], ("", ""))
        print(f"  ACK seq={ack['seq']} ok={ack['ok']} "
              f"err={ACK_ERR[ack['err']]} key={ack['key']} val={value}")
        if ack["ok"] and sent[0] == "diag":
            print(f"  DIAG {sent[1]} = {value} ADC counts "
                  f"({DIAG_KEYS[sent[1]][1]})")
        elif ack["ok"] and ack["key"] in IR_CAL_KEYS:
            counts = ack["val"] * 4
            verdict = "PASS" if counts >= IR_MIN_CONTRAST else "FAIL"
            print(f"  {IR_CAL_KEYS[ack['key']]} contrast ~{counts} ADC counts"
                  f" (need {IR_MIN_CONTRAST}) -> {verdict}")
    elif leaf == "bctrace" and len(data) == TRACE_BYTES:
        tr = decode_trace(data)
        if tr["chunk"] == 0:
            print(f"  BCTRACE {tr['id']}: {tr['chunks']} frames, "
                  f"{tr['bin_um'] / 1000:.3f} mm per sample")
    elif leaf == "status":
        online = (data == bytes([1]))
        print("  STATUS", "ONLINE" if online else "OFFLINE")
    else:
        print(f"  {topic}: bad length {len(data)}")


def build_command(args, seq):
    cmd = CMDS[args[0]]
    key = 0
    val = 0
    if args[0] == "drive":
        try:
            val = int(args[1])
        except (IndexError, ValueError):
            sys.exit("drive needs a distance in mm, e.g. drive 600")
        if not 1 <= val <= DRIVE_MAX_MM:
            sys.exit(f"drive distance must be 1..{DRIVE_MAX_MM} mm")
    elif args[0] == "diag":
        if len(args) < 2 or args[1] not in DIAG_KEYS:
            sys.exit(f"diag needs one of: {', '.join(DIAG_KEYS)}")
        key = DIAG_KEYS[args[1]][0]
        SENT[seq] = ("diag", args[1])
    elif args[0] in ("set", "get"):
        key, encode = KEYS[args[1]]
        if args[0] == "set":
            if encode is None:
                sys.exit(f"{args[1]} is read-only")
            val = encode(args[2])
    return pack({"ver": 1, "seq": seq, "cmd": cmd, "key": key, "val": val},
                [("ver", 4), ("seq", 8), ("cmd", 3), ("key", 3),
                 ("val", 10), ("pad", 4)])


def verify(path):
    """A10: decode frames packed by the firmware's own C code."""
    frames = dict(line.split() for line in open(path) if line.strip())
    tlm = decode_telemetry(bytes.fromhex(frames["telemetry"]))
    expected = {"t": 123456, "spd_l": -120, "spd_r": 250, "enc_l": 65535,
                "enc_r": 1234, "dist": 54321, "ir": 5, "err_mm": -5.0,
                "bc": 3, "bc_dir": 1, "hump_cur": 12, "hump_max": 127,
                "obs_d": 2047, "obs_brg": 24, "flt": 0x2201, "drop": 15}
    state = {"system": 6, "motion": 1, "mode": 2, "line": 3, "barcode": 1,
             "obstacle": 9, "imu": 4, "comms": 5}
    errors = [k for k, v in expected.items() if tlm[k] != v]
    errors += [k for k, v in state.items() if tlm["state"][k] != v]
    ev = decode_event(bytes.fromhex(frames["event"]))
    if (ev["name"], ev["peak"], ev["n"], ev["dist"], ev["rej"], ev["t"]) != \
            ("MISSION_END", 37, 3, 40000, 2, 99):
        errors.append("event")
    ack, _ = unpack(bytes.fromhex(frames["ack"]), ACK)
    if (ack["seq"], ack["ok"], ack["err"], ack["key"], ack["val"]) != \
            (200, 0, 6, 4, 777):
        errors.append("ack")
    if "bctrace" in frames:
        tr = decode_trace(bytes.fromhex(frames["bctrace"]))
        want = [(0, 0), (4095, 0), (4095, 1), (160, 0), (340, 1)] + \
            [(0, 0)] * 6
        if (tr["id"], tr["chunk"], tr["chunks"], tr["bin_um"],
                tr["samples"]) != (7, 3, 94, 483, want):
            errors.append("bctrace")
    print("A10 decode check:", "PASS" if not errors else f"FAIL {errors}")
    return 0 if not errors else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--broker", default="127.0.0.1")
    # Repeat --send to queue several commands, sent in order with
    # consecutive sequence numbers, e.g.
    #   --send set kp 1.0 --send set ki 0.05 --send set base_speed 150
    parser.add_argument("--send", nargs="+", action="append")
    parser.add_argument("--seq", type=int, default=int(time.time()) % 256)
    # Repeat one DIAG read every 0.5 s, e.g. --watch bc_raw while moving
    # the sensor between the black line and the white floor.
    parser.add_argument("--watch", choices=sorted(DIAG_KEYS))
    parser.add_argument("--verify")
    args = parser.parse_args()

    if args.verify:
        return verify(args.verify)

    import paho.mqtt.client as mqtt   # Only needed for live use.

    # Unique id per run: a second window sending commands must not kick
    # the live dashboard off the broker (same client id = disconnect).
    client = mqtt.Client(client_id=f"is05-dashboard-{os.getpid()}")
    client.on_message = lambda c, u, m: show(m.topic, m.payload)
    client.connect(args.broker, 1883, 10)
    client.subscribe("is05/car/#", qos=1)
    for idx, command in enumerate(args.send or []):
        client.publish("is05/car/cmd",
                       build_command(command, (args.seq + idx) % 256), qos=1)
    if not args.watch:
        client.loop_forever()
        return 0
    client.loop_start()
    seq = args.seq + len(args.send or [])
    try:
        while True:
            client.publish("is05/car/cmd",
                           build_command(["diag", args.watch], seq % 256),
                           qos=1)
            seq += 1
            time.sleep(0.5)
    except KeyboardInterrupt:
        client.loop_stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
