#!/usr/bin/env python3
"""IS05 web dashboard bridge: MQTT <-> browser (tools/dashboard.html).

Browsers cannot speak raw MQTT (TCP 1883), so this bridge:
  * subscribes to is05/car/# on Mosquitto and streams every frame
    (topic, hex, decoded fields) to the page over Server-Sent Events;
  * streams the exact log lines dashboard.py prints (same show() code),
    and appends them to dashboard.log as before;
  * publishes start / stop / estop / set / get / diag commands from the page,
    packed by dashboard.build_command() with a rolling sequence number;
  * records the barcode sensor's live readings (page: "Start live") to
    bc_logs/bc_live_<date>_<time>.csv, one row per bc_raw reading, with
    the car's distance, wheel speeds and state from the latest telemetry;
  * reassembles each barcode trace (is05/car/bctrace: the whole pass over
    a code, ~0.5 mm per sample, sent by the car after every read or on
    "diag bc_trace"), saves it as bc_logs/bc_trace_<date>_<time>.csv and
    sends it to the page for plotting.

Decoders are imported from dashboard.py, so the field tables still have
one source of truth (mqtt_fmt.h -> dashboard.py).

Usage:
    pip install paho-mqtt
    python tools/dashboard_web.py --broker 192.168.1.10
    -> open http://localhost:8080

Options: --port 8080  --log dashboard.log  --no-log  --bc-dir bc_logs
"""

import argparse
import contextlib
import io
import json
import os
import queue
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import dashboard as dash  # noqa: E402  (same folder)

HTML_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         "dashboard.html")
HISTORY_MAX = 600              # Lines replayed to a newly opened page.
TOPIC_ROOT = "is05/car/"
BC_LOG_HEADER = ("time,elapsed_s,bc_raw,dist_mm,spd_l_mm_s,spd_r_mm_s,"
                 "car_ms,state\n")


class BcRecorder:
    """Barcode sensor live log: one CSV row per bc_raw DIAG reading."""

    def __init__(self, folder):
        self.folder = folder
        self.lock = threading.Lock()
        self.fh = None
        self.path = None
        self.rows = 0
        self.t0 = 0.0
        self.tlm = None            # Latest decoded telemetry frame.

    def start(self):
        with self.lock:
            if self.fh:
                return self.path   # Already recording (second page).
            os.makedirs(self.folder, exist_ok=True)
            name = time.strftime("bc_live_%Y%m%d_%H%M%S.csv")
            self.path = os.path.abspath(os.path.join(self.folder, name))
            self.fh = open(self.path, "w", encoding="utf-8", buffering=1)
            self.fh.write(BC_LOG_HEADER)
            self.rows = 0
            self.t0 = time.time()
            return self.path

    def stop(self):
        with self.lock:
            path, rows = self.path, self.rows
            if self.fh:
                self.fh.close()
                self.fh = None
            return path, rows

    def telemetry(self, tlm):
        self.tlm = tlm

    def sample(self, raw):
        with self.lock:
            if not self.fh:
                return
            now = time.time()
            tlm = self.tlm or {}
            state = tlm.get("state", {}).get("system")
            name = (dash.SYSTEM[state] if state is not None and
                    state < len(dash.SYSTEM) else "")
            stamp = (time.strftime("%H:%M:%S", time.localtime(now)) +
                     f".{int((now % 1) * 1000):03d}")
            self.fh.write(f"{stamp},{now - self.t0:.3f},{raw},"
                          f"{tlm.get('dist', '')},{tlm.get('spd_l', '')},"
                          f"{tlm.get('spd_r', '')},"
                          f"{tlm['t'] * 10 if 't' in tlm else ''},{name}\n")
            self.rows += 1


class TraceAssembler:
    """Collect the frames of one barcode trace; save and plot when done."""

    TIMEOUT_S = 3.0            # Finish a trace whose last frame was lost.

    def __init__(self, hub, folder):
        self.hub = hub
        self.folder = folder
        self.lock = threading.Lock()
        self.cur = None
        self.timer = None
        self.last_barcode = ""

    def barcode(self, ev):
        """Remember the latest BARCODE event, to label the next trace."""
        name = ev.get("code") or dash.barcode_label(ev)
        self.last_barcode = name + (" reverse" if ev.get("dir") else "")

    def frame(self, data):
        try:
            tr = dash.decode_trace(data)
        except (IndexError, KeyError, ValueError):
            return
        finished = []
        with self.lock:
            if self.cur is not None and self.cur["id"] != tr["id"]:
                finished.append(self.cur)      # Its last frame was lost.
                self.cur = None
            if self.cur is None:
                self.cur = {"id": tr["id"], "chunks": tr["chunks"],
                            "bin_um": tr["bin_um"], "frames": {},
                            "t": time.time()}
            self.cur["frames"][tr["chunk"]] = tr["samples"]
            if tr["chunk"] + 1 >= tr["chunks"]:
                finished.append(self.cur)
                self.cur = None
                if self.timer:
                    self.timer.cancel()
            else:
                self.arm_timer()
        for cur in finished:
            self.finish(cur)

    def arm_timer(self):
        if self.timer:
            self.timer.cancel()
        self.timer = threading.Timer(self.TIMEOUT_S, self.flush)
        self.timer.daemon = True
        self.timer.start()

    def flush(self):
        with self.lock:
            cur, self.cur = self.cur, None
        if cur is not None:
            self.finish(cur)

    def finish(self, cur):
        per = dash.TRACE_PER_FRAME
        total = min(cur["chunks"] * per, dash.TRACE_BINS)
        vals, black = [None] * total, [None] * total
        for chunk, samples in cur["frames"].items():
            for k, (level, bit) in enumerate(samples):
                idx = chunk * per + k
                if idx < total:
                    vals[idx], black[idx] = level, bit
        missing = cur["chunks"] - len(cur["frames"])
        bin_mm = cur["bin_um"] / 1000.0
        stamp = time.strftime("%Y%m%d_%H%M%S", time.localtime(cur["t"]))
        path = None
        if self.folder:
            try:
                os.makedirs(self.folder, exist_ok=True)
                path = os.path.abspath(os.path.join(
                    self.folder, f"bc_trace_{stamp}_{cur['id']}.csv"))
                with open(path, "w", encoding="utf-8") as fh:
                    fh.write("pos_mm,blackness,black\n")
                    for idx in range(total):
                        fh.write(f"{idx * bin_mm:.2f},"
                                 f"{'' if vals[idx] is None else vals[idx]},"
                                 f"{'' if black[idx] is None else black[idx]}"
                                 "\n")
            except OSError as exc:
                self.hub.log(f"# bc trace: not saved ({exc})", kind="sys")
                path = None
        label = self.last_barcode
        self.last_barcode = ""
        self.hub.log(f"# bc trace {cur['id']}: {total} samples x "
                     f"{bin_mm:.3f} mm"
                     + (f", {missing} of {cur['chunks']} frames lost"
                        if missing else "")
                     + (f", after barcode {label}" if label else "")
                     + (f" -> {path}" if path else ""), kind="sys")
        self.hub.publish({"type": "bctrace", "ts": cur["t"],
                          "id": cur["id"], "bin_mm": bin_mm,
                          "vals": vals, "black": black,
                          "missing": missing, "path": path,
                          "barcode": label}, leaf="bctrace")


class Hub:
    """Fan-out of JSON messages to every open page, plus a replay buffer."""

    def __init__(self, log_path):
        self.lock = threading.Lock()
        self.clients = []
        self.history = []
        self.latest = {}       # Last frame per topic leaf, for new pages.
        self.log_file = None
        if log_path:
            self.log_file = open(log_path, "a", encoding="utf-8",
                                 buffering=1)

    def add(self):
        q = queue.Queue(maxsize=2000)
        with self.lock:
            self.clients.append(q)
            backlog = list(self.history) + list(self.latest.values())
        return q, backlog

    def remove(self, q):
        with self.lock:
            if q in self.clients:
                self.clients.remove(q)

    def publish(self, msg, keep=False, leaf=None):
        data = json.dumps(msg, separators=(",", ":"))
        with self.lock:
            if keep:
                self.history.append(data)
                del self.history[:-HISTORY_MAX]
            if leaf:
                self.latest[leaf] = data
            clients = list(self.clients)
        for q in clients:
            try:
                q.put_nowait(data)
            except queue.Full:
                pass           # Slow page: drop rather than block MQTT.

    def log(self, text, kind="log"):
        for line in text.rstrip("\n").split("\n"):
            if self.log_file:
                self.log_file.write(line + "\n")
            self.publish({"type": kind, "ts": time.time(), "text": line},
                         keep=True)


def decode(leaf, data):
    """Structured fields for the page (None when the length is wrong)."""
    try:
        if leaf == "telemetry" and len(data) == 22:
            tlm = dash.decode_telemetry(data)
            tlm["faults"] = dash.fault_names(tlm["flt"])
            return tlm
        if leaf == "heartbeat" and len(data) == 4:
            return dash.unpack(data, dash.HEARTBEAT)[0]
        if leaf == "event":
            ev = dash.decode_event(data)
            if ev["name"] == "FAULT":
                ev["faults"] = dash.fault_names(ev["flt"])
            return ev
        if leaf == "ack" and len(data) == 4:
            ack = dash.unpack(data, dash.ACK)[0]
            # Full value for the page: valhi carries bits 10-11 (DIAG).
            ack["val"] |= ack.pop("val_hi") << 10
            return ack
        if leaf == "status":
            return {"online": data == bytes([1])}
    except (IndexError, KeyError, ValueError):
        return None
    return None


def make_client(client_id):
    import warnings
    import paho.mqtt.client as mqtt
    warnings.filterwarnings("ignore", category=DeprecationWarning)
    try:                        # paho-mqtt 2.x
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION1,
                           client_id=client_id)
    except AttributeError:      # paho-mqtt 1.x
        return mqtt.Client(client_id=client_id)


class Bridge:
    def __init__(self, hub, broker, port, recorder=None, traces=None):
        self.hub = hub
        self.recorder = recorder
        self.traces = traces
        self.broker = broker
        self.seq = int(time.time()) % 256
        self.seq_lock = threading.Lock()
        self.connected = False
        self.client = make_client(f"is05-webdash-{os.getpid()}")
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        self.client.on_message = self.on_message
        self.client.reconnect_delay_set(1, 5)
        self.port = port

    def start(self):
        self.client.connect_async(self.broker, self.port, 10)
        self.client.loop_start()

    def status(self):
        self.hub.publish({"type": "bridge", "broker": self.broker,
                          "connected": self.connected}, leaf="_bridge")

    def on_connect(self, client, userdata, flags, rc, *extra):
        self.connected = (rc == 0)
        client.subscribe(TOPIC_ROOT + "#", qos=1)
        self.hub.log(f"# bridge: connected to {self.broker}:{self.port} "
                     f"rc={rc}", kind="sys")
        self.status()

    def on_disconnect(self, client, userdata, rc, *extra):
        self.connected = False
        self.hub.log(f"# bridge: disconnected rc={rc}, retrying", kind="sys")
        self.status()

    def on_message(self, client, userdata, msg):
        leaf = msg.topic.rsplit("/", 1)[-1]
        data = bytes(msg.payload)
        if leaf == "bctrace":
            # ~94 frames per trace: assembled here, not logged one by one.
            if self.traces is not None:
                self.traces.frame(data)
            return
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            try:
                dash.show(msg.topic, data)   # Exact dashboard.py output.
            except (IndexError, KeyError, ValueError) as exc:
                print(f"  {msg.topic}: decode error {exc}")
        frame = {"type": "frame", "ts": time.time(), "topic": msg.topic,
                 "leaf": leaf, "hex": data.hex(), "len": len(data),
                 "retain": bool(msg.retain), "d": decode(leaf, data)}
        self.record(leaf, frame["d"])
        if (self.traces is not None and leaf == "event" and frame["d"] and
                frame["d"].get("name") == "BARCODE"):
            self.traces.barcode(frame["d"])
        keep_leaf = leaf if leaf in ("telemetry", "status",
                                     "heartbeat") else None
        # Events (barcode reads) are replayed to a reopened page.
        self.hub.publish(frame, keep=(leaf == "event"), leaf=keep_leaf)
        if buf.getvalue():
            self.hub.log(buf.getvalue())

    def record(self, leaf, fields):
        """Feed telemetry and bc_raw DIAG acks to the barcode log."""
        if self.recorder is None or fields is None:
            return
        if leaf == "telemetry":
            self.recorder.telemetry(fields)
        elif (leaf == "ack" and fields.get("ok") and
              dash.SENT.get(fields.get("seq")) == ("diag", "bc_raw")):
            self.recorder.sample(fields["val"])

    def send(self, words):
        """words: e.g. ["start"] or ["set", "base_speed", "200"]."""
        if not words or words[0] not in dash.CMDS:
            raise ValueError(f"unknown command {words!r}")
        if words[0] == "drive":
            try:
                mm = int(words[1]) if len(words) > 1 else 0
            except ValueError:
                mm = 0
            if not 1 <= mm <= dash.DRIVE_MAX_MM:
                raise ValueError(f"drive needs 1..{dash.DRIVE_MAX_MM} mm")
        if words[0] in ("set", "get"):
            if len(words) < 2 or words[1] not in dash.KEYS:
                raise ValueError(f"unknown key in {words!r}")
            if words[0] == "set":
                if len(words) < 3 or dash.KEYS[words[1]][1] is None:
                    raise ValueError(f"{words[1]} needs a writable value")
        if not self.connected:
            raise RuntimeError("not connected to broker")
        with self.seq_lock:
            self.seq = (self.seq + 1) % 256
            seq = self.seq
        try:
            payload = dash.build_command(words, seq)
        except SystemExit as exc:          # build_command exits on misuse.
            raise ValueError(str(exc)) from None
        self.client.publish(TOPIC_ROOT + "cmd", payload, qos=1)
        self.hub.log(f"> SEND {' '.join(words)} seq={seq} "
                     f"hex={payload.hex()}", kind="cmd")
        return seq


def make_handler(hub, bridge, recorder):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, fmt, *args):
            pass                           # Keep the terminal quiet.

        def cors(self):
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Headers", "Content-Type")
            self.send_header("Access-Control-Allow-Methods", "GET, POST")

        def do_OPTIONS(self):
            self.send_response(204)
            self.cors()
            self.send_header("Content-Length", "0")
            self.end_headers()

        def reply(self, code, body, ctype="application/json"):
            raw = body.encode("utf-8")
            self.send_response(code)
            self.cors()
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(raw)))
            self.end_headers()
            self.wfile.write(raw)

        def do_GET(self):
            if self.path in ("/", "/index.html", "/dashboard.html"):
                try:
                    with open(HTML_PATH, encoding="utf-8") as fh:
                        self.reply(200, fh.read(), "text/html; charset=utf-8")
                except OSError:
                    self.reply(404, "dashboard.html not found next to "
                               "dashboard_web.py", "text/plain")
            elif self.path == "/stream":
                self.stream()
            else:
                self.reply(404, "{}")

        def stream(self):
            self.send_response(200)
            self.cors()
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Connection", "keep-alive")
            self.end_headers()
            q, backlog = hub.add()
            bridge.status()
            try:
                for data in backlog:
                    self.wfile.write(f"data: {data}\n\n".encode())
                self.wfile.flush()
                while True:
                    try:
                        data = q.get(timeout=15)
                        self.wfile.write(f"data: {data}\n\n".encode())
                    except queue.Empty:
                        self.wfile.write(b": ping\n\n")
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError,
                    ConnectionAbortedError, OSError):
                pass
            finally:
                hub.remove(q)

        def do_POST(self):
            if self.path == "/bclog":
                self.bc_log()
                return
            if self.path != "/cmd":
                self.reply(404, "{}")
                return
            length = int(self.headers.get("Content-Length", "0"))
            try:
                body = json.loads(self.rfile.read(length) or b"{}")
                words = [str(w) for w in body.get("args", [])]
                seq = bridge.send(words)
                self.reply(200, json.dumps({"ok": True, "seq": seq}))
            except (ValueError, RuntimeError, TypeError) as exc:
                hub.log(f"> SEND rejected: {exc}", kind="sys")
                self.reply(400, json.dumps({"ok": False,
                                            "error": str(exc)}))

        def bc_log(self):
            """{"on": true} starts the barcode CSV, {"on": false} ends it."""
            length = int(self.headers.get("Content-Length", "0"))
            try:
                body = json.loads(self.rfile.read(length) or b"{}")
            except ValueError:
                body = {}
            if recorder is None:
                self.reply(400, json.dumps({"ok": False,
                                            "error": "bridge run with "
                                                     "--no-log"}))
                return
            try:
                if body.get("on"):
                    path = recorder.start()
                    hub.log(f"# bc log: recording to {path}", kind="sys")
                    self.reply(200, json.dumps({"ok": True, "path": path}))
                else:
                    path, rows = recorder.stop()
                    if path:
                        hub.log(f"# bc log: {rows} readings saved to {path}",
                                kind="sys")
                    self.reply(200, json.dumps({"ok": True, "path": path,
                                                "rows": rows}))
            except OSError as exc:
                self.reply(500, json.dumps({"ok": False,
                                            "error": str(exc)}))

    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--broker", default="127.0.0.1")
    parser.add_argument("--mqtt-port", type=int, default=1883)
    parser.add_argument("--port", type=int, default=8080,
                        help="HTTP port for the page")
    parser.add_argument("--host", default="127.0.0.1",
                        help="0.0.0.0 to open the page from another device")
    parser.add_argument("--log", default="dashboard.log")
    parser.add_argument("--no-log", action="store_true")
    parser.add_argument("--bc-dir", default="bc_logs",
                        help="folder for the barcode sensor live CSVs")
    args = parser.parse_args()

    hub = Hub(None if args.no_log else args.log)
    recorder = None if args.no_log else BcRecorder(args.bc_dir)
    traces = TraceAssembler(hub, None if args.no_log else args.bc_dir)
    bridge = Bridge(hub, args.broker, args.mqtt_port, recorder, traces)
    bridge.start()
    server = ThreadingHTTPServer((args.host, args.port),
                                 make_handler(hub, bridge, recorder))
    server.daemon_threads = True
    url = f"http://{'localhost' if args.host in ('127.0.0.1', '0.0.0.0') else args.host}:{args.port}"
    print(f"IS05 web dashboard: {url}  (broker {args.broker}:"
          f"{args.mqtt_port}, Ctrl+C to quit)")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        if recorder is not None:
            recorder.stop()
        bridge.client.loop_stop()
        server.server_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
