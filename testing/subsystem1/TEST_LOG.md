# Subsystem 1 — test log

**Subsystem:** Communications (T1, MQTT bridge)  ·  **Owner:** Jonathan Tan Jiang Ming  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/communications/`

See `testing/README.md` for how to run tests and record results. Automatic
tests are in `app_program/communications/ss1_comms.c` (`ss1_*_test()`), built with
`make NET=lwip MQTT=1 TEST=1`. Host-side unit tests of the same modules are in
`testing/host/` (`make -C testing/host`).

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S1-U01 | out-frame header keeps QoS and retain flags | IF-08, NFR11 | auto | `ss1_comms.c` | `[TEST] S1-U01 PASS` |
| S1-U02 | known topic index maps to an MQTT topic | IF-08, NFR11 | auto | `ss1_comms.c` | `[TEST] S1-U02 PASS` |
| S1-U03 | unknown topic index is rejected (no publish) | IF-08, NFR11 | auto | `ss1_comms.c` | `[TEST] S1-U03 PASS` |
| S1-U04 | bitpack round-trips an 11-bit field at a bit offset | NFR11 | auto | `ss1_comms.c` | `[TEST] S1-U04 PASS` |
| S1-U05 | car connects: `is05/car/status` = 01 (retained), telemetry at 5 Hz, heartbeat at 1 Hz | FR7, FR13 | manual | 1. Flash `NET=lwip MQTT=1` image. 2. `python3 tools/dashboard.py`. 3. Count frames for 10 s. | STATUS ONLINE; 50 ± 2 telemetry, 10 ± 1 heartbeats |
| S1-U06 | pull Pico W power: broker publishes will `status` = 00 within 15 s | NFR5 | manual | Watch the dashboard after unplugging. | STATUS OFFLINE |

*Method:* **auto** = `test_log_check()` in code, result printed on the console;
**manual** = a person observes the result (scope, dashboard, tape measure).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S1-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
