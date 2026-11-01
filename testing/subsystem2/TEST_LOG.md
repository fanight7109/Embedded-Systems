# Subsystem 2 — test log

**Subsystem:** Motion control (T2)  ·  **Owner:** Peh Jun Wei Joel  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/motion_control/`

See `testing/README.md` for how to run tests and record results. Automatic
tests are in `app_program/motion_control/ss2_motion.c` (`ss2_*_test()`), built with
`make NET=lwip MQTT=1 TEST=1`. Host-side unit tests of the same modules are in
`testing/host/` (`make -C testing/host`).

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S2-U01 | 500 mm → encoder edges → mm within 5 mm | FR1, FR12 | auto | `ss2_motion.c` | `[TEST] S2-U01 PASS` |
| S2-U02 | P-only PID step with Kp 1.00 returns the error | FR1 | auto | `ss2_motion.c` | `[TEST] S2-U02 PASS` |
| S2-U03 | zero error from reset gives zero output | FR1 | auto | `ss2_motion.c` | `[TEST] S2-U03 PASS` |
| S2-U04 | car is stationary at test start (odometer < 10 mm) | FR10 | auto | `ss2_motion.c` | `[TEST] S2-U04 PASS` |
| S2-U05 | E-STOP (MQTT `estop`) zeroes both PWM outputs within 20 ms | NFR9, FR8 | manual | Scope on GP8/GP10, send `estop` while driving (GP20 is now START/STOP, not E-STOP). | PWM low ≤ 20 ms after T1 receives the command |

*Method:* **auto** = `test_log_check()` in code, result printed on the console;
**manual** = a person observes the result (scope, dashboard, tape measure).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S2-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
