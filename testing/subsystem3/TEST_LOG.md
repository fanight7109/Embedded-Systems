# Subsystem 3 — test log

**Subsystem:** Line following and barcode (T3)  ·  **Owner:** Lim Jun Hao  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/line_barcode/`

See `testing/README.md` for how to run tests and record results. Automatic
tests are in `app_program/line_barcode/ss3_line.c` (`ss3_*_test()`), built with
`make NET=lwip MQTT=1 TEST=1`. Host-side unit tests of the same modules are in
`testing/host/` (`make -C testing/host`).

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S3-U01 | symbol A maps to LEFT | FR3 | auto | `ss3_line.c` | `[TEST] S3-U01 PASS` |
| S3-U02 | symbol D maps to U-TURN | FR3 | auto | `ss3_line.c` | `[TEST] S3-U02 PASS` |
| S3-U03 | unknown symbol is rejected | FR2 | auto | `ss3_line.c` | `[TEST] S3-U03 PASS` |
| S3-U04 | 3-element capture rejected with ERR_LENGTH | FR2 | auto | `ss3_line.c` | `[TEST] S3-U04 PASS` |
| S3-U05 | T3 running, reject counter readable | FR1 | auto | `ss3_line.c` | `[TEST] S3-U05 PASS` |
| S3-U06 | barcode decode rate at 100/200/300 mm/s | FR2, NFR8, gate A7 | manual | Drive over the sample sheet 10× per speed, log events. | ≥ 9/10 per speed |
| S3-U07 | *B* crossed: decoded on distance | FR2, NFR8 | auto | `ss3_line.c` (full `barcode_scan_*` pipeline, target compiler) | `[TEST] S3-U07 PASS` |
| S3-U08 | *B* with bars 40 % too wide decodes | FR2, NFR8 | auto | `ss3_line.c` | `[TEST] S3-U08 PASS` |
| S3-U09 | hand-swipe read on the bench (car still, time domain) | FR2 | manual | Car in READY, wheels still. Swipe the sample sheet under the barcode sensor 10× each way at a steady hand speed; read `bc:` lines on USB serial. | ≥ 9/10 `bc: *A* … by time` (and *Z* on pages 3–4, `no command`) |
| S3-U10 | slow and stop-start read | FR2, NFR8 | manual | Push the car by hand over the sample sheet 10×: slow (~50 mm/s), and with a stop of ≥ 1 s mid-code. | ≥ 9/10 `bc: … by distance` |
| S3-U11 | faint print (bars ~350 vs paper ~160) through slicer + capture, 50–300 mm/s, 3 mm and 6 mm narrow, 2 s stop on a bar, tape line before the code, noise | FR2, NFR8, gate A7 | auto (host) | `make -C testing/host` (`test_barcode_analog`) | all `analog:` checks pass |
| S3-U12 | faint print read on the car | FR2, NFR8 | manual | Flash, CALIBRATE, drive over the printed sample sheet 10× at 100/200/300 mm/s. Watch `bc:` lines and DIAG `bc_black_on` / `bc_white_on` (should sit ~85 / ~50 above paper). | ≥ 9/10 `bc: *A* …` per speed, reported as the last bar ends |
| S3-U13 | whole-barcode trace reaches the dashboard | FR2, NFR8 | manual | Flash, run `tools/dashboard_web.py`, drive over the sample sheet at 100 mm/s. | Plot appears in the barcode sensor card ~4 cm after the code, CSV in `bc_logs/`, 0 frames lost; 15 shaded bars for `*A*` |

*Method:* **auto** = `test_log_check()` in code, result printed on the console;
**manual** = a person observes the result (scope, dashboard, tape measure).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S3-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
