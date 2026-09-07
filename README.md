# Exoskeleton Controller

ESP32 firmware for multiple L12 linear actuators controlled through a PCA9685. Position commands are sent over UART0 at 115200 baud. The default experiment enables two motors.

## Install ESP-IDF

Install ESP-IDF v6.0.2 using Espressif's official process:

https://docs.espressif.com/projects/esp-idf/en/v6.0.2/esp32/get-started/index.html

Activate ESP-IDF in every terminal used for this project. On the original development machine:

```bash
source "$HOME/.espressif/tools/activate_idf_v6.0.2.sh"
```

A different computer will have a different activation path. Use the one created by that machine's install.

```bash
idf.py --version
```

Expected: `ESP-IDF v6.0.2`

## Get the project

```bash
git clone https://github.com/beckham921219/exoskeleton_controller.git
cd exoskeleton_controller
```
Source is in `main/main.cpp`.

## Build, flash, and monitor

```bash
idf.py build
ls /dev/ttyUSB*
idf.py -p /dev/ttyUSB0 flash
idf.py -p /dev/ttyUSB0 monitor
```

Replace `/dev/ttyUSB0` if the board appears on another port. To exit the monitor: `Ctrl+]`.

If flashing reports a permission error, add the user to `dialout` and log out/in:

```bash
sudo usermod -a -G dialout $USER
```

Do not use `sudo` with `idf.py flash`.

## Serial commands

Wait for:

```
EXOSKELETON: Serial ready. Send: P,<position_mm>,<motor_id> (IDs 0-1)
```

The firmware initializes I2C, PCA9685, ADC1 and UART0, then commands each configured motor to 50% of its physical stroke. It accepts commands immediately after the prompt; startup feedback sampling runs alongside serial processing.

Command format (`P` must be uppercase). Position is clamped to 0 through the physical stroke minus 3 mm: 0–47 mm for a 50 mm actuator, or 0–27 mm for a 30 mm actuator. Out-of-range commands produce a warning and apply the clamped target.

Configure each motor in the `MOTORS` table in `main/main.cpp`. Table index is the zero-based motor ID, independent of its PWM channel. These are proposed wiring assignments; confirm them against your hardware before flashing:

| Motor ID | PCA9685 channel | Physical stroke | Command range | Startup | Feedback GPIO |
|---|---|---|---|---|---|
| 0 | 0 | 50 mm | 0–47 mm | 25 mm | 34 |
| 1 | 1 | 30 mm | 0–27 mm | 15 mm | 35 |

Keep physical stroke at 30 or 50, not 27 or 47: PWM conversion and startup use physical stroke. Only the command limit subtracts 3 mm. The minimum remains 0 mm. This is a software target limit, not a measured-position cutoff or stall protection.

The command is `P,<target position in mm>,<motor ID>`. Input echo and Backspace are supported. Missing/unknown IDs, extra fields, nonfinite numbers and overlong lines are rejected without updating any motor. The old two-field format is no longer accepted.

```
P,20,0
P,25,1
```

Use small position changes first.

To check independent limits, send `P,60,0` (warning, target 47 mm) and `P,35,1` (warning, target 27 mm). `P,-10,0` clamps to 0 mm. `P,20,2` is rejected with the current two-motor table. Verify the other motor's target does not change. No physical test has been performed as part of this code change.

Commands do not sleep for two seconds: each motor schedules its own `CAL` report approximately two seconds after its latest successful command. A newer command for the same motor replaces that pending report. Other motors remain responsive. ADC sampling runs roughly every 100 ms; initial `SAMPLE` logs stop after 30 sample cycles. Routine reports are deferred/suppressed while a command is being typed so they do not overwrite the input line. Feedback is observational and does not adjust PWM.

Each motor has independent calibration fields: `feedback_calibrated`, `adc_at_zero`, and `adc_counts_per_mm`. Both motors start **uncalibrated**, so logs show raw ADC rather than potentially incorrect millimeters/velocity. After measuring each motor, enable its calibration using `position_mm = (adc_at_zero - adc_raw) / adc_counts_per_mm`. Do not copy the old 50 mm calibration to the 30 mm motor. Use `feedback_pin = -1` for any motor whose feedback is not connected.

## Expanding to nine motors

Add seven entries to `MOTORS` when ready, with the actual 30/50 mm strokes and unique PWM channels (typically 2–8). The accepted IDs automatically become 0–8. Unlisted channels are not commanded. Duplicate channels, duplicate enabled feedback pins, and unsupported strokes fail compile-time validation.

This implementation shares one ADC1 handle across feedback channels and explicitly rejects ADC2 pins. Nine PWM outputs fit on one PCA9685, but nine direct feedback inputs cannot all use ESP32 ADC1 (and not all ADC1 pins are available on every board). Additional motors may use `-1` for no feedback; collecting all nine feedback signals requires a separate hardware/driver extension, such as an external ADC or analog multiplexer. Check the module's signal levels and motor supply capacity for the expanded setup.

## Host logic tests

These tests check strict parsing, IDs, 30/50 mm limits, and PWM conversion without hardware:

```bash
g++ -std=c++17 -Wall -Wextra -Werror tests/actuator_logic_test.cpp -o /tmp/exoskeleton_actuator_logic_test
/tmp/exoskeleton_actuator_logic_test
```

## Common problems

| Symptom | Fix |
|---|---|
| `idf.py: command not found` | Activate ESP-IDF, then `idf.py --version` |
| `Permission denied: /dev/ttyUSB0` | User needs `dialout`; log out and back in |
| `/dev/ttyUSB0` missing | `ls /dev/ttyUSB* /dev/ttyACM*`; unplug and reconnect |
| `p,25,0` or `P,25` → `Invalid command` | Use uppercase `P` and include the ID: `P,25,0` |
| Commands ignored after reset | Wait for `Serial ready` |
