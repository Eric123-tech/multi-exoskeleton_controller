# Multi-Exoskeleton Controller

ESP32 firmware for controlling ten Actuonix L12 linear actuators through a PCA9685 PWM controller. The firmware supports individual position commands, a hard-coded hand-closing pose, a return-to-open pose, and a traveling-wave test.

## Attribution

This repository is a fork of [beckham921219/exoskeleton_controller](https://github.com/beckham921219/exoskeleton_controller). Many thanks to the original author for providing the initial controller implementation on which this project is based.

## Main features

- Controls ten L12 actuators using PCA9685 channels 0 through 9.
- Accepts commands over UART0 at 115200 baud.
- Uses zero-based motor IDs from 0 through 9.
- Supports 30 mm and 50 mm physical actuator strokes.
- Applies a 3 mm software extension margin, giving limits of 27 mm and 47 mm.
- Starts in the fully open pose, with every motor commanded to 0 mm.
- Runs `close` and `open` with a 150 ms delay between finger groups to reduce simultaneous startup load.
- Retains the manual position and traveling-wave diagnostic commands.

## Hardware configuration

The current firmware assumes:

| Device | Configuration |
|---|---|
| Controller | ESP32 |
| PWM driver | PCA9685 at I2C address `0x40` |
| I2C SDA | GPIO21 |
| I2C SCL | GPIO22 |
| PWM frequency | 50 Hz |
| Serial interface | UART0 at 115200 baud |

The motor ID is the index in the `MOTORS` array in `main/main.cpp`. The current wiring assumes that the motor ID is also the PCA9685 channel number. Verify every physical connection before running an automatic motion.

## Motor positions and close targets

The motors are ordered by finger: thumb, index, middle, ring, and little finger. Within each finger, the motor nearer the palm comes first.

| ID | Finger | Position | PCA9685 channel | Physical stroke | Allowed command range | `close` target |
|---:|---|---|---:|---:|---:|---:|
| 0 | Thumb | Near palm | 0 | 30 mm | 0–27 mm | 10 mm |
| 1 | Thumb | Far from palm | 1 | 30 mm | 0–27 mm | 27 mm |
| 2 | Index | Near palm | 2 | 50 mm | 0–47 mm | 10 mm |
| 3 | Index | Far from palm | 3 | 30 mm | 0–27 mm | 27 mm |
| 4 | Middle | Near palm | 4 | 50 mm | 0–47 mm | 10 mm |
| 5 | Middle | Far from palm | 5 | 30 mm | 0–27 mm | 27 mm |
| 6 | Ring | Near palm | 6 | 50 mm | 0–47 mm | 10 mm |
| 7 | Ring | Far from palm | 7 | 30 mm | 0–27 mm | 27 mm |
| 8 | Little | Near palm | 8 | 30 mm | 0–27 mm | 10 mm |
| 9 | Little | Far from palm | 9 | 30 mm | 0–27 mm | 27 mm |

The physical stroke must remain configured as 30 mm or 50 mm. Do not replace those values with 27 mm or 47 mm: the physical stroke is used for PWM conversion, while the 3 mm margin is applied only to the allowed command range.

## Serial commands

Enter one command per line in the ESP-IDF serial monitor.

### Close the hand

```text
close
```

`close` moves each motor from its current commanded position to the target in the table above. Each finger follows a three-second smooth trajectory. The two motors belonging to one finger start together, and consecutive fingers start 150 ms apart:

| Finger | Motor IDs | Start delay |
|---|---:|---:|
| Thumb | 0, 1 | 0 ms |
| Index | 2, 3 | 150 ms |
| Middle | 4, 5 | 300 ms |
| Ring | 6, 7 | 450 ms |
| Little | 8, 9 | 600 ms |

The final finger completes after approximately 3.6 seconds.

### Open the hand

```text
open
```

`open` returns every motor to 0 mm. It uses the same finger order, 150 ms stagger, and three-second trajectory per finger, so the complete motion also takes approximately 3.6 seconds.

### Control one motor

```text
P,<position_mm>,<motor_id>
```

Examples:

```text
P,5,0
P,10,2
P,27,9
```

`P` must be uppercase. Positions outside the configured range are clamped and reported. For example, `P,60,2` is clamped to 47 mm, `P,60,3` is clamped to 27 mm, and `P,-10,0` is clamped to 0 mm. Invalid IDs, malformed numbers, extra fields, and incomplete commands are rejected.

A valid manual `P` command immediately interrupts `close`, `open`, or `wave` and gives control to the selected motor.

### Run the traveling-wave test

```text
wave
wave,10
```

`wave` runs for 10 seconds by default. `wave,<seconds>` accepts durations from 1 through 60 seconds. Before starting, the firmware commands all motors to 0 mm and waits for one second. When the wave finishes, all motors return to 0 mm.

Only one automatic motion can control the motors at a time. Starting `close`, `open`, or `wave` interrupts the previous automatic motion. Repeating the currently active pose command does not restart it.

## Clone the repository

```bash
cd ~/linux_projects
git clone https://github.com/Eric123-tech/multi-exoskeleton_controller.git exoskeleton_controller
cd ~/linux_projects/exoskeleton_controller
```

The main firmware source is `main/main.cpp`.

## ESP-IDF setup

This project has been tested with ESP-IDF v6.0.2. Install ESP-IDF by following the [official ESP32 setup guide](https://docs.espressif.com/projects/esp-idf/en/v6.0.2/esp32/get-started/index.html).

On the development machine used for this project, activate the Conda environment and ESP-IDF as follows:

```bash
conda activate esp-idf-py
cd ~/linux_projects/exoskeleton_controller
source ~/esp/esp-idf/export.sh
```

Confirm the environment:

```bash
idf.py --version
```

Expected output includes `ESP-IDF v6.0.2`.

## Build, flash, and monitor

Run the following commands in order:

```bash
conda activate esp-idf-py
cd ~/linux_projects/exoskeleton_controller
source ~/esp/esp-idf/export.sh
idf.py build
idf.py -p /dev/ttyUSB0 -b 115200 flash
idf.py -p /dev/ttyUSB0 monitor
```

Replace `/dev/ttyUSB0` if the ESP32 appears on a different port. Check available ports with:

```bash
ls /dev/ttyUSB* /dev/ttyACM*
```

To stop `idf.py -p /dev/ttyUSB0 monitor`, press:

```text
Ctrl+]
```

If flashing reports a serial-port permission error, add the current user to the `dialout` group, then log out and back in:

```bash
sudo usermod -a -G dialout "$USER"
```

Do not run `idf.py flash` with `sudo`.

## Feedback

IDs 0 and 1 currently use GPIO34 and GPIO35 for analog position feedback. They are marked uncalibrated, so the monitor reports raw ADC values. Feedback is disabled for IDs 2 through 9.

The current feedback is observational only: it does not correct PWM commands or stop a motor. Reading feedback from all ten actuators will require additional hardware, such as a suitable external ADC or analog multiplexer.

## Safety notes

- Verify each motor ID and direction using small manual movements before running `close` or `open`.
- The 3 mm software margin limits commanded extension but is not a measured-position cutoff.
- The firmware does not currently provide force, current, collision, or stall protection.
- The 150 ms stagger reduces simultaneous startup demand but does not replace adequate motor power, grounding, decoupling, and electrical-noise control.
- Keep a reliable method of removing motor power available during initial tests.

## Host logic tests

The host tests cover command parsing, motor IDs, position limits, PWM conversion, and interpolation helpers without requiring an ESP32:

```bash
g++ -std=c++17 -Wall -Wextra -Werror tests/actuator_logic_test.cpp -o /tmp/exoskeleton_actuator_logic_test
/tmp/exoskeleton_actuator_logic_test
```
