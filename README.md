# Exoskeleton Controller

ESP32 firmware for an L12 linear actuator driven through a PCA9685. Position commands are sent over serial. Feedback is read from GPIO 34.

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
EXOSKELETON: Serial ready. Send: P,<position_mm>
```

The firmware initializes I2C, probes the PCA9685, commands 25 mm, then collects 30 position/velocity samples before this prompt.

Command format (`P` must be uppercase). Position is clamped to 0–50 mm:

```
P,25
P,30
```

Use small position changes first.

## Common problems

| Symptom | Fix |
|---|---|
| `idf.py: command not found` | Activate ESP-IDF, then `idf.py --version` |
| `Permission denied: /dev/ttyUSB0` | User needs `dialout`; log out and back in |
| `/dev/ttyUSB0` missing | `ls /dev/ttyUSB* /dev/ttyACM*`; unplug and reconnect |
| `p,25` → `Invalid command` | Use `P,25` |
| Commands ignored after reset | Wait for `Serial ready` |
