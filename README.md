# Exoskeleton Controller

ESP32 firmware for an exoskeleton actuator controller. Drives an L12 linear actuator through a PCA9685 PWM driver and estimates position from analog feedback.

## Hardware

- ESP32
- PCA9685 (I2C: SDA GPIO 21, SCL GPIO 22)
- Actuonix L12 linear actuator (PWM channel 0, feedback on GPIO 34)

## Build and flash

Requires [ESP-IDF v6.0.2](https://docs.espressif.com/projects/esp-idf/en/v6.0.2/esp32/get-started/index.html).

```bash
cd ~/robotics/repos/personal/exoskeleton_controller/
source "$HOME/.espressif/tools/activate_idf_v6.0.2.sh"
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

Source is in `main/main.cpp`.
