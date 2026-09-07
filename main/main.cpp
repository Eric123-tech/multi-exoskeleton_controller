#include <cstdio>

#include "actuator_logic.h"
#include "driver/uart.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


namespace {

// ============================================================
// General
// ============================================================

constexpr char TAG[] = "EXOSKELETON";


// ============================================================
// I2C
// ============================================================

constexpr gpio_num_t I2C_SDA_PIN = GPIO_NUM_21;
constexpr gpio_num_t I2C_SCL_PIN = GPIO_NUM_22;

constexpr uint32_t I2C_SPEED_HZ = 100000;
constexpr int I2C_TIMEOUT_MS = 1000;


// ============================================================
// PCA9685
// ============================================================

constexpr uint8_t PCA9685_ADDRESS = 0x40;

constexpr uint8_t PCA9685_MODE1 = 0x00;
constexpr uint8_t PCA9685_LED0_ON_L = 0x06;
constexpr uint8_t PCA9685_PRESCALE = 0xFE;

constexpr uint8_t PCA9685_MODE_SLEEP = 0x11;
constexpr uint8_t PCA9685_MODE_WAKE_AI = 0x21;

constexpr uint8_t PCA9685_PRESCALE_50HZ = 0x79;

constexpr float PWM_PERIOD_MS = 20.0f;
constexpr float PWM_COUNTS = 4096.0f;


// ============================================================
// L12 actuator
// ============================================================

// Table index is the public zero-based motor ID. Only listed motors are driven.
// Set feedback_pin to -1 if no feedback wire is connected.
// Calibration values are per motor; enable only after measuring that motor.
struct MotorConfig {
    uint8_t pwm_channel;
    float stroke_mm;
    int feedback_pin;
    bool feedback_calibrated;
    float adc_at_zero;
    float adc_counts_per_mm;
};

constexpr MotorConfig MOTORS[] = {
    // channel, stroke, feedback GPIO, calibrated, ADC at 0 mm, ADC counts/mm
    {0, 50.0f, 34, false, 0.0f, 0.0f},
    {1, 30.0f, 35, false, 0.0f, 0.0f},
};
constexpr size_t MOTOR_COUNT = sizeof(MOTORS) / sizeof(MOTORS[0]);
static_assert(MOTOR_COUNT > 0 && MOTOR_COUNT <= 16,
              "One PCA9685 supports 1 through 16 configured motors");

constexpr bool valid_motor_config()
{
    for (size_t i = 0; i < MOTOR_COUNT; ++i) {
        if (MOTORS[i].pwm_channel >= 16 ||
            (MOTORS[i].stroke_mm != 30.0f && MOTORS[i].stroke_mm != 50.0f) ||
            (MOTORS[i].feedback_calibrated &&
             MOTORS[i].adc_counts_per_mm == 0.0f)) return false;
        for (size_t j = 0; j < i; ++j) {
            if (MOTORS[i].pwm_channel == MOTORS[j].pwm_channel ||
                (MOTORS[i].feedback_pin >= 0 &&
                 MOTORS[i].feedback_pin == MOTORS[j].feedback_pin)) return false;
        }
    }
    return true;
}
static_assert(valid_motor_config(), "Invalid or duplicate motor configuration");

struct MotorState {
    adc_channel_t adc_channel = {};
    float target_mm = 0.0f;
    float previous_position_mm = 0.0f;
    float filtered_velocity_mm_s = 0.0f;
    int64_t previous_time_us = 0;
    int64_t report_due_us = 0;
    int adc_raw = 0;
    bool feedback_ok = false;
};

MotorState motor_states[MOTOR_COUNT];
constexpr int ADC_AVERAGE_SAMPLES = 16;
constexpr float VELOCITY_ALPHA = 0.5f;
constexpr int64_t SAMPLE_PERIOD_US = 100000;
constexpr int64_t SETTLE_TIME_US = 2000000;

// ============================================================
// Error helper
// ============================================================

bool check_ok(esp_err_t err, const char *operation)
{
    if (err == ESP_OK) {
        return true;
    }

    ESP_LOGE(
        TAG,
        "%s failed: %s",
        operation,
        esp_err_to_name(err)
    );

    return false;
}


// ============================================================
// PCA9685 register helpers
// ============================================================

esp_err_t pca_write_register(
    i2c_master_dev_handle_t device,
    uint8_t reg,
    uint8_t value)
{
    uint8_t data[2] = {
        reg,
        value
    };

    return i2c_master_transmit(
        device,
        data,
        sizeof(data),
        I2C_TIMEOUT_MS
    );
}


esp_err_t pca_read_register(
    i2c_master_dev_handle_t device,
    uint8_t reg,
    uint8_t &value)
{
    return i2c_master_transmit_receive(
        device,
        &reg,
        1,
        &value,
        1,
        I2C_TIMEOUT_MS
    );
}


// ============================================================
// PCA9685 initialization
// ============================================================

bool initialize_pca9685(
    i2c_master_dev_handle_t device)
{
    // PRE_SCALE may only be changed while the oscillator sleeps.
    if (!check_ok(
            pca_write_register(
                device,
                PCA9685_MODE1,
                PCA9685_MODE_SLEEP),
            "Put PCA9685 to sleep")) {
        return false;
    }

    if (!check_ok(
            pca_write_register(
                device,
                PCA9685_PRESCALE,
                PCA9685_PRESCALE_50HZ),
            "Configure PCA9685 frequency")) {
        return false;
    }

    // Wake oscillator and enable register auto-increment.
    if (!check_ok(
            pca_write_register(
                device,
                PCA9685_MODE1,
                PCA9685_MODE_WAKE_AI),
            "Wake PCA9685")) {
        return false;
    }

    // Give oscillator time to stabilize.
    esp_rom_delay_us(1000);

    uint8_t mode1 = 0;
    uint8_t prescale = 0;

    if (!check_ok(
            pca_read_register(
                device,
                PCA9685_MODE1,
                mode1),
            "Read PCA9685 MODE1")) {
        return false;
    }

    if (!check_ok(
            pca_read_register(
                device,
                PCA9685_PRESCALE,
                prescale),
            "Read PCA9685 PRE_SCALE")) {
        return false;
    }

    ESP_LOGI(
        TAG,
        "PCA9685 ready: MODE1=0x%02X PRE_SCALE=0x%02X",
        mode1,
        prescale
    );

    return true;
}


// ============================================================
// L12 command conversion
// ============================================================

esp_err_t set_pwm_count(
    i2c_master_dev_handle_t device,
    uint8_t channel,
    uint16_t off_count)
{
    const uint8_t first_register =
        PCA9685_LED0_ON_L + 4 * channel;

    uint8_t data[5] = {
        first_register,

        0x00, // ON_L
        0x00, // ON_H

        static_cast<uint8_t>(off_count & 0xFF),
        static_cast<uint8_t>((off_count >> 8) & 0x0F)
    };

    return i2c_master_transmit(
        device,
        data,
        sizeof(data),
        I2C_TIMEOUT_MS
    );
}


bool set_position_mm(
    i2c_master_dev_handle_t device,
    size_t motor_id,
    float requested_mm)
{
    if (motor_id >= MOTOR_COUNT || !std::isfinite(requested_mm)) return false;
    const auto &config = MOTORS[motor_id];
    auto &state = motor_states[motor_id];
    const float target_mm = actuator::clamp_position(requested_mm, config.stroke_mm);
    if (target_mm != requested_mm) {
        ESP_LOGW(TAG, "Motor %u: %.2f mm clamped to %.2f mm (range 0-%.2f mm)",
                 static_cast<unsigned>(motor_id), requested_mm, target_mm,
                 config.stroke_mm - actuator::EXTENSION_MARGIN_MM);
    }

    const float pulse = actuator::pulse_ms(target_mm, config.stroke_mm);
    const uint16_t count = actuator::pwm_count(pulse);
    if (!check_ok(set_pwm_count(device, config.pwm_channel, count), "Set L12 position")) {
        ESP_LOGE(TAG, "Motor %u command failed", static_cast<unsigned>(motor_id));
        return false;
    }

    state.target_mm = target_mm;
    // A newer command for this motor replaces its pending calibration report.
    state.report_due_us = esp_timer_get_time() + SETTLE_TIME_US;
    ESP_LOGI(TAG, "Motor %u channel=%u: Command %.2f mm -> %.3f ms -> %u counts",
             static_cast<unsigned>(motor_id), config.pwm_channel, target_mm, pulse, count);
    return true;
}


// ============================================================
// L12 feedback
// ============================================================

bool read_average_adc(
    adc_oneshot_unit_handle_t adc_handle,
    adc_channel_t adc_channel,
    int &average_raw)
{
    int sum = 0;

    for (int i = 0; i < ADC_AVERAGE_SAMPLES; ++i) {
        int sample = 0;

        esp_err_t err = adc_oneshot_read(
            adc_handle,
            adc_channel,
            &sample
        );

        if (!check_ok(err, "Read L12 feedback ADC")) {
            return false;
        }

        sum += sample;
    }

    average_raw = sum / ADC_AVERAGE_SAMPLES;

    return true;
}

bool initialize_feedback(adc_oneshot_unit_handle_t &adc_handle)
{
    // This experiment uses ADC1 only. Additional motors can omit feedback (-1).
    // Sharing one ADC unit handle is required for multiple channels.
    for (size_t id = 0; id < MOTOR_COUNT; ++id) {
        const auto &config = MOTORS[id];
        if (config.feedback_pin < 0) continue;
        adc_unit_t unit;
        auto &state = motor_states[id];
        if (!check_ok(adc_oneshot_io_to_channel(
                          config.feedback_pin, &unit, &state.adc_channel),
                      "Map feedback GPIO")) return false;
        if (unit != ADC_UNIT_1) {
            ESP_LOGE(TAG, "Motor %u: GPIO%d must use ADC1; use -1 for no feedback",
                     static_cast<unsigned>(id), config.feedback_pin);
            return false;
        }
        if (adc_handle == nullptr) {
            adc_oneshot_unit_init_cfg_t init = {};
            init.unit_id = ADC_UNIT_1;
            init.ulp_mode = ADC_ULP_MODE_DISABLE;
            if (!check_ok(adc_oneshot_new_unit(&init, &adc_handle), "Initialize ADC1"))
                return false;
        }
        adc_oneshot_chan_cfg_t channel = {};
        channel.atten = ADC_ATTEN_DB_12;
        channel.bitwidth = ADC_BITWIDTH_DEFAULT;
        if (!check_ok(adc_oneshot_config_channel(
                          adc_handle, state.adc_channel, &channel),
                      "Configure feedback channel")) return false;
    }
    return true;
}

void sample_feedback(adc_oneshot_unit_handle_t adc_handle)
{
    for (size_t id = 0; id < MOTOR_COUNT; ++id) {
        const auto &config = MOTORS[id];
        auto &state = motor_states[id];
        if (config.feedback_pin < 0) continue;
        state.feedback_ok = read_average_adc(adc_handle, state.adc_channel, state.adc_raw);
        if (!state.feedback_ok) {
            ESP_LOGW(TAG, "Motor %u feedback unavailable", static_cast<unsigned>(id));
            state.previous_time_us = 0;
            continue;
        }
        if (!config.feedback_calibrated) continue;
        const float position =
            (config.adc_at_zero - state.adc_raw) / config.adc_counts_per_mm;
        const int64_t now = esp_timer_get_time();
        if (state.previous_time_us > 0 && now > state.previous_time_us) {
            const float dt = (now - state.previous_time_us) / 1000000.0f;
            const float velocity = (position - state.previous_position_mm) / dt;
            state.filtered_velocity_mm_s =
                VELOCITY_ALPHA * velocity +
                (1.0f - VELOCITY_ALPHA) * state.filtered_velocity_mm_s;
        } else {
            state.filtered_velocity_mm_s = 0.0f;
        }
        state.previous_position_mm = position;
        state.previous_time_us = now;
    }
}

void report_feedback(size_t id, const char *label)
{
    const auto &config = MOTORS[id];
    const auto &state = motor_states[id];
    if (config.feedback_pin < 0) {
        ESP_LOGI(TAG, "%s: motor=%u command=%.2f mm feedback=disabled",
                 label, static_cast<unsigned>(id), state.target_mm);
    } else if (!state.feedback_ok) {
        ESP_LOGW(TAG, "%s: motor=%u command=%.2f mm feedback=error",
                 label, static_cast<unsigned>(id), state.target_mm);
    } else if (!config.feedback_calibrated) {
        ESP_LOGI(TAG, "%s: motor=%u command=%.2f mm adc=%d (uncalibrated)",
                 label, static_cast<unsigned>(id), state.target_mm, state.adc_raw);
    } else {
        ESP_LOGI(TAG, "%s: motor=%u command=%.2f mm adc=%d pos=%.2f mm vel=%.2f mm/s",
                 label, static_cast<unsigned>(id), state.target_mm, state.adc_raw,
                 state.previous_position_mm, state.filtered_velocity_mm_s);
    }
}

} // namespace


// ============================================================
// Main
// ============================================================

extern "C" void app_main()
{
    ESP_LOGI(TAG, "Exoskeleton controller starting!");


    // --------------------------------------------------------
    // I2C bus
    // --------------------------------------------------------

    i2c_master_bus_config_t bus_config = {};

    bus_config.i2c_port = I2C_NUM_0;
    bus_config.sda_io_num = I2C_SDA_PIN;
    bus_config.scl_io_num = I2C_SCL_PIN;
    bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_config.glitch_ignore_cnt = 7;
    bus_config.flags.enable_internal_pullup = true;

    i2c_master_bus_handle_t bus_handle = nullptr;

    if (!check_ok(
            i2c_new_master_bus(
                &bus_config,
                &bus_handle),
            "Create I2C bus")) {
        return;
    }


    // --------------------------------------------------------
    // PCA9685
    // --------------------------------------------------------

    if (!check_ok(
            i2c_master_probe(
                bus_handle,
                PCA9685_ADDRESS,
                I2C_TIMEOUT_MS),
            "Probe PCA9685")) {
        return;
    }

    i2c_device_config_t device_config = {};

    device_config.dev_addr_length =
        I2C_ADDR_BIT_LEN_7;

    device_config.device_address =
        PCA9685_ADDRESS;

    device_config.scl_speed_hz =
        I2C_SPEED_HZ;

    i2c_master_dev_handle_t pca_handle = nullptr;

    if (!check_ok(
            i2c_master_bus_add_device(
                bus_handle,
                &device_config,
                &pca_handle),
            "Add PCA9685 device")) {
        return;
    }

    if (!initialize_pca9685(pca_handle)) {
        return;
    }


    // Configure all feedback inputs before issuing startup movement commands.
    adc_oneshot_unit_handle_t adc_handle = nullptr;
    if (!initialize_feedback(adc_handle)) return;

    // Explicit buffered, nonblocking UART receive: feedback does not depend on typing.
    // stdout keeps using the default ESP-IDF UART0 console for logs and echo.
    uart_config_t uart_config = {};
    uart_config.baud_rate = 115200;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.source_clk = UART_SCLK_DEFAULT;
    if (!check_ok(uart_param_config(UART_NUM_0, &uart_config), "Configure UART0") ||
        !check_ok(uart_driver_install(UART_NUM_0, 1024, 0, 0, nullptr, 0),
                  "Install UART0 driver")) return;

    for (size_t id = 0; id < MOTOR_COUNT; ++id) {
        const auto &config = MOTORS[id];
        ESP_LOGI(TAG, "Motor %u: channel=%u stroke=%.0f mm limit=%.0f mm feedback=GPIO%d",
                 static_cast<unsigned>(id), config.pwm_channel, config.stroke_mm,
                 config.stroke_mm - actuator::EXTENSION_MARGIN_MM, config.feedback_pin);
        if (!set_position_mm(pca_handle, id, config.stroke_mm * actuator::STARTUP_RATIO))
            return;
    }

    ESP_LOGI(TAG, "Serial ready. Send: P,<position_mm>,<motor_id> (IDs 0-%u)",
             static_cast<unsigned>(MOTOR_COUNT - 1));

    char command_buffer[64] = {};
    size_t command_length = 0;
    bool line_overflow = false;
    int startup_samples_remaining = 30;
    int64_t next_sample_us = esp_timer_get_time();

    while (true) {
        uint8_t incoming[64];
        const int received = uart_read_bytes(UART_NUM_0, incoming, sizeof(incoming), 0);
        for (int i = 0; i < received; ++i) {
            const int c = incoming[i];
            if (c == '\n' || c == '\r') {
                if (command_length > 0 || line_overflow) {
                    putchar('\n');
                    fflush(stdout);
                    command_buffer[command_length] = '\0';
                    actuator::Command command = {};
                    if (line_overflow) {
                        ESP_LOGW(TAG, "Command too long; entire line rejected");
                    } else if (!actuator::parse_command(command_buffer, MOTOR_COUNT, command)) {
                        ESP_LOGW(TAG, "Invalid command: %s. Use P,<position_mm>,<motor_id>; IDs 0-%u",
                                 command_buffer, static_cast<unsigned>(MOTOR_COUNT - 1));
                    } else {
                        set_position_mm(pca_handle, command.motor_id, command.position_mm);
                    }
                    command_length = 0;
                    line_overflow = false;
                }
            } else if (c == '\b' || c == 0x7F) {
                if (!line_overflow && command_length > 0) {
                    --command_length;
                    fputs("\b \b", stdout);
                    fflush(stdout);
                }
            } else if (c >= 0x20 && c <= 0x7E) {
                if (!line_overflow && command_length < sizeof(command_buffer) - 1) {
                    command_buffer[command_length++] = static_cast<char>(c);
                    putchar(c);
                    fflush(stdout);
                } else {
                    // Never execute a truncated prefix of an overlong command.
                    line_overflow = true;
                }
            } else {
                // Reject unsupported control bytes instead of silently changing a command.
                line_overflow = true;
            }
        }

        const int64_t now = esp_timer_get_time();
        if (now >= next_sample_us) {
            sample_feedback(adc_handle);
            next_sample_us = esp_timer_get_time() + SAMPLE_PERIOD_US;
            // Avoid interleaving routine feedback logs with a partly typed command.
            if (startup_samples_remaining > 0) {
                if (command_length == 0 && !line_overflow) {
                    for (size_t id = 0; id < MOTOR_COUNT; ++id)
                        report_feedback(id, "SAMPLE");
                }
                --startup_samples_remaining;
            }
        }
        if (command_length == 0 && !line_overflow) {
            for (size_t id = 0; id < MOTOR_COUNT; ++id) {
                auto &state = motor_states[id];
                if (state.report_due_us > 0 && now >= state.report_due_us) {
                    report_feedback(id, "CAL");
                    state.report_due_us = 0;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
