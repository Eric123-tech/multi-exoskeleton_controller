#include <cstdio>

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

constexpr float L12_STROKE_MM = 50.0f;

constexpr float L12_MIN_PULSE_MS = 1.0f;
constexpr float L12_MAX_PULSE_MS = 2.0f;

constexpr uint8_t L12_PWM_CHANNEL = 0;


// Empirical purple-wire feedback calibration.
//
// Approximate model:
//
//     ADC = 3923 - 78 * position_mm
//
// Therefore:
//
//     position_mm = (3923 - ADC) / 78
//
constexpr float L12_ADC_AT_0_MM = 3923.0f;
constexpr float L12_ADC_COUNTS_PER_MM = 78.0f;


// ============================================================
// ADC / estimator
// ============================================================

constexpr gpio_num_t L12_FEEDBACK_PIN = GPIO_NUM_34;

constexpr int ADC_AVERAGE_SAMPLES = 16;

constexpr float VELOCITY_ALPHA = 0.5f;

constexpr int CONTROL_PERIOD_MS = 100;
constexpr TickType_t CONTROL_PERIOD_TICKS =
    pdMS_TO_TICKS(CONTROL_PERIOD_MS);


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

float clamp_position_mm(float position_mm)
{
    if (position_mm < 0.0f) {
        return 0.0f;
    }

    if (position_mm > L12_STROKE_MM) {
        return L12_STROKE_MM;
    }

    return position_mm;
}


float position_to_pulse_ms(float position_mm)
{
    position_mm = clamp_position_mm(position_mm);

    const float position_ratio =
        position_mm / L12_STROKE_MM;

    return
        L12_MIN_PULSE_MS +
        position_ratio *
        (L12_MAX_PULSE_MS - L12_MIN_PULSE_MS);
}


uint16_t pulse_ms_to_pwm_count(float pulse_ms)
{
    return static_cast<uint16_t>(
        (pulse_ms / PWM_PERIOD_MS) * PWM_COUNTS + 0.5f
    );
}


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
    uint8_t channel,
    float target_position_mm)
{
    target_position_mm =
        clamp_position_mm(target_position_mm);

    const float pulse_ms =
        position_to_pulse_ms(target_position_mm);

    const uint16_t pwm_count =
        pulse_ms_to_pwm_count(pulse_ms);

    if (!check_ok(
            set_pwm_count(
                device,
                channel,
                pwm_count),
            "Set L12 position")) {
        return false;
    }

    ESP_LOGI(
        TAG,
        "Command: %.2f mm -> %.3f ms -> %u counts",
        target_position_mm,
        pulse_ms,
        pwm_count
    );

    return true;
}


// ============================================================
// L12 feedback
// ============================================================

float adc_to_position_mm(int adc_raw)
{
    return
        (L12_ADC_AT_0_MM - static_cast<float>(adc_raw)) /
        L12_ADC_COUNTS_PER_MM;
}


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


    // --------------------------------------------------------
    // Command one actuator
    // --------------------------------------------------------

    constexpr float TARGET_POSITION_MM = 40.0f;

    if (!set_position_mm(
            pca_handle,
            L12_PWM_CHANNEL,
            TARGET_POSITION_MM)) {
        return;
    }


    // --------------------------------------------------------
    // ADC setup
    // --------------------------------------------------------

    adc_unit_t adc_unit;
    adc_channel_t adc_channel;

    if (!check_ok(
            adc_oneshot_io_to_channel(
                L12_FEEDBACK_PIN,
                &adc_unit,
                &adc_channel),
            "Map L12 feedback GPIO to ADC")) {
        return;
    }

    adc_oneshot_unit_init_cfg_t adc_init_config = {};

    adc_init_config.unit_id = adc_unit;
    adc_init_config.ulp_mode = ADC_ULP_MODE_DISABLE;

    adc_oneshot_unit_handle_t adc_handle = nullptr;

    if (!check_ok(
            adc_oneshot_new_unit(
                &adc_init_config,
                &adc_handle),
            "Initialize ADC")) {
        return;
    }

    adc_oneshot_chan_cfg_t adc_channel_config = {};

    adc_channel_config.atten = ADC_ATTEN_DB_12;
    adc_channel_config.bitwidth = ADC_BITWIDTH_DEFAULT;

    if (!check_ok(
            adc_oneshot_config_channel(
                adc_handle,
                adc_channel,
                &adc_channel_config),
            "Configure ADC channel")) {
        return;
    }


    // --------------------------------------------------------
    // Position / velocity estimator
    // --------------------------------------------------------

    float previous_position_mm = 0.0f;
    int64_t previous_time_us = 0;

    float filtered_velocity_mm_s = 0.0f;

    bool have_previous_sample = false;

    TickType_t last_wake_time =
        xTaskGetTickCount();


    // Keep 30 samples for now so behavior matches our tests.
    for (int i = 0; i < 30; ++i) {

        int adc_raw = 0;

        if (!read_average_adc(
                adc_handle,
                adc_channel,
                adc_raw)) {
            return;
        }

        const float position_mm =
            adc_to_position_mm(adc_raw);

        const int64_t current_time_us =
            esp_timer_get_time();


        if (have_previous_sample) {

            const float dt_s =
                static_cast<float>(
                    current_time_us - previous_time_us
                ) / 1000000.0f;

            const float raw_velocity_mm_s =
                (position_mm - previous_position_mm) /
                dt_s;

            filtered_velocity_mm_s =
                VELOCITY_ALPHA * raw_velocity_mm_s +
                (1.0f - VELOCITY_ALPHA) *
                filtered_velocity_mm_s;

            ESP_LOGI(
                TAG,
                "ADC=%d pos=%.2f mm vel_raw=%.2f mm/s "
                "vel_filtered=%.2f mm/s dt=%.4f s",
                adc_raw,
                position_mm,
                raw_velocity_mm_s,
                filtered_velocity_mm_s,
                dt_s
            );

        } else {

            ESP_LOGI(
                TAG,
                "ADC=%d pos=%.2f mm",
                adc_raw,
                position_mm
            );
        }


        previous_position_mm = position_mm;
        previous_time_us = current_time_us;
        have_previous_sample = true;


        xTaskDelayUntil(
            &last_wake_time,
            CONTROL_PERIOD_TICKS
        );
    }
    
    ESP_LOGI(TAG, "Serial ready. Send: P,<position_mm>");

    char command_buffer[64] = {};
    size_t command_length = 0;

    while (true) {
        int c = getchar();

        if (c != EOF) {

            if (c == '\n' || c == '\r') {

                if (command_length > 0) {

                    command_buffer[command_length] = '\0';

                    float target_position_mm = 0.0f;

                    if (sscanf(
                            command_buffer,
                            "P,%f",
                            &target_position_mm) == 1) {

                        ESP_LOGI(
                            TAG,
                            "Parsed position command: %.2f mm",
                            target_position_mm
                        );
                        if (!set_position_mm(
                            pca_handle,
                            L12_PWM_CHANNEL,
                            target_position_mm)) {
                    
                        ESP_LOGE(TAG, "Failed to apply position command");
                    
                        } else {
                        
                            // Wait for actuator to settle before calibration reading.
                            vTaskDelay(pdMS_TO_TICKS(2000));
                        
                            int calibration_adc = 0;
                        
                            if (read_average_adc(
                                    adc_handle,
                                    adc_channel,
                                    calibration_adc)) {
                        
                                ESP_LOGI(
                                    TAG,
                                    "CAL: command=%.2f mm adc=%d",
                                    target_position_mm,
                                    calibration_adc
                                );
                            }
                        }
                    } else {

                        ESP_LOGW(
                            TAG,
                            "Invalid command: %s",
                            command_buffer
                        );
                    }

                    command_length = 0;
                }

            } else if (
                command_length < sizeof(command_buffer) - 1
            ) {

                command_buffer[command_length] =
                    static_cast<char>(c);

                ++command_length;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}