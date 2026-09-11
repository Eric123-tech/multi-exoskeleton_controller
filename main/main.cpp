#include <cstdio>
#include <cmath>
#include <cctype>
#include <cstdlib>

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


// ============================================================
// Motor configuration
// ============================================================

struct MotorConfig {
    uint8_t pwm_channel;
    float stroke_mm;
    int feedback_pin;
    bool feedback_calibrated;
    float adc_at_zero;
    float adc_counts_per_mm;
};


constexpr MotorConfig MOTORS[] = {
    // ID/channel, stroke, feedback GPIO, calibrated, ADC at zero, counts/mm
    // Each finger is ordered near-palm first, far-from-palm second.
    {0, 30.0f, 34, false, 0.0f, 0.0f},  // 0: thumb near
    {1, 30.0f, 35, false, 0.0f, 0.0f},  // 1: thumb far
    {2, 50.0f, -1, false, 0.0f, 0.0f},  // 2: index near
    {3, 30.0f, -1, false, 0.0f, 0.0f},  // 3: index far
    {4, 50.0f, -1, false, 0.0f, 0.0f},  // 4: middle near
    {5, 30.0f, -1, false, 0.0f, 0.0f},  // 5: middle far
    {6, 50.0f, -1, false, 0.0f, 0.0f},  // 6: ring near
    {7, 30.0f, -1, false, 0.0f, 0.0f},  // 7: ring far
    {8, 30.0f, -1, false, 0.0f, 0.0f},  // 8: little near
    {9, 30.0f, -1, false, 0.0f, 0.0f},  // 9: little far
};


constexpr size_t MOTOR_COUNT =
    sizeof(MOTORS) / sizeof(MOTORS[0]);


static_assert(
    MOTOR_COUNT > 0 && MOTOR_COUNT <= 16,
    "One PCA9685 supports 1 through 16 configured motors"
);


constexpr bool valid_motor_config()
{
    for (size_t i = 0; i < MOTOR_COUNT; ++i) {

        if (MOTORS[i].pwm_channel >= 16) {
            return false;
        }

        if (MOTORS[i].stroke_mm != 30.0f &&
            MOTORS[i].stroke_mm != 50.0f) {
            return false;
        }

        if (MOTORS[i].feedback_calibrated &&
            MOTORS[i].adc_counts_per_mm == 0.0f) {
            return false;
        }

        for (size_t j = 0; j < i; ++j) {

            if (MOTORS[i].pwm_channel ==
                MOTORS[j].pwm_channel) {
                return false;
            }

            if (MOTORS[i].feedback_pin >= 0 &&
                MOTORS[i].feedback_pin ==
                MOTORS[j].feedback_pin) {
                return false;
            }
        }
    }

    return true;
}


static_assert(
    valid_motor_config(),
    "Invalid or duplicate motor configuration"
);


// ============================================================
// Motor runtime state
// ============================================================

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
// Wave configuration
// ============================================================

constexpr float PI = 3.14159265358979323846f;

// One complete temporal cycle takes 4 seconds.
constexpr float WAVE_PERIOD_S = 4.0f;

// 0.45 gives normalized range:
//
// 0.5 - 0.45 = 0.05
// 0.5 + 0.45 = 0.95
//
constexpr float WAVE_AMPLITUDE_RATIO = 0.45f;

// Update motor targets at 50 Hz.
constexpr int64_t WAVE_UPDATE_PERIOD_US = 20000;

// Before wave motion begins, all motors are first commanded to 0 mm.
constexpr float WAVE_ZERO_SETTLE_S = 1.0f;

// Wave amplitude ramps up/down smoothly.
constexpr float WAVE_RAMP_S = 1.0f;

// "wave" without a number defaults to 10 seconds.
constexpr float DEFAULT_WAVE_DURATION_S = 10.0f;

constexpr float MIN_WAVE_DURATION_S = 1.0f;
constexpr float MAX_WAVE_DURATION_S = 60.0f;


// ============================================================
// Hard-coded grasp configuration
// ============================================================

// Near-palm motors travel 10 mm. Far motors use their configured
// maximum (27 mm for a 30 mm actuator with the 3 mm margin).
constexpr float CLOSE_TARGET_MM[] = {
    10.0f, 27.0f,  // thumb
    10.0f, 27.0f,  // index
    10.0f, 27.0f,  // middle
    10.0f, 27.0f,  // ring
    10.0f, 27.0f,  // little
};

static_assert(
    sizeof(CLOSE_TARGET_MM) / sizeof(CLOSE_TARGET_MM[0]) == MOTOR_COUNT,
    "Close pose must contain one target per motor"
);

constexpr bool valid_close_pose()
{
    for (size_t id = 0; id < MOTOR_COUNT; ++id) {
        if (CLOSE_TARGET_MM[id] < 0.0f ||
            CLOSE_TARGET_MM[id] >
                MOTORS[id].stroke_mm - actuator::EXTENSION_MARGIN_MM) {
            return false;
        }
    }
    return true;
}

static_assert(valid_close_pose(), "Close pose exceeds a motor safety limit");

constexpr float GRASP_MOVE_DURATION_S = 3.0f;
constexpr int64_t GRASP_UPDATE_PERIOD_US = 20000;

// Each finger owns two adjacent motor IDs. During CLOSE and OPEN,
// the next finger begins 150 ms later to reduce simultaneous startup load.
constexpr size_t MOTORS_PER_FINGER = 2;
constexpr int64_t FINGER_STAGGER_US = 150000;

static_assert(
    MOTOR_COUNT % MOTORS_PER_FINGER == 0,
    "Each finger must contain exactly two motors"
);

constexpr size_t FINGER_COUNT =
    MOTOR_COUNT / MOTORS_PER_FINGER;


enum class GraspGoal {
    OPEN,
    CLOSE
};


struct GraspState {
    bool active = false;
    GraspGoal goal = GraspGoal::OPEN;
    int64_t start_us = 0;
    int64_t duration_us = 0;
    int64_t next_update_us = 0;
    float start_mm[MOTOR_COUNT] = {};
};


GraspState grasp_state;


// ============================================================
// Wave state machine
// ============================================================

enum class WavePhase {
    IDLE,
    ZERO_SETTLING,
    RUNNING
};


struct WaveState {

    WavePhase phase = WavePhase::IDLE;

    int64_t zero_start_us = 0;

    int64_t run_start_us = 0;

    int64_t duration_us = 0;

    int64_t next_update_us = 0;

    float duration_s = 0.0f;
};


WaveState wave_state;


bool wave_is_active()
{
    return wave_state.phase != WavePhase::IDLE;
}


// ============================================================
// Error helper
// ============================================================

bool check_ok(
    esp_err_t err,
    const char *operation)
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


    if (!check_ok(
            pca_write_register(
                device,
                PCA9685_MODE1,
                PCA9685_MODE_WAKE_AI),
            "Wake PCA9685")) {

        return false;
    }


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
// Set PCA9685 PWM count
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

        0x00,
        0x00,

        static_cast<uint8_t>(
            off_count & 0xFF
        ),

        static_cast<uint8_t>(
            (off_count >> 8) & 0x0F
        )
    };


    return i2c_master_transmit(
        device,
        data,
        sizeof(data),
        I2C_TIMEOUT_MS
    );
}


// ============================================================
// Set one motor position
// ============================================================

bool set_position_mm(
    i2c_master_dev_handle_t device,
    size_t motor_id,
    float requested_mm,
    bool verbose = true)
{
    if (motor_id >= MOTOR_COUNT ||
        !std::isfinite(requested_mm)) {

        return false;
    }


    const auto &config =
        MOTORS[motor_id];

    auto &state =
        motor_states[motor_id];


    const float target_mm =
        actuator::clamp_position(
            requested_mm,
            config.stroke_mm
        );


    if (verbose &&
        target_mm != requested_mm) {

        ESP_LOGW(
            TAG,

            "Motor %u: %.2f mm clamped to %.2f mm "
            "(range 0-%.2f mm)",

            static_cast<unsigned>(motor_id),

            requested_mm,

            target_mm,

            config.stroke_mm -
                actuator::EXTENSION_MARGIN_MM
        );
    }


    const float pulse =
        actuator::pulse_ms(
            target_mm,
            config.stroke_mm
        );


    const uint16_t count =
        actuator::pwm_count(
            pulse
        );


    if (!check_ok(
            set_pwm_count(
                device,
                config.pwm_channel,
                count),

            "Set L12 position")) {

        ESP_LOGE(
            TAG,
            "Motor %u command failed",
            static_cast<unsigned>(motor_id)
        );

        return false;
    }


    state.target_mm =
        target_mm;


    if (verbose) {

        state.report_due_us =
            esp_timer_get_time() +
            SETTLE_TIME_US;


        ESP_LOGI(
            TAG,

            "Motor %u channel=%u: "
            "Command %.2f mm -> %.3f ms -> %u counts",

            static_cast<unsigned>(motor_id),

            config.pwm_channel,

            target_mm,

            pulse,

            count
        );
    }


    return true;
}


// ============================================================
// Move all motors to zero
// ============================================================

bool move_all_to_zero(
    i2c_master_dev_handle_t device,
    bool verbose)
{
    for (size_t id = 0;
         id < MOTOR_COUNT;
         ++id) {

        if (!set_position_mm(
                device,
                id,
                0.0f,
                verbose)) {

            return false;
        }
    }

    return true;
}


// ============================================================
// Hard-coded grasp motion
// ============================================================

const char *grasp_goal_name(
    GraspGoal goal)
{
    return goal == GraspGoal::CLOSE
        ? "CLOSE"
        : "OPEN";
}


bool grasp_is_active()
{
    return grasp_state.active;
}


void cancel_grasp()
{
    if (!grasp_is_active()) {
        return;
    }


    ESP_LOGI(
        TAG,
        "%s interrupted",
        grasp_goal_name(grasp_state.goal)
    );


    grasp_state.active = false;
}


float grasp_target_mm(
    GraspGoal goal,
    size_t motor_id)
{
    return goal == GraspGoal::CLOSE
        ? CLOSE_TARGET_MM[motor_id]
        : 0.0f;
}


int64_t grasp_start_delay_us(
    size_t motor_id)
{
    const size_t finger_id =
        motor_id / MOTORS_PER_FINGER;


    return static_cast<int64_t>(finger_id) *
        FINGER_STAGGER_US;
}


int64_t grasp_total_duration_us(
    int64_t movement_duration_us)
{
    return movement_duration_us +
        static_cast<int64_t>(FINGER_COUNT - 1) *
        FINGER_STAGGER_US;
}


void start_grasp(
    GraspGoal goal)
{
    if (grasp_is_active() &&
        grasp_state.goal == goal) {

        ESP_LOGI(
            TAG,
            "%s already in progress",
            grasp_goal_name(goal)
        );

        return;
    }


    bool already_at_target = true;


    for (size_t id = 0;
         id < MOTOR_COUNT;
         ++id) {

        const float target_mm =
            grasp_target_mm(goal, id);


        if (std::fabs(
                motor_states[id].target_mm -
                target_mm) > 0.001f) {

            already_at_target = false;
        }
    }


    if (already_at_target) {

        grasp_state.active = false;

        ESP_LOGI(
            TAG,
            "%s already at target",
            grasp_goal_name(goal)
        );

        return;
    }


    const int64_t now =
        esp_timer_get_time();


    grasp_state.active = true;
    grasp_state.goal = goal;
    grasp_state.start_us = now;
    grasp_state.duration_us =
        static_cast<int64_t>(
            GRASP_MOVE_DURATION_S *
            1000000.0f
        );
    grasp_state.next_update_us = now;


    for (size_t id = 0;
         id < MOTOR_COUNT;
         ++id) {

        grasp_state.start_mm[id] =
            motor_states[id].target_mm;
    }


    const float total_duration_s =
        grasp_total_duration_us(
            grasp_state.duration_us
        ) /
        1000000.0f;


    ESP_LOGI(
        TAG,
        "%s started: %.2f s per finger, "
        "%lld ms stagger, %.2f s total",
        grasp_goal_name(goal),
        GRASP_MOVE_DURATION_S,
        static_cast<long long>(
            FINGER_STAGGER_US /
            1000
        ),
        total_duration_s
    );


    for (size_t id = 0;
         id < MOTOR_COUNT;
         ++id) {

        ESP_LOGI(
            TAG,
            "  motor %u: %.2f -> %.2f mm, delay=%lld ms",
            static_cast<unsigned>(id),
            grasp_state.start_mm[id],
            grasp_target_mm(goal, id),
            static_cast<long long>(
                grasp_start_delay_us(id) /
                1000
            )
        );
    }
}


void update_grasp(
    i2c_master_dev_handle_t device,
    int64_t now)
{
    if (!grasp_is_active() ||
        now < grasp_state.next_update_us) {

        return;
    }


    const int64_t elapsed_us =
        now - grasp_state.start_us;


    for (size_t id = 0;
         id < MOTOR_COUNT;
         ++id) {


        const int64_t start_delay_us =
            grasp_start_delay_us(
                id
            );


        if (elapsed_us < start_delay_us) {
            continue;
        }


        const int64_t local_elapsed_us =
            elapsed_us - start_delay_us;


        float progress =
            static_cast<float>(local_elapsed_us) /
            static_cast<float>(grasp_state.duration_us);


        if (progress > 1.0f) {
            progress = 1.0f;
        }


        const float blend =
            actuator::smoothstep01(progress);


        const float final_mm =
            grasp_target_mm(
                grasp_state.goal,
                id
            );


        const float target_mm =
            grasp_state.start_mm[id]
            +
            blend *
            (final_mm - grasp_state.start_mm[id]);


        if (!set_position_mm(
                device,
                id,
                target_mm,
                false)) {

            ESP_LOGE(
                TAG,
                "%s motor %u update failed",
                grasp_goal_name(grasp_state.goal),
                static_cast<unsigned>(id)
            );


            grasp_state.active = false;


            return;
        }
    }


    const int64_t total_duration_us =
        grasp_total_duration_us(
            grasp_state.duration_us
        );


    if (elapsed_us >= total_duration_us) {

        ESP_LOGI(
            TAG,
            "%s complete",
            grasp_goal_name(grasp_state.goal)
        );


        grasp_state.active = false;


        return;
    }


    grasp_state.next_update_us =
        now + GRASP_UPDATE_PERIOD_US;
}


// ============================================================
// Feedback
// ============================================================

bool read_average_adc(
    adc_oneshot_unit_handle_t adc_handle,
    adc_channel_t adc_channel,
    int &average_raw)
{
    int sum = 0;


    for (int i = 0;
         i < ADC_AVERAGE_SAMPLES;
         ++i) {

        int sample = 0;


        const esp_err_t err =
            adc_oneshot_read(
                adc_handle,
                adc_channel,
                &sample
            );


        if (!check_ok(
                err,
                "Read L12 feedback ADC")) {

            return false;
        }


        sum += sample;
    }


    average_raw =
        sum / ADC_AVERAGE_SAMPLES;


    return true;
}


// ============================================================
// Initialize feedback
// ============================================================

bool initialize_feedback(
    adc_oneshot_unit_handle_t &adc_handle)
{
    for (size_t id = 0;
         id < MOTOR_COUNT;
         ++id) {


        const auto &config =
            MOTORS[id];


        if (config.feedback_pin < 0) {
            continue;
        }


        adc_unit_t unit;


        auto &state =
            motor_states[id];


        if (!check_ok(
                adc_oneshot_io_to_channel(
                    config.feedback_pin,
                    &unit,
                    &state.adc_channel),

                "Map feedback GPIO")) {

            return false;
        }


        if (unit != ADC_UNIT_1) {

            ESP_LOGE(
                TAG,

                "Motor %u: GPIO%d must use ADC1; "
                "use -1 for no feedback",

                static_cast<unsigned>(id),

                config.feedback_pin
            );

            return false;
        }


        if (adc_handle == nullptr) {

            adc_oneshot_unit_init_cfg_t init = {};

            init.unit_id =
                ADC_UNIT_1;

            init.ulp_mode =
                ADC_ULP_MODE_DISABLE;


            if (!check_ok(
                    adc_oneshot_new_unit(
                        &init,
                        &adc_handle),

                    "Initialize ADC1")) {

                return false;
            }
        }


        adc_oneshot_chan_cfg_t channel = {};

        channel.atten =
            ADC_ATTEN_DB_12;

        channel.bitwidth =
            ADC_BITWIDTH_DEFAULT;


        if (!check_ok(
                adc_oneshot_config_channel(
                    adc_handle,
                    state.adc_channel,
                    &channel),

                "Configure feedback channel")) {

            return false;
        }
    }


    return true;
}


// ============================================================
// Sample feedback
// ============================================================

void sample_feedback(
    adc_oneshot_unit_handle_t adc_handle)
{
    for (size_t id = 0;
         id < MOTOR_COUNT;
         ++id) {


        const auto &config =
            MOTORS[id];

        auto &state =
            motor_states[id];


        if (config.feedback_pin < 0) {
            continue;
        }


        state.feedback_ok =
            read_average_adc(
                adc_handle,
                state.adc_channel,
                state.adc_raw
            );


        if (!state.feedback_ok) {

            ESP_LOGW(
                TAG,

                "Motor %u feedback unavailable",

                static_cast<unsigned>(id)
            );


            state.previous_time_us = 0;

            continue;
        }


        if (!config.feedback_calibrated) {
            continue;
        }


        const float position =
            (
                config.adc_at_zero -
                state.adc_raw
            )
            /
            config.adc_counts_per_mm;


        const int64_t now =
            esp_timer_get_time();


        if (state.previous_time_us > 0 &&
            now > state.previous_time_us) {


            const float dt =
                (
                    now -
                    state.previous_time_us
                )
                /
                1000000.0f;


            const float velocity =
                (
                    position -
                    state.previous_position_mm
                )
                /
                dt;


            state.filtered_velocity_mm_s =

                VELOCITY_ALPHA *
                velocity

                +

                (1.0f - VELOCITY_ALPHA) *
                state.filtered_velocity_mm_s;
        }

        else {

            state.filtered_velocity_mm_s =
                0.0f;
        }


        state.previous_position_mm =
            position;


        state.previous_time_us =
            now;
    }
}


// ============================================================
// Feedback report
// ============================================================

void report_feedback(
    size_t id,
    const char *label)
{
    const auto &config =
        MOTORS[id];

    const auto &state =
        motor_states[id];


    if (config.feedback_pin < 0) {

        ESP_LOGI(
            TAG,

            "%s: motor=%u command=%.2f mm "
            "feedback=disabled",

            label,

            static_cast<unsigned>(id),

            state.target_mm
        );
    }

    else if (!state.feedback_ok) {

        ESP_LOGW(
            TAG,

            "%s: motor=%u command=%.2f mm "
            "feedback=error",

            label,

            static_cast<unsigned>(id),

            state.target_mm
        );
    }

    else if (!config.feedback_calibrated) {

        ESP_LOGI(
            TAG,

            "%s: motor=%u command=%.2f mm "
            "adc=%d (uncalibrated)",

            label,

            static_cast<unsigned>(id),

            state.target_mm,

            state.adc_raw
        );
    }

    else {

        ESP_LOGI(
            TAG,

            "%s: motor=%u command=%.2f mm "
            "adc=%d pos=%.2f mm vel=%.2f mm/s",

            label,

            static_cast<unsigned>(id),

            state.target_mm,

            state.adc_raw,

            state.previous_position_mm,

            state.filtered_velocity_mm_s
        );
    }
}


// ============================================================
// Wave command parser
// ============================================================
//
// Accepted:
//
// wave
// wave,15
// wave,2.5
// WAVE,10
//
// Returns:
//
//  0 : not a wave command
//  1 : valid wave command
// -1 : looks like wave, but invalid
//

int parse_wave_command(
    const char *text,
    float &duration_s)
{
    const char *p = text;


    while (*p &&
           std::isspace(
               static_cast<unsigned char>(*p))) {

        ++p;
    }


    const char word[] = "wave";


    for (int i = 0; i < 4; ++i) {

        if (p[i] == '\0') {
            return 0;
        }


        if (std::tolower(
                static_cast<unsigned char>(p[i]))
            != word[i]) {

            return 0;
        }
    }


    p += 4;


    while (*p &&
           std::isspace(
               static_cast<unsigned char>(*p))) {

        ++p;
    }


    // "wave"
    if (*p == '\0') {

        duration_s =
            DEFAULT_WAVE_DURATION_S;

        return 1;
    }


    // Must be wave,<seconds>
    if (*p != ',') {
        return -1;
    }


    ++p;


    while (*p &&
           std::isspace(
               static_cast<unsigned char>(*p))) {

        ++p;
    }


    if (*p == '\0') {
        return -1;
    }


    char *end = nullptr;


    const float value =
        std::strtof(
            p,
            &end
        );


    if (end == p ||
        !std::isfinite(value)) {

        return -1;
    }


    while (*end &&
           std::isspace(
               static_cast<unsigned char>(*end))) {

        ++end;
    }


    if (*end != '\0') {
        return -1;
    }


    if (value < MIN_WAVE_DURATION_S ||
        value > MAX_WAVE_DURATION_S) {

        return -1;
    }


    duration_s = value;

    return 1;
}


// ============================================================
// Start wave
// ============================================================

bool start_wave(
    i2c_master_dev_handle_t device,
    float duration_s)
{
    if (wave_is_active()) {

        ESP_LOGI(
            TAG,
            "Restarting active WAVE"
        );
    }


    wave_state.phase =
        WavePhase::IDLE;


    ESP_LOGI(
        TAG,

        "WAVE requested: %.2f s",

        duration_s
    );


    ESP_LOGI(
        TAG,

        "Moving all motors to 0 mm before WAVE"
    );


    if (!move_all_to_zero(
            device,
            true)) {

        ESP_LOGE(
            TAG,
            "Failed to move motors to zero"
        );

        return false;
    }


    const int64_t now =
        esp_timer_get_time();


    wave_state.phase =
        WavePhase::ZERO_SETTLING;


    wave_state.zero_start_us =
        now;


    wave_state.duration_s =
        duration_s;


    wave_state.duration_us =
        static_cast<int64_t>(
            duration_s *
            1000000.0f
        );


    wave_state.next_update_us =
        0;


    ESP_LOGI(
        TAG,

        "WAVE zero settling: %.2f s",

        WAVE_ZERO_SETTLE_S
    );


    return true;
}


// ============================================================
// Cancel wave
// ============================================================

void cancel_wave()
{
    if (!wave_is_active()) {
        return;
    }


    if (wave_state.phase ==
        WavePhase::RUNNING) {


        const float elapsed_s =
            (
                esp_timer_get_time() -
                wave_state.run_start_us
            )
            /
            1000000.0f;


        ESP_LOGI(
            TAG,

            "WAVE interrupted at %.2f s",

            elapsed_s
        );
    }

    else {

        ESP_LOGI(
            TAG,

            "WAVE interrupted during zero settling"
        );
    }


    wave_state.phase =
        WavePhase::IDLE;
}


// ============================================================
// Update wave state machine
// ============================================================

void update_wave(
    i2c_master_dev_handle_t device,
    int64_t now)
{
    // --------------------------------------------------------
    // IDLE
    // --------------------------------------------------------

    if (wave_state.phase ==
        WavePhase::IDLE) {

        return;
    }


    // --------------------------------------------------------
    // Waiting at zero
    // --------------------------------------------------------

    if (wave_state.phase ==
        WavePhase::ZERO_SETTLING) {


        const int64_t zero_settle_us =
            static_cast<int64_t>(
                WAVE_ZERO_SETTLE_S *
                1000000.0f
            );


        if (now -
            wave_state.zero_start_us
            >= zero_settle_us) {


            wave_state.phase =
                WavePhase::RUNNING;


            wave_state.run_start_us =
                now;


            wave_state.next_update_us =
                now;


            ESP_LOGI(
                TAG,

                "WAVE started: duration=%.2f s, "
                "period=%.2f s, amplitude=%.2f",

                wave_state.duration_s,

                WAVE_PERIOD_S,

                WAVE_AMPLITUDE_RATIO
            );
        }


        return;
    }


    // --------------------------------------------------------
    // Wave running
    // --------------------------------------------------------

    if (wave_state.phase !=
        WavePhase::RUNNING) {

        return;
    }


    const int64_t elapsed_us =
        now -
        wave_state.run_start_us;


    // --------------------------------------------------------
    // Wave complete
    // --------------------------------------------------------

    if (elapsed_us >=
        wave_state.duration_us) {


        wave_state.phase =
            WavePhase::IDLE;


        ESP_LOGI(
            TAG,

            "WAVE finished after %.2f s",

            wave_state.duration_s
        );


        ESP_LOGI(
            TAG,

            "Returning all motors to 0 mm"
        );


        move_all_to_zero(
            device,
            true
        );


        return;
    }


    // --------------------------------------------------------
    // Only update targets at 50 Hz
    // --------------------------------------------------------

    if (now <
        wave_state.next_update_us) {

        return;
    }


    wave_state.next_update_us =
        now +
        WAVE_UPDATE_PERIOD_US;


    const float elapsed_s =
        elapsed_us /
        1000000.0f;


    // ========================================================
    // Smooth amplitude envelope
    // ========================================================
    //
    // For normal long waves:
    //
    // 0 -> 1 during first second
    // stays at 1
    // 1 -> 0 during final second
    //
    // For very short wave durations, ramp time is automatically
    // reduced to half the requested wave duration.
    // ========================================================

    float ramp_s =
        WAVE_RAMP_S;


    if (wave_state.duration_s <
        2.0f * ramp_s) {

        ramp_s =
            wave_state.duration_s *
            0.5f;
    }


    float ramp_up = 1.0f;


    if (ramp_s > 0.0f &&
        elapsed_s < ramp_s) {

        ramp_up =
            elapsed_s /
            ramp_s;
    }


    const float remaining_s =
        wave_state.duration_s -
        elapsed_s;


    float ramp_down = 1.0f;


    if (ramp_s > 0.0f &&
        remaining_s < ramp_s) {

        ramp_down =
            remaining_s /
            ramp_s;
    }


    float envelope =
        std::fmin(
            ramp_up,
            ramp_down
        );


    if (envelope < 0.0f) {
        envelope = 0.0f;
    }


    if (envelope > 1.0f) {
        envelope = 1.0f;
    }


    // ========================================================
    // Traveling sine wave
    // ========================================================

    const float omega =
        2.0f *
        PI /
        WAVE_PERIOD_S;


    const float phase_step =
        2.0f *
        PI /
        static_cast<float>(
            MOTOR_COUNT
        );


    for (size_t id = 0;
         id < MOTOR_COUNT;
         ++id) {


        const auto &config =
            MOTORS[id];


        // Safe usable length:
        //
        // 50 mm actuator -> 47 mm
        // 30 mm actuator -> 27 mm

        const float safe_length_mm =
            config.stroke_mm -
            actuator::EXTENSION_MARGIN_MM;


        // The minus sign makes the wave travel:
        //
        // 0 -> 1 -> 2 -> ... -> 9

        const float phase =
            omega *
            elapsed_s

            -

            phase_step *
            static_cast<float>(id);


        // Full-amplitude normalized range:
        //
        // 0.05 -> 0.95

        const float normalized_position =
            0.5f

            +

            WAVE_AMPLITUDE_RATIO *
            std::sin(phase);


        // envelope causes:
        //
        // start: all motors at 0
        // ramp up smoothly
        // run traveling wave
        // ramp down smoothly
        // end: all motors at 0

        const float target_mm =
            envelope *
            safe_length_mm *
            normalized_position;


        // Silent command during wave.
        //
        // Otherwise 10 motors x 50 Hz would produce
        // 500 log messages per second.

        if (!set_position_mm(
                device,
                id,
                target_mm,
                false)) {


            ESP_LOGE(
                TAG,

                "WAVE motor %u update failed",

                static_cast<unsigned>(id)
            );


            wave_state.phase =
                WavePhase::IDLE;


            return;
        }
    }
}


// ============================================================
// Dispatch one complete serial command
// ============================================================

void handle_serial_command(
    i2c_master_dev_handle_t device,
    const char *text)
{
    if (actuator::matches_command_word(
            text,
            "close")) {

        if (wave_is_active()) {
            cancel_wave();
        }

        start_grasp(
            GraspGoal::CLOSE
        );

        return;
    }


    if (actuator::matches_command_word(
            text,
            "open")) {

        if (wave_is_active()) {
            cancel_wave();
        }

        start_grasp(
            GraspGoal::OPEN
        );

        return;
    }


    float wave_duration_s = 0.0f;


    const int wave_result =
        parse_wave_command(
            text,
            wave_duration_s
        );


    if (wave_result == 1) {

        if (grasp_is_active()) {
            cancel_grasp();
        }

        start_wave(
            device,
            wave_duration_s
        );

        return;
    }


    if (wave_result == -1) {

        ESP_LOGW(
            TAG,
            "Invalid wave command. "
            "Use wave or wave,<seconds>; "
            "range %.0f-%.0f s",
            MIN_WAVE_DURATION_S,
            MAX_WAVE_DURATION_S
        );

        return;
    }


    actuator::Command command = {};


    if (!actuator::parse_command(
            text,
            MOTOR_COUNT,
            command)) {

        ESP_LOGW(
            TAG,
            "Invalid command: %s. Use close, open, "
            "P,<position_mm>,<motor_id>, or wave,<seconds>",
            text
        );

        return;
    }


    bool automatic_motion_interrupted = false;


    if (wave_is_active()) {
        cancel_wave();
        automatic_motion_interrupted = true;
    }


    if (grasp_is_active()) {
        cancel_grasp();
        automatic_motion_interrupted = true;
    }


    if (automatic_motion_interrupted) {

        ESP_LOGI(
            TAG,
            "Manual P command now has control"
        );
    }


    set_position_mm(
        device,
        command.motor_id,
        command.position_mm,
        true
    );
}


} // namespace



// ============================================================
// Main
// ============================================================

extern "C" void app_main()
{
    ESP_LOGI(
        TAG,
        "Exoskeleton controller starting!"
    );


    // ========================================================
    // I2C bus
    // ========================================================

    i2c_master_bus_config_t bus_config = {};


    bus_config.i2c_port =
        I2C_NUM_0;


    bus_config.sda_io_num =
        I2C_SDA_PIN;


    bus_config.scl_io_num =
        I2C_SCL_PIN;


    bus_config.clk_source =
        I2C_CLK_SRC_DEFAULT;


    bus_config.glitch_ignore_cnt =
        7;


    bus_config.flags.enable_internal_pullup =
        true;


    i2c_master_bus_handle_t bus_handle =
        nullptr;


    if (!check_ok(
            i2c_new_master_bus(
                &bus_config,
                &bus_handle),

            "Create I2C bus")) {

        return;
    }


    // ========================================================
    // PCA9685
    // ========================================================

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


    i2c_master_dev_handle_t pca_handle =
        nullptr;


    if (!check_ok(
            i2c_master_bus_add_device(
                bus_handle,
                &device_config,
                &pca_handle),

            "Add PCA9685 device")) {

        return;
    }


    if (!initialize_pca9685(
            pca_handle)) {

        return;
    }


    // ========================================================
    // ADC feedback
    // ========================================================

    adc_oneshot_unit_handle_t adc_handle =
        nullptr;


    if (!initialize_feedback(
            adc_handle)) {

        return;
    }


    // ========================================================
    // UART0
    // ========================================================

    uart_config_t uart_config = {};


    uart_config.baud_rate =
        115200;


    uart_config.data_bits =
        UART_DATA_8_BITS;


    uart_config.parity =
        UART_PARITY_DISABLE;


    uart_config.stop_bits =
        UART_STOP_BITS_1;


    uart_config.flow_ctrl =
        UART_HW_FLOWCTRL_DISABLE;


    uart_config.source_clk =
        UART_SCLK_DEFAULT;


    if (!check_ok(
            uart_param_config(
                UART_NUM_0,
                &uart_config),

            "Configure UART0")

        ||

        !check_ok(
            uart_driver_install(
                UART_NUM_0,
                1024,
                0,
                0,
                nullptr,
                0),

            "Install UART0 driver")) {

        return;
    }


    // ========================================================
    // Startup: all motors to open pose
    // ========================================================

    ESP_LOGI(
        TAG,

        "Configured %u motors",

        static_cast<unsigned>(
            MOTOR_COUNT
        )
    );


    for (size_t id = 0;
         id < MOTOR_COUNT;
         ++id) {


        const auto &config =
            MOTORS[id];


        ESP_LOGI(
            TAG,

            "Motor %u: channel=%u "
            "stroke=%.0f mm "
            "limit=%.0f mm "
            "feedback=GPIO%d",

            static_cast<unsigned>(id),

            config.pwm_channel,

            config.stroke_mm,

            config.stroke_mm -
            actuator::EXTENSION_MARGIN_MM,

            config.feedback_pin
        );
    }


    ESP_LOGI(
        TAG,
        "Moving all motors to OPEN pose (0 mm)"
    );


    if (!move_all_to_zero(
            pca_handle,
            true)) {

        return;
    }


    // ========================================================
    // Command information
    // ========================================================

    ESP_LOGI(
        TAG,

        "Commands:"
    );


    ESP_LOGI(
        TAG,

        "  close                       150 ms staggered grasp"
    );


    ESP_LOGI(
        TAG,

        "  open                        150 ms staggered return"
    );


    ESP_LOGI(
        TAG,

        "  P,<position_mm>,<motor_id>   IDs 0-%u",

        static_cast<unsigned>(
            MOTOR_COUNT - 1
        )
    );


    ESP_LOGI(
        TAG,

        "  wave                        default %.0f s",

        DEFAULT_WAVE_DURATION_S
    );


    ESP_LOGI(
        TAG,

        "  wave,<seconds>              range %.0f-%.0f s",

        MIN_WAVE_DURATION_S,

        MAX_WAVE_DURATION_S
    );


    // ========================================================
    // UART command buffer
    // ========================================================

    char command_buffer[64] = {};

    size_t command_length = 0;

    bool line_overflow = false;

    int startup_samples_remaining = 30;

    int64_t next_sample_us =
        esp_timer_get_time();


    // ========================================================
    // Main loop
    // ========================================================

    while (true) {


        // ====================================================
        // UART input
        // ====================================================

        uint8_t incoming[64];


        const int received =
            uart_read_bytes(
                UART_NUM_0,
                incoming,
                sizeof(incoming),
                0
            );


        for (int i = 0;
             i < received;
             ++i) {


            const int c =
                incoming[i];


            // ------------------------------------------------
            // Enter
            // ------------------------------------------------

            if (c == '\n' ||
                c == '\r') {


                if (command_length > 0 ||
                    line_overflow) {


                    putchar('\n');
                    fflush(stdout);


                    command_buffer[
                        command_length
                    ] = '\0';


                    if (line_overflow) {


                        ESP_LOGW(
                            TAG,

                            "Command too long; "
                            "entire line rejected"
                        );
                    }

                    else {

                        handle_serial_command(
                            pca_handle,
                            command_buffer
                        );
                    }


                    command_length = 0;

                    line_overflow = false;
                }
            }


            // ------------------------------------------------
            // Backspace
            // ------------------------------------------------

            else if (
                c == '\b' ||
                c == 0x7F) {


                if (!line_overflow &&
                    command_length > 0) {


                    --command_length;


                    fputs(
                        "\b \b",
                        stdout
                    );


                    fflush(stdout);
                }
            }


            // ------------------------------------------------
            // Printable characters
            // ------------------------------------------------

            else if (
                c >= 0x20 &&
                c <= 0x7E) {


                if (!line_overflow &&
                    command_length <
                    sizeof(command_buffer) - 1) {


                    command_buffer[
                        command_length++
                    ] =
                        static_cast<char>(c);


                    putchar(c);

                    fflush(stdout);
                }

                else {

                    line_overflow = true;
                }
            }


            else {

                line_overflow = true;
            }
        }


        // ====================================================
        // Timing
        // ====================================================

        const int64_t now =
            esp_timer_get_time();


        // ====================================================
        // Non-blocking automatic motion updates
        // ====================================================

        update_wave(
            pca_handle,
            now
        );


        update_grasp(
            pca_handle,
            now
        );


        // ====================================================
        // Feedback sampling
        // ====================================================

        if (now >=
            next_sample_us) {


            sample_feedback(
                adc_handle
            );


            next_sample_us =
                esp_timer_get_time() +
                SAMPLE_PERIOD_US;


            if (startup_samples_remaining > 0) {


                // Avoid destroying the line being typed.
                if (command_length == 0 &&
                    !line_overflow) {


                    for (size_t id = 0;
                         id < MOTOR_COUNT;
                         ++id) {


                        report_feedback(
                            id,
                            "SAMPLE"
                        );
                    }
                }


                --startup_samples_remaining;
            }
        }


        // ====================================================
        // Delayed reports
        // ====================================================

        if (command_length == 0 &&
            !line_overflow) {


            for (size_t id = 0;
                 id < MOTOR_COUNT;
                 ++id) {


                auto &state =
                    motor_states[id];


                if (state.report_due_us > 0 &&
                    now >= state.report_due_us) {


                    report_feedback(
                        id,
                        "CAL"
                    );


                    state.report_due_us = 0;
                }
            }
        }


        vTaskDelay(
            pdMS_TO_TICKS(10)
        );
    }
}
