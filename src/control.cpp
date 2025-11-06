#include "config.h"
#include "control.h"
#include "crsf_rc.h"
#include "driver/ledc.h"
#include "driver/rmt.h"

static bool armed = false;

static void setup_pwm(uint8_t channel, uint8_t pin, uint32_t freq)
{
    ledc_timer_config_t timer {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = (ledc_timer_bit_t)PWM_RES_BITS,
        .timer_num = (ledc_timer_t)channel,
        .freq_hz = freq,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t ledc_channel {
        .gpio_num = pin,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = (ledc_channel_t)channel,
        .timer_sel = (ledc_timer_t)channel,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&ledc_channel);
}

void motor_init()
{
    setup_pwm(EDF_CHANNEL, EDF_PIN, PWM_FREQ);
    setup_pwm(SERVO1_CHANNEL, SERVO1_PIN, PWM_FREQ);
    setup_pwm(SERVO2_CHANNEL, SERVO2_PIN, PWM_FREQ);
    setup_pwm(SERVO3_CHANNEL, SERVO3_PIN, PWM_FREQ);
    setup_pwm(SERVO4_CHANNEL, SERVO4_PIN, PWM_FREQ);

    ESP_LOGI("CHANNELS", "PWM channels initialized");
}

static void write_us(uint8_t channel, uint16_t usec)
{
    uint32_t max_duty = PWM_MAX_DUTY;
    uint32_t duty = (uint32_t)((float)usec / 20000.0f * max_duty); // 20ms period
    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)channel);
}

static int map_crsf(int value, int in_min, int in_max, int out_min, int out_max)
{
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void motor_arm(bool arm)
{
    armed = arm;
    if (!armed) {
        // kill outputs
        for (int i = 0; i < 5; i++) write_us(i, SERVO_MIN_US);
        ESP_LOGW(TAG, "Motors disarmed");
    } else {
        ESP_LOGI(TAG, "Motors armed");
    }
}

void motor_update_from_crsf()
{
    if (!armed) return;

    // get RC inputs from channels
    int16_t rx_throttle = crsf_get_channel(0);
    int16_t rx_roll     = crsf_get_channel(1);
    int16_t rx_pitch    = crsf_get_channel(2);
    int16_t rx_yaw      = crsf_get_channel(3);

    // normalize digital values
    auto norm = [](int16_t val)
    {
        return (float)(val - 992) / 820.0f; // center at midpoint (992? for 172-1811)
    };
    float roll = norm(rx_roll);
    float pitch = norm(rx_pitch);
    float yaw = norm(rx_yaw);
    float throttle = (float)(rx_throttle - 172) / (1811 - 172);

    // EDF PWM
    uint16_t throttle_us = (uint16_t)(1000 + throttle*1000);
    write_us(EDF_CHANNEL, throttle_us);

    // servo mixing
    const int16_t SERVO_RANGE = 250;
    uint16_t base_us = 1500;

    uint16_t servo1_us = base_us + (pitch + roll - yaw) * SERVO_RANGE;
    uint16_t servo2_us = base_us + (pitch - roll - yaw) * SERVO_RANGE;
    uint16_t servo3_us = base_us + (pitch - roll - yaw) * SERVO_RANGE;
    uint16_t servo4_us = base_us + (pitch + roll + yaw) * SERVO_RANGE;

    // clamping for safe limits
    auto clamp = [](uint16_t val)
    {
        return (uint16_t)std::max(1000, std::min(2000, (int)val));
    };
    servo1_us = clamp(servo1_us);
    servo2_us = clamp(servo2_us);
    servo3_us = clamp(servo3_us);
    servo4_us = clamp(servo4_us);


    // write to outputs
    write_us(SERVO1_CHANNEL, servo1_us);
    write_us(SERVO2_CHANNEL, servo2_us);
    write_us(SERVO3_CHANNEL, servo3_us);
    write_us(SERVO4_CHANNEL, servo4_us); 
}

static uint16_t readPWM_us(int pin)
{
    // blocking for simplicity; use RMT for real control loops
    return pulseIn(pin, HIGH, 25000);  // 25ms timeout, returns pulse width in µs
}

void motor_update_from_pwm()
{
    if (!armed) return;

    uint16_t rx_throttle = readPWM_us(RX_THROTTLE_PIN);
    uint16_t rx_roll     = readPWM_us(RX_ROLL_PIN);
    uint16_t rx_pitch    = readPWM_us(RX_PITCH_PIN);
    uint16_t rx_yaw      = readPWM_us(RX_YAW_PIN);

    auto norm = [](uint16_t val)
    {
        return ((float)val - 1500.0f) / 500.0f; // -1.0 to +1.0
    };
    float roll = norm(rx_roll);
    float pitch = norm(rx_pitch);
    float yaw = norm(rx_yaw);
    float throttle = ((float)rx_throttle - 1000.0f) / 1000.0f; // 0.0 to 1.0

    uint16_t throttle_us = (uint16_t)(1000 + throttle * 1000);
    write_us(EDF_CHANNEL, throttle_us);

    const int16_t SERVO_RANGE = 250;
    uint16_t base_us = 1500;

    uint16_t servo1_us = base_us + (pitch + roll - yaw) * SERVO_RANGE;
    uint16_t servo2_us = base_us + (pitch - roll - yaw) * SERVO_RANGE;
    uint16_t servo3_us = base_us + (pitch - roll - yaw) * SERVO_RANGE;
    uint16_t servo4_us = base_us + (pitch + roll + yaw) * SERVO_RANGE;

    auto clamp = [](uint16_t val)
    {
        return (uint16_t)std::max(1000, std::min(2000, (int)val));
    };
    servo1_us = clamp(servo1_us);
    servo2_us = clamp(servo2_us);
    servo3_us = clamp(servo3_us);
    servo4_us = clamp(servo4_us);

    write_us(SERVO1_CHANNEL, servo1_us);
    write_us(SERVO2_CHANNEL, servo2_us);
    write_us(SERVO3_CHANNEL, servo3_us);
    write_us(SERVO4_CHANNEL, servo4_us);
}

void motorUpdate()
{
    if (!armed) return;

#if USE_CRSF
    motor_update_from_crsf();
#else
    motor_update_from_pwm();
#endif
}

void setupPWMOutput()
{
    // configure timer
    ledcSetup(0, PWM_FREQ, PWM_RES_BITS);

    // attach pins to channels
    ledcAttachPin(EDF_PIN, EDF_CHANNEL);
    ledcAttachPin(SERVO1_PIN, SERVO1_CHANNEL);
    ledcAttachPin(SERVO2_PIN, SERVO2_CHANNEL);
    ledcAttachPin(SERVO3_PIN, SERVO3_CHANNEL);
    ledcAttachPin(SERVO4_PIN, SERVO4_CHANNEL);

    // initialize all to neutral
    write_us(0, 1000); // ESC off?
    for (int ch = 1; ch <= 4; ch++)
    {
            write_us(ch, 1500);
    }
}