#include "config.h"
#include "control.h"
#include "crsf_rc.h"
#include "driver/ledc.h"

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

    // get RC inputs
    int16_t rx_throttle = crsf_get_channel(0);
    int16_t rx_roll     = crsf_get_channel(1);
    int16_t rx_pitch    = crsf_get_channel(2);
    int16_t rx_yaw      = crsf_get_channel(3);

    // map rx value to duty in microseconds (172-1811 -> 1000-2000us)
    uint16_t throttle_us = map_crsf(rx_throttle, 172, 1811, 1000, 2000);
    uint16_t roll_us     = map_crsf(rx_roll,     172, 1811, 1000, 2000);
    uint16_t pitch_us    = map_crsf(rx_pitch,    172, 1811, 1000, 2000);
    uint16_t yaw_us      = map_crsf(rx_yaw,      172, 1811, 1000, 2000);

    // write to outputs
    write_us(EDF_CHANNEL, throttle_us);
    write_us(SERVO1_CHANNEL, roll_us);
    write_us(SERVO2_CHANNEL, pitch_us);
    write_us(SERVO3_CHANNEL, yaw_us);
    write_us(SERVO4_CHANNEL, 1500); // neutral for now
}