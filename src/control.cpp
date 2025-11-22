#include "config.h"
#include "control.h"
#include "crsf_rc.h"
#include "driver/ledc.h"
#include "driver/rmt.h"

static bool armed = false;

void setup_pwm_timer()
{
    ledc_timer_config_t timer {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_14_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    esp_err_t err = ledc_timer_config(&timer);
    if (err != ESP_OK) {
        Serial.printf("Timer config failed: %d\n", err);
    }
}

void setup_pwm_channel(uint8_t channel, uint8_t pin)
{
    ledc_channel_config_t ledc_channel {
        .gpio_num = pin,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = (ledc_channel_t)channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };
    esp_err_t err = ledc_channel_config(&ledc_channel);
    if (err != ESP_OK) {
        Serial.printf("Channel %d config failed: %d\n", channel, err);
    }
}

void motor_init()
{
    setup_pwm_timer();
    setup_pwm_channel(EDF_CHANNEL, EDF_PIN);
    setup_pwm_channel(SERVO1_CHANNEL, SERVO1_PIN);
    setup_pwm_channel(SERVO2_CHANNEL, SERVO2_PIN);
    setup_pwm_channel(SERVO3_CHANNEL, SERVO3_PIN);
    setup_pwm_channel(SERVO4_CHANNEL, SERVO4_PIN);

    ESP_LOGI("CHANNELS", "PWM channels initialized");
}

void write_us(uint8_t channel, uint16_t usec)
{
    const uint32_t max_duty = (1 << 14) - 1;  // 16383
    const uint32_t period_us = 1000000 / PWM_FREQ;
    uint32_t duty = (usec * max_duty) / period_us;
    
    Serial.printf("Ch%d: %uus -> duty %lu\n", channel, usec, duty);
    
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
    int16_t rx_throttle = crsf_get_channel(2);
    int16_t rx_roll     = crsf_get_channel(0);
    int16_t rx_pitch    = crsf_get_channel(1);
    int16_t rx_yaw      = crsf_get_channel(3);

    // normalize digital values
    auto norm = [](int16_t val)
    {
        return (float)(val - 992) / 820.0f; // center at midpoint (992? for 172-1811)
    };
    float roll = norm(rx_roll);
    float pitch = norm(rx_pitch);
    float yaw = norm(rx_yaw);
    //float throttle = (float)(rx_throttle - 172) / (1811 - 172);

    // EDF PWM
    //uint16_t throttle_us = (uint16_t)(1000 + throttle*1000);
    //write_us(EDF_CHANNEL, throttle_us);

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