
#include "cf.h"
#include "metric.h"
#include "config.h"
#include "control.h"
#include "crsf_rc.h"
#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <MPU9250.h>

Performance perf;
FlightState state;
CF cf(ALPHA);
int channels[] = {EDF_CHANNEL, SERVO1_CHANNEL, SERVO2_CHANNEL, SERVO3_CHANNEL, SERVO4_CHANNEL};
int pins[] = {EDF_PIN, SERVO1_PIN, SERVO2_PIN, SERVO3_PIN, SERVO4_PIN};

// library MPU9250 object
MPU9250 mpu;

// mutex for thread-safe access to our flight state
SemaphoreHandle_t state_mutex;

// helper function for LEDs indicating FC status
void setrgb(uint8_t red, uint8_t green, uint8_t blue)
{
  neopixelWrite(RGB_PIN, red, green, blue);
}

// helper function to convert microseconds to duty cycle for PWM
uint32_t servoMicrosecondsToDuty(uint16_t microseconds)
{
    const uint32_t max_duty = (1 << PWM_RES_BITS) - 1;
    return (microseconds * max_duty) / 20000; // 20ms period
}

// helper function to set servo position in microseconds
void setServoMicroseconds(uint8_t channel, uint16_t microseconds)
{
    if (microseconds < 1000) microseconds = 1000;
    if (microseconds > 2000) microseconds = 2000;

    uint32_t duty = servoMicrosecondsToDuty(microseconds);
    ledcWrite(channel, duty);
}

// ===== CORE 0 =====
// the work done on this core will be dedicated to processing the control loop of our monocopter.
// ==================
void controlLoop(void *parameter)
{
    Wire.begin(SDA, SCL, 400000);

    const uint32_t rate_period_us = 1000000 / RATE_LOOP_HZ;

    const TickType_t period = pdMS_TO_TICKS(1000 / RATE_LOOP_HZ);
    TickType_t prev_wake = xTaskGetTickCount();

    // performance tracking
    uint32_t loops_in_window = 0;
    uint32_t stats_window_start = millis();

    // local containers to avoid repeated allocations/locks
    float lgx, lgy, lgz;
    float lax, lay, laz;
    float lmx, lmy, lmz;

    Serial.println("[ControlLoop] Task started on Core 0");

    // ==== BEGIN CONTROL LOOP ====
    while (true)
    {
        uint32_t loop_start = micros();

        // === GRAB SENSOR DATA ===
        uint32_t t0 = micros();
        bool ok = mpu.update();
        uint32_t mpu_read_us = micros() - t0;

        if (ok)
        {
            lgx = mpu.getGyroX() - GX_BIAS;
            lgy = mpu.getGyroY() - GY_BIAS;
            lgz = mpu.getGyroZ() - GZ_BIAS;
            lax = mpu.getAccX();
            lay = mpu.getAccY();
            laz = mpu.getAccZ();

            lmx = mpu.getMagX();
            lmy = mpu.getMagY();
            lmz = mpu.getMagZ();

            cf.update(lgx, lgy, lgz, lax, lay, laz, lmx, lmy, lmz, 1.0f / RATE_LOOP_HZ);

            float roll = cf.getRoll();
            float pitch = cf.getPitch();
            float yaw = cf.getYaw();

            if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(2)) == pdTRUE)
            {
                state.roll = roll;
                state.pitch = pitch;
                state.yaw = yaw;

                state.gyro_x = lgx;
                state.gyro_y = lgy;
                state.gyro_z = lgz;

                state.timestamp_ms = millis();
                state.data_ready = true;
                xSemaphoreGive(state_mutex);
            }
        }

        // === CONTROL ===
        crsf_update(); // update RC channels


        // === METRICS ===
        perf.mpu_read_us = mpu_read_us;
        perf.loop_time_us = micros() - loop_start;
        loops_in_window++;

        if (millis() - stats_window_start >= 1000)
        {
            perf.loop_count = loops_in_window;
            perf.free_heap = ESP.getFreeHeap();
            loops_in_window = 0;
            stats_window_start = millis();
        }
    }
}

void setup() 
{
    Serial.begin(BAUD_RATE); // start serial comm for USB
    
    Serial.println("\n\nInitializing peripherals...\n\n");

    setrgb(255, 255, 0);

    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);

    Serial.print("Initializing MPU9250 sensor...");
    // initialize MPU9250
    if (!mpu.setup(0x68))
    {
        Serial.print("FAILED.\n");
        setrgb(255, 0, 0);
        while(1)
        {
            delay(1);
        }
    }
    Serial.print("OK...");

    // calibrate sensors (assumes device is stationary, optional in the future)
    mpu.calibrateAccelGyro();
    Serial.print("GYRO/ACCEL OK...");
    delay(1000);
    mpu.calibrateMag();
    Serial.print("MAG OK.\n");

    Wire.setClock(400000);

    //mpu.setMagneticDeclination(0);
    
    /*
    Serial.print("SETTING FILTER TO MADGWICK...");
    mpu.selectFilter(QuatFilterSel::MADGWICK);
    Serial.print("OK\n");
    */

    mpu.setFilterIterations(1);

    /*
    Serial.println("Initializing web server...");
    if(initServer() < 0)
    {
        Serial.print("FAILED.");
        while (true)
        {
            delay(1);
        }
    }
    Serial.print("OK.");
    */

    // create mutex 
    state_mutex = xSemaphoreCreateMutex();
    if (!state_mutex)
    {
        Serial.println("ERROR: Failed to create mutex");
        setrgb(255, 0, 0);
        while (true) delay(1);
    }

    // bind control loop task to its own core (Core 0) 
    xTaskCreatePinnedToCore(
        controlLoop,  
        "ControLoop",
        8192,   // memory size (deterministic and no dynamic allocation so shouldn't matter)
        NULL,
        3,      // higher priority than logging task
        NULL,
        0
    );

    /*
    // === PWM CHANNEL INIT ===
    Serial.println("Initializing EDF + servo PWM channels...");

    // EDF output
    ledcSetup(0, PWM_FREQ, PWM_RES_BITS);
    ledcAttachPin(EDF_PIN, 0);

    for (int i = 0; i < 5; i++)
    {
        ledcSetup(channels[i], PWM_FREQ, PWM_RES_BITS);
        ledcAttachPin(pins[i], channels[i]);
    }

    ledcWrite(EDF_CHANNEL, 0); // EDF off
    for (int i = 1; i <= 4; i++)
    {
        ledcWrite(i, servoMicrosecondsToDuty(1500)); // neutral position
    }
    */

    Serial.println("Initializing UART CRSF receiver...");
    crsf_init();
    Serial.println("OK.\n");

    setrgb(0, 255, 0);
}

// ===== CORE 1 =====
// the work done on this core will be dedicated to logging and grabbing data from our monocopter
// ==================
void loop() 
{
    static uint32_t last_print_time = 0;
    static uint32_t last_stats_time = 0;
    static float last_roll = 0, last_pitch = 0, last_yaw = 0;
    const float THRESHOLD_DEG = 1.5f;      // degrees change required to trigger print
    const uint32_t PRINT_INTERVAL_MS = 100; // minimum interval between prints
    const uint32_t STATS_INTERVAL_MS = 5000; // summary print every 5s

    /* Check for significant attitude change
    if (millis() - last_print_time >= PRINT_INTERVAL_MS)
    {
        if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            float d_roll  = fabs(state.roll  - last_roll);
            float d_pitch = fabs(state.pitch - last_pitch);
            float d_yaw   = fabs(state.yaw   - last_yaw);

            if (d_roll > THRESHOLD_DEG || d_pitch > THRESHOLD_DEG || d_yaw > THRESHOLD_DEG)
            {
                Serial.printf("Δ roll: %.2f°, pitch: %.2f°, yaw: %.2f°  |  RPY: %.2f, %.2f, %.2f\n",
                              d_roll, d_pitch, d_yaw, state.roll, state.pitch, state.yaw);

                last_roll = state.roll;
                last_pitch = state.pitch;
                last_yaw = state.yaw;
                last_print_time = millis();
            }

            xSemaphoreGive(state_mutex);
        }
    }
    */

    static uint32_t lastPrint = 0;
    if (millis() - lastPrint >= 50) { // ~20 Hz print rate
        lastPrint = millis();

        if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            Serial.printf("Roll:%.2f, Pitch:%.2f, Yaw:%.2f\n", state.roll, state.pitch, state.yaw);
            xSemaphoreGive(state_mutex);
        }
    }

    // Print a short summary every few seconds
    if (millis() - last_stats_time >= STATS_INTERVAL_MS)
    {
        last_stats_time = millis();
        Serial.printf("[Core1] Loop alive | Heap: %lu bytes | Stack watermark: %lu | Read time: %lu\n", esp_get_free_heap_size(), uxTaskGetStackHighWaterMark(NULL), perf.mpu_read_us);
    }

    vTaskDelay(pdMS_TO_TICKS(5)); // don’t hog CPU
}