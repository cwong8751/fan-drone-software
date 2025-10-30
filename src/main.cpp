#include "metric.h"
#include "config.h"
#include "control.h"
#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <MPU9250.h>

Performance perf;
FlightState state;
uint32_t last_print_time = 0;

// library MPU9250 object
MPU9250 mpu;

// mutex for thread-safe access to our flight state
SemaphoreHandle_t state_mutex;

// helper function for LEDs indicating FC status
void setrgb(uint8_t red, uint8_t green, uint8_t blue)
{
  neopixelWrite(RGB_pin, red, green, blue);
}

// ===== CORE 0 =====
// the work done on this core will be dedicated to processing the control loop of our monocopter.
// ==================
#define DEBUG_MPU_TOGGLE_PIN 2
void controlLoop(void *parameter)
{
    const uint32_t rate_period_us = 1000000 / RATE_LOOP_HZ;
    TickType_t prev_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1000 / RATE_LOOP_HZ);

    uint32_t loop_counter = 0;
    uint32_t max_mpu_read_time_us = 0;

    // performance tracking
    uint32_t stats_window_start = millis();
    uint32_t loops_in_window = 0;

    // jitter tracking
    uint32_t prev_loop_time = 0;
    uint32_t max_jitter = 0;
    uint32_t total_jitter = 0;
    
    pinMode(DEBUG_MPU_TOGGLE_PIN, OUTPUT);
    digitalWrite(DEBUG_MPU_TOGGLE_PIN, LOW);

    Serial.println("[ControlLoop] Task started on Core 0");

    while (true)
    {
        uint32_t loop_start = micros();

        digitalWrite(DEBUG_MPU_TOGGLE_PIN, HIGH);

        uint32_t mpu_start = micros();
        bool ok = mpu.update();
        uint32_t mpu_read_time_us = micros() - mpu_start;

        digitalWrite(DEBUG_MPU_TOGGLE_PIN, LOW);

        if (ok)
        {
            if (mpu_read_time_us > max_mpu_read_time_us) max_mpu_read_time_us = mpu_read_time_us;
            
            if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(5)) == pdTRUE)
            {
                // library madgwick/mahony filter sensor fusion
                static uint32_t orientation_counter = 0;
                if (orientation_counter++ % 10 == 0) {  // Every 10th loop = 25Hz
                    state.roll = mpu.getRoll();
                    state.pitch = mpu.getPitch();
                    state.yaw = mpu.getYaw();
                }

                state.gyro_x = mpu.getGyroX();
                state.gyro_y = mpu.getGyroY();
                state.gyro_z = mpu.getGyroZ();

                state.timestamp_ms = millis();
                state.data_ready = true;
                
                xSemaphoreGive(state_mutex);
            }

            // === PID compute ===
            uint32_t pid_start = micros();
            // TODO: call to PID calculation function
            uint32_t pid_time_us = micros() - pid_start;

            // === motor control ===
            uint32_t motor_start = micros();
            // TODO: call to update motor function
            uint32_t motor_time_us = micros() - motor_start;

            // total loop time (us) calculation
            uint32_t total_loop_time_us = micros() - loop_start;
            
            // update performance metrics
            perf.loop_time_us = total_loop_time_us;
            if (total_loop_time_us > perf.loop_time_max_us) perf.loop_time_max_us = total_loop_time_us;
            if (total_loop_time_us < perf.loop_time_min_us) perf.loop_time_min_us = total_loop_time_us;
            
            perf.mpu_read_us = mpu_read_time_us;
            perf.pid_compute_us = pid_time_us;
            perf.motor_update_us = motor_time_us;
            
            if (prev_loop_time > 0) {
                uint32_t jitter = abs((int32_t)total_loop_time_us - (int32_t)prev_loop_time);
                total_jitter += jitter;
                if (jitter > max_jitter) {
                    max_jitter = jitter;
                }
            }
            prev_loop_time = total_loop_time_us;

            loops_in_window++;
            loop_counter++;

            // stats snapshot every second 
            if (millis() - stats_window_start >= 1000)
            {
                uint32_t elapsed_ms = millis() - stats_window_start;
                float avg_loop_hz = (loops_in_window * 1000.0f) / elapsed_ms;
                perf.free_heap = ESP.getFreeHeap();
                
                float avg_jitter = loops_in_window > 1 ? total_jitter / (float)(loops_in_window - 1) : 0;
                perf.jitter_avg_us = (uint32_t)avg_jitter;
                perf.jitter_max_us = max_jitter;
                perf.loop_count = loops_in_window;
                
                // store current stats for Core 1 to print
                
                // reset window
                stats_window_start = millis();
                loops_in_window = 0;
                max_mpu_read_time_us = 0;
                perf.loop_time_max_us = 0;
                perf.loop_time_min_us = UINT32_MAX;
                max_jitter = 0;
                total_jitter = 0;
                perf.overruns = 0; // reset overrun counter after reporting period
            }
        }

        vTaskDelayUntil(&prev_wake, period);
    }
}

void setup() 
{
    Serial.begin(BAUD_RATE); // start serial comm for USB
    
    Serial.println("\n\nInitializing peripherals...\n\n");

    setrgb(255, 255, 0);

    Serial.print("Initializing I2C0 bus interface...");
    // initialize first i2c bus to 8 (SDA) and 9 (SCL)
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000); 
    Serial.print("OK.\n");

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

    Wire.setClock(400000); 

    // calibrate sensors (assumes device is stationary, optional in the future)
    mpu.calibrateAccelGyro();
    Serial.print("GYRO/ACCEL OK...");
    delay(1000);
    mpu.calibrateMag();
    Serial.print("MAG OK.\n");

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
        4096,
        NULL,
        3,
        NULL,
        0
    );

    setrgb(0, 255, 0);
}

// ===== CORE 1 =====
// the work done on this core will be dedicated to logging and grabbing data from our monocopter
// ==================
void loop() 
{
    static uint32_t last_stats_print = 0;
    static uint32_t loop_count_core1 = 0;
    static uint32_t last_perf_print = 0;

    if (millis() - last_print_time >= PRINT_INTERVAL_MS)
    {
        last_print_time = millis();
        
        if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            Serial.print("roll:");
            Serial.print(state.roll);
            Serial.print(",");
            Serial.print("pitch:");
            Serial.print(state.pitch);
            Serial.print(",");
            Serial.print("yaw:");
            Serial.println(state.yaw);

            xSemaphoreGive(state_mutex);
        }
        
        loop_count_core1++;
    }
    
    // Print detailed performance stats every 1 second
    if (millis() - last_perf_print >= 1000)
    {
        last_perf_print = millis();
        
        // Calculate percentages
        float mpu_pct = perf.loop_time_us > 0 ? (perf.mpu_read_us / (float)perf.loop_time_us) * 100.0f : 0;
        float pid_pct = perf.loop_time_us > 0 ? (perf.pid_compute_us / (float)perf.loop_time_us) * 100.0f : 0;
        float motor_pct = perf.loop_time_us > 0 ? (perf.motor_update_us / (float)perf.loop_time_us) * 100.0f : 0;
        float other_pct = 100.0f - mpu_pct - pid_pct - motor_pct;
        
        Serial.println("\n=== Performance Stats ===");
        Serial.printf("Loop Rate: %.1f Hz (target: %d Hz)\n", 
                      perf.loop_count / 1.0f, RATE_LOOP_HZ);
        Serial.printf("CPU Core0: %.1f%% (%.1f us / %.1f us budget)\n", 
                      perf.cpu_usage_core0, 
                      (float)perf.loop_time_us,
                      1000000.0f / RATE_LOOP_HZ);
        Serial.printf("Loop Time: %lu us (min: %lu, max: %lu)\n", 
                     perf.loop_time_us, perf.loop_time_min_us, perf.loop_time_max_us);
        Serial.printf("  MPU Read:     %4lu us (%.1f%%)\n", perf.mpu_read_us, mpu_pct);
        Serial.printf("  PID Compute:  %4lu us (%.1f%%)\n", perf.pid_compute_us, pid_pct);
        Serial.printf("  Motor Update: %4lu us (%.1f%%)\n", perf.motor_update_us, motor_pct);
        Serial.printf("  Other/Overhead: %.1f%%\n", other_pct);
        Serial.printf("Jitter: avg %lu us, max %lu us\n", perf.jitter_avg_us, perf.jitter_max_us);
        Serial.printf("Overruns: %lu\n", perf.overruns);
        Serial.printf("Free Heap: %lu bytes\n", perf.free_heap);
        Serial.printf("Stack High Water (Core0): %lu bytes\n", perf.free_heap);
        
        // Print warnings
        if (perf.cpu_usage_core0 > 80.0f) {
            Serial.println("⚠️  WARNING: CPU usage >80%");
        }
        if (perf.mpu_read_us > 200) {
            Serial.printf("⚠️  MPU read slow: %lu us\n", perf.mpu_read_us);
        }
        if (perf.overruns > 0) {
            Serial.printf("⚠️  OVERRUNS detected: %lu in last second\n", perf.overruns);
        }
        
        Serial.println("========================\n");
    }

    // Print Core 1 stats every 5 seconds
    if (millis() - last_stats_print >= 5000) 
    {
        last_stats_print = millis();
        
        Serial.println("\n=== Core 1 (Loop) Stats ===");
        Serial.printf("Logging Rate: %.1f Hz\n", loop_count_core1 / 5.0f);
        Serial.printf("Stack High Water: %lu bytes\n", uxTaskGetStackHighWaterMark(NULL));
        Serial.println("===========================\n");
        
        loop_count_core1 = 0;
    }
    
    vTaskDelay(pdMS_TO_TICKS(1));
}
