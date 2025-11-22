#ifndef METRIC_H
#define METRIC_H

#include <Arduino.h>

// performance structure
struct Performance 
{
    // control loop metrics
    uint32_t loop_time_us = 0;
    uint32_t loop_time_max_us = 0;
    uint32_t loop_time_min_us = UINT32_MAX;
    
    // component timings
    uint32_t mpu_read_us = 0;
    uint32_t pid_compute_us = 0;
    uint32_t motor_update_us = 0;
    
    // system metrics
    float cpu_usage_core0 = 0.0f;
    float cpu_usage_core1 = 0.0f;
    uint32_t free_heap = 0;
    
    // loop statistics
    uint32_t loop_count = 0;
    uint32_t overruns = 0;
    
    // jitter tracking
    uint32_t jitter_avg_us = 0;
    uint32_t jitter_max_us = 0;
};

extern Performance perf;

// high-resolution timing
extern hw_timer_t *rate_loop_timer;
extern hw_timer_t *angle_loop_timer;
extern unsigned long currTime;
extern unsigned long prevTime;
extern uint32_t last_print_time;

const uint32_t PRINT_INTERVAL_MS = 50; // 20 Hz
const long timeoutTime = 2000;

#endif // METRIC_H