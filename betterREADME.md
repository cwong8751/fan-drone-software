# Monocopter Flight Controller - Complete Documentation

ESP32-S3 based flight controller firmware for monocopter configurations with real-time sensor fusion and dual-core control architecture.

---

## Quick Start

### Features

- **Dual-core RTOS**: Core 0 handles 250Hz control loop, Core 1 manages logging
- **9-DOF sensor fusion**: Madgwick AHRS algorithm with gyro, accelerometer, and magnetometer
- **CRSF receiver**: Low-latency radio control input via UART
- **PID stabilization**: Roll, pitch, and yaw control loops
- **USB-CDC debugging**: Native USB serial communication
- **Thread-safe**: Mutex-protected state sharing between cores

### Hardware Requirements

#### Development Board
- **ESP32-S3 DevKit-C-1** (or compatible ESP32-S3 board)
- USB-C cable for programming and serial communication

#### Sensors
- **IMU**: LSM6DSOX (6-axis) or MPU9250 (9-axis)
  - I2C address: 0x6B
  - Measures: 3-axis gyroscope, 3-axis accelerometer
- **Magnetometer**: LIS3MDL (3-axis)
  - I2C address: 0x1C
  - Connected to I2C Bus 1
- **Barometer**: MS5607-02BA or MS5611
  - Connected to I2C Bus 0
  - Measures: temperature and pressure

#### Actuators
- 1x Electric Ducted Fan (EDF) or motor
- 4x Servo motors for control surfaces

#### Radio Control
- CRSF-compatible receiver (TBS Crossfire, ExpressLRS, etc.)
- Connected via UART

### Pin Configuration

All pins are defined in `include/config.h`. Key connections:

```cpp
// I2C Bus 0 (IMU + Barometer)
SDA_IMU_PIN
SCL_IMU_PIN

// I2C Bus 1 (Magnetometer)
SDA_MAG_PIN
SCL_MAG_PIN

// PWM Outputs
EDF_PIN       // Main motor/EDF
SERVO1_PIN    // Control surface 1
SERVO2_PIN    // Control surface 2
SERVO3_PIN    // Control surface 3
SERVO4_PIN    // Control surface 4

// Status LED
RGB_PIN       // Neopixel RGB LED

// CRSF Receiver
CRSF_RX_PIN   // UART RX (from receiver)
CRSF_TX_PIN   // UART TX (for telemetry - optional)
```

### Software Setup

#### Prerequisites

1. **PlatformIO**: Install via VS Code extension or CLI

2. **USB Driver**: ESP32-S3 uses native USB CDC 

#### Build and Upload

1. **Clone the repository**
2. **Open in PlatformIO**
3. **Configure hardware** (if needed):
   - Edit `include/config.h` for pin assignments
   - Adjust PID gains for your specific airframe
   - Set control loop rate (`RATE_LOOP_HZ`)
4. **Build firmware**
5. **Upload to board**
6. **Monitor serial output**


### Status LED Indicators

| Color | Status |
|-------|--------|
| **Yellow** | System initializing |
| **Red** | Initialization error (check serial output) |
| **Red blinking** | Motors armed during initialization (safety halt) |
| **Green** | System armed and ready |

---

## Table of Contents

1. [Repository Structure](#repository-structure)
2. [Build Configuration](#build-configuration)
3. [Architecture Overview](#architecture-overview)
4. [Core Components](#core-components)
5. [Data Structures](#data-structures)
6. [System Initialization](#system-initialization)
7. [Dual-Core Task Architecture](#dual-core-task-architecture)
8. [Sensor Integration](#sensor-integration)
9. [Control Flow](#control-flow)
10. [Communication Protocols](#communication-protocols)
11. [Performance Monitoring](#performance-monitoring)
12. [Thread Safety](#thread-safety)
13. [Hardware Configuration](#hardware-configuration)
14. [Configuration Guide](#configuration-guide)
15. [Safety Features](#safety-features)
16. [Troubleshooting](#troubleshooting)
17. [Future Enhancements](#future-enhancements)
18. [API Reference](#api-reference)
19. [Glossary](#glossary)

---

## Repository Structure

```
monocopter-fc/
├── platformio.ini          # PlatformIO configuration
├── README.md              # This file
├── include/               # Header files
│   ├── cf.h              # Complementary filter interface
│   ├── config.h          # Hardware pins, loop rates, PID defaults, PWM settings
│   ├── control.h         # Motor control API (init/update/arm)
│   ├── crsf_rc.h         # CRSF receiver interface
│   ├── metric.h          # Performance metrics structures
│   └── pid.h             # PID controller declarations
└── src/                  # Source files
    ├── main.cpp          # Program entry point and task setup
    ├── cf.cpp            # Complementary filter implementation
    ├── control.cpp       # Motor control and control loop logic
    ├── crsf_rc.cpp       # CRSF receiver parsing and channel API
    ├── pid.cpp           # PID controller implementation
    └── wifi.cpp          # Optional web server/telemetry (commented out)
```

### Module Responsibilities

| Module | Purpose |
|--------|---------|
| **main.cpp** | System initialization, sensor setup, RTOS task creation, Madgwick filter integration |
| **cf.cpp/h** | Complementary filter implementation (alternative to Madgwick) |
| **control.cpp/h** | Motor/servo PWM control, arming logic, control loop implementation |
| **crsf_rc.cpp/h** | CRSF protocol receiver parsing, channel data API |
| **pid.cpp/h** | PID controller for stabilization loops |
| **metric.h** | Performance tracking structures |
| **config.h** | Centralized hardware configuration (pins, rates, constants) |
| **wifi.cpp** | Web server/telemetry features (currently unused) |

---

## Build Configuration

**Target Board**: ESP32-S3 DevKit-C-1  
**Framework**: Arduino  
**Build Tool**: PlatformIO

### platformio.ini Settings

```ini
[env:esp32-s3-devkitc-1]
platform = espressif32
board = esp32-s3-devkitc-1
framework = arduino

; USB CDC for serial communication
build_flags = -DARDUINO_USB_CDC_ON_BOOT=1

; Flash and PSRAM configuration
; (specific settings for ESP32-S3 memory management)
```

**Key Features**:
- USB-CDC enabled for native USB serial communication
- Custom flash and PSRAM settings optimized for sensor processing
- Sensor libraries integrated via PlatformIO dependencies

**Dependencies** (from platformio.ini):
- Adafruit LSM6DSOX (6-axis IMU)
- Adafruit LIS3MDL (magnetometer)
- Adafruit AHRS (Madgwick filter)
- MS5x (barometric pressure sensor)
- Adafruit Unified Sensor

---

## Architecture Overview

This flight controller firmware implements a real-time control system using FreeRTOS on the ESP32-S3's dual-core architecture. The system is designed for a monocopter configuration with the following key characteristics:

- **Dual-core processing**: Core 0 handles real-time control loops, Core 1 manages logging and monitoring
- **Multi-sensor fusion**: IMU (accelerometer + gyroscope), magnetometer, and barometer data fusion
- **CRSF protocol**: Direct receiver communication via UART for low-latency control input
- **Real-time control**: 250Hz control loop rate (`RATE_LOOP_HZ`) with deterministic timing
- **Thread-safe data sharing**: Mutex-protected state exchange between cores
- **USB-CDC communication**: Native USB serial for debugging and telemetry

### System Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                      ESP32-S3                               │
├──────────────────────────┬──────────────────────────────────┤
│      CORE 0              │         CORE 1                   │
│  (Control Loop)          │    (Logging/Monitor)             │
│   @ 250 Hz               │                                  │
│                          │                                  │
│  ┌────────────────┐      │                                  │
│  │ Sensor Reading │      │     ┌──────────────┐            │
│  │  - LSM6DSOX    │      │     │ USB-CDC      │            │
│  │    (or MPU9250)│      │     │ Serial       │            │
│  │  - LIS3MDL     │      │     │ Logging      │            │
│  │  - MS5x        │      │     │              │            │
│  └────────┬───────┘      │     └──────────────┘            │
│           │              │                                  │
│  ┌────────▼───────┐      │                                  │
│  │ Sensor Fusion  │      │                                  │
│  │  (Madgwick)    │      │                                  │
│  └────────┬───────┘      │                                  │
│           │              │                                  │
│  ┌────────▼───────┐      │                                  │
│  │ State Update   │◄─────┼──────── Mutex ─────────────────►│
│  │  (Thread-safe) │      │                                  │
│  └────────┬───────┘      │                                  │
│           │              │                                  │
│  ┌────────▼───────┐      │                                  │
│  │ PID Control    │      │                                  │
│  │  + CRSF Input  │      │                                  │
│  └────────┬───────┘      │                                  │
│           │              │                                  │
│  ┌────────▼───────┐      │                                  │
│  │ Motor/Servo    │      │                                  │
│  │  PWM Output    │      │                                  │
│  └────────────────┘      │                                  │
│                          │                                  │
└──────────────────────────┴──────────────────────────────────┘
         │                            │
         ▼                            ▼
    ┌─────────┐                 ┌──────────┐
    │ Motors  │                 │   USB    │
    │ Servos  │                 │  Serial  │
    └─────────┘                 └──────────┘
```

---

## Core Components

### External Dependencies

```cpp
// Sensor Libraries
#include "Adafruit_LSM6DSOX.h"    // 6-axis IMU (accel + gyro)
#include <Adafruit_LIS3MDL.h>     // 3-axis magnetometer
#include <Adafruit_Sensor.h>      // Unified sensor interface
#include <Adafruit_AHRS.h>        // Attitude/Heading Reference System
#include <MS5x.h>                 // Barometric pressure sensor

// System Libraries
#include <Arduino.h>              // Arduino framework
#include <WiFi.h>                 // WiFi (currently unused)
#include <Wire.h>                 // I2C communication
#include "driver/ledc.h"          // ESP32 LED PWM controller
```

### Internal Modules

```cpp
#include "cf.h"        // Complementary filter implementation
#include "metric.h"    // Performance metrics structures
#include "config.h"    // System configuration constants (pins, rates, PID defaults)
#include "control.h"   // Motor/servo control functions
#include "crsf_rc.h"   // CRSF receiver protocol
#include "pid.h"       // PID controller implementation
```

**Note**: `wifi.cpp` exists for optional web server/telemetry features but is currently commented out in main.cpp.

---

## Data Structures

### FlightState Structure

Thread-safe container for current vehicle attitude and sensor data:

```cpp
struct FlightState {
    float roll;          // Roll angle (degrees)
    float pitch;         // Pitch angle (degrees)
    float yaw;           // Yaw angle (degrees)
    
    float gyro_x;        // Gyroscope X-axis (rad/s)
    float gyro_y;        // Gyroscope Y-axis (rad/s)
    float gyro_z;        // Gyroscope Z-axis (rad/s)
    
    uint32_t timestamp_ms;  // Data timestamp
    bool data_ready;        // Data validity flag
};
```

**Access Pattern**: Protected by `state_mutex` semaphore for thread-safe read/write operations.

### Performance Metrics Structure

Real-time performance monitoring data:

```cpp
struct Performance {
    uint32_t mpu_read_us;    // IMU read time (microseconds)
    uint32_t loop_time_us;   // Total control loop time (microseconds)
    uint32_t loop_count;     // Loops completed per second
    uint32_t free_heap;      // Available heap memory (bytes)
};
```

**Update Frequency**: Metrics computed every 1000ms window.

### Sensor Fusion

```cpp
Adafruit_Madgwick mad_filter;  // Madgwick AHRS algorithm
```

**Configuration**:
- Update rate: `RATE_LOOP_HZ` (250Hz from config.h)
- Inputs: 9-axis (gyro, accel, magnetometer)
- Outputs: Roll, pitch, yaw (Euler angles)

**Alternative**: Complementary filter (cf.cpp) available but Madgwick is currently active.

### PID Controllers

Defined in `pid.h` and implemented in `pid.cpp`:

```cpp
struct PIDController {
    float kp;           // Proportional gain
    float ki;           // Integral gain
    float kd;           // Derivative gain
    float integral;     // Accumulated integral term
    float prev_error;   // Previous error for derivative
    // ... additional state
};
```

**Default Values**: Set in `config.h` for roll, pitch, and yaw stabilization loops.

**Usage**: Applied in control loop to generate corrective motor/servo commands based on attitude error.

### Hardware Objects

```cpp
// I2C Sensors (actual sensor may vary by build)
MS5x barometer(&Wire);              // MS5x series barometer on I2C bus 0
Adafruit_LIS3MDL lis3;             // LIS3MDL magnetometer
Adafruit_LSM6DSOX sox;             // LSM6DSOX IMU (accel + gyro)
// OR
// MPU9250 mpu;                      // MPU9250 alternative IMU

// I2C Buses
Wire   // Bus 0: Barometer, IMU
Wire1  // Bus 1: Magnetometer
```

**Note**: The codebase supports both LSM6DSOX and MPU9250 IMUs. The actual sensor used depends on hardware configuration.

### Control Arrays

```cpp
// PWM channel assignments
int channels[] = {
    EDF_CHANNEL,      // Electric ducted fan
    SERVO1_CHANNEL,   // Servo 1
    SERVO2_CHANNEL,   // Servo 2
    SERVO3_CHANNEL,   // Servo 3
    SERVO4_CHANNEL    // Servo 4
};

// GPIO pin assignments
int pins[] = {
    EDF_PIN,      // EDF motor pin
    SERVO1_PIN,   // Servo 1 pin
    SERVO2_PIN,   // Servo 2 pin
    SERVO3_PIN,   // Servo 3 pin
    SERVO4_PIN    // Servo 4 pin
};
```

---

## System Initialization

### Initialization Sequence

The `setup()` function executes on Core 1 and performs the following initialization sequence:

```
1. Serial Communication (BAUD_RATE)
   ↓
2. 3-second startup delay
   ↓
3. Status LED (Yellow = initializing)
   ↓
4. I2C Bus Configuration
   - Wire (Bus 0): SDA_IMU_PIN, SCL_IMU_PIN
   - Wire1 (Bus 1): SDA_MAG_PIN, SCL_MAG_PIN
   ↓
5. Sensor Initialization
   - LSM6DSOX (IMU) @ 0x6B
   - LIS3MDL (Magnetometer) @ 0x1C
   - MS5607 (Barometer)
   ↓
6. I2C Clock Speed (400kHz)
   ↓
7. Mutex Creation (state_mutex)
   ↓
8. Control Loop Task Creation (Core 0, Priority 3)
   ↓
9. Motor/Servo Initialization
   ↓
10. CRSF Receiver Initialization
    ↓
11. Madgwick Filter Setup (RATE_LOOP_HZ)
    ↓
12. Motor Arming
    ↓
13. Status LED (Green = ready)
```

### Error Handling

All initialization steps include error checking with infinite loop halt on failure:

```cpp
if (!sox.begin_I2C(0x6B))
{
    Serial.print("FAILED.\n");
    setrgb(255, 0, 0);  // Red LED
    while (true) delay(1);  // Halt system
}
```

**Status LED Colors**:
- Yellow (255, 255, 0): Initializing
- Red (255, 0, 0): Initialization failure
- Green (0, 255, 0): Ready/Armed

---

## Dual-Core Task Architecture

### Core 0: Control Loop Task

**Task Name**: `ControlLoop`  
**Stack Size**: 8192 bytes  
**Priority**: 3 (higher than logging)  
**Execution Rate**: 250Hz (`RATE_LOOP_HZ`)

**Responsibilities**:
1. Sensor data acquisition
2. Sensor fusion (Madgwick filter)
3. State update (thread-safe)
4. CRSF input processing
5. Motor/servo output generation
6. Performance metrics collection

**Timing Characteristics**:
```cpp
const uint32_t rate_period_us = 1000000 / RATE_LOOP_HZ;  // 4000μs @ 250Hz
const TickType_t period = pdMS_TO_TICKS(1000 / RATE_LOOP_HZ);  // 4ms
```

Uses FreeRTOS `vTaskDelayUntil()` for deterministic periodic execution.

### Core 1: Main Loop

**Execution**: Standard Arduino `loop()` function  
**Current State**: Disabled (commented out)  
**Intended Purpose**: Logging and monitoring

**Potential Functionality** (currently commented):
- Attitude change detection and logging
- Performance statistics printing
- Heap/stack watermark monitoring

---

## Sensor Integration

### LSM6DSOX (6-Axis IMU)

**Interface**: I2C Bus 0  
**Address**: 0x6B  
**Sensors**:
- 3-axis accelerometer (m/s²)
- 3-axis gyroscope (rad/s)
- Temperature sensor

**Data Acquisition**:
```cpp
sensors_event_t accel, gyro, temp;
sox.getEvent(&accel, &gyro, &temp);

// Access data
float ax = accel.acceleration.x;  // m/s²
float gx = gyro.gyro.x;           // rad/s
```

### LIS3MDL (3-Axis Magnetometer)

**Interface**: I2C Bus 1  
**Address**: 0x1C  
**Sensors**: 3-axis magnetometer (μTesla)

**Data Acquisition**:
```cpp
sensors_event_t mag;
lis3.getEvent(&mag);

float mx = mag.magnetic.x;  // μTesla
```

### MS5607-02BA (Barometric Pressure Sensor)

**Interface**: I2C Bus 0  
**Measurements**:
- Temperature (°C)
- Pressure (Pa)
- Altitude (meters, if enabled)

**Data Acquisition**:
```cpp
barometer.checkUpdates();
if (barometer.isReady())
{
    float temperature = barometer.GetTemp();
    float pressure = barometer.GetPres();
}
```

**Note**: Altitude calculation is currently disabled but can be enabled for altitude-hold functionality.

### I2C Bus Configuration

```cpp
// Bus 0 (Wire): IMU + Barometer
Wire.setPins(SDA_IMU_PIN, SCL_IMU_PIN);
Wire.begin();
Wire.setClock(400000);  // 400kHz

// Bus 1 (Wire1): Magnetometer
Wire1.setPins(SDA_MAG_PIN, SCL_MAG_PIN);
Wire1.begin();
```

**Rationale**: Separate buses prevent I2C address conflicts and improve performance.

---

## Control Flow

### Sensor Fusion Pipeline

```
┌─────────────────┐
│ Raw Sensor Data │
│  - Gyro (3-axis)│
│  - Accel(3-axis)│
│  - Mag  (3-axis)│
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ Madgwick Filter │
│  (9-DOF AHRS)   │
│   @ 250 Hz      │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│  Euler Angles   │
│  - Roll         │
│  - Pitch        │
│  - Yaw          │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│  FlightState    │
│  (Protected)    │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ PID Controllers │
│ - Roll PID      │
│ - Pitch PID     │
│ - Yaw PID       │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ Motor Commands  │
└─────────────────┘
```

**Update Rate**: 250Hz for smooth attitude estimation and control response

**PID Integration**: Attitude errors (desired vs actual angles) are fed into PID controllers which generate corrective motor/servo commands.

### CRSF Input Processing

When armed, the system processes CRSF receiver channels:

```cpp
// Channel mapping
int16_t rx_throttle = crsf_get_channel(2);  // Channel 3 → index 2
int16_t rx_roll     = crsf_get_channel(0);  // Channel 1 → index 0
int16_t rx_pitch    = crsf_get_channel(1);  // Channel 2 → index 1
int16_t rx_yaw      = crsf_get_channel(3);  // Channel 4 → index 3
```

**Normalization**:
```cpp
auto norm = [](int16_t val) {
    return (float)(val - 992) / 820.0f;
};

// Range: 172-1811 (CRSF standard)
// Midpoint: 992
// Normalized: -1.0 to +1.0
```

**Throttle Mapping**:
```cpp
float throttle = (float)(rx_throttle - 172) / (1811 - 172);
// Range: 0.0 to 1.0
```

### Motor/Servo Output

**PWM Configuration**:
- Resolution: `PWM_RES_BITS` (typically 12-bit = 4096 levels)
- Period: 20ms (50Hz, standard RC servo)
- Pulse width: 1000-2000μs

**Conversion Functions**:
```cpp
// Microseconds to duty cycle
uint32_t servoMicrosecondsToDuty(uint16_t microseconds)
{
    const uint32_t max_duty = (1 << PWM_RES_BITS) - 1;
    return (microseconds * max_duty) / 20000;
}

// Set servo position (auto-clamped to 1000-2000μs)
void setServoMicroseconds(uint8_t channel, uint16_t microseconds);
```

---

## Communication Protocols

### CRSF (Crossfire Protocol)

**Interface**: UART  
**Function**: RC receiver communication  
**Features**:
- Low latency (< 10ms typical)
- High resolution (11-bit channels, 172-1811 range)
- Telemetry support (not yet implemented)

**API**:
```cpp
crsf_init();                        // Initialize UART
crsf_update();                      // Process incoming data
int16_t crsf_get_channel(uint8_t); // Read channel value
```

### Serial Debug Output

**Baud Rate**: `BAUD_RATE` (defined in config.h)  
**Purpose**: System diagnostics and telemetry

**Output Format** (when armed):
```
Throttle:1234,Roll:567,Pitch:890,Yaw:345
```

**Print Rate**: Every 10 control loops (~ 25Hz @ 250Hz loop)

---

## Performance Monitoring

### Metrics Collection

```cpp
struct Performance {
    uint32_t mpu_read_us;    // IMU read latency
    uint32_t loop_time_us;   // Total loop execution time
    uint32_t loop_count;     // Loops per second
    uint32_t free_heap;      // Available RAM
};
```

**Update Logic**:
```cpp
// Per-loop metrics
perf.mpu_read_us = imu_read_us;
perf.loop_time_us = micros() - loop_start;

// Periodic metrics (1 Hz)
if (millis() - stats_window_start >= 1000)
{
    perf.loop_count = loops_in_window;
    perf.free_heap = ESP.getFreeHeap();
    // Reset counters
}
```

### Performance Targets

| Metric | Good | Warning | Critical |
|--------|------|---------|----------|
| Loop rate | 250 Hz | < 240 Hz | < 225 Hz |
| Loop time | < 3500μs | 3500-4000μs | > 4000μs |
| IMU read time | < 400μs | 400-600μs | > 600μs |
| Free heap | > 80 KB | 50-80 KB | < 50 KB |

**Note**: At 250Hz, each control loop iteration must complete within 4ms to maintain real-time performance.

---

## Thread Safety

### Mutex Protection

```cpp
SemaphoreHandle_t state_mutex;
```

**Protected Resource**: `FlightState state`

**Usage Pattern**:
```cpp
// Core 0: Write attitude data
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

// Core 1: Read attitude data
if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(10)) == pdTRUE)
{
    float current_roll = state.roll;
    // ... process data
    xSemaphoreGive(state_mutex);
}
```

**Timeout Values**:
- Core 0 (control): 2ms timeout (priority to real-time control)
- Core 1 (logging): 10ms timeout (more tolerant)

### Critical Sections

No additional critical sections are required as:
1. Sensor objects are accessed only from Core 0
2. Motor/servo control functions are called only from Core 0
3. Serial output is non-critical and uses hardware buffering

---

## Hardware Configuration

### Pin Assignments

Defined in `config.h` (referenced but not shown):

```cpp
// I2C Pins
SDA_IMU_PIN, SCL_IMU_PIN  // Bus 0: IMU + Barometer
SDA_MAG_PIN, SCL_MAG_PIN  // Bus 1: Magnetometer

// PWM Output Pins
EDF_PIN      // Electric ducted fan
SERVO1_PIN   // Control surface 1
SERVO2_PIN   // Control surface 2
SERVO3_PIN   // Control surface 3
SERVO4_PIN   // Control surface 4

// Status LED
RGB_PIN      // Neopixel RGB LED
```

### PWM Channels

```cpp
// LEDC channel assignments
EDF_CHANNEL      // Motor speed control
SERVO1_CHANNEL   // Servo position control
SERVO2_CHANNEL
SERVO3_CHANNEL
SERVO4_CHANNEL
```

**Configuration**:
- Resolution: `PWM_RES_BITS`
- Frequency: 50Hz (20ms period)
- Duty cycle: Calculated from 1000-2000μs pulse width

### System Constants (config.h)

```cpp
// Control loop configuration
RATE_LOOP_HZ    // Control loop frequency (250Hz)

// Sensor fusion
ALPHA           // Complementary filter coefficient (if used)

// PID defaults
// - Roll PID gains (kp, ki, kd)
// - Pitch PID gains (kp, ki, kd)
// - Yaw PID gains (kp, ki, kd)

// Hardware pins
// - I2C pins (SDA/SCL for both buses)
// - PWM output pins (EDF + servos)
// - Status LED pin
// - CRSF UART pins

// PWM configuration
PWM_RES_BITS    // PWM resolution (12-bit = 4096 levels)
// - PWM frequency settings
// - Servo pulse width limits (1000-2000μs)

// Serial communication
BAUD_RATE       // USB-CDC serial speed
```

**Design Philosophy**: All tunable parameters centralized in config.h for easy modification without changing core logic.

---

## Configuration Guide

### Control Loop Rate

Default: **250Hz** (`RATE_LOOP_HZ` in `config.h`)

This is a balance between:
- **Higher rates** (500Hz+): More responsive but higher CPU load
- **Lower rates** (100Hz): More CPU headroom but less responsive

### PID Tuning

PID gains are defined in `config.h`:

```cpp
// Roll PID
ROLL_KP, ROLL_KI, ROLL_KD

// Pitch PID  
PITCH_KP, PITCH_KI, PITCH_KD

// Yaw PID
YAW_KP, YAW_KI, YAW_KD
```

**Tuning procedure**:
1. Start with conservative gains (low Kp, zero Ki and Kd)
2. Increase Kp until oscillations appear, then reduce by 20-30%
3. Add Kd to dampen oscillations
4. Add Ki slowly to eliminate steady-state error
5. Test in controlled environment before full flight

### Sensor Fusion

Two algorithms available:

1. **Madgwick Filter** (default, active)
   - More accurate, especially with magnetometer
   - Handles gyro drift well
   - Slightly higher CPU usage

2. **Complementary Filter** (alternative)
   - Simpler, faster computation
   - Good for high-rate loops
   - Enable in `src/main.cpp` control loop

### CRSF Channel Mapping

Channels are mapped in `src/main.cpp`:

```cpp
Throttle = Channel 3 (array index 2)
Roll     = Channel 1 (array index 0)
Pitch    = Channel 2 (array index 1)
Yaw      = Channel 4 (array index 3)
```

Modify if your transmitter uses different channel assignments.

---

## Safety Features

### Arming Sequence

```cpp
bool armed;  // Global arming state

// During initialization
if (motor_arm())
{
    Serial.print("\n!!!MOTORS ARMED...FLIP MOTOR SWITCH!!!\n");
    setrgb(255, 0, 0);  // Red warning
    while (true) delay(1);  // Halt if motors armed during init
}

// After successful init
motor_arm();
armed = true;
```

**Safety Logic**: System halts if motors are armed during initialization to prevent unintended activation.

### Sensor Validation

```cpp
if (!sox.getEvent(&accel, &gyro, &temp))
{
    Serial.println("\n*** Failed to get SOX event ***\n");
    ready = false;
    continue;  // Skip control loop iteration
}
```

**Behavior**: Control outputs are only updated when all sensors provide valid data.

### Input Clamping

```cpp
void setServoMicroseconds(uint8_t channel, uint16_t microseconds)
{
    if (microseconds < 1000) microseconds = 1000;
    if (microseconds > 2000) microseconds = 2000;
    // ... set PWM duty cycle
}
```

**Range Enforcement**: All servo commands are clamped to safe 1000-2000μs range.

### Summary

1. **Arming check**: Motors must be disarmed during initialization
2. **Sensor validation**: Control loop skips if sensor reads fail
3. **Watchdog protection**: FreeRTOS task monitoring
4. **Input clamping**: All servo commands limited to safe 1000-2000μs range
5. **Mutex protection**: Thread-safe data access between cores

---

## Troubleshooting

### Build Issues

**Problem**: "Library not found"
- **Solution**: Run `pio pkg install` to fetch dependencies

**Problem**: Upload fails
- **Solution**: Hold BOOT button while connecting USB, or check USB cable

**Problem**: Compilation errors in sensor libraries
- **Solution**: Check PlatformIO library versions in platformio.ini
- **Solution**: Update libraries with `pio pkg update`

### Runtime Issues

**Problem**: Red LED on startup
- **Solution**: Check serial output for specific sensor initialization error
- Verify I2C connections and sensor addresses
- Use `i2cdetect` to scan for devices

**Problem**: Gyro drift or wrong orientation
- **Solution**: Check sensor mounting orientation
- Verify magnetometer calibration
- Adjust Madgwick filter parameters

**Problem**: Oscillations or unstable flight
- **Solution**: Reduce PID gains, especially Kp and Kd
- Check for mechanical vibrations affecting sensors
- Verify control surface directions
- Ensure CG (center of gravity) is correct

**Problem**: No CRSF receiver data
- **Solution**: Check UART connections
- Verify receiver is bound and powered
- Confirm baud rate matches receiver (typically 416666)
- Check for correct TX/RX pin assignment

**Problem**: Erratic sensor readings
- **Solution**: Check I2C pull-up resistors (typically 4.7kΩ)
- Verify power supply stability (sensors need clean 3.3V)
- Check for electromagnetic interference from motors

**Problem**: System freezes or watchdog resets
- **Solution**: Monitor loop timing with performance metrics
- Check for blocking operations in control loop
- Verify adequate stack size for tasks

---

## Future Enhancements

### Planned Features

- [ ] **Altitude hold**: Enable barometer altitude calculation and implement vertical PID
- [ ] **WiFi telemetry**: Activate `wifi.cpp` for ground station communication
- [ ] **Blackbox logging**: Add SD card support for flight data recording
- [ ] **In-flight tuning**: Web interface for PID adjustment
- [ ] **GPS integration**: Add position hold and waypoint navigation
- [ ] **Failsafe modes**: Loss of signal handling and return-to-home

### Altitude Hold Implementation

Code stub exists for barometric altitude calculation:

```cpp
// Currently disabled
// if (seaLevelPressure == 0) 
//     seaLevelPressure = barometer.getSeaLevel(current_temperature);
// float altitude = barometer.getAltitude(true);
```

**Implementation Path**: Enable altitude calculation and implement PID altitude controller.

### Telemetry Logging

Core 1 loop currently disabled but designed for:
- Attitude change detection
- Performance statistics
- System health monitoring

```cpp
// Commented implementation exists
// - Delta-based attitude logging (threshold: 1.5°)
// - 5-second summary statistics
// - Heap/stack monitoring
```

### Network Interface

WiFi library included but unused:
```cpp
#include <WiFi.h>
// Could enable:
// - Web-based configuration
// - Real-time telemetry streaming
// - Ground station communication
```

---

## Development Notes

### Complementary Filter

```cpp
CF cf(ALPHA);  // Declared but unused in favor of Madgwick
```

**Status**: The codebase includes a complementary filter implementation (cf.cpp/h) as an alternative to the Madgwick filter.

**Current Configuration**: Madgwick filter is the active sensor fusion algorithm.

**Use Case**: Complementary filters are simpler and computationally lighter than Madgwick but may be less accurate. The CF implementation exists as a fallback or alternative for systems with tighter processing constraints.

**Switching**: To use CF instead of Madgwick, modify the sensor fusion section in main.cpp's control loop.

### Code Cleanup Opportunities

1. **WiFi Module**: wifi.cpp exists but is commented out in main.cpp. Decision needed:
   - Remove if not planned for production
   - Implement web-based configuration interface
   - Add telemetry streaming capability

2. **Sensor Fusion Strategy**: Both Madgwick and CF available:
   - Document switching procedure
   - Performance comparison testing
   - Consider removing unused option

3. **Core 1 Logging**: Currently disabled. Options:
   - Enable delta-based attitude logging
   - Implement performance monitoring dashboard
   - Add flight data recording

4. **Channel Mapping Documentation**: Add comments explaining CRSF channel remapping rationale (why 3→2, 1→3, etc.)

5. **PID Tuning**: Document PID gain tuning procedure and flight test methodology

### Performance Optimization

1. **Local Variables**: Control loop uses local containers (`lgx`, `lgy`, etc.) to minimize mutex contention
2. **Batch Sensor Reads**: Single `getEvent()` call retrieves all IMU data
3. **Minimal Locking**: Mutex held only during state structure updates (< 50μs typical)

---

## API Reference

### Helper Functions

#### RGB Status LED
```cpp
void setrgb(uint8_t red, uint8_t green, uint8_t blue)
```
Sets Neopixel LED color (0-255 per channel).

#### PWM Conversion
```cpp
uint32_t servoMicrosecondsToDuty(uint16_t microseconds)
```
Converts servo pulse width (μs) to PWM duty cycle value.

#### Servo Control
```cpp
void setServoMicroseconds(uint8_t channel, uint16_t microseconds)
```
Sets servo position with automatic 1000-2000μs clamping.

### External Module Functions

#### Motor Control (control.h/cpp)
```cpp
void motor_init();                    // Initialize PWM channels
bool motor_arm();                     // Check arming switch state
void motor_update_from_crsf();        // Update outputs from receiver
// Additional control loop implementation pieces
```

#### CRSF Protocol (crsf_rc.h/cpp)
```cpp
void crsf_init();                     // Initialize UART receiver
void crsf_update();                   // Process incoming data
int16_t crsf_get_channel(uint8_t n);  // Read channel value (172-1811)
// Channel parsing and API implementation
```

#### PID Controller (pid.h/cpp)
```cpp
void pid_init(PIDController *pid, float kp, float ki, float kd);
float pid_compute(PIDController *pid, float setpoint, float measured, float dt);
void pid_reset(PIDController *pid);
// PID implementation for stabilization loops
```

#### Complementary Filter (cf.h/cpp)
```cpp
void cf_init(CF *filter, float alpha);
void cf_update(CF *filter, float gyro, float accel, float dt);
float cf_get_angle(CF *filter);
// Alternative to Madgwick for sensor fusion
```

---

## Glossary

| Term | Definition |
|------|------------|
| **AHRS** | Attitude and Heading Reference System |
| **CF** | Complementary Filter (sensor fusion algorithm) |
| **CRSF** | Crossfire (TBS Crossfire RC protocol) |
| **DXA** | Twentieths of a point (1/1440 inch) |
| **EDF** | Electric Ducted Fan |
| **ESP32-S3** | Espressif's dual-core Xtensa LX7 microcontroller with WiFi/BLE |
| **IMU** | Inertial Measurement Unit |
| **LEDC** | LED PWM Controller (ESP32 peripheral) |
| **Madgwick** | Sensor fusion algorithm by Sebastian Madgwick |
| **PID** | Proportional-Integral-Derivative controller |
| **PSRAM** | Pseudo-Static RAM (external memory on ESP32-S3) |
| **PWM** | Pulse Width Modulation |
| **RTOS** | Real-Time Operating System (FreeRTOS) |
| **USB-CDC** | USB Communications Device Class (virtual serial port) |

---

## Resources

- [ESP32-S3 Technical Reference](https://www.espressif.com/sites/default/files/documentation/esp32-s3_technical_reference_manual_en.pdf)
- [PlatformIO Documentation](https://docs.platformio.org/)
- [Madgwick AHRS Algorithm](https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/)
- [CRSF Protocol](https://github.com/crsf-wg/crsf/wiki)
- [PID Tuning Guide](https://oscarliang.com/pid/)
- [FreeRTOS Documentation](https://www.freertos.org/Documentation/RTOS_book.html)

---

## Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/new-feature`)
3. Commit changes (`git commit -am 'Add new feature'`)
4. Push to branch (`git push origin feature/new-feature`)
5. Open a Pull Request

## License

[Specify your license here]

## Authors

[Your name/team]

## Acknowledgments

- Adafruit for sensor libraries
- TBS/ExpressLRS for CRSF protocol
- Sebastian Madgwick for AHRS algorithm
- FreeRTOS team
- Espressif ESP32 community

---

## Revision History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2026-01-31 | Initial comprehensive documentation |

---

**End of Documentation**
