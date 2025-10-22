#include "config.h"
#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <MPU9250.h>

// constants
#define BAUD_RATE 115200
#define FAIL -1
#define SUCCESS 0

// library MPU9250 object
MPU9250 mpu;

// flight state structure (shared b/n our two cores)
struct FlightState 
{
    // orientation (using library sensor fusion)
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;

    // gyro rates (deg/s)
    float gyro_x = 0.0f;
    float gyro_y = 0.0f;
    float gyro_z = 0.0f;

    // accelerometer (g's)
    float acc_x = 0.0f;
    float acc_y = 0.0f;
    float acc_z = 0.0f;

    // magnetometer (mG)
    float mag_x = 0.0f;
    float mag_y = 0.0f;
    float mag_z = 0.0f;

    // timestamp
    uint32_t timestamp_ms = 0;

    // data ready flag
    volatile bool data_ready = false;
} state;

// mutex for thread-safe access to our flight state
SemaphoreHandle_t state_mutex;

// wifi-related
const char *ssid = "wurc2.4";
const char *password = "robotics@washu";
WiFiServer server(80);
String header;

// high-resolution timing
hw_timer_t *rate_loop_timer = NULL;
hw_timer_t *angle_loop_timer = NULL;
unsigned long currTime = 0;
unsigned long prevTime = 0;
static uint32_t last_print_time = 0;
const uint32_t PRINT_INTERVAL_MS = 50; // 20 Hz
const long timeoutTime = 2000;

// helper function for LEDs indicating FC status
void setrgb(uint8_t red, uint8_t green, uint8_t blue)
{
  neopixelWrite(RGB_pin, red, green, blue);
}

// helper function to init wifi
int initServer()
{
    Serial.print("[WiFi] connecting to ");
    Serial.print(ssid);
    Serial.print("...");
    if(!WiFi.begin(ssid, password))
    {
        Serial.println("ERROR: coulnd't set up WiFi connection. ");
        return FAIL;
    }
    Serial.print("OK.");

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }

    // print local IP address and start web server
    Serial.println("");
    Serial.println("WiFi connected.");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    server.begin();

    return SUCCESS;
}

void updateServer()
{
    WiFiClient client = server.available(); // Listen for incoming clients

    if (client)
    { // If a new client connects,
        currTime = millis();
        prevTime = currTime;
        Serial.println("New Client."); // print a message out in the serial port
        String currentLine = "";       // make a String to hold incoming data from the client
        while (client.connected() && currTime - prevTime <= timeoutTime)
        { // loop while the client's connected
            currTime = millis();
            if (client.available())
            {                           // if there's bytes to read from the client,
                char c = client.read(); // read a byte, then
                Serial.write(c);        // print it out the serial monitor
                header += c;
                if (c == '\n')
                { // if the byte is a newline character
                    // if the current line is blank, you got two newline characters in a row.
                    // that's the end of the client HTTP request, so send a response:
                    if (currentLine.length() == 0)
                    {
                        if (header.indexOf("GET /data") >= 0)
                        {
                            // todo: update roll/pitch/yaw

                            client.println("HTTP/1.1 200 OK");
                            client.println("Content-type: application/json");
                            client.println("Connection: close");
                            client.println();
                            client.print("{\"roll\":");
                            client.print(state.roll);
                            client.print(",\"pitch\":");
                            client.print(state.pitch);
                            client.print(",\"yaw\":");
                            client.print(0);
                            client.println("}");
                            client.println();
                            break;
                        }
                        else
                        {
                            client.println("HTTP/1.1 200 OK");
                            client.println("Content-type:text/html");
                            client.println("Connection: close");
                            client.println();
                            client.println("<!DOCTYPE html><html>");
                            client.println("<head>");
                            client.println("<meta name='viewport' content='width=device-width, initial-scale=1'>");
                            client.println("<script src='https://cdn.jsdelivr.net/npm/chart.js'></script>");
                            client.println("<style>");
                            client.println("body { font-family: Helvetica; text-align: center; margin: 20px; }");
                            client.println("canvas { max-width: 100%; height: auto; }");
                            client.println("</style>");
                            client.println("</head>");
                            client.println("<body>");
                            client.println("<h1>ESP32 Orientation Plot</h1>");
                            client.println("<canvas id='chart' width='400' height='200'></canvas>");
                            client.println("<script>");
                            client.println("const ctx = document.getElementById('chart').getContext('2d');");
                            client.println("const chart = new Chart(ctx, {type: 'line', data: {");
                            client.println("labels: [], datasets: [");
                            client.println("{label:'Roll', borderColor:'red', fill:false, data:[]},");
                            client.println("{label:'Pitch', borderColor:'green', fill:false, data:[]},");
                            client.println("{label:'Yaw', borderColor:'blue', fill:false, data:[]}");
                            client.println("]}, options: {");
                            client.println("responsive: true, animation: false,");
                            client.println("scales: {y: {min:-180, max:180, title:{display:true, text:'Degrees'}}}");
                            client.println("}});");
                            client.println("function fetchData(){fetch('/data').then(r=>r.json()).then(d=>{");
                            client.println("const t = new Date().toLocaleTimeString();");
                            client.println("chart.data.labels.push(t);");
                            client.println("chart.data.datasets[0].data.push(d.roll);");
                            client.println("chart.data.datasets[1].data.push(d.pitch);");
                            client.println("chart.data.datasets[2].data.push(d.yaw);");
                            client.println("if(chart.data.labels.length>50){");
                            client.println("chart.data.labels.shift();");
                            client.println("chart.data.datasets.forEach(s=>s.data.shift());}");
                            client.println("chart.update();");
                            client.println("});}");
                            client.println("setInterval(fetchData, 500);"); // update every 0.5 sec
                            client.println("</script>");
                            client.println("</body></html>");
                            client.println();
                            break;
                        }
                    }
                    else
                    { // if you got a newline, then clear currentLine
                        currentLine = "";
                    }
                }
                else if (c != '\r')
                {                     // if you got anything else but a carriage return character,
                    currentLine += c; // add it to the end of the currentLine
                }
            }
        }
        // Clear the header variable
        header = "";
        // Close the connection
        client.stop();
        Serial.println("Client disconnected.");
        Serial.println("");
    }
}

// ===== CORE 0 =====
void controlLoop(void *parameter)
{
    const uint32_t rate_period_us = 1000000 / RATE_LOOP_HZ;

    TickType_t prev_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1000 / RATE_LOOP_HZ);

    uint32_t loop_counter = 0;

    // performance tracking
    uint32_t mpu_read_time_us = 0;
    uint32_t max_mpu_read_time_us = 0;
    uint32_t total_loop_time_us = 0;

    Serial.println("[ControlLoop] Task started on Core 0");

    while (true)
    {
        uint32_t loop_start = micros();

        uint32_t mpu_start = micros();

        if (mpu.update())
        {
            mpu_read_time_us = micros() - mpu_start;

            if (mpu_read_time_us > max_mpu_read_time_us)
            {
                max_mpu_read_time_us = mpu_read_time_us;
            }

            if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(5)) == pdTRUE)
            {
                // library madgwick/mahony filter sensor fusion
                state.roll = mpu.getRoll();
                state.pitch = mpu.getPitch();
                state.yaw = mpu.getYaw();

                state.gyro_x = mpu.getGyroX();
                state.gyro_y = mpu.getGyroY();
                state.gyro_z = mpu.getGyroZ();

                /*
                state.acc_x = mpu.getAccX();
                state.acc_y = mpu.getAccY();
                state.acc_z = mpu.getAccZ();

                state.mag_x = mpu.getMagX();
                state.mag_y = mpu.getMagY();
                state.mag_z = mpu.getMagZ();
                */

                state.timestamp_ms = millis();
                state.data_ready = true;
                
                xSemaphoreGive(state_mutex);
            }

            total_loop_time_us = micros() - loop_start;
            float cpu_usage = (total_loop_time_us / (float)period) * 100.0f;

            loop_counter++;
            if (loop_counter % 1000 == 0)
            {
                Serial.printf("[ControlLoop] Rate: %d Hz | CPU: %.1f%% | MPU Read: %lu us (max: %lu us) | total loop: %lu us\n", RATE_LOOP_HZ, cpu_usage, mpu_read_time_us, max_mpu_read_time_us, total_loop_time_us);
                max_mpu_read_time_us = 0;
            }

            vTaskDelayUntil(&prev_wake, period);
        }
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
    Wire.setClock(I2C_FREQ); 
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

    // calibrate sensors (assumes device is stationary, optional in the future)
    mpu.calibrateAccelGyro();
    Serial.print("GYRO/ACCEL OK...");
    delay(1000);
    mpu.calibrateMag();
    Serial.print("MAG OK.\n");

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
        2,
        NULL,
        0
    );

    setrgb(0, 255, 0);
}

// ===== CORE 1 =====
void loop() 
{
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
        
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
