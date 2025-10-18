// #include "ekf.h"
#include "cf.h"
#include <WiFi.h>
#include <Arduino.h>
#include <Wire.h>
#include <MPU9250.h>
#include <cmath>

// constants
#define RGB_pin 48
#define SDA_pin 8
#define SCL_pin 9
#define BAUD_RATE 115200
#define ALPHA 0.98f
#define DT 0.01f
#define GX_BIAS 2.50f
#define GY_BIAS -4.65f
#define GZ_BIAS -1.13f

// filtered angles
float roll = 0.0f;
float pitch = 0.0f;

// raw sensor values
float ax, ay, az;
float gx, gy, gz;

// wifi related
const char *ssid = "wurc2.4";
const char *password = "robotics@washu";
WiFiServer server(80);
String header;
unsigned long currentTime = millis();
unsigned long previousTime = 0;
const long timeoutTime = 2000;

MPU9250 mpu;
CF cf(ALPHA);

// helper function for LEDs indicating FC status
void setrgb(uint8_t red, uint8_t green, uint8_t blue)
{
    neopixelWrite(RGB_pin, red, green, blue);
}

// helper function to init wifi
void initServer()
{
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    // Print local IP address and start web server
    Serial.println("");
    Serial.println("WiFi connected.");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    server.begin();
}

void updateServer()
{
    WiFiClient client = server.available(); // Listen for incoming clients

    if (client)
    { // If a new client connects,
        currentTime = millis();
        previousTime = currentTime;
        Serial.println("New Client."); // print a message out in the serial port
        String currentLine = "";       // make a String to hold incoming data from the client
        while (client.connected() && currentTime - previousTime <= timeoutTime)
        { // loop while the client's connected
            currentTime = millis();
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
                            client.print(roll);
                            client.print(",\"pitch\":");
                            client.print(pitch);
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

void setup()
{
    Serial.begin(BAUD_RATE); // start serial comm at baud rate 115200 for USB

    Serial.println("Initializing peripherals...\n\n");

    Serial.println("Initializing I2C0 bus interface...");

    // intilialize first i2c bus to 8 (SDA) and 9 (SCL)
    Wire.begin(SDA_pin, SCL_pin);
    Wire.setClock(400000); // 400kHz fast mode with MPU9250

    Serial.print("OK.\n");

    Serial.println("Initializing MPU9250 sensor...");

    // initialize MPU9250
    if (!mpu.setup(0x68))
    {
        Serial.println("FAILED.\n");
        setrgb(255, 0, 0);
        while (1)
            delay(100);
    }

    Serial.print("OK.\n");

    Serial.println("Calibrating gyro, accel, and mag sensors...");

    // calibrate sensors (assumes device is stationary, optional in the future)
    mpu.calibrateAccelGyro();

    Serial.print("GYRO/ACCEL OK...");

    mpu.calibrateMag();
    delay(3000);

    Serial.print("OK.\n");

    setrgb(0, 255, 0);

    Serial.println("gyro bias calculation...");

    float gx_bias = 0, gy_bias = 0, gz_bias = 0;
    for (int i = 0; i < 500; i++)
    {
        mpu.update_accel_gyro();
        gx_bias += mpu.getGyroX();
        gy_bias += mpu.getGyroY();
        gz_bias += mpu.getGyroZ();
        delay(2);
    }
    gx_bias /= 500;
    gy_bias /= 500;
    gz_bias /= 500;

    Serial.print("OK");

    Serial.println();
    Serial.print("Biases: ");
    Serial.print(gx_bias);
    Serial.print(", ");
    Serial.print(gy_bias);
    Serial.print(", ");
    Serial.println(gz_bias);

    initServer();

    // embed gyro biases to calculations
    // trial 1: 2.55, -4.69, -1.09
    // trial 2: 2.46, -4.62, -1.17
    // trial 3: 2.49, -4.65, -1.13
    // avg: gx bias of 2.50, gy bias of -4.65, gz bias of -1.13
}

void loop()
{
    if (mpu.update())
    {

        // extract raw gyro and accel data from MPU9250
        float gx = mpu.getGyroX() - GX_BIAS;
        float gy = mpu.getGyroY() - GY_BIAS;
        float gz = mpu.getGyroZ() - GZ_BIAS;

        float ax = mpu.getAccX();
        float ay = mpu.getAccY();
        float az = mpu.getAccZ();

        cf.update(gx, gy, gz, ax, ay, az, DT);

        Serial.print("roll:");
        Serial.print(cf.getRoll());
        Serial.print(",");
        Serial.print("pitch:");
        Serial.println(cf.getPitch());

        // set update
        roll = cf.getRoll();
        pitch = cf.getPitch();

        /*
        float roll = gx;
        float pitch = gy;
        float yaw = gz;

        Serial.print("roll:");
        Serial.print(roll);
        Serial.print(",");
        Serial.print("pitch:");
        Serial.print(pitch);
        Serial.print(",");
        Serial.print("yaw:");
        Serial.println(yaw);
        */

        // update webserver
        updateServer();

        delay(10);
    }
}