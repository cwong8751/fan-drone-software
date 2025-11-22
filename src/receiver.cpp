// this is a pure receiver code, only receives and replies with a "message received" message.
// we can add other things to it later to what its going to do with this data it received

#include <esp_now.h>
#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>

bool simulate = true; // Set to false when using real ESP-NOW
unsigned long lastSim = 0;

WebServer server(80);

int valT = 0, valR = 0, valP = 0, valY = 0;
bool hasTRPY = false;

// ⚙️ Replace with the MAC address of the *other* ESP32
// Example MACs (replace with your own):
// Board 1: 50:78:7D:16:34:48
// Board 2: 98:88:E0:14:D6:F0
uint8_t peerAddress[] = {0x10, 0x51, 0xDB, 0x82, 0x71, 0x18}; // <-- change this for each board

const char *ssid = "wurc2.4";
const char *password = "robotics@washu";

// Message buffer
String messageToSend;
String messageReceived;

// Peer info
esp_now_peer_info_t peerInfo;

void parseTRPY(String msg)
{
  // Example: "T:192,R:992,P:988,Y:992"
  if (!msg.startsWith("T:"))
    return;

  int tIndex = msg.indexOf("T:");
  int rIndex = msg.indexOf("R:");
  int pIndex = msg.indexOf("P:");
  int yIndex = msg.indexOf("Y:");

  if (tIndex < 0 || rIndex < 0 || pIndex < 0 || yIndex < 0)
    return;

  valT = msg.substring(tIndex + 2, rIndex - 1).toInt();
  valR = msg.substring(rIndex + 2, pIndex - 1).toInt();
  valP = msg.substring(pIndex + 2, yIndex - 1).toInt();
  valY = msg.substring(yIndex + 2).toInt();

  hasTRPY = true;
}

void generateFakeTRPY()
{
  int T = random(150, 2000); // throttle range
  int R = random(900, 1100); // roll
  int P = random(900, 1100); // pitch
  int Y = random(900, 1100); // yaw

  String fakeMsg = "T:" + String(T) +
                   ",R:" + String(R) +
                   ",P:" + String(P) +
                   ",Y:" + String(Y);

  Serial.print("FAKE INPUT → ");
  Serial.println(fakeMsg);

  parseTRPY(fakeMsg);
}

// 📤 Callback: when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.print("Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

// 📥 Callback: when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  char incomingMessage[250];
  memcpy(incomingMessage, incomingData, len);
  incomingMessage[len] = '\0'; // null-terminate

  Serial.print("Received: ");
  Serial.println(incomingMessage);

  parseTRPY(String(incomingMessage));

  // 📤 Send reply message
  String replyMessage = "message received";
  esp_err_t result = esp_now_send(mac, (uint8_t *)replyMessage.c_str(), replyMessage.length() + 1);

  if (result == ESP_OK)
  {
    Serial.println("Reply sent: message received");
  }
  else
  {
    Serial.println("Error sending reply");
  }
}

void handleRoot()
{
  server.send(200, "text/html",
              "<html>"
              "<head>"
              "<title>ESP32 Live Telemetry</title>"
              "<meta name='viewport' content='width=device-width, initial-scale=1'>"
              "<style>"
              "body { font-family: Arial; padding:20px; }"
              ".box { border:1px solid #444; padding:15px; width:260px; margin-bottom:20px; }"
              "canvas { width:100%; max-width:800px; height:400px; }"
              "</style>"
              "<script src='/chart.js'></script>"
              "</head>"

              "<body>"
              "<h2>ESP32 Live Telemetry</h2>"

              "<div class='box'>"
              "<b>Throttle:</b> <span id='t'>--</span><br>"
              "<b>Roll:</b> <span id='r'>--</span><br>"
              "<b>Pitch:</b> <span id='p'>--</span><br>"
              "<b>Yaw:</b> <span id='y'>--</span><br>"
              "</div>"

              "<canvas id='chart'></canvas>"

              "<script>"
              "const ctx = document.getElementById('chart').getContext('2d');"

              "const chart = new Chart(ctx, {"
              "type: 'line',"
              "data: {"
              "labels: [],"
              "datasets: ["
              "{ label: 'Throttle', borderColor:'red', data:[], fill:false },"
              "{ label: 'Roll', borderColor:'green', data:[], fill:false },"
              "{ label: 'Pitch', borderColor:'blue', data:[], fill:false },"
              "{ label: 'Yaw', borderColor:'orange', data:[], fill:false }"
              "]"
              "},"
              "options: {"
              "animation: false,"
              "responsive: true,"
              "scales: {"
              "x: { display: false },"
              "y: { beginAtZero:false }"
              "}"
              "}"
              "});"

              "function updateData(){"
              "fetch('/data')"
              ".then(res => res.json())"
              ".then(j => {"
              "document.getElementById('t').innerHTML = j.T;"
              "document.getElementById('r').innerHTML = j.R;"
              "document.getElementById('p').innerHTML = j.P;"
              "document.getElementById('y').innerHTML = j.Y;"

              "// ---- Push new data to graph ----"
              "chart.data.labels.push(''); /* blank label for scrolling */"
              "chart.data.datasets[0].data.push(j.T);"
              "chart.data.datasets[1].data.push(j.R);"
              "chart.data.datasets[2].data.push(j.P);"
              "chart.data.datasets[3].data.push(j.Y);"

              "// ---- Limit graph length ----"
              "if (chart.data.labels.length > 80) {" // keep last 80 samples
              "chart.data.labels.shift();"
              "chart.data.datasets.forEach(ds => ds.data.shift());"
              "}"

              "chart.update();"
              "});"
              "}"

              "setInterval(updateData, 200); /* refresh rate */"
              "</script>"
              "</body>"
              "</html>");
}

// ---- JSON TELEMETRY ENDPOINT ----
void handleData()
{
  String json = "{";
  json += "\"T\":" + String(valT) + ",";
  json += "\"R\":" + String(valR) + ",";
  json += "\"P\":" + String(valP) + ",";
  json += "\"Y\":" + String(valY);
  json += "}";

  server.send(200, "application/json", json);
}

void setup()
{
  Serial.begin(115200);

  // mount littlefs for chart.js
  if (!LittleFS.begin(true))
  {
    Serial.println("LittleFS Mount Failed");
    return;
  }
  Serial.println("LittleFS mounted.");

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_AP_STA);

  // connect to wurc wifi so the base station can connect to the internet for chart.js
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi network.");

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println("\nConnected to router!");
    Serial.print("STA IP: ");
    Serial.println(WiFi.localIP());

    // IMPORTANT: match AP channel to router channel for ESP-NOW
    int routerChannel = WiFi.channel();
    Serial.print("Router WiFi channel: ");
    Serial.println(routerChannel);

    // Start the AP *on the same channel*
    WiFi.softAP("esp receiver", "12345678", routerChannel);
  }
  else
  {
    Serial.println("\nCould not connect to router.");

    // fallback AP channel 1
    WiFi.softAP("esp receiver", "12345678", 1);
    Serial.println("Fallback AP started on channel 1");
  }

  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  // Print this board's MAC so you can copy it to the other
  Serial.print("This board MAC Address: ");
  Serial.println(WiFi.macAddress());

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register callbacks
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  // Register peer
  memcpy(peerInfo.peer_addr, peerAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }

  server.on("/", handleRoot);
  server.on("/data", handleData);

  // handler for chart.js we're getting it from littlefs 
  server.on("/chart.js", HTTP_GET, []()
            {
  File file = LittleFS.open("/chart.js", "r");
  if (!file) {
    server.send(500, "text/plain", "Failed to load chart.js");
    return;
  }
  server.streamFile(file, "application/javascript");
  file.close(); });

  server.begin();
  Serial.println("Webserver started.");

  Serial.println("Setup complete. Ready to send and receive messages!");
}

void loop()
{
  server.handleClient();

  if (simulate)
  {
    if (millis() - lastSim >= 300)
    { // generate data every 300ms
      lastSim = millis();
      generateFakeTRPY();
    }
  }
}
