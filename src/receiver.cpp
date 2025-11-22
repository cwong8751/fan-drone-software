// this is a pure receiver code, only receives and replies with a "message received" message.
// we can add other things to it later to what its going to do with this data it received

#include <esp_now.h>
#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>

bool simulate = false; // Set to false when using real ESP-NOW
unsigned long lastSim = 0;

WebServer server(80);

int valT = 0, valR = 0, valP = 0, valY = 0;
bool hasTRPY = false;

float fcRoll = 0, fcPitch = 0, fcYaw = 0;
bool hasFC = false;

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

void parseFC(String msg)
{
  // Example: "R:12.34,P:-4.0,Y:180.2"
  if (!msg.startsWith("R:"))
    return;

  int rIndex = msg.indexOf("R:");
  int pIndex = msg.indexOf("P:");
  int yIndex = msg.indexOf("Y:");

  if (rIndex < 0 || pIndex < 0 || yIndex < 0)
    return;

  fcRoll = msg.substring(rIndex + 2, pIndex - 1).toFloat();
  fcPitch = msg.substring(pIndex + 2, yIndex - 1).toFloat();
  fcYaw = msg.substring(yIndex + 2).toFloat();

  hasFC = true;
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

  if (String(incomingMessage).startsWith("T:"))
  {
    parseTRPY(String(incomingMessage));
  }
  else if (String(incomingMessage).startsWith("R:"))
  {
    parseFC(String(incomingMessage));
  }

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
              "<html><head>"
              "<meta name='viewport' content='width=device-width, initial-scale=1'>"
              "<title>ESP32 Telemetry</title>"
              "<style>"
              "body{font-family:Arial;padding:20px;}"
              ".box{border:1px solid #444;padding:15px;width:260px;margin-bottom:20px;}"
              "canvas{border:1px solid #444;width:100%;max-width:800px;height:300px;}"
              "</style>"
              "</head><body>"

              "<h2>ESP32 Live Telemetry</h2>"
              "<div class='box'>"
              "<b>Throttle:</b> <span id='t'>--</span><br>"
              "<b>Roll:</b> <span id='r'>--</span><br>"
              "<b>Pitch:</b> <span id='p'>--</span><br>"
              "<b>Yaw:</b> <span id='y'>--</span><br>"
              "</div>"

              "<canvas id='chart' width='800' height='300'></canvas>"

              // "<script>"
              // "let c=document.getElementById('chart');"
              // "let ctx=c.getContext('2d');"
              // "let maxPts=200;"
              // "let t=[],r=[],p=[],y=[];"

              // "function drawLine(arr,color){"
              // " if(arr.length<2) return;"
              // " ctx.beginPath(); ctx.strokeStyle=color;"
              // " for(let i=0;i<arr.length;i++){"
              // "   let x=(i/maxPts)*c.width;"
              // "   let yy=c.height - (arr[i]/2000)*c.height;"
              // "   if(i==0) ctx.moveTo(x,yy); else ctx.lineTo(x,yy);"
              // " }"
              // " ctx.stroke();"
              // "}"

              // "function draw(){"
              // " ctx.clearRect(0,0,c.width,c.height);"
              // " drawLine(t,'red');"
              // " drawLine(r,'green');"
              // " drawLine(p,'blue');"
              // " drawLine(y,'orange');"
              // "}"

              // "function update(){"
              // " fetch('/data').then(x=>x.json()).then(j=>{"
              // "   document.getElementById('t').innerHTML=j.T;"
              // "   document.getElementById('r').innerHTML=j.R;"
              // "   document.getElementById('p').innerHTML=j.P;"
              // "   document.getElementById('y').innerHTML=j.Y;"

              // "   t.push(j.T); r.push(j.R); p.push(j.P); y.push(j.Y);"
              // "   if(t.length>maxPts){t.shift();r.shift();p.shift();y.shift();}"

              // "   draw();"
              // " });"
              // "}"

              // "setInterval(update,200);"
              // "</script>"

              "<script>"
              "let c=document.getElementById('chart');"
              "let ctx=c.getContext('2d');"

              "let c2=document.getElementById('fcchart');"
              "let ctx2=c2.getContext('2d');"

              "let maxPts=200;"

              // TRPY arrays (RC transmitter)
              "let t=[],r=[],p=[],y=[];"

              // FC attitude arrays (from monocopter)
              "let fr=[], fp=[], fy=[];"

              // ----- DRAW TRPY GRAPH -----
              "function drawLine(ctx, arr, color, scale){"
              " if(arr.length<2) return;"
              " ctx.beginPath(); ctx.strokeStyle=color;"
              " for(let i=0;i<arr.length;i++){"
              "   let x=(i/maxPts)*ctx.canvas.width;"
              "   let yy=ctx.canvas.height - (arr[i]/scale)*ctx.canvas.height;"
              "   if(i===0) ctx.moveTo(x,yy); else ctx.lineTo(x,yy);"
              " }"
              " ctx.stroke();"
              "}"

              "function drawTRPY(){"
              " ctx.clearRect(0,0,c.width,c.height);"
              " drawLine(ctx,t,'red',2000);"    // throttle
              " drawLine(ctx,r,'green',2000);"  // roll
              " drawLine(ctx,p,'blue',2000);"   // pitch
              " drawLine(ctx,y,'orange',2000);" // yaw
              "}"

              // ----- DRAW FC ORIENTATION GRAPH -----
              "function drawFC(){"
              " ctx2.clearRect(0,0,c2.width,c2.height);"

              " function dl(a,color){"
              "   if(a.length<2) return;"
              "   ctx2.beginPath(); ctx2.strokeStyle=color;"
              "   for(let i=0;i<a.length;i++){"
              "     let x=(i/maxPts)*c2.width;"
              "     let yy=c2.height - ((a[i] + 180) / 360) * c2.height;" // map -180..180
              "     if(i===0) ctx2.moveTo(x,yy); else ctx2.lineTo(x,yy);"
              "   }"
              "   ctx2.stroke();"
              " }"

              " dl(fr,'magenta');"
              " dl(fp,'cyan');"
              " dl(fy,'yellow');"
              "}"

              // ----- UPDATE TRPY DATA (/data) -----
              "function updateTRPY(){"
              " fetch('/data').then(r=>r.json()).then(j=>{"
              "   document.getElementById('t').innerHTML=j.T;"
              "   document.getElementById('r').innerHTML=j.R;"
              "   document.getElementById('p').innerHTML=j.P;"
              "   document.getElementById('y').innerHTML=j.Y;"

              "   t.push(j.T); r.push(j.R); p.push(j.P); y.push(j.Y);"
              "   if(t.length>maxPts){t.shift();r.shift();p.shift();y.shift();}"

              "   drawTRPY();"
              " });"
              "}"

              // ----- UPDATE FLIGHT CONTROLLER DATA (/fcdata) -----
              "function updateFC(){"
              " fetch('/fcdata').then(r=>r.json()).then(j=>{"

              "   fr.push(j.roll);"
              "   fp.push(j.pitch);"
              "   fy.push(j.yaw);"

              "   if(fr.length>maxPts){fr.shift();fp.shift();fy.shift();}"

              "   drawFC();"
              " });"
              "}"

              // ----- CALL BOTH -----
              "setInterval(() => { updateTRPY(); updateFC(); }, 200);"

              "</script>"

              "</body></html>");
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

void handleFCData()
{
  String json = "{";
  json += "\"roll\":" + String(fcRoll) + ",";
  json += "\"pitch\":" + String(fcPitch) + ",";
  json += "\"yaw\":" + String(fcYaw);
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
  server.on("/fcdata", handleFCData);

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
