// Load Wi-Fi library
#include <WiFi.h>

// Replace with your network credentials
const char* ssid = "wurc2.4";
const char* password = "robotics@washu";

// Set web server port number to 80
WiFiServer server(80);

// Variable to store the HTTP request
String header;

// Current time
unsigned long currentTime = millis();
// Previous time
unsigned long previousTime = 0; 
// Define timeout time in milliseconds (example: 2000ms = 2s)
const long timeoutTime = 2000;

void setup() {
  Serial.begin(115200);

  // Connect to Wi-Fi network with SSID and password
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
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

void loop(){
  WiFiClient client = server.available();   // Listen for incoming clients

  if (client) {                             // If a new client connects,
    currentTime = millis();
    previousTime = currentTime;
    Serial.println("New Client.");          // print a message out in the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected() && currentTime - previousTime <= timeoutTime) {  // loop while the client's connected
      currentTime = millis();
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        header += c;
        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            if (header.indexOf("GET /data") >= 0) {
              //todo: update roll/pitch/yaw 

              client.println("HTTP/1.1 200 OK");
              client.println("Content-type: application/json");
              client.println("Connection: close");
              client.println();
              client.print("{\"roll\":");
              client.print("replace with actual value");
              client.print(",\"pitch\":");
              client.print("replace with actual value");
              client.print(",\"yaw\":");
              client.print("replace with actual value");
              client.println("}");
              client.println();
              break;
            }
            else {
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
              client.println("setInterval(fetchData, 500);");  // update every 0.5 sec
              client.println("</script>");
              client.println("</body></html>");
              client.println();
              break;
            }
          } else { // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
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