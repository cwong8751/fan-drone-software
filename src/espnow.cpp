// #include <esp_now.h>
// #include <WiFi.h>

// // ⚙️ Replace with the MAC address of the *other* ESP32
// // Example MACs (replace with your own):
// // Board 1: 50:78:7D:16:34:48
// // Board 2: 98:88:E0:14:D6:F0
// uint8_t peerAddress[] = {0x98, 0x88, 0xE0, 0x14, 0xD6, 0xF0}; // <-- change this for each board

// // Message buffer
// String messageToSend;
// String messageReceived;

// // Peer info
// esp_now_peer_info_t peerInfo;

// // 📤 Callback: when data is sent
// void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
//   Serial.print("Send Status:\t");
//   Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
// }

// // 📥 Callback: when data is received
// void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
//   char incomingMessage[250];
//   memcpy(incomingMessage, incomingData, len);
//   incomingMessage[len] = '\0'; // null-terminate

//   Serial.print("Received: ");
//   Serial.println(incomingMessage);
// }

// void setup() {
//   Serial.begin(115200);

//   // Set device as a Wi-Fi Station
//   WiFi.mode(WIFI_STA);

//   // Print this board's MAC so you can copy it to the other
//   Serial.print("This board MAC Address: ");
//   Serial.println(WiFi.macAddress());

//   // Init ESP-NOW
//   if (esp_now_init() != ESP_OK) {
//     Serial.println("Error initializing ESP-NOW");
//     return;
//   }

//   // Register callbacks
//   esp_now_register_send_cb(OnDataSent);
//   esp_now_register_recv_cb(OnDataRecv);

//   // Register peer
//   memcpy(peerInfo.peer_addr, peerAddress, 6);
//   peerInfo.channel = 0;
//   peerInfo.encrypt = false;

//   if (esp_now_add_peer(&peerInfo) != ESP_OK) {
//     Serial.println("Failed to add peer");
//     return;
//   }

//   Serial.println("Setup complete. Ready to send and receive messages!");
// }

// void loop() {
//   // Send a message every 5 seconds
//   messageToSend = "Hello from " + WiFi.macAddress();
//   esp_err_t result = esp_now_send(peerAddress, (uint8_t *)messageToSend.c_str(), messageToSend.length() + 1);

//   if (result == ESP_OK) {
//     Serial.print("Sent: ");
//     Serial.println(messageToSend);
//   } else {
//     Serial.println("Error sending message");
//   }

//   delay(5000);
// }