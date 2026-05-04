#include <esp_now.h>
#include <WiFi.h>

// ==========================================================
// IMPORTANT: REPLACE THIS WITH YOUR RECEIVER'S MAC ADDRESS
// ==========================================================
uint8_t receiverAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// The data structure MUST match the receiver exactly
typedef struct struct_message {
  int ch1; // Roll
  int ch2; // Pitch
  int ch3; // Throttle
  int ch4; // Yaw
  int ch5; // Aux 1 (Arming switch, etc.)
  int ch6; // Aux 2 (Flight modes, etc.)
} struct_message;

struct_message rcData;
esp_now_peer_info_t peerInfo;

// --- Hardware Setup: Define your joystick/switch pins ---
// Analog pins for joysticks
const int pinThrottle = 34; // Left stick Vertical
const int pinYaw      = 35; // Left stick Horizontal
const int pinPitch    = 32; // Right stick Vertical
const int pinRoll     = 33; // Right stick Horizontal

// Digital pins for toggle switches
const int pinAux1     = 25; 
const int pinAux2     = 26; 

// Callback function to let you know if the data actually reached the drone
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Uncomment the lines below for debugging, but leave them commented for flight 
  // so the serial prints don't slow down your control loop!
  
  // Serial.print("\r\nLast Packet Send Status:\t");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  Serial.begin(115200);

  // Configure Aux switch pins with internal pullups
  pinMode(pinAux1, INPUT_PULLUP);
  pinMode(pinAux2, INPUT_PULLUP);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register the send callback
  esp_now_register_send_cb(OnDataSent);

  // Register the drone (peer)
  memcpy(peerInfo.peer_addr, receiverAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add the drone to the network      
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}

void loop() {
  // 1. Read Analog Joysticks (ESP32 ADC is 12-bit, so it outputs 0-4095)
  int rawThrottle = analogRead(pinThrottle);
  int rawYaw      = analogRead(pinYaw);
  int rawPitch    = analogRead(pinPitch);
  int rawRoll     = analogRead(pinRoll);

  // 2. Map the ADC values to standard RC PWM ranges (1000 to 2000)
  // NOTE: If a stick moves the wrong way on your drone, invert the mapping here.
  // Example inversion: map(rawRoll, 0, 4095, 2000, 1000);
  rcData.ch3 = map(rawThrottle, 0, 4095, 1000, 2000); 
  rcData.ch4 = map(rawYaw,      0, 4095, 1000, 2000); 
  rcData.ch2 = map(rawPitch,    0, 4095, 1000, 2000); 
  rcData.ch1 = map(rawRoll,     0, 4095, 1000, 2000); 

  // 3. Read Aux switches (Mapping HIGH/LOW states to 2000/1000 RC values)
  rcData.ch5 = digitalRead(pinAux1) == HIGH ? 2000 : 1000;
  rcData.ch6 = digitalRead(pinAux2) == HIGH ? 2000 : 1000;

  // 4. Send the data payload via ESP-NOW
  esp_err_t result = esp_now_send(receiverAddress, (uint8_t *) &rcData, sizeof(rcData));
   
  // Wait 10 milliseconds before sending the next packet (creates a 100Hz refresh rate)
  delay(10); 
}