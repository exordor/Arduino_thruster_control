#include <WiFiS3.h>
#include <Servo.h>

// WiFi credentials
const char* ssid = "ISSE_2.4";
const char* password = "TheAnswerIs42";

// Static IP configuration (optional but recommended)
IPAddress local_IP(192, 168, 50, 100);
IPAddress gateway(192, 168, 50, 1);
IPAddress subnet(255, 255, 255, 0);

// TCP Server configuration
const unsigned int server_port = 8888;
WiFiServer wifiServer(server_port);
WiFiClient rosClient;

// ESC pins
const int ESC_LEFT_OUT = 10;
const int ESC_RIGHT_OUT = 9;

// ESC ranges
const int ESC_MIN = 1100;
const int ESC_MID = 1500;
const int ESC_MAX = 1900;

// Servo objects
Servo escL, escR;

// Thruster state
int currentLeftUs = ESC_MID;
int currentRightUs = ESC_MID;
unsigned long lastStatusSendMs = 0;
const unsigned long STATUS_SEND_INTERVAL_MS = 100;  // Send status every 100ms (~10Hz)

void handleClientConnections() {
  // Check for new client connections
  if (!rosClient.connected()) {
    rosClient = wifiServer.available();
    if (rosClient) {
      Serial.println("New ROS client connected!");
    }
  }
}

void sendStatus() {
  if (!rosClient.connected()) {
    return;  // Cannot send if not connected
  }
  
  unsigned long now = millis();
  if (now - lastStatusSendMs < STATUS_SEND_INTERVAL_MS) {
    return;  // Not time to send yet
  }
  
  lastStatusSendMs = now;
  
  // Send status: "S <left_us> <right_us>"
  char statusBuf[50];
  snprintf(statusBuf, sizeof(statusBuf), "S %d %d\n", currentLeftUs, currentRightUs);
  
  if (rosClient.print(statusBuf)) {
    // Status sent successfully (optional: log for debugging)
  } else {
    Serial.println("Failed to send status to ROS");
  }
}

void setup() {
  // Initialize Serial for debugging
  Serial.begin(115200);
  delay(2000);  // Wait for Serial to be ready
  
  Serial.println("\n\nStarting WiFi connection...");
  
  // Configure static IP (optional)
  WiFi.config(local_IP, gateway, subnet);
  Serial.println("Static IP configured");
  
  // Connect to WiFi
  WiFi.begin(ssid, password);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected to WiFi!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.print("RSSI (signal strength): ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
  } else {
    Serial.println("\nFailed to connect to WiFi");
  }
  
  // Start TCP server
  wifiServer.begin();
  Serial.print("TCP Server started on port ");
  Serial.println(server_port);
  Serial.print("Waiting for ROS connection at ");
  Serial.print(WiFi.localIP());
  Serial.print(":");
  Serial.println(server_port);
  
  // Initialize ESCs
  escL.attach(ESC_LEFT_OUT);
  escR.attach(ESC_RIGHT_OUT);
  
  // Set neutral position for 2 seconds (ESC calibration)
  escL.writeMicroseconds(ESC_MID);
  escR.writeMicroseconds(ESC_MID);
  Serial.println("ESCs initialized to neutral (1500 µs)");
  delay(2000);
}

void loop() {
  // Check WiFi connection status
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected! Attempting to reconnect...");
    WiFi.begin(ssid, password);
    delay(10000);
  }
  
  // Handle client connections
  handleClientConnections();
  
  // Read incoming data from ROS if connected
  if (rosClient.connected()) {
    static String inputBuffer = "";
    
    while (rosClient.available()) {
      char c = rosClient.read();
      
      if (c == '\n') {
        // Process complete command
        Serial.print("Received: ");
        Serial.println(inputBuffer);
        
        // Parse command: "C <left_us> <right_us>"
        if (inputBuffer.startsWith("C ")) {
          int leftUs = 0, rightUs = 0;
          if (sscanf(inputBuffer.c_str(), "C %d %d", &leftUs, &rightUs) == 2) {
            leftUs = constrain(leftUs, ESC_MIN, ESC_MAX);
            rightUs = constrain(rightUs, ESC_MIN, ESC_MAX);
            
            currentLeftUs = leftUs;
            currentRightUs = rightUs;
            
            escL.writeMicroseconds(leftUs);
            escR.writeMicroseconds(rightUs);
            
            Serial.print("ESC Command: Left=");
            Serial.print(leftUs);
            Serial.print(" Right=");
            Serial.println(rightUs);
          }
        }
        inputBuffer = "";
      } else if (c != '\r') {
        inputBuffer += c;
        if (inputBuffer.length() > 50) {
          inputBuffer = "";  // Prevent buffer overflow
        }
      }
    }
  } else {
    // Set thrusters to neutral if not connected
    escL.writeMicroseconds(ESC_MID);
    escR.writeMicroseconds(ESC_MID);
    currentLeftUs = ESC_MID;
    currentRightUs = ESC_MID;
  }
  
  // Send status to ROS
  sendStatus();
  static unsigned long lastPrintMs = 0;
  if (millis() - lastPrintMs > 5000) {
    lastPrintMs = millis();
    Serial.print("WiFi: ");
    Serial.print(WiFi.status() == WL_CONNECTED ? "CONNECTED" : "DISCONNECTED");
    Serial.print(" | IP: ");
    Serial.print(WiFi.localIP());
    Serial.print(" | ROS: ");
    Serial.println(rosClient.connected() ? "CONNECTED" : "DISCONNECTED");
  }
  
  delay(10);
}
