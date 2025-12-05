#include <WiFiS3.h>
#include <Servo.h>

/*
 * Dual Thrusters Control - WiFi + RC Hybrid Mode
 * 
 * Control Priority:
 *   1. WiFi/ROS commands (if connected and receiving)
 *   2. RC receiver (if WiFi timeout or disconnected)
 *   3. Neutral failsafe (if both unavailable)
 * 
 * Communication:
 *   - WiFi: TCP Server on port 8888
 *   - Command format: C <left_us> <right_us>\n
 *   - Status format: S <mode> <left_us> <right_us>\n
 *   - Mode: 0=RC, 1=WiFi
 */

// === WiFi Configuration ===
const char* ssid = "ISSE_2.4";
const char* password = "TheAnswerIs42";

IPAddress local_IP(192, 168, 50, 100);
IPAddress gateway(192, 168, 50, 1);
IPAddress subnet(255, 255, 255, 0);

const unsigned int server_port = 8888;
WiFiServer wifiServer(server_port);
WiFiClient rosClient;

// === Pin Configuration ===
const int CH_LEFT_IN = 2;      // RC Left channel PWM input
const int CH_RIGHT_IN = 3;     // RC Right channel PWM input
const int ESC_LEFT_OUT = 10;   // Left ESC signal output
const int ESC_RIGHT_OUT = 9;   // Right ESC signal output

// === Timing Constants ===
const unsigned long PULSE_TIMEOUT_US = 25000;        // pulseIn timeout
const unsigned long RC_FAILSAFE_MS = 200;            // RC signal timeout
const unsigned long WIFI_CMD_TIMEOUT_MS = 500;       // WiFi command timeout
const unsigned long STATUS_SEND_INTERVAL_MS = 100;   // Status update rate (10Hz)

// === ESC Configuration ===
const int RX_VALID_MIN = 950;
const int RX_VALID_MAX = 2000;
const int ESC_MIN = 1100;
const int ESC_MID = 1500;
const int ESC_MAX = 1900;
const int DEADBAND_US = 25;

// === Servo Objects ===
Servo escL, escR;

// === State Variables ===
// RC state
unsigned long lastRcUpdateL = 0;
unsigned long lastRcUpdateR = 0;
int rcOutL = ESC_MID;
int rcOutR = ESC_MID;

// WiFi state
unsigned long lastWifiCmdMs = 0;
int wifiOutL = ESC_MID;
int wifiOutR = ESC_MID;
bool haveWifiCmd = false;

// Current output
int currentLeftUs = ESC_MID;
int currentRightUs = ESC_MID;
int currentMode = 0;  // 0=RC, 1=WiFi

// Status sending
unsigned long lastStatusSendMs = 0;

// === Helper Functions ===

int mapLinearToEsc(long pulseUs) {
  // Out of valid range → neutral
  if (pulseUs < RX_VALID_MIN || pulseUs > RX_VALID_MAX) {
    return ESC_MID;
  }
  
  // Map RX range to ESC range
  long mapped = map(pulseUs, RX_VALID_MIN, RX_VALID_MAX, ESC_MIN, ESC_MAX);
  mapped = constrain(mapped, ESC_MIN, ESC_MAX);
  
  // Apply deadband around center
  int rxCenter = (RX_VALID_MIN + RX_VALID_MAX) / 2;
  int distFromCenter = abs((int)pulseUs - rxCenter);
  
  if (distFromCenter < DEADBAND_US) {
    return ESC_MID;
  }
  
  return (int)mapped;
}

void readRcInputs() {
  unsigned long now = millis();
  
  // Read left channel
  unsigned long inL = pulseIn(CH_LEFT_IN, HIGH, PULSE_TIMEOUT_US);
  if (inL >= RX_VALID_MIN && inL <= RX_VALID_MAX) {
    rcOutL = mapLinearToEsc((long)inL);
    lastRcUpdateL = now;
  }
  
  // Read right channel
  unsigned long inR = pulseIn(CH_RIGHT_IN, HIGH, PULSE_TIMEOUT_US);
  if (inR >= RX_VALID_MIN && inR <= RX_VALID_MAX) {
    rcOutR = mapLinearToEsc((long)inR);
    lastRcUpdateR = now;
  }
  
  // Apply RC failsafe
  if (now - lastRcUpdateL > RC_FAILSAFE_MS) {
    rcOutL = ESC_MID;
  }
  if (now - lastRcUpdateR > RC_FAILSAFE_MS) {
    rcOutR = ESC_MID;
  }
}

void handleClientConnections() {
  if (!rosClient.connected()) {
    rosClient = wifiServer.available();
    if (rosClient) {
      Serial.println("New WiFi client connected!");
      haveWifiCmd = false;  // Reset command flag
    }
  }
}

void readWifiCommands() {
  if (!rosClient.connected()) {
    return;
  }
  
  static String inputBuffer = "";
  
  while (rosClient.available()) {
    char c = rosClient.read();
    
    if (c == '\n') {
      // Process complete command
      if (inputBuffer.startsWith("C ")) {
        int leftUs = 0, rightUs = 0;
        if (sscanf(inputBuffer.c_str(), "C %d %d", &leftUs, &rightUs) == 2) {
          wifiOutL = constrain(leftUs, ESC_MIN, ESC_MAX);
          wifiOutR = constrain(rightUs, ESC_MIN, ESC_MAX);
          lastWifiCmdMs = millis();
          haveWifiCmd = true;
          
          Serial.print("WiFi Command: Left=");
          Serial.print(wifiOutL);
          Serial.print(" Right=");
          Serial.println(wifiOutR);
        }
      }
      inputBuffer = "";
    } else if (c != '\r') {
      inputBuffer += c;
      if (inputBuffer.length() > 50) {
        inputBuffer = "";  // Prevent overflow
      }
    }
  }
}

void determineControlMode() {
  unsigned long now = millis();
  
  // Check if WiFi commands are active and recent
  bool wifiActive = haveWifiCmd && 
                    rosClient.connected() && 
                    (now - lastWifiCmdMs < WIFI_CMD_TIMEOUT_MS);
  
  if (wifiActive) {
    // WiFi has priority
    currentMode = 1;
    currentLeftUs = wifiOutL;
    currentRightUs = wifiOutR;
  } else {
    // Fallback to RC
    currentMode = 0;
    currentLeftUs = rcOutL;
    currentRightUs = rcOutR;
  }
}

void updateThrusters() {
  escL.writeMicroseconds(currentLeftUs);
  escR.writeMicroseconds(currentRightUs);
}

void sendStatus() {
  if (!rosClient.connected()) {
    return;
  }
  
  unsigned long now = millis();
  if (now - lastStatusSendMs < STATUS_SEND_INTERVAL_MS) {
    return;
  }
  
  lastStatusSendMs = now;
  
  // Send status: "S <mode> <left_us> <right_us>\n"
  char statusBuf[50];
  snprintf(statusBuf, sizeof(statusBuf), "S %d %d %d\n", 
           currentMode, currentLeftUs, currentRightUs);
  
  rosClient.print(statusBuf);
}

// === Setup ===

void setup() {
  // Initialize Serial
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("\n=== WiFi + RC Thruster Control ===");
  
  // Configure RC input pins
  pinMode(CH_LEFT_IN, INPUT);
  pinMode(CH_RIGHT_IN, INPUT);
  Serial.println("RC input pins configured");
  
  // Configure WiFi
  WiFi.config(local_IP, gateway, subnet);
  Serial.println("Static IP configured");
  
  // Connect to WiFi
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✓ Connected to WiFi!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.print("RSSI: ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
    
    // Start TCP server
    wifiServer.begin();
    Serial.print("TCP Server started on port ");
    Serial.println(server_port);
  } else {
    Serial.println("\n✗ WiFi connection failed - RC only mode");
  }
  
  // Initialize ESCs
  escL.attach(ESC_LEFT_OUT);
  escR.attach(ESC_RIGHT_OUT);
  
  // ESC calibration - neutral for 2 seconds
  escL.writeMicroseconds(ESC_MID);
  escR.writeMicroseconds(ESC_MID);
  Serial.println("ESCs initialized to neutral (1500 µs)");
  delay(2000);
  
  Serial.println("\n=== System Ready ===");
  Serial.println("Control Priority: WiFi > RC > Failsafe");
  Serial.println();
}

// === Main Loop ===

void loop() {
  unsigned long now = millis();
  
  // 1. Handle WiFi client connections
  handleClientConnections();
  
  // 2. Read RC inputs (blocking but fast ~20ms max)
  readRcInputs();
  
  // 3. Read WiFi commands (non-blocking)
  readWifiCommands();
  
  // 4. Determine control mode and outputs
  determineControlMode();
  
  // 5. Update thrusters
  updateThrusters();
  
  // 6. Send status to WiFi client
  sendStatus();
  
  // 7. Connection state transitions (WiFi link vs client connection)
  bool wifiLink = (WiFi.status() == WL_CONNECTED);
  bool clientLink = rosClient.connected();
  static bool prevWifiLink = false;
  static bool prevClientLink = false;
  static bool stateInit = false;
  if (!stateInit) {
    prevWifiLink = wifiLink;
    prevClientLink = clientLink;
    stateInit = true;
  }
  if (wifiLink != prevWifiLink) {
    Serial.print("WiFi link ");
    Serial.println(wifiLink ? "CONNECTED" : "DISCONNECTED");
    prevWifiLink = wifiLink;
  }
  if (clientLink != prevClientLink) {
    Serial.print("Client ");
    Serial.println(clientLink ? "CONNECTED" : "DISCONNECTED");
    prevClientLink = clientLink;
  }
  
  // 8. Print debug status every 5 seconds
  static unsigned long lastDebugMs = 0;
  if (now - lastDebugMs > 5000) {
    lastDebugMs = now;
    
    Serial.print("Mode: ");
    Serial.print(currentMode == 1 ? "WiFi" : "RC");
    Serial.print(" | WiFi link: ");
    Serial.print(wifiLink ? "CONNECTED" : "DISCONNECTED");
    Serial.print(" | Client: ");
    Serial.print(clientLink ? "CONNECTED" : "DISCONNECTED");
    Serial.print(" | L=");
    Serial.print(currentLeftUs);
    Serial.print(" R=");
    Serial.println(currentRightUs);
  }
  
  delay(5);
}
