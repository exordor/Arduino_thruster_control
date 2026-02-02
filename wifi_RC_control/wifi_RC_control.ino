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

// WiFi network list - add all available networks here
// System will try to connect in order until successful
const int MAX_WIFI_NETWORKS = 5;

struct WifiNetwork {
  const char* ssid;
  const char* password;
  IPAddress local_ip;
  IPAddress gateway;
  IPAddress subnet;
  bool use_dhcp;  // true = use DHCP, false = use static IP
};

WifiNetwork wifiNetworks[MAX_WIFI_NETWORKS] = {
    // Network 1: IGE (static IP)
  {
    .ssid = "IGE-Geomatics-sense-mobile",
    .password = "kmhxTFWNQKH-BCiBz9Co",
    .local_ip = IPAddress(192, 168, 50, 100),
    .gateway = IPAddress(192, 168, 50, 1),
    .subnet = IPAddress(255, 255, 255, 0),
    .use_dhcp = false
  },
  // Network 2: ISSE (static IP)
  {
    .ssid = "ISSE_2.4",
    .password = "TheAnswerIs42",
    .local_ip = IPAddress(192, 168, 50, 100),
    .gateway = IPAddress(192, 168, 50, 1),
    .subnet = IPAddress(255, 255, 255, 0),
    .use_dhcp = false
  },
  // Network 3: Example with DHCP
  // {
  //   .ssid = "Your-WiFi-Name",
  //   .password = "Your-Password",
  //   .local_ip = IPAddress(0, 0, 0, 0),
  //   .gateway = IPAddress(0, 0, 0, 0),
  //   .subnet = IPAddress(0, 0, 0, 0),
  //   .use_dhcp = true
  // },
  // Network 4: Another network (uncomment and configure)
  // {
  //   .ssid = "Another-WiFi",
  //   .password = "password",
  //   .local_ip = IPAddress(192, 168, 1, 100),
  //   .gateway = IPAddress(192, 168, 1, 1),
  //   .subnet = IPAddress(255, 255, 255, 0),
  //   .use_dhcp = false
  // },
  // Network 5: Add more networks as needed (max 5)
};

// Track which network is currently connected
int currentNetworkIndex = -1;

// WiFi auto-reconnect configuration
const unsigned long WIFI_CHECK_INTERVAL_MS = 2000;  // Check WiFi every 2 seconds
const unsigned long WIFI_RECONNECT_DELAY_MS = 5000; // Wait 5s before reconnect attempt
const int MAX_RECONNECT_ATTEMPTS = 3;               // Max reconnect attempts per network
unsigned long lastWifiCheckMs = 0;
unsigned long lastWifiDisconnectMs = 0;
int reconnectAttemptCount = 0;
bool reconnectInProgress = false;

const unsigned int server_port = 8888;
WiFiServer wifiServer(server_port);
WiFiClient rosClient;

// === Pin Configuration ===
const int CH_RIGHT_IN = 2;     // RC Right channel PWM input (Pin 2)
const int CH_LEFT_IN = 3;      // RC Left channel PWM input (Pin 3)
const int ESC_RIGHT_OUT = 9;   // Right ESC signal output
const int ESC_LEFT_OUT = 10;   // Left ESC signal output

// === Timing Constants ===
const unsigned long RC_FAILSAFE_MS = 200;            // RC signal timeout
const unsigned long WIFI_CMD_TIMEOUT_MS = 1000;      // WiFi command timeout (more tolerant)
const unsigned long WIFI_GRACE_MS = 400;             // Hold-last grace after timeout
const int DECAY_STEP_US = 5;                         // Soft decay step toward 1500
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
// RC state - interrupt-based capture
volatile unsigned long rRiseMicros = 0, rPulseMicros = 0;
volatile unsigned long lRiseMicros = 0, lPulseMicros = 0;
unsigned long lastRcUpdateL = 0;
unsigned long lastRcUpdateR = 0;

// RC filtering and smoothing
int rcAvgL = ESC_MID;
int rcAvgR = ESC_MID;
const int FILTER_ALPHA = 20;    // Filter strength (20%)
const int MAX_STEP_US = 10;     // Ramp limiting (10µs per cycle)

// === PWM 档位分级设置 ===
const bool ENABLE_GEAR_MODE = true;  // 是否启用档位模式（false=连续模式）
const int NUM_GEARS = 9;             // 档位数量（含中位档）

// 档位定义：9档模式（后退4档 | 中位 | 前进4档）- 每档间隔100µs
const int GEAR_VALUES[NUM_GEARS] = {
  1100,  // 档位1：后退最高速 (-400µs)
  1200,  // 档位2：后退高速   (-300µs)
  1300,  // 档位3：后退中速   (-200µs)
  1400,  // 档位4：后退低速   (-100µs)
  1500,  // 档位5：中位停止   (0µs)
  1600,  // 档位6：前进低速   (+100µs)
  1700,  // 档位7：前进中速   (+200µs)
  1800,  // 档位8：前进高速   (+300µs)
  1900   // 档位9：前进最高速 (+400µs)
};

// 档位切换阈值（基于实际遥控器输出范围平均分配）
// 假设实际范围约 1000-2000µs，每档约 111µs
const int GEAR_THRESHOLDS[NUM_GEARS - 1] = {
  1056,  // < 1056: 档位1 (1000 + 56)
  1167,  // 1056-1167: 档位2 (+111)
  1278,  // 1167-1278: 档位3 (+111)
  1389,  // 1278-1389: 档位4 (+111)
  1500,  // 1389-1500: 档位5 中位 (+111)
  1611,  // 1500-1611: 档位6 (+111)
  1722,  // 1611-1722: 档位7 (+111)
  1833   // 1722-1833: 档位8 (+111), > 1833: 档位9
};

int rcOutL = ESC_MID;
int rcOutR = ESC_MID;

// WiFi state
unsigned long lastWifiCmdMs = 0;
int wifiOutL = ESC_MID;
int wifiOutR = ESC_MID;
bool haveWifiCmd = false;

// WiFi command buffer (using char array instead of String to avoid memory fragmentation)
#define CMD_BUFFER_SIZE 64
static char cmdBuffer[CMD_BUFFER_SIZE];
static int cmdBufferIndex = 0;

// WiFi command rate limiting (prevent excessive commands from overheating motors/ESCs)
const unsigned long MIN_CMD_INTERVAL_MS = 100;  // Minimum 100ms between commands (max 10 commands/sec)
unsigned long lastWifiCommandSentMs = 0;

// WiFi filtering and smoothing
int wifiAvgL = ESC_MID;
int wifiAvgR = ESC_MID;

// Current output
int currentLeftUs = ESC_MID;
int currentRightUs = ESC_MID;
int currentMode = 0;  // 0=RC, 1=WiFi

// Status sending
unsigned long lastStatusSendMs = 0;

// === Helper Functions ===

// Right channel interrupt handler
void onRightChange() {
  int level = digitalRead(CH_RIGHT_IN);
  unsigned long now = micros();
  if (level == HIGH) {
    rRiseMicros = now;
  } else {
    unsigned long width = now - rRiseMicros;
    // Only accept valid pulse widths (800-2200µs)
    if (width >= 800 && width <= 2200) {
      rPulseMicros = width;
    }
  }
}

// Left channel interrupt handler
void onLeftChange() {
  int level = digitalRead(CH_LEFT_IN);
  unsigned long now = micros();
  if (level == HIGH) {
    lRiseMicros = now;
  } else {
    unsigned long width = now - lRiseMicros;
    if (width >= 800 && width <= 2200) {
      lPulseMicros = width;
    }
  }
}

// 将输入信号映射到档位
int mapToGear(unsigned long inUs) {
  if (inUs == 0) return ESC_MID; // 超时/无脉冲
  if ((int)inUs < RX_VALID_MIN || (int)inUs > RX_VALID_MAX) return ESC_MID; // 非法范围
  if (abs((int)inUs - 1500) <= DEADBAND_US) return ESC_MID; // 中位保持
  
  // 根据输入值选择档位（9档）
  for (int i = 0; i < NUM_GEARS - 1; i++) {
    if (inUs < GEAR_THRESHOLDS[i]) {
      return GEAR_VALUES[i];
    }
  }
  return GEAR_VALUES[NUM_GEARS - 1];  // 最高档位
}

int mapLinearToEsc(long pulseUs) {
  if (pulseUs == 0) return ESC_MID; // Timeout/no pulse
  
  // Out of valid range → neutral
  if (pulseUs < RX_VALID_MIN || pulseUs > RX_VALID_MAX) {
    return ESC_MID;
  }
  
  // Center deadband
  if (abs((int)pulseUs - 1500) <= DEADBAND_US) {
    return ESC_MID;
  }
  
  // Map RX range to ESC range
  long mapped = map(pulseUs, RX_VALID_MIN, RX_VALID_MAX, ESC_MIN, ESC_MAX);
  mapped = constrain(mapped, ESC_MIN, ESC_MAX);
  
  return (int)mapped;
}

// 统一映射函数：根据模式选择连续或档位映射
int mapToEsc(unsigned long inUs) {
  if (ENABLE_GEAR_MODE) {
    return mapToGear(inUs);
  } else {
    return mapLinearToEsc(inUs);
  }
}

void readRcInputs() {
  unsigned long now = millis();
  
  // Read pulse widths from interrupt capture (non-blocking)
  noInterrupts();
  unsigned long inR = rPulseMicros;
  unsigned long inL = lPulseMicros;
  interrupts();
  
  // Update timestamp if valid
  if (inR >= RX_VALID_MIN && inR <= RX_VALID_MAX) lastRcUpdateR = now;
  if (inL >= RX_VALID_MIN && inL <= RX_VALID_MAX) lastRcUpdateL = now;
  
  // Map to ESC range (using gear or linear mode)
  int outR = mapToEsc((long)inR);
  int outL = mapToEsc((long)inL);
  
  // Apply low-pass filter
  int filtR = (rcAvgR * (100 - FILTER_ALPHA) + outR * FILTER_ALPHA) / 100;
  int filtL = (rcAvgL * (100 - FILTER_ALPHA) + outL * FILTER_ALPHA) / 100;
  
  // Apply soft-start ramp limiting
  int deltaR = filtR - rcAvgR;
  if (deltaR > MAX_STEP_US) deltaR = MAX_STEP_US;
  if (deltaR < -MAX_STEP_US) deltaR = -MAX_STEP_US;
  rcAvgR += deltaR;
  
  int deltaL = filtL - rcAvgL;
  if (deltaL > MAX_STEP_US) deltaL = MAX_STEP_US;
  if (deltaL < -MAX_STEP_US) deltaL = -MAX_STEP_US;
  rcAvgL += deltaL;
  
  // Apply RC failsafe
  if (now - lastRcUpdateR > RC_FAILSAFE_MS) rcAvgR = ESC_MID;
  if (now - lastRcUpdateL > RC_FAILSAFE_MS) rcAvgL = ESC_MID;
  
  rcOutR = rcAvgR;
  rcOutL = rcAvgL;
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
  
  while (rosClient.available()) {
    char c = rosClient.read();
    
    if (c == '\n') {
      // Null-terminate the string
      cmdBuffer[cmdBufferIndex] = '\0';
      
      // Process complete command
      if (cmdBufferIndex > 0 && cmdBuffer[0] == 'C' && cmdBuffer[1] == ' ') {
        // Check command rate limit
        unsigned long now = millis();
        if (now - lastWifiCommandSentMs < MIN_CMD_INTERVAL_MS) {
          unsigned long waitTime = MIN_CMD_INTERVAL_MS - (now - lastWifiCommandSentMs);
          Serial.print("⚠ Command too frequent, ignored (wait ");
          Serial.print(waitTime);
          Serial.println("ms)");
          cmdBufferIndex = 0;
          continue;
        }
        
        int leftUs = 0, rightUs = 0;
        if (sscanf(cmdBuffer, "C %d %d", &leftUs, &rightUs) == 2) {
          // Constrain and store raw WiFi commands
          int rawL = constrain(leftUs, ESC_MIN, ESC_MAX);
          int rawR = constrain(rightUs, ESC_MIN, ESC_MAX);
          
          // Apply low-pass filter to WiFi inputs
          int filtL = (wifiAvgL * (100 - FILTER_ALPHA) + rawL * FILTER_ALPHA) / 100;
          int filtR = (wifiAvgR * (100 - FILTER_ALPHA) + rawR * FILTER_ALPHA) / 100;
          
          // Apply soft-start ramp limiting
          int deltaL = filtL - wifiAvgL;
          if (deltaL > MAX_STEP_US) deltaL = MAX_STEP_US;
          if (deltaL < -MAX_STEP_US) deltaL = -MAX_STEP_US;
          wifiAvgL += deltaL;
          
          int deltaR = filtR - wifiAvgR;
          if (deltaR > MAX_STEP_US) deltaR = MAX_STEP_US;
          if (deltaR < -MAX_STEP_US) deltaR = -MAX_STEP_US;
          wifiAvgR += deltaR;
          
          // Smoothed WiFi outputs
          wifiOutL = wifiAvgL;
          wifiOutR = wifiAvgR;
          lastWifiCmdMs = millis();
          lastWifiCommandSentMs = now;  // Update command rate timestamp
          haveWifiCmd = true;
          
          Serial.print("WiFi Command: Left=");
          Serial.print(wifiOutL);
          Serial.print(" Right=");
          Serial.println(wifiOutR);
        }
      }
      // Reset buffer for next command
      cmdBufferIndex = 0;
    } else if (c != '\r') {
      // Add character to buffer if space available
      if (cmdBufferIndex < CMD_BUFFER_SIZE - 1) {
        cmdBuffer[cmdBufferIndex++] = c;
      } else {
        // Buffer overflow - reset
        Serial.println("⚠ Command buffer overflow, reset");
        cmdBufferIndex = 0;
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
    // WiFi has priority, preserve smoothing state
    currentMode = 1;
    currentLeftUs = wifiOutL;
    currentRightUs = wifiOutR;
  } else {
    // No recent WiFi command — apply grace hold then soft decay
    unsigned long age = haveWifiCmd ? (now - lastWifiCmdMs) : WIFI_CMD_TIMEOUT_MS + WIFI_GRACE_MS + 1;
    if (age <= WIFI_CMD_TIMEOUT_MS + WIFI_GRACE_MS) {
      // Hold last WiFi filtered values, decay toward neutral
      currentMode = 1; // still treat as WiFi during grace
      currentLeftUs = wifiOutL;
      currentRightUs = wifiOutR;
      if (currentLeftUs > ESC_MID) currentLeftUs -= DECAY_STEP_US; else if (currentLeftUs < ESC_MID) currentLeftUs += DECAY_STEP_US;
      if (currentRightUs > ESC_MID) currentRightUs -= DECAY_STEP_US; else if (currentRightUs < ESC_MID) currentRightUs += DECAY_STEP_US;
      // Keep wifiAvg tracking the decayed values to avoid jumps when WiFi resumes
      wifiAvgL = currentLeftUs;
      wifiAvgR = currentRightUs;
      wifiOutL = wifiAvgL;
      wifiOutR = wifiAvgR;
    } else {
      // Fallback to RC after grace expires
      currentMode = 0;
      currentLeftUs = rcOutL;
      currentRightUs = rcOutR;
    }
  }
}

void updateThrusters() {
  escR.writeMicroseconds(currentRightUs);
  escL.writeMicroseconds(currentLeftUs);
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

// === WiFi Multi-Network Management ===

// Try to reconnect to WiFi (attempt previously connected network first)
bool reconnectWiFi() {
  unsigned long now = millis();
  
  // Wait before reconnect attempt
  if (now - lastWifiDisconnectMs < WIFI_RECONNECT_DELAY_MS) {
    return false;
  }
  
  Serial.println("\n=== WiFi Reconnection Attempt ===");
  
  // Try to reconnect to the previously connected network first
  if (currentNetworkIndex >= 0) {
    Serial.print("Reconnecting to: ");
    Serial.println(wifiNetworks[currentNetworkIndex].ssid);
    
    // Configure IP
    if (!wifiNetworks[currentNetworkIndex].use_dhcp) {
      WiFi.config(wifiNetworks[currentNetworkIndex].local_ip, 
                  wifiNetworks[currentNetworkIndex].gateway, 
                  wifiNetworks[currentNetworkIndex].subnet);
    }
    
    // Disconnect first, then attempt reconnection
    WiFi.disconnect();
    delay(100);
    WiFi.begin(wifiNetworks[currentNetworkIndex].ssid, 
               wifiNetworks[currentNetworkIndex].password);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 10) {
      delay(500);
      Serial.print(".");
      attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\n✓ Reconnected!");
      reconnectAttemptCount = 0;
      reconnectInProgress = false;
      return true;
    }
    
    Serial.println("\n✗ Reconnect failed, trying other networks...");
  }
  
  // If reconnect to previous network failed, try all networks
  reconnectAttemptCount++;
  if (reconnectAttemptCount >= MAX_RECONNECT_ATTEMPTS) {
    Serial.println("Max reconnect attempts reached. Waiting...");
    reconnectInProgress = false;
    reconnectAttemptCount = 0;
    return false;
  }
  
  // Try all networks in order
  for (int i = 0; i < MAX_WIFI_NETWORKS; i++) {
    // Skip empty entries
    if (wifiNetworks[i].ssid == nullptr || strlen(wifiNetworks[i].ssid) == 0) {
      continue;
    }
    
    // Skip the network we just tried to reconnect to
    if (i == currentNetworkIndex) {
      continue;
    }
    
    Serial.print("Trying network [");
    Serial.print(i + 1);
    Serial.print("/");
    Serial.print(MAX_WIFI_NETWORKS);
    Serial.print("]: ");
    Serial.println(wifiNetworks[i].ssid);
    
    // Configure IP
    if (!wifiNetworks[i].use_dhcp) {
      WiFi.config(wifiNetworks[i].local_ip, 
                  wifiNetworks[i].gateway, 
                  wifiNetworks[i].subnet);
    } else {
      WiFi.config(IPAddress(0, 0, 0, 0), IPAddress(0, 0, 0, 0), IPAddress(0, 0, 0, 0));
    }
    
    // Attempt connection
    WiFi.begin(wifiNetworks[i].ssid, wifiNetworks[i].password);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 10) {
      delay(500);
      Serial.print(".");
      attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\n✓ Connected!");
      Serial.print("  Network: ");
      Serial.println(wifiNetworks[i].ssid);
      Serial.print("  IP Address: ");
      Serial.println(WiFi.localIP());
      Serial.print("  RSSI: ");
      Serial.print(WiFi.RSSI());
      Serial.println(" dBm");
      
      currentNetworkIndex = i;
      reconnectAttemptCount = 0;
      reconnectInProgress = false;
      return true;
    }
    
    WiFi.disconnect();
    delay(500);
  }
  
  Serial.println("\n✗ All networks unavailable");
  return false;
}

// Check WiFi status and trigger reconnection if needed
void checkWiFiStatus() {
  unsigned long now = millis();
  
  // Only check at intervals
  if (now - lastWifiCheckMs < WIFI_CHECK_INTERVAL_MS) {
    return;
  }
  lastWifiCheckMs = now;
  
  bool wifiConnected = (WiFi.status() == WL_CONNECTED);
  
  if (!wifiConnected && !reconnectInProgress) {
    // WiFi disconnected - start reconnection process
    if (lastWifiDisconnectMs == 0) {
      lastWifiDisconnectMs = now;
      Serial.println("\n⚠ WiFi link lost!");
    }
    
    reconnectInProgress = true;
    reconnectWiFi();
  } else if (wifiConnected) {
    // WiFi connected - reset disconnect timer
    if (lastWifiDisconnectMs > 0) {
      Serial.println("\n✓ WiFi link restored");
    }
    lastWifiDisconnectMs = 0;
    reconnectInProgress = false;
    reconnectAttemptCount = 0;
  }
}

// Try to connect to WiFi networks in order until successful
bool connectToWiFi() {
  Serial.println("\n=== Attempting WiFi Connection ===");
  
  for (int i = 0; i < MAX_WIFI_NETWORKS; i++) {
    // Skip empty entries
    if (wifiNetworks[i].ssid == nullptr || strlen(wifiNetworks[i].ssid) == 0) {
      continue;
    }
    
    Serial.print("Trying network [");
    Serial.print(i + 1);
    Serial.print("/");
    Serial.print(MAX_WIFI_NETWORKS);
    Serial.print("]: ");
    Serial.println(wifiNetworks[i].ssid);
    
    // Configure IP based on network settings
    if (!wifiNetworks[i].use_dhcp) {
      Serial.print("  Using static IP: ");
      Serial.println(wifiNetworks[i].local_ip);
      WiFi.config(wifiNetworks[i].local_ip, wifiNetworks[i].gateway, wifiNetworks[i].subnet);
    } else {
      Serial.println("  Using DHCP");
      WiFi.config(IPAddress(0, 0, 0, 0), IPAddress(0, 0, 0, 0), IPAddress(0, 0, 0, 0));
    }
    
    // Attempt connection
    WiFi.begin(wifiNetworks[i].ssid, wifiNetworks[i].password);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
      delay(500);
      Serial.print(".");
      attempts++;
    }
    
    // Check if connection successful
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\n✓ Connected!");
      Serial.print("  Network: ");
      Serial.println(wifiNetworks[i].ssid);
      Serial.print("  IP Address: ");
      Serial.println(WiFi.localIP());
      Serial.print("  Gateway: ");
      Serial.println(WiFi.gatewayIP());
      Serial.print("  Subnet: ");
      Serial.println(WiFi.subnetMask());
      Serial.print("  RSSI: ");
      Serial.print(WiFi.RSSI());
      Serial.println(" dBm");
      
      currentNetworkIndex = i;
      return true;
    } else {
      Serial.println("\n✗ Connection failed");
    }
    
    // Disconnect before trying next network
    WiFi.disconnect();
    delay(500);
  }
  
  Serial.println("\n✗ All WiFi connection attempts failed");
  currentNetworkIndex = -1;
  return false;
}

// === Setup ===

void setup() {
  // Initialize Serial
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("\n=== WiFi + RC Thruster Control ===");
  Serial.print("RC Control Mode: ");
  Serial.println(ENABLE_GEAR_MODE ? "Gear Mode (9 gears, 100µs intervals)" : "Continuous Mode");
  Serial.println();
  
  // Configure RC input pins
  pinMode(CH_RIGHT_IN, INPUT);
  pinMode(CH_LEFT_IN, INPUT);
  Serial.println("RC input pins configured");
  
  // Attach interrupts for PWM capture
  attachInterrupt(digitalPinToInterrupt(CH_RIGHT_IN), onRightChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH_LEFT_IN), onLeftChange, CHANGE);
  Serial.println("RC interrupts attached");
  
  // Connect to WiFi (try all networks in order)
  bool wifiConnected = connectToWiFi();
  
  if (wifiConnected) {
    // Start TCP server
    wifiServer.begin();
    Serial.print("TCP Server started on port ");
    Serial.println(server_port);
    Serial.println("Ready for WiFi control commands");
  } else {
    Serial.println("WiFi unavailable - Running in RC only mode");
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
  
  // 0. Check WiFi status and auto-reconnect if needed
  checkWiFiStatus();
  
  // 1. Handle WiFi client connections
  handleClientConnections();
  
  // 2. Read RC inputs (non-blocking with interrupts)
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
    if (wifiLink && currentNetworkIndex >= 0) {
      Serial.print(" (");
      Serial.print(wifiNetworks[currentNetworkIndex].ssid);
      Serial.print(")");
    }
    Serial.print(" | Client: ");
    Serial.print(clientLink ? "CONNECTED" : "DISCONNECTED");
    Serial.print(" | CmdAge=");
    if (haveWifiCmd) {
      Serial.print(now - lastWifiCmdMs);
    } else {
      Serial.print("N/A");
    }
    Serial.print("ms");
    Serial.print(" | L=");
    Serial.print(currentLeftUs);
    Serial.print(" R=");
    Serial.print(currentRightUs);
    Serial.print(" | WiFiRaw L="); Serial.print(wifiOutL);
    Serial.print(" R="); Serial.print(wifiOutR);
    Serial.print(" | WiFiAvg L="); Serial.print(wifiAvgL);
    Serial.print(" R="); Serial.println(wifiAvgR);
  }
  
  delay(5);
}
