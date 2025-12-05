#include <WiFiS3.h>
#include <Servo.h>
#include <EEPROM.h>

/*
 * ============================================
 * Arduino UNO R4 WiFi - Multi-WiFi Control
 * Version: 2.0 - Multi-Profile Management
 * ============================================
 * 
 * Features:
 *   • Multiple WiFi profiles (up to 5)
 *   • Priority-based profile management
 *   • Web-based configuration portal
 *   • RC + WiFi hybrid control
 *   • Persistent EEPROM storage
 *   • Dual thruster ESC control
 * 
 * Default AP:
 *   SSID: ArduinoR4-Config
 *   Password: 12345678
 *   URL: http://192.168.4.1
 * 
 * Serial Commands:
 *   CONFIG - Start WiFi configuration AP
 *   RESET - Clear all EEPROM data
 *   STATUS - Show WiFi connection status
 *   LIST - List all saved profiles
 *   SWITCH <idx> - Switch to profile
 *   DELETE <idx> - Delete profile
 */

// ========== EEPROM Storage Layout ==========
#define EEPROM_MAGIC_ADDR 0
#define EEPROM_VERSION_ADDR 1
#define EEPROM_CONFIG_COUNT_ADDR 2
#define EEPROM_ACTIVE_INDEX_ADDR 3
#define EEPROM_CONFIGS_START 4

#define EEPROM_MAGIC_VALUE 0xA5  // single-byte magic marker
#define EEPROM_VERSION 1
#define MAX_WIFI_CONFIGS 5
#define CONFIG_SLOT_SIZE 163

// Profile field offsets within each slot
#define CONFIG_PRIORITY_OFFSET 0      // 1 byte
#define CONFIG_SSID_OFFSET 1          // 32 bytes
#define CONFIG_PASS_OFFSET 33         // 64 bytes
#define CONFIG_IP_OFFSET 97           // 4 bytes
#define CONFIG_GATEWAY_OFFSET 101     // 4 bytes
#define CONFIG_SUBNET_OFFSET 105      // 4 bytes
#define CONFIG_PORT_OFFSET 109        // 2 bytes

// ========== WiFi Profile Structure ==========
struct WiFiProfile {
  uint8_t priority;
  char ssid[32];
  char password[64];
  uint8_t ip[4];
  uint8_t gateway[4];
  uint8_t subnet[4];
  uint16_t port;
};

// ========== Global Variables ==========
WiFiProfile currentProfile;
WiFiProfile savedProfiles[MAX_WIFI_CONFIGS];
uint8_t configCount = 0;
uint8_t activeConfigIndex = 0;

// WiFi Server and Client
WiFiServer wifiServer(8888);
WiFiClient rosClient;
IPAddress staticIP;
IPAddress gateway;
IPAddress subnet;
uint16_t serverPort = 8888;

// WiFi communication state
unsigned long lastWifiCmdMs = 0;
bool haveWifiCmd = false;

// ========== Pin Configuration ==========
const int CH_LEFT_IN = 2;
const int CH_RIGHT_IN = 3;
const int ESC_LEFT_OUT = 10;
const int ESC_RIGHT_OUT = 9;

// ========== Timing Constants ==========
const unsigned long PULSE_TIMEOUT_US = 25000;
const unsigned long RC_FAILSAFE_MS = 200;
const unsigned long WIFI_CMD_TIMEOUT_MS = 500;
const unsigned long STATUS_SEND_INTERVAL_MS = 100;

// ========== ESC Configuration ==========
const int RX_VALID_MIN = 950;
const int RX_VALID_MAX = 2000;
const int ESC_MIN = 1100;
const int ESC_MID = 1500;
const int ESC_MAX = 1900;

// ========== Servo Objects ==========
Servo escL, escR;

// ========== State Variables ==========
unsigned long lastRcUpdateL = 0;
unsigned long lastRcUpdateR = 0;
int rcOutL = ESC_MID;
int rcOutR = ESC_MID;

int wifiOutL = ESC_MID;
int wifiOutR = ESC_MID;

int currentLeftUs = ESC_MID;
int currentRightUs = ESC_MID;
int currentMode = 0;

unsigned long lastStatusSendMs = 0;
unsigned long lastWifiRetryMs = 0;
const unsigned long WIFI_RETRY_INTERVAL_MS = 30000; // Retry WiFi every 30 seconds if disconnected

// ========== EEPROM Management Functions ==========

void loadAllWiFiConfigs() {
  Serial.println("\n[EEPROM] Loading configurations...");
  
  uint8_t magic = EEPROM.read(EEPROM_MAGIC_ADDR);
  uint8_t version = EEPROM.read(EEPROM_VERSION_ADDR);
  
  if (magic != EEPROM_MAGIC_VALUE || version != EEPROM_VERSION) {
    Serial.println("[EEPROM] No valid data found - using defaults");
    configCount = 0;
    activeConfigIndex = 0;
    setDefaultProfile();
    return;
  }
  
  configCount = EEPROM.read(EEPROM_CONFIG_COUNT_ADDR);
  activeConfigIndex = EEPROM.read(EEPROM_ACTIVE_INDEX_ADDR);
  
  if (configCount > MAX_WIFI_CONFIGS) configCount = 0;
  if (activeConfigIndex >= configCount) activeConfigIndex = 0;
  
  Serial.print("[EEPROM] Found ");
  Serial.print(configCount);
  Serial.print(" profile(s)");
  
  for (uint8_t i = 0; i < configCount; i++) {
    loadWiFiProfile(i);
  }
  
  if (configCount > 0) {
    loadWiFiProfileAsCurrent(activeConfigIndex);
  } else {
    setDefaultProfile();
  }
}

void loadWiFiProfile(uint8_t index) {
  if (index >= MAX_WIFI_CONFIGS) return;
  
  uint16_t offset = EEPROM_CONFIGS_START + (index * CONFIG_SLOT_SIZE);
  
  savedProfiles[index].priority = EEPROM.read(offset + CONFIG_PRIORITY_OFFSET);
  
  for (int i = 0; i < 32; i++) {
    savedProfiles[index].ssid[i] = EEPROM.read(offset + CONFIG_SSID_OFFSET + i);
  }
  savedProfiles[index].ssid[31] = '\0';
  
  for (int i = 0; i < 64; i++) {
    savedProfiles[index].password[i] = EEPROM.read(offset + CONFIG_PASS_OFFSET + i);
  }
  savedProfiles[index].password[63] = '\0';
  
  for (int i = 0; i < 4; i++) {
    savedProfiles[index].ip[i] = EEPROM.read(offset + CONFIG_IP_OFFSET + i);
    savedProfiles[index].gateway[i] = EEPROM.read(offset + CONFIG_GATEWAY_OFFSET + i);
    savedProfiles[index].subnet[i] = EEPROM.read(offset + CONFIG_SUBNET_OFFSET + i);
  }
  
  uint8_t portHigh = EEPROM.read(offset + CONFIG_PORT_OFFSET);
  uint8_t portLow = EEPROM.read(offset + CONFIG_PORT_OFFSET + 1);
  savedProfiles[index].port = (portHigh << 8) | portLow;
  if (savedProfiles[index].port == 0 || savedProfiles[index].port == 0xFFFF) {
    savedProfiles[index].port = 8888;
  }
}

void loadWiFiProfileAsCurrent(uint8_t index) {
  if (index >= configCount || index >= MAX_WIFI_CONFIGS) {
    setDefaultProfile();
    return;
  }
  
  currentProfile = savedProfiles[index];
  activeConfigIndex = index;
  
  staticIP = IPAddress(currentProfile.ip[0], currentProfile.ip[1],
                       currentProfile.ip[2], currentProfile.ip[3]);
  gateway = IPAddress(currentProfile.gateway[0], currentProfile.gateway[1],
                      currentProfile.gateway[2], currentProfile.gateway[3]);
  subnet = IPAddress(currentProfile.subnet[0], currentProfile.subnet[1],
                     currentProfile.subnet[2], currentProfile.subnet[3]);
  serverPort = currentProfile.port;
  
  EEPROM.write(EEPROM_ACTIVE_INDEX_ADDR, activeConfigIndex);
  
  Serial.print("[WiFi] Loaded profile ");
  Serial.print(index);
  Serial.print(": ");
  Serial.println(currentProfile.ssid);
}

void saveWiFiProfile(uint8_t index, WiFiProfile &profile) {
  if (index >= MAX_WIFI_CONFIGS) return;
  
  uint16_t offset = EEPROM_CONFIGS_START + (index * CONFIG_SLOT_SIZE);
  
  EEPROM.write(offset + CONFIG_PRIORITY_OFFSET, profile.priority);
  
  for (int i = 0; i < 32; i++) {
    EEPROM.write(offset + CONFIG_SSID_OFFSET + i, profile.ssid[i]);
  }
  
  for (int i = 0; i < 64; i++) {
    EEPROM.write(offset + CONFIG_PASS_OFFSET + i, profile.password[i]);
  }
  
  for (int i = 0; i < 4; i++) {
    EEPROM.write(offset + CONFIG_IP_OFFSET + i, profile.ip[i]);
    EEPROM.write(offset + CONFIG_GATEWAY_OFFSET + i, profile.gateway[i]);
    EEPROM.write(offset + CONFIG_SUBNET_OFFSET + i, profile.subnet[i]);
  }
  
  EEPROM.write(offset + CONFIG_PORT_OFFSET, (profile.port >> 8) & 0xFF);
  EEPROM.write(offset + CONFIG_PORT_OFFSET + 1, profile.port & 0xFF);
  
  savedProfiles[index] = profile;
  
  if (index >= configCount) {
    configCount = index + 1;
    EEPROM.write(EEPROM_CONFIG_COUNT_ADDR, configCount);
  }
  
  EEPROM.write(EEPROM_MAGIC_ADDR, EEPROM_MAGIC_VALUE);
  EEPROM.write(EEPROM_VERSION_ADDR, EEPROM_VERSION);
  
  Serial.print("[EEPROM] Saved profile ");
  Serial.print(index);
  Serial.print(": ");
  Serial.println(profile.ssid);
}

void setActiveProfile(uint8_t index) {
  if (index >= configCount) {
    Serial.println("[Error] Invalid profile index");
    return;
  }
  
  loadWiFiProfileAsCurrent(index);
  EEPROM.write(EEPROM_ACTIVE_INDEX_ADDR, activeConfigIndex);
}

void deleteWiFiProfile(uint8_t index) {
  if (index >= configCount) return;
  
  Serial.print("[EEPROM] Deleting profile ");
  Serial.print(index);
  
  // Shift profiles down
  for (uint8_t i = index; i < configCount - 1; i++) {
    uint16_t src = EEPROM_CONFIGS_START + ((i + 1) * CONFIG_SLOT_SIZE);
    uint16_t dst = EEPROM_CONFIGS_START + (i * CONFIG_SLOT_SIZE);
    
    for (uint16_t j = 0; j < CONFIG_SLOT_SIZE; j++) {
      EEPROM.write(dst + j, EEPROM.read(src + j));
    }
  }
  
  configCount--;
  EEPROM.write(EEPROM_CONFIG_COUNT_ADDR, configCount);
  
  if (activeConfigIndex >= configCount && configCount > 0) {
    activeConfigIndex = configCount - 1;
  }
  EEPROM.write(EEPROM_ACTIVE_INDEX_ADDR, activeConfigIndex);
  
  if (configCount > 0) {
    loadWiFiProfileAsCurrent(activeConfigIndex);
  } else {
    setDefaultProfile();
  }
}

void printWiFiProfiles() {
  Serial.println("\n========== WiFi Profiles ==========");
  Serial.print("Total: ");
  Serial.print(configCount);
  Serial.print(" / ");
  Serial.println(MAX_WIFI_CONFIGS);
  
  if (configCount == 0) {
    Serial.println("No profiles saved");
    return;
  }
  
  for (uint8_t i = 0; i < configCount; i++) {
    Serial.print("\n[");
    Serial.print(i);
    Serial.print("] ");
    Serial.print(i == activeConfigIndex ? "✓ ACTIVE" : " ");
    Serial.println();
    
    Serial.print("  SSID: ");
    Serial.println(savedProfiles[i].ssid);
    Serial.print("  Priority: ");
    Serial.println(savedProfiles[i].priority);
    Serial.print("  IP: ");
    Serial.print(savedProfiles[i].ip[0]); Serial.print(".");
    Serial.print(savedProfiles[i].ip[1]); Serial.print(".");
    Serial.print(savedProfiles[i].ip[2]); Serial.print(".");
    Serial.println(savedProfiles[i].ip[3]);
    Serial.print("  Port: ");
    Serial.println(savedProfiles[i].port);
  }
  Serial.println("\n===================================\n");
}

void clearAllWiFiConfigs() {
  Serial.println("[EEPROM] Clearing all data...");
  
  for (uint16_t i = EEPROM_MAGIC_ADDR; i < 1024; i++) {
    EEPROM.write(i, 0xFF);
  }
  
  configCount = 0;
  activeConfigIndex = 0;
  setDefaultProfile();
}

void setDefaultProfile() {
  currentProfile.priority = 0;
  strcpy(currentProfile.ssid, "");
  strcpy(currentProfile.password, "");
  currentProfile.ip[0] = 192;
  currentProfile.ip[1] = 168;
  currentProfile.ip[2] = 4;
  currentProfile.ip[3] = 1;
  currentProfile.gateway[0] = 192;
  currentProfile.gateway[1] = 168;
  currentProfile.gateway[2] = 4;
  currentProfile.gateway[3] = 1;
  currentProfile.subnet[0] = 255;
  currentProfile.subnet[1] = 255;
  currentProfile.subnet[2] = 255;
  currentProfile.subnet[3] = 0;
  currentProfile.port = 8888;
  
  staticIP = IPAddress(192, 168, 4, 1);
  gateway = IPAddress(192, 168, 4, 1);
  subnet = IPAddress(255, 255, 255, 0);
  serverPort = 8888;
}

// ========== WiFi Connection ==========

bool connectToWiFi() {
  if (strlen(currentProfile.ssid) == 0) {
    Serial.println("[WiFi] No SSID configured");
    return false;
  }
  
  Serial.print("[WiFi] Connecting to: ");
  Serial.print(currentProfile.ssid);
  Serial.print(" (Priority: ");
  Serial.print(currentProfile.priority);
  Serial.println(")");
  
  WiFi.config(staticIP, gateway, subnet);
  WiFi.begin(currentProfile.ssid, currentProfile.password);
  
  int attempts = 0;
  // Allow up to 30 seconds (60 attempts × 500ms) for WiFi connection
  while (WiFi.status() != WL_CONNECTED && attempts < 60) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[WiFi] ✓ Connected!");
    Serial.print("[WiFi] IP: ");
    Serial.println(WiFi.localIP());
    return true;
  } else {
    Serial.println("\n[WiFi] ✗ Failed (will retry in background)");
    return false;
  }
}

// Priority-based WiFi connection: try all profiles by priority
bool connectToWiFiByPriority() {
  if (configCount == 0) {
    Serial.println("[WiFi] No profiles available");
    return false;
  }
  
  // Create index array and sort by priority (descending)
  uint8_t indices[MAX_WIFI_CONFIGS];
  for (uint8_t i = 0; i < configCount; i++) {
    indices[i] = i;
  }
  
  // Simple bubble sort by priority (highest first)
  for (uint8_t i = 0; i < configCount - 1; i++) {
    for (uint8_t j = 0; j < configCount - i - 1; j++) {
      if (savedProfiles[indices[j]].priority < savedProfiles[indices[j + 1]].priority) {
        uint8_t temp = indices[j];
        indices[j] = indices[j + 1];
        indices[j + 1] = temp;
      }
    }
  }
  
  // Try each profile in priority order
  Serial.println("\n[WiFi] Attempting connections in priority order:");
  for (uint8_t i = 0; i < configCount; i++) {
    uint8_t profileIdx = indices[i];
    Serial.print("[WiFi] Attempt ");
    Serial.print(i + 1);
    Serial.print("/");
    Serial.print(configCount);
    Serial.print(" - Priority ");
    Serial.print(savedProfiles[profileIdx].priority);
    Serial.print(": ");
    Serial.println(savedProfiles[profileIdx].ssid);
    
    // Load profile and try connecting
    loadWiFiProfileAsCurrent(profileIdx);
    if (connectToWiFi()) {
      return true;
    }
  }
  
  Serial.println("[WiFi] ✗ All profiles failed to connect");
  return false;
}

// ========== Configuration AP ==========

void startConfigAP() {
  Serial.println("\n[AP] Starting Configuration Access Point");
  // Force AP to use 192.168.4.1/24 so the phone gets an IP in the same subnet.
  IPAddress apIP(192, 168, 4, 1);
  IPAddress apGateway(192, 168, 4, 1);
  IPAddress apSubnet(255, 255, 255, 0);
  WiFi.config(apIP, apGateway, apSubnet);

  WiFi.beginAP("ArduinoR4-Config", "12345678");
  IPAddress actualAP = WiFi.localIP();
  Serial.println("[AP] SSID: ArduinoR4-Config, Password: 12345678");
  Serial.print("[AP] Visit: http://");
  Serial.println(actualAP);
  
  WiFiServer configServer(80);
  configServer.begin();
  
  unsigned long startTime = millis();
  const unsigned long TIMEOUT = 300000; // 5 minutes
  bool exitRequested = false;   // leave AP early after a POST action
  bool reconnectNeeded = false; // reconnect STA after config change
  
  while (!exitRequested && (millis() - startTime < TIMEOUT)) {
    WiFiClient client = configServer.available();
    
    if (client) {
      String request = "";
      unsigned long timeout = millis();
      
      while (client.connected() && millis() - timeout < 2000) {
        if (client.available()) {
          char c = client.read();
          request += c;
          timeout = millis();
        }
      }
      
      if (request.indexOf("GET") == 0) {
        sendConfigPage(client);
      }
      else if (request.indexOf("POST") == 0) {
        if (request.indexOf("action=add") > 0) {
          handleAddProfile(client, request);
          exitRequested = true;
          reconnectNeeded = true;
        }
        else if (request.indexOf("action=delete") > 0) {
          handleDeleteProfile(client, request);
          exitRequested = true;
          reconnectNeeded = true;
        }
        else if (request.indexOf("action=switch") > 0) {
          handleSwitchProfile(client, request);
          exitRequested = true;
          reconnectNeeded = true;
        }
      }
      
      client.stop();
    }
    
    delay(50);
  }
  
  if (exitRequested) {
    Serial.println("[AP] Closing AP (config applied)");
  } else {
    Serial.println("[AP] Timeout - Closing AP");
  }
  WiFi.end();

  if (reconnectNeeded) {
    // Reload configs from EEPROM and reconnect with the active profile
    loadAllWiFiConfigs();
    if (strlen(currentProfile.ssid) > 0) {
      connectToWiFi();
    }
    wifiServer.begin();
  }
}

void sendConfigPage(WiFiClient &client) {
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html; charset=UTF-8");
  client.println("Connection: close");
  client.println();

  String html;
  html.reserve(4000);

  html += F("<!DOCTYPE html><html><head><meta charset=\"UTF-8\"><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\"><title>Arduino WiFi Config</title><style>");
  html += F("body{font-family:Arial;background:#1a1a1a;color:#fff;margin:0;padding:20px;} .container{max-width:800px;margin:0 auto;} h1{color:#4CAF50;} .section{background:#2a2a2a;padding:20px;margin:20px 0;border-radius:8px;border-left:4px solid #4CAF50;} .form-group{margin:15px 0;} label{display:block;margin-bottom:5px;font-weight:bold;} input,select{width:100%;padding:10px;margin-bottom:10px;background:#1a1a1a;border:1px solid #4CAF50;color:#fff;border-radius:4px;box-sizing:border-box;} button{padding:10px 20px;background:#4CAF50;color:white;border:none;border-radius:4px;cursor:pointer;font-weight:bold;margin:5px 5px 5px 0;} button:hover{background:#45a049;} button.danger{background:#f44336;} button.danger:hover{background:#da190b;} .profile-item{background:#3a3a3a;padding:15px;margin:10px 0;border-left:3px solid #4CAF50;border-radius:4px;} .profile-item.active{border-left-color:#ffc107;} .button-group{display:flex;flex-wrap:wrap;gap:5px;} .tabs{display:flex;gap:10px;margin-bottom:20px;} .tab-button{padding:10px 20px;background:#2a2a2a;border:2px solid #4CAF50;color:#fff;cursor:pointer;border-radius:4px;} .tab-button.active{background:#4CAF50;} .tab-content{display:none;} .tab-content.active{display:block;}");
  html += F("</style></head><body><div class=\"container\"><h1>🌐 WiFi Configuration</h1><div class=\"tabs\"><button class=\"tab-button active\" onclick=\"switchTab('profiles')\">Profiles</button><button class=\"tab-button\" onclick=\"switchTab('add')\">Add WiFi</button></div>");

  // Profiles tab with server-rendered list
  html += F("<div id=\"profiles\" class=\"tab-content active\"><div class=\"section\"><h2>Saved Profiles</h2><div id=\"profileList\">");
  if (configCount == 0) {
    html += F("<p>No profiles saved.</p>");
  } else {
    for (uint8_t i = 0; i < configCount; i++) {
      html += F("<div class=\"profile-item");
      if (i == activeConfigIndex) html += F(" active");
      html += F("\">");
      html += F("<div><strong>SSID:</strong> ");
      html += savedProfiles[i].ssid;
      html += F("</div>");
      html += F("<div><strong>Priority:</strong> ");
      html += String(savedProfiles[i].priority);
      html += F("</div><div><strong>IP:</strong> ");
      html += String(savedProfiles[i].ip[0]); html += '.'; html += String(savedProfiles[i].ip[1]); html += '.'; html += String(savedProfiles[i].ip[2]); html += '.'; html += String(savedProfiles[i].ip[3]);
      html += F("</div><div><strong>Port:</strong> ");
      html += String(savedProfiles[i].port);
      html += F("</div><div class=\"button-group\">");
      // Switch form
      html += F("<form method=\"POST\" style=\"display:inline\"><input type=\"hidden\" name=\"action\" value=\"switch\"><input type=\"hidden\" name=\"index\" value=\"");
      html += String(i);
      html += F("\"><button type=\"submit\">Switch</button></form>");
      // Delete form
      html += F("<form method=\"POST\" style=\"display:inline\"><input type=\"hidden\" name=\"action\" value=\"delete\"><input type=\"hidden\" name=\"index\" value=\"");
      html += String(i);
      html += F("\"><button type=\"submit\" class=\"danger\">Delete</button></form>");
      html += F("</div></div>");
    }
  }
  html += F("</div></div></div>");

  // Add form tab
  html += F("<div id=\"add\" class=\"tab-content\"><div class=\"section\"><h2>Add New WiFi</h2><form method=\"POST\"><input type=\"hidden\" name=\"action\" value=\"add\">");
  html += F("<div class=\"form-group\"><label>WiFi SSID</label><input type=\"text\" name=\"ssid\" required></div>");
  html += F("<div class=\"form-group\"><label>Password</label><input type=\"password\" name=\"password\"></div>");
  html += F("<div class=\"form-group\"><label>Priority (0-255)</label><input type=\"number\" name=\"priority\" min=\"0\" max=\"255\" value=\"100\"></div>");
  html += F("<div class=\"form-group\"><label>Static IP</label><input type=\"text\" name=\"ip\" value=\"192.168.4.1\"></div>");
  html += F("<div class=\"form-group\"><label>Gateway</label><input type=\"text\" name=\"gateway\" value=\"192.168.4.1\"></div>");
  html += F("<div class=\"form-group\"><label>Subnet</label><input type=\"text\" name=\"subnet\" value=\"255.255.255.0\"></div>");
  html += F("<div class=\"form-group\"><label>Port</label><input type=\"number\" name=\"port\" min=\"1\" max=\"65535\" value=\"8888\"></div>");
  html += F("<button type=\"submit\">Add Profile</button></form></div></div>");

  // Simple tab switcher (no fetch needed)
  html += F("<script>function switchTab(t){document.querySelectorAll('.tab-content').forEach(e=>e.classList.remove('active'));document.querySelectorAll('.tab-button').forEach(e=>e.classList.remove('active'));document.getElementById(t).classList.add('active');event.target.classList.add('active');}</script>");
  html += F("</div></body></html>");

  client.write((uint8_t*)html.c_str(), html.length());
}

void handleAddProfile(WiFiClient &client, String &request) {
  int bodyStart = request.indexOf("\r\n\r\n");
  if (bodyStart < 0) {
    client.println("HTTP/1.1 400 Bad Request\r\nConnection: close\r\n\r\nError");
    return;
  }
  
  String body = request.substring(bodyStart + 4);
  WiFiProfile profile = {};
  // Defaults so missing fields don't leave garbage
  profile.priority = 100;
  profile.port = 8888;
  profile.ip[0] = 192; profile.ip[1] = 168; profile.ip[2] = 4; profile.ip[3] = 1;
  profile.gateway[0] = 192; profile.gateway[1] = 168; profile.gateway[2] = 4; profile.gateway[3] = 1;
  profile.subnet[0] = 255; profile.subnet[1] = 255; profile.subnet[2] = 255; profile.subnet[3] = 0;
  
  parseFormData(body, "ssid=", profile.ssid, 32);
  parseFormData(body, "password=", profile.password, 64);
  profile.priority = parseIntField(body, "priority=");
  parseIPAddress(body, "ip=", profile.ip);
  parseIPAddress(body, "gateway=", profile.gateway);
  parseIPAddress(body, "subnet=", profile.subnet);
  profile.port = parseIntField(body, "port=");
  if (profile.port == 0) profile.port = 8888;
  
  if (configCount < MAX_WIFI_CONFIGS) {
    saveWiFiProfile(configCount, profile);
    client.println("HTTP/1.1 200 OK\r\nConnection: close\r\n\r\nAdded!");
  } else {
    client.println("HTTP/1.1 400 Bad Request\r\nConnection: close\r\n\r\nFull!");
  }
}

void handleDeleteProfile(WiFiClient &client, String &request) {
  int idx = parseIntField(request, "index=");
  deleteWiFiProfile(idx);
  client.println("HTTP/1.1 200 OK\r\nConnection: close\r\n\r\nDeleted!");
}

void handleSwitchProfile(WiFiClient &client, String &request) {
  int idx = parseIntField(request, "index=");
  setActiveProfile(idx);
  client.println("HTTP/1.1 200 OK\r\nConnection: close\r\n\r\nSwitched!");
}

// ========== Utility Functions ==========

void parseFormData(String &body, const char* key, char* value, int maxLen) {
  int pos = body.indexOf(key);
  if (pos < 0) return;
  
  int start = pos + strlen(key);
  int end = body.indexOf('&', start);
  if (end < 0) end = body.length();
  
  String encoded = body.substring(start, end);
  urlDecode(encoded, value, maxLen);
}

int parseIntField(String &body, const char* key) {
  char temp[20];
  parseFormData(body, key, temp, 20);
  return atoi(temp);
}

void parseIPAddress(String &body, const char* key, uint8_t* octets) {
  char temp[20];
  parseFormData(body, key, temp, 20);
  
  int idx = 0;
  for (int i = 0; i < 4 && idx < strlen(temp); i++) {
    int val = 0;
    while (idx < strlen(temp) && temp[idx] != '.' && isdigit(temp[idx])) {
      val = val * 10 + (temp[idx] - '0');
      idx++;
    }
    octets[i] = constrain(val, 0, 255);
    if (idx < strlen(temp) && temp[idx] == '.') idx++;
  }
}

void urlDecode(String &encoded, char* decoded, int maxLen) {
  int decodedLen = 0;
  for (int i = 0; i < encoded.length() && decodedLen < maxLen - 1; i++) {
    if (encoded[i] == '+') {
      decoded[decodedLen++] = ' ';
    } else if (encoded[i] == '%' && i + 2 < encoded.length()) {
      char hex[3];
      hex[0] = encoded[i + 1];
      hex[1] = encoded[i + 2];
      hex[2] = '\0';
      decoded[decodedLen++] = (char)strtol(hex, NULL, 16);
      i += 2;
    } else {
      decoded[decodedLen++] = encoded[i];
    }
  }
  decoded[decodedLen] = '\0';
}

// ========== RC & Thruster Control ==========

void readRCChannels() {
  unsigned long pulseL = pulseIn(CH_LEFT_IN, HIGH, PULSE_TIMEOUT_US);
  unsigned long pulseR = pulseIn(CH_RIGHT_IN, HIGH, PULSE_TIMEOUT_US);
  
  if (pulseL > 0 && pulseL >= RX_VALID_MIN && pulseL <= RX_VALID_MAX) {
    rcOutL = constrain(pulseL, ESC_MIN, ESC_MAX);
    lastRcUpdateL = millis();
  }
  
  if (pulseR > 0 && pulseR >= RX_VALID_MIN && pulseR <= RX_VALID_MAX) {
    rcOutR = constrain(pulseR, ESC_MIN, ESC_MAX);
    lastRcUpdateR = millis();
  }
}

void handleClientConnections() {
  if (!rosClient.connected()) {
    rosClient = wifiServer.available();
    if (rosClient) {
      Serial.println("[TCP] New WiFi client connected!");
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
          
          Serial.print("[TCP] WiFi Command: Left=");
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

void parseThrusterCommand(String cmd) {
  // Not used - kept for compatibility
}

void updateOutputs() {
  unsigned long now = millis();
  
  unsigned long rcTimeoutL = now - lastRcUpdateL;
  unsigned long rcTimeoutR = now - lastRcUpdateR;
  unsigned long wifiTimeout = now - lastWifiCmdMs;
  
  bool rcValidL = rcTimeoutL < RC_FAILSAFE_MS;
  bool rcValidR = rcTimeoutR < RC_FAILSAFE_MS;
  bool wifiValid = wifiTimeout < WIFI_CMD_TIMEOUT_MS && haveWifiCmd && rosClient.connected();
  
  if (wifiValid) {
    // WiFi has priority
    currentLeftUs = wifiOutL;
    currentRightUs = wifiOutR;
    currentMode = 1;
  } else if (rcValidL && rcValidR) {
    // RC fallback
    currentLeftUs = rcOutL;
    currentRightUs = rcOutR;
    currentMode = 0;
  } else {
    // Safe failsafe
    currentLeftUs = ESC_MID;
    currentRightUs = ESC_MID;
    currentMode = 0;
  }
  
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

void tryReconnectWiFi() {
  // Auto-retry WiFi connection if disconnected and enough time has passed
  if (WiFi.status() == WL_CONNECTED) {
    lastWifiRetryMs = millis();
    return;
  }
  
  if (strlen(currentProfile.ssid) == 0) return;
  
  if (millis() - lastWifiRetryMs >= WIFI_RETRY_INTERVAL_MS) {
    Serial.println("[WiFi] Retrying connection...");
    connectToWiFi();
  }
}

// ========== Serial Commands ==========

void handleSerialCommands() {
  if (!Serial.available()) return;
  
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  
  if (cmd.equals("CONFIG")) {
    startConfigAP();
  }
  else if (cmd.equals("RESET")) {
    Serial.println("[System] Resetting...");
    clearAllWiFiConfigs();
    delay(1000);
    NVIC_SystemReset();
  }
  else if (cmd.equals("STATUS")) {
    Serial.print("[WiFi] Status: ");
    if (WiFi.status() == WL_CONNECTED) {
      Serial.print("CONNECTED (");
      Serial.print(WiFi.RSSI());
      Serial.println(" dBm)");
    } else {
      Serial.println("DISCONNECTED");
    }
  }
  else if (cmd.equals("LIST")) {
    printWiFiProfiles();
  }
  else if (cmd.startsWith("SWITCH ")) {
    int idx = cmd.substring(7).toInt();
    setActiveProfile(idx);
  }
  else if (cmd.startsWith("DELETE ")) {
    int idx = cmd.substring(7).toInt();
    deleteWiFiProfile(idx);
  }
}

// ========== Setup & Loop ==========

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("\n\n==== Arduino R4 WiFi - Multi-WiFi Control ====");
  Serial.println("Commands: CONFIG, RESET, STATUS, LIST, SWITCH <i>, DELETE <i>");
  
  escL.attach(ESC_LEFT_OUT);
  escR.attach(ESC_RIGHT_OUT);
  escL.writeMicroseconds(ESC_MID);
  escR.writeMicroseconds(ESC_MID);
  
  pinMode(CH_LEFT_IN, INPUT);
  pinMode(CH_RIGHT_IN, INPUT);
  
  loadAllWiFiConfigs();
  
  if (strlen(currentProfile.ssid) == 0) {
    Serial.println("[System] No WiFi profile found - entering CONFIG mode");
    startConfigAP();
  } else {
    // Use priority-based connection to try all profiles in order
    connectToWiFiByPriority();
  }
  
  wifiServer.begin();
  lastWifiRetryMs = millis(); // Initialize retry timer
  Serial.print("[System] Ready! TCP Server on port ");
  Serial.println(serverPort);
  
  Serial.println("\n=== System Status ===");
  Serial.print("WiFi: ");
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("CONNECTED (");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm)");
  } else {
    Serial.println("DISCONNECTED");
  }
  printWiFiProfiles();
}

void loop() {
  readRCChannels();
  handleClientConnections();
  readWifiCommands();
  determineControlMode();
  updateOutputs();
  sendStatus();
  handleSerialCommands();
  tryReconnectWiFi();
  
  // Periodic debug output every 5 seconds
  static unsigned long lastDebugMs = 0;
  if (millis() - lastDebugMs >= 5000) {
    lastDebugMs = millis();
    Serial.print("[Status] Mode: ");
    Serial.print(currentMode ? "WiFi" : "RC");
    Serial.print(" | L: ");
    Serial.print(currentLeftUs);
    Serial.print(" | R: ");
    Serial.print(currentRightUs);
    Serial.print(" | WiFi: ");
    Serial.print(WiFi.status() == WL_CONNECTED ? "✓" : "✗");
    Serial.print(" | Client: ");
    Serial.println(rosClient && rosClient.connected() ? "✓" : "✗");
  }
  
  delay(5);
}
