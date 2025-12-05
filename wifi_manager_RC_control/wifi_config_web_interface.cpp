#if 0
/*
 * Reference-only helper for the WiFi config web UI.
 *
 * NOTE: This file is not used by the active build. The Arduino sketch
 * `wifi_manager_RC_control.ino` already contains the integrated web UI
 * implementation. Keeping this file compiled causes duplicate symbols and
 * missing includes, so we disable it with `#if 0`.
 */

void startConfigAP() {
  Serial.println("\n=== Starting Configuration AP ===");
  
  // Start Access Point
  WiFi.beginAP("ArduinoR4-Config", "12345678");
  
  Serial.println("[AP] Started - Connect to 'ArduinoR4-Config' (password: 12345678)");
  Serial.println("[AP] Open http://192.168.4.1 in your browser");
  
  WiFiServer configServer(80);
  configServer.begin();
  
  bool configComplete = false;
  unsigned long configStartTime = millis();
  const unsigned long CONFIG_TIMEOUT = 300000; // 5 minutes
  
  while (!configComplete && (millis() - configStartTime < CONFIG_TIMEOUT)) {
    WiFiClient client = configServer.available();
    
    if (client) {
      Serial.println("[AP] Client connected");
      
      String request = "";
      unsigned long clientTimeout = millis();
      
      while (client.connected() && (millis() - clientTimeout < 2000)) {
        if (client.available()) {
          char c = client.read();
          request += c;
          clientTimeout = millis();
        }
      }
      
      Serial.print("[AP] Request size: ");
      Serial.print(request.length());
      Serial.println(" bytes");
      
      // Parse request
      if (request.indexOf("GET") == 0) {
        // Send configuration page
        sendConfigPage(client);
      }
      else if (request.indexOf("POST") == 0) {
        // Handle form submission
        if (request.indexOf("action=add") > 0) {
          handleAddProfile(client, request);
        }
        else if (request.indexOf("action=delete") > 0) {
          handleDeleteProfile(client, request);
        }
        else if (request.indexOf("action=switch") > 0) {
          handleSwitchProfile(client, request);
        }
        else if (request.indexOf("action=priority") > 0) {
          handleSetPriority(client, request);
        }
      }
      
      client.stop();
      Serial.println("[AP] Client disconnected");
    }
    
    delay(100);
  }
  
  Serial.println("[AP] Configuration timeout - Exiting AP mode");
  WiFi.end();  // Stop AP mode
}

void sendConfigPage(WiFiClient &client) {
  // Send HTTP response
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html; charset=UTF-8");
  client.println("Content-Length: 3500");
  client.println("Connection: close");
  client.println();
  
  // HTML content
  String html = R"(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Arduino WiFi Config</title>
  <style>
    body { font-family: Arial, sans-serif; background: #1a1a1a; color: #fff; margin: 0; padding: 20px; }
    .container { max-width: 800px; margin: 0 auto; }
    h1 { color: #4CAF50; }
    .section { background: #2a2a2a; padding: 20px; margin: 20px 0; border-radius: 8px; border-left: 4px solid #4CAF50; }
    .form-group { margin: 15px 0; }
    label { display: block; margin-bottom: 5px; font-weight: bold; }
    input, select, textarea { width: 100%; padding: 10px; margin-bottom: 10px; background: #1a1a1a; border: 1px solid #4CAF50; color: #fff; border-radius: 4px; box-sizing: border-box; }
    button { padding: 10px 20px; background: #4CAF50; color: white; border: none; border-radius: 4px; cursor: pointer; font-weight: bold; margin: 5px 5px 5px 0; }
    button:hover { background: #45a049; }
    button.danger { background: #f44336; }
    button.danger:hover { background: #da190b; }
    .profile-list { background: #3a3a3a; padding: 15px; border-radius: 4px; margin: 10px 0; }
    .profile-item { background: #2a2a2a; padding: 15px; margin: 10px 0; border-left: 3px solid #4CAF50; border-radius: 4px; }
    .profile-item.active { border-left-color: #ffc107; }
    .profile-info { font-size: 0.9em; color: #aaa; }
    .button-group { display: flex; flex-wrap: wrap; gap: 5px; }
    .success { color: #4CAF50; }
    .error { color: #f44336; }
    small { display: block; color: #999; margin-top: 5px; }
    .tabs { display: flex; gap: 10px; margin-bottom: 20px; }
    .tab-button { padding: 10px 20px; background: #2a2a2a; border: 2px solid #4CAF50; color: #fff; cursor: pointer; border-radius: 4px; }
    .tab-button.active { background: #4CAF50; }
    .tab-content { display: none; }
    .tab-content.active { display: block; }
  </style>
</head>
<body>
  <div class="container">
    <h1>🌐 Arduino WiFi Configuration</h1>
    
    <div class="tabs">
      <button class="tab-button active" onclick="switchTab('profiles')">Saved Profiles</button>
      <button class="tab-button" onclick="switchTab('add')">Add New WiFi</button>
    </div>
    
    <!-- Profiles Tab -->
    <div id="profiles" class="tab-content active">
      <div class="section">
        <h2>Saved WiFi Profiles</h2>
        <div id="profileList" class="profile-list">
          Loading profiles...
        </div>
      </div>
    </div>
    
    <!-- Add Profile Tab -->
    <div id="add" class="tab-content">
      <div class="section">
        <h2>Add New WiFi Profile</h2>
        <form onsubmit="addProfile(event)">
          <div class="form-group">
            <label>WiFi SSID</label>
            <input type="text" id="ssid" placeholder="Network name" required>
            <small>The name of your WiFi network</small>
          </div>
          
          <div class="form-group">
            <label>Password</label>
            <input type="password" id="password" placeholder="WiFi password" required>
            <small>Leave empty for open networks</small>
          </div>
          
          <div class="form-group">
            <label>Priority (0-255)</label>
            <input type="number" id="priority" min="0" max="255" value="100" required>
            <small>Higher value = higher priority</small>
          </div>
          
          <div class="form-group">
            <label>Static IP Address</label>
            <input type="text" id="ip" placeholder="192.168.1.100" value="192.168.4.1" required>
            <small>Leave 192.168.4.1 for DHCP during AP mode</small>
          </div>
          
          <div class="form-group">
            <label>Gateway</label>
            <input type="text" id="gateway" placeholder="192.168.1.1" value="192.168.4.1" required>
          </div>
          
          <div class="form-group">
            <label>Subnet Mask</label>
            <input type="text" id="subnet" placeholder="255.255.255.0" value="255.255.255.0" required>
          </div>
          
          <div class="form-group">
            <label>Port</label>
            <input type="number" id="port" min="1" max="65535" value="8888" required>
            <small>TCP server port for commands</small>
          </div>
          
          <button type="submit" onclick="submitAddProfile()">Add Profile</button>
        </form>
      </div>
    </div>
  </div>
  
  <script>
    function switchTab(tabName) {
      const contents = document.querySelectorAll('.tab-content');
      const buttons = document.querySelectorAll('.tab-button');
      
      contents.forEach(c => c.classList.remove('active'));
      buttons.forEach(b => b.classList.remove('active'));
      
      document.getElementById(tabName).classList.add('active');
      event.target.classList.add('active');
    }
    
    function submitAddProfile() {
      const ssid = document.getElementById('ssid').value;
      const password = document.getElementById('password').value;
      const priority = document.getElementById('priority').value;
      const ip = document.getElementById('ip').value;
      const gateway = document.getElementById('gateway').value;
      const subnet = document.getElementById('subnet').value;
      const port = document.getElementById('port').value;
      
      const formData = new FormData();
      formData.append('action', 'add');
      formData.append('ssid', ssid);
      formData.append('password', password);
      formData.append('priority', priority);
      formData.append('ip', ip);
      formData.append('gateway', gateway);
      formData.append('subnet', subnet);
      formData.append('port', port);
      
      fetch(window.location.href, {
        method: 'POST',
        body: new URLSearchParams(formData)
      }).then(() => {
        alert('Profile added!');
        location.reload();
      }).catch(err => alert('Error: ' + err));
    }
    
    function deleteProfile(index) {
      if (confirm('Delete this profile?')) {
        fetch(window.location.href, {
          method: 'POST',
          body: 'action=delete&index=' + index
        }).then(() => {
          alert('Profile deleted!');
          location.reload();
        }).catch(err => alert('Error: ' + err));
      }
    }
    
    function switchProfile(index) {
      fetch(window.location.href, {
        method: 'POST',
        body: 'action=switch&index=' + index
      }).then(() => {
        alert('Switched to profile ' + index);
        location.reload();
      }).catch(err => alert('Error: ' + err));
    }
    
    function setPriority(index) {
      const priority = prompt('Enter new priority (0-255):', '100');
      if (priority !== null) {
        fetch(window.location.href, {
          method: 'POST',
          body: 'action=priority&index=' + index + '&priority=' + priority
        }).then(() => {
          alert('Priority updated!');
          location.reload();
        }).catch(err => alert('Error: ' + err));
      }
    }
  </script>
</body>
</html>
)";
  
  client.write((uint8_t*)html.c_str(), html.length());
}

void handleAddProfile(WiFiClient &client, String &request) {
  Serial.println("[AP] Processing: Add Profile");
  
  // Parse form data from POST request
  int bodyStart = request.indexOf("\r\n\r\n");
  if (bodyStart < 0) {
    sendSimpleResponse(client, "Error: Invalid request");
    return;
  }
  
  String body = request.substring(bodyStart + 4);
  WiFiProfile newProfile;
  
  // Parse URL-encoded parameters
  parseFormData(body, "ssid=", newProfile.ssid, 32);
  parseFormData(body, "password=", newProfile.password, 64);
  parseFormData(body, "priority=", &newProfile.priority);
  
  // Parse IP addresses
  uint8_t tempIp[4];
  parseIPAddress(body, "ip=", tempIp);
  memcpy(newProfile.ip, tempIp, 4);
  
  parseIPAddress(body, "gateway=", tempIp);
  memcpy(newProfile.gateway, tempIp, 4);
  
  parseIPAddress(body, "subnet=", tempIp);
  memcpy(newProfile.subnet, tempIp, 4);
  
  // Parse port
  char portStr[6];
  parseFormData(body, "port=", portStr, 6);
  newProfile.port = atoi(portStr);
  
  // Save profile
  if (configCount < MAX_WIFI_CONFIGS) {
    saveWiFiProfile(configCount, newProfile);
    sendSimpleResponse(client, "Profile added successfully!");
  } else {
    sendSimpleResponse(client, "Error: Maximum profiles reached!");
  }
}

void handleDeleteProfile(WiFiClient &client, String &request) {
  Serial.println("[AP] Processing: Delete Profile");
  
  int indexStart = request.indexOf("index=");
  if (indexStart > 0) {
    int index = atoi(request.substring(indexStart + 6).c_str());
    deleteWiFiProfile(index);
    sendSimpleResponse(client, "Profile deleted!");
  } else {
    sendSimpleResponse(client, "Error: Invalid index");
  }
}

void handleSwitchProfile(WiFiClient &client, String &request) {
  Serial.println("[AP] Processing: Switch Profile");
  
  int indexStart = request.indexOf("index=");
  if (indexStart > 0) {
    int index = atoi(request.substring(indexStart + 6).c_str());
    setActiveProfile(index);
    sendSimpleResponse(client, "Switched to profile!");
  } else {
    sendSimpleResponse(client, "Error: Invalid index");
  }
}

void handleSetPriority(WiFiClient &client, String &request) {
  Serial.println("[AP] Processing: Set Priority");
  
  int indexStart = request.indexOf("index=");
  int priorityStart = request.indexOf("priority=");
  
  if (indexStart > 0 && priorityStart > 0) {
    int index = atoi(request.substring(indexStart + 6).c_str());
    int priority = atoi(request.substring(priorityStart + 9).c_str());
    
    if (index < configCount) {
      savedProfiles[index].priority = constrain(priority, 0, 255);
      saveWiFiProfile(index, savedProfiles[index]);
      sendSimpleResponse(client, "Priority updated!");
    } else {
      sendSimpleResponse(client, "Error: Invalid index");
    }
  } else {
    sendSimpleResponse(client, "Error: Missing parameters");
  }
}

void sendSimpleResponse(WiFiClient &client, const char* message) {
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/plain");
  client.println("Connection: close");
  client.println();
  client.println(message);
}

// Utility functions for parsing

void parseFormData(String &body, const char* key, char* value, int maxLen) {
  int keyPos = body.indexOf(key);
  if (keyPos < 0) return;
  
  int startPos = keyPos + strlen(key);
  int endPos = body.indexOf('&', startPos);
  if (endPos < 0) endPos = body.length();
  
  String encoded = body.substring(startPos, endPos);
  urlDecode(encoded, value, maxLen);
}

void parseFormData(String &body, const char* key, uint8_t* value) {
  char temp[6];
  parseFormData(body, key, temp, 6);
  *value = constrain(atoi(temp), 0, 255);
}

void parseIPAddress(String &body, const char* key, uint8_t* octets) {
  char temp[20];
  parseFormData(body, key, temp, 20);
  
  int pos = 0;
  for (int i = 0; i < 4; i++) {
    int dotPos = temp[pos] == '.' ? pos : -1;
    for (int j = pos; j < strlen(temp) && temp[j] != '.'; j++) {
      dotPos = j;
    }
    octets[i] = atoi(temp + pos);
    pos = dotPos + 2;
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

#endif  // reference-only (disabled)
