#include <ArduinoBLE.h>
#include "pwm.h"

const char* DEVICE_NAME = "UNO-R4-WiFi";
const char* LOCAL_NAME = "UNO-R4-BLE-Thruster";

const int LED_PIN = LED_BUILTIN;
const int ESC_RIGHT_OUT = 9;
const int ESC_LEFT_OUT = 10;

const int ESC_MIN = 1100;
const int ESC_MID = 1500;
const int ESC_MAX = 1900;

// Start in a conservative test window so the first BLE tests are less aggressive.
const int SAFE_TEST_MIN = 1300;
const int SAFE_TEST_MAX = 1700;

const unsigned long ESC_PWM_PERIOD_US = 20000;
const unsigned long ESC_SAFE_BOOT_NEUTRAL_MS = 2000;
const unsigned long STATUS_INTERVAL_MS = 1000;
const unsigned long DEFAULT_MOTION_TIMEOUT_MS = 3000;
const unsigned long MIN_PULSE_MS = 100;
const unsigned long MAX_PULSE_MS = 5000;

BLEService testService("19B10010-E8F2-537E-4F6C-D104768A1214");
BLEStringCharacteristic rxCharacteristic(
  "19B10011-E8F2-537E-4F6C-D104768A1214",
  BLERead | BLEWrite | BLEWriteWithoutResponse,
  96
);
BLEStringCharacteristic txCharacteristic(
  "19B10012-E8F2-537E-4F6C-D104768A1214",
  BLERead | BLENotify,
  96
);

PwmOut escL(ESC_LEFT_OUT);
PwmOut escR(ESC_RIGHT_OUT);

bool escReady = false;
bool armed = false;
bool safeMode = true;
int currentLeftUs = ESC_MID;
int currentRightUs = ESC_MID;
unsigned long lastStatusMs = 0;
unsigned long motionTimeoutAtMs = 0;
unsigned long pulseEndsAtMs = 0;

int activeMinUs() {
  return safeMode ? SAFE_TEST_MIN : ESC_MIN;
}

int activeMaxUs() {
  return safeMode ? SAFE_TEST_MAX : ESC_MAX;
}

void sendText(const String& text) {
  txCharacteristic.writeValue(text.c_str());
  Serial.print("TX -> ");
  Serial.println(text);
}

String statusText() {
  String text = "status armed=";
  text += armed ? "1" : "0";
  text += " mode=";
  text += safeMode ? "safe" : "full";
  text += " left=";
  text += currentLeftUs;
  text += " right=";
  text += currentRightUs;

  if (pulseEndsAtMs > millis()) {
    text += " pulse_ms=";
    text += (pulseEndsAtMs - millis());
  }

  return text;
}

void applyThrusters() {
  int outLeftUs = ESC_MID;
  int outRightUs = ESC_MID;

  if (armed) {
    outLeftUs = constrain(currentLeftUs, ESC_MIN, ESC_MAX);
    outRightUs = constrain(currentRightUs, ESC_MIN, ESC_MAX);
  }

  if (escReady) {
    escL.pulseWidth_us(outLeftUs);
    escR.pulseWidth_us(outRightUs);
  }

  digitalWrite(LED_PIN, armed ? HIGH : LOW);

  Serial.print("ESC -> left=");
  Serial.print(outLeftUs);
  Serial.print(" right=");
  Serial.println(outRightUs);
}

void neutralize(bool keepArmed) {
  currentLeftUs = ESC_MID;
  currentRightUs = ESC_MID;
  pulseEndsAtMs = 0;
  motionTimeoutAtMs = 0;

  if (!keepArmed) {
    armed = false;
  }

  applyThrusters();
}

void armThrusters() {
  armed = true;
  currentLeftUs = ESC_MID;
  currentRightUs = ESC_MID;
  pulseEndsAtMs = 0;
  motionTimeoutAtMs = 0;
  applyThrusters();
}

void setThrusters(int leftUs, int rightUs) {
  currentLeftUs = constrain(leftUs, activeMinUs(), activeMaxUs());
  currentRightUs = constrain(rightUs, activeMinUs(), activeMaxUs());
  pulseEndsAtMs = 0;

  if (currentLeftUs == ESC_MID && currentRightUs == ESC_MID) {
    motionTimeoutAtMs = 0;
  } else {
    motionTimeoutAtMs = millis() + DEFAULT_MOTION_TIMEOUT_MS;
  }

  applyThrusters();
}

void pulseThrusters(int leftUs, int rightUs, unsigned long durationMs) {
  currentLeftUs = constrain(leftUs, activeMinUs(), activeMaxUs());
  currentRightUs = constrain(rightUs, activeMinUs(), activeMaxUs());
  pulseEndsAtMs = millis() + durationMs;
  motionTimeoutAtMs = pulseEndsAtMs;
  applyThrusters();
}

String normalizeCommand(String text) {
  text.trim();
  text.toLowerCase();
  return text;
}

void sendHelp() {
  sendText("cmd: arm disarm stop safe full status set L R pulse L R ms");
}

void handleIncomingText(const String& rawText) {
  String command = normalizeCommand(rawText);
  char buffer[96];
  command.toCharArray(buffer, sizeof(buffer));

  Serial.print("RX <- ");
  Serial.println(rawText);

  if (strcmp(buffer, "help") == 0) {
    sendHelp();
    return;
  }

  if (strcmp(buffer, "ping") == 0) {
    sendText("pong");
    return;
  }

  if (strcmp(buffer, "status") == 0) {
    sendText(statusText());
    return;
  }

  if (strcmp(buffer, "safe") == 0) {
    safeMode = true;
    currentLeftUs = constrain(currentLeftUs, SAFE_TEST_MIN, SAFE_TEST_MAX);
    currentRightUs = constrain(currentRightUs, SAFE_TEST_MIN, SAFE_TEST_MAX);
    applyThrusters();
    sendText("ok mode=safe range=1300..1700");
    return;
  }

  if (strcmp(buffer, "full") == 0) {
    safeMode = false;
    applyThrusters();
    sendText("ok mode=full range=1100..1900");
    return;
  }

  if (strcmp(buffer, "arm") == 0) {
    armThrusters();
    sendText(String("ok armed range=") + activeMinUs() + ".." + activeMaxUs());
    return;
  }

  if (strcmp(buffer, "disarm") == 0) {
    neutralize(false);
    sendText("ok disarmed");
    return;
  }

  if (strcmp(buffer, "stop") == 0 || strcmp(buffer, "center") == 0) {
    neutralize(true);
    sendText("ok stop left=1500 right=1500");
    return;
  }

  int leftUs = 0;
  int rightUs = 0;
  int durationMs = 0;

  if (sscanf(buffer, "set %d %d", &leftUs, &rightUs) == 2) {
    if (!armed) {
      sendText("err arm first");
      return;
    }

    setThrusters(leftUs, rightUs);
    sendText(String("ok set left=") + currentLeftUs + " right=" + currentRightUs);
    return;
  }

  if (sscanf(buffer, "pulse %d %d %d", &leftUs, &rightUs, &durationMs) == 3) {
    if (!armed) {
      sendText("err arm first");
      return;
    }

    unsigned long clampedDurationMs = constrain(durationMs, (int)MIN_PULSE_MS, (int)MAX_PULSE_MS);
    pulseThrusters(leftUs, rightUs, clampedDurationMs);
    sendText(
      String("ok pulse left=") + currentLeftUs +
      " right=" + currentRightUs +
      " ms=" + clampedDurationMs
    );
    return;
  }

  sendText("err use: help");
}

void setup() {
  Serial.begin(115200);
  delay(1200);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  currentLeftUs = ESC_MID;
  currentRightUs = ESC_MID;
  escReady = escL.begin(ESC_PWM_PERIOD_US, ESC_MID) && escR.begin(ESC_PWM_PERIOD_US, ESC_MID);

  if (!escReady) {
    Serial.println("ESC PWM init failed.");
    while (true) {
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);
      delay(100);
    }
  }

  Serial.print("ESC neutral hold for ");
  Serial.print(ESC_SAFE_BOOT_NEUTRAL_MS);
  Serial.println(" ms");
  applyThrusters();
  delay(ESC_SAFE_BOOT_NEUTRAL_MS);

  if (!BLE.begin()) {
    Serial.println("BLE start failed.");
    Serial.println("Check UNO R4 WiFi connectivity firmware.");

    while (true) {
      digitalWrite(LED_PIN, HIGH);
      delay(150);
      digitalWrite(LED_PIN, LOW);
      delay(150);
    }
  }

  BLE.setDeviceName(DEVICE_NAME);
  BLE.setLocalName(LOCAL_NAME);
  BLE.setAdvertisedService(testService);

  testService.addCharacteristic(rxCharacteristic);
  testService.addCharacteristic(txCharacteristic);
  BLE.addService(testService);

  rxCharacteristic.writeValue("ready");
  sendText("ready");
  BLE.advertise();

  Serial.println();
  Serial.println("BLE thruster test started.");
  Serial.print("Advertising as: ");
  Serial.println(LOCAL_NAME);
  Serial.println("Thruster pins: left=D10 right=D9");
  Serial.println("Commands: help, arm, disarm, stop, safe, full, status");
  Serial.println("Examples: set 1600 1600 | pulse 1600 1600 1000");
}

void loop() {
  BLEDevice central = BLE.central();
  if (!central) {
    return;
  }

  Serial.print("Connected to central: ");
  Serial.println(central.address());
  sendHelp();
  sendText(statusText());
  lastStatusMs = millis();

  while (central.connected()) {
    BLE.poll();

    if (rxCharacteristic.written()) {
      String incoming = rxCharacteristic.value();
      handleIncomingText(incoming);
      lastStatusMs = millis();
    }

    unsigned long now = millis();

    if (pulseEndsAtMs > 0 && now >= pulseEndsAtMs) {
      neutralize(true);
      sendText("ok pulse done left=1500 right=1500");
    } else if (motionTimeoutAtMs > 0 && now >= motionTimeoutAtMs) {
      neutralize(true);
      sendText("ok timeout stop");
    }

    if (now - lastStatusMs >= STATUS_INTERVAL_MS) {
      sendText(statusText());
      lastStatusMs = now;
    }
  }

  Serial.print("Disconnected from central: ");
  Serial.println(central.address());
  neutralize(false);
  BLE.advertise();
}
