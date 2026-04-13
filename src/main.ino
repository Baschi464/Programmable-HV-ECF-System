#include <Wire.h>
#include <Adafruit_MCP4728.h>
#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

Adafruit_MCP4728 dac;

const uint8_t RELAY_COUNT = 8;
const uint8_t RELAY_PINS[RELAY_COUNT] = {42, 44, 46, 48, 50, 52, 51, 53};

const float HV_MIN_VOLTAGE = 0.0f;
const float HV_COMMAND_MAX_VOLTAGE = 6000.0f;
const float DAC_REFERENCE_VOLTAGE = 5.0f;
const float HV_AMPLIFIER_GAIN = 1333.33f;
const float MAX_PHYSICAL_HV_VOLTAGE = DAC_REFERENCE_VOLTAGE * HV_AMPLIFIER_GAIN;
const uint16_t DAC_MAX_VALUE = 4095;

const unsigned long WATCHDOG_TIMEOUT_MS = 1000;

const uint8_t DEFAULT_MAP[RELAY_COUNT] = {1, 1, 1, 1, 1, 1, 2, 2};

const byte RX_BUFFER_SIZE = 128;
char rxBuffer[RX_BUFFER_SIZE];
bool frameReady = false;

uint8_t relayState[RELAY_COUNT] = {0};
uint8_t channelMap[RELAY_COUNT] = {1, 1, 1, 1, 1, 1, 2, 2};
float hvSetpoints[2] = {0.0f, 0.0f};

unsigned long lastControlFrameMs = 0;
bool watchdogTripped = false;
bool mapLocked = false;

char *trimToken(char *token) {
  while (*token != '\0' && isspace((unsigned char)*token)) {
    token++;
  }

  size_t len = strlen(token);
  while (len > 0 && isspace((unsigned char)token[len - 1])) {
    token[len - 1] = '\0';
    len--;
  }

  return token;
}

bool parseUIntToken(const char *token, long minValue, long maxValue, uint8_t &outValue) {
  char *endPtr = NULL;
  long value = strtol(token, &endPtr, 10);

  if (endPtr == token || *endPtr != '\0') {
    return false;
  }
  if (value < minValue || value > maxValue) {
    return false;
  }

  outValue = (uint8_t)value;
  return true;
}

bool parseFloatToken(const char *token, float &outValue) {
  if (token == NULL || *token == '\0') {
    return false;
  }

  bool hasDigit = false;
  for (const char *p = token; *p != '\0'; p++) {
    char c = *p;
    if (c >= '0' && c <= '9') {
      hasDigit = true;
      continue;
    }
    if (c == '+' || c == '-') {
      if (p == token || *(p - 1) == 'e' || *(p - 1) == 'E') {
        continue;
      }
      return false;
    }
    if (c == '.' || c == 'e' || c == 'E') {
      continue;
    }
    return false;
  }

  if (!hasDigit) {
    return false;
  }

  outValue = (float)atof(token);
  if (isnan(outValue) || isinf(outValue)) {
    return false;
  }

  return true;
}

float clampHv(float voltage) {
  if (voltage < HV_MIN_VOLTAGE) {
    return HV_MIN_VOLTAGE;
  }
  if (voltage > HV_COMMAND_MAX_VOLTAGE) {
    return HV_COMMAND_MAX_VOLTAGE;
  }
  return voltage;
}

uint16_t hvToDac(float voltage) {
  if (DAC_REFERENCE_VOLTAGE <= 0.0f || HV_AMPLIFIER_GAIN <= 0.0f) {
    return 0;
  }

  float hv = clampHv(voltage);
  if (hv > MAX_PHYSICAL_HV_VOLTAGE) {
    hv = MAX_PHYSICAL_HV_VOLTAGE;
  }

  float dacVoltage = hv / HV_AMPLIFIER_GAIN;
  if (dacVoltage < 0.0f) {
    dacVoltage = 0.0f;
  }
  if (dacVoltage > DAC_REFERENCE_VOLTAGE) {
    dacVoltage = DAC_REFERENCE_VOLTAGE;
  }

  float scaled = (dacVoltage / DAC_REFERENCE_VOLTAGE) * DAC_MAX_VALUE;
  if (scaled < 0.0f) {
    scaled = 0.0f;
  }
  if (scaled > DAC_MAX_VALUE) {
    scaled = DAC_MAX_VALUE;
  }
  return (uint16_t)(scaled + 0.5f);
}

void applyDacOutputs() {
  bool hv1Enabled = false;
  bool hv2Enabled = false;

  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    if (!relayState[i]) {
      continue;
    }
    if (channelMap[i] == 1) {
      hv1Enabled = true;
    } else {
      hv2Enabled = true;
    }
  }

  float hv1Output = hv1Enabled ? hvSetpoints[0] : 0.0f;
  float hv2Output = hv2Enabled ? hvSetpoints[1] : 0.0f;

  dac.setChannelValue(MCP4728_CHANNEL_A, hvToDac(hv1Output));
  dac.setChannelValue(MCP4728_CHANNEL_B, hvToDac(hv2Output));
}

void applyRelayOutputs() {
  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    digitalWrite(RELAY_PINS[i], relayState[i] ? HIGH : LOW);
  }
}

void applySafeOutputs() {
  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    relayState[i] = 0;
  }
  hvSetpoints[0] = 0.0f;
  hvSetpoints[1] = 0.0f;
  applyRelayOutputs();
  applyDacOutputs();
}

void sendError(const char *code) {
  Serial.print("ERR:");
  Serial.println(code);
}

void sendActEcho() {
  Serial.print("ACT:");
  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    Serial.print(relayState[i]);
    Serial.print(",");
  }
  Serial.print(hvSetpoints[0], 1);
  Serial.print(",");
  Serial.println(hvSetpoints[1], 1);
}

void sendMapEcho() {
  Serial.print("MAP:");
  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    Serial.print(channelMap[i]);
    if (i < RELAY_COUNT - 1) {
      Serial.print(",");
    }
  }
  Serial.println();
}

void receiveFrame() {
  static bool receiveInProgress = false;
  static byte index = 0;

  while (Serial.available() > 0 && !frameReady) {
    char c = (char)Serial.read();

    if (receiveInProgress) {
      if (c == '>') {
        rxBuffer[index] = '\0';
        receiveInProgress = false;
        index = 0;
        frameReady = true;
      } else {
        if (index < RX_BUFFER_SIZE - 1) {
          rxBuffer[index++] = c;
        } else {
          receiveInProgress = false;
          index = 0;
          sendError("MALFORMED");
        }
      }
    } else if (c == '<') {
      receiveInProgress = true;
      index = 0;
    }
  }
}

bool handleMapFrame(char *tokens[], uint8_t tokenCount) {
  if (mapLocked) {
    sendError("MAP_LOCKED");
    return false;
  }

  if (tokenCount != 9) {
    sendError("MALFORMED");
    return false;
  }

  uint8_t parsedMap[RELAY_COUNT];
  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    if (!parseUIntToken(tokens[i + 1], 1, 2, parsedMap[i])) {
      sendError("BAD_MAP");
      return false;
    }
  }

  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    channelMap[i] = parsedMap[i];
  }

  sendMapEcho();
  return true;
}

bool handleControlFrame(char *tokens[], uint8_t tokenCount) {
  if (tokenCount != 10) {
    sendError("MALFORMED");
    return false;
  }

  uint8_t parsedRelays[RELAY_COUNT];
  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    if (!parseUIntToken(tokens[i], 0, 1, parsedRelays[i])) {
      sendError("MALFORMED");
      return false;
    }
  }

  float requestedHv1 = 0.0f;
  float requestedHv2 = 0.0f;
  if (!parseFloatToken(tokens[8], requestedHv1) || !parseFloatToken(tokens[9], requestedHv2)) {
    sendError("MALFORMED");
    return false;
  }

  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    relayState[i] = parsedRelays[i];
  }
  hvSetpoints[0] = clampHv(requestedHv1);
  hvSetpoints[1] = clampHv(requestedHv2);

  applyRelayOutputs();
  applyDacOutputs();

  mapLocked = true;
  lastControlFrameMs = millis();
  watchdogTripped = false;
  sendActEcho();
  return true;
}

void processFrame() {
  char *tokens[12];
  uint8_t tokenCount = 0;

  char *token = strtok(rxBuffer, ",");
  while (token != NULL && tokenCount < 12) {
    tokens[tokenCount++] = trimToken(token);
    token = strtok(NULL, ",");
  }

  if (token != NULL || tokenCount == 0) {
    sendError("MALFORMED");
    return;
  }

  if (strcmp(tokens[0], "MAP") == 0) {
    handleMapFrame(tokens, tokenCount);
    return;
  }

  handleControlFrame(tokens, tokenCount);
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10);

  Wire.begin();
  Wire.setClock(100000);

  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    pinMode(RELAY_PINS[i], OUTPUT);
    digitalWrite(RELAY_PINS[i], LOW);
    channelMap[i] = DEFAULT_MAP[i];
  }

  if (!dac.begin(0x60)) {
    sendError("DAC_INIT");
    while (1) {
      delay(100);
    }
  }

  applySafeOutputs();
  lastControlFrameMs = millis();
  watchdogTripped = false;
  mapLocked = false;

  Serial.println("<ARDUINO_READY_HV_8CH>");
  sendMapEcho();
  sendActEcho();
}

void loop() {
  receiveFrame();
  if (frameReady) {
    processFrame();
    frameReady = false;
  }

  if ((millis() - lastControlFrameMs) > WATCHDOG_TIMEOUT_MS) {
    if (!watchdogTripped) {
      applySafeOutputs();
      watchdogTripped = true;
      sendError("WATCHDOG");
      sendActEcho();
    }
  }
}