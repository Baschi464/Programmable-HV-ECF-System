#include <Wire.h>
#include <Adafruit_MCP4728.h>
#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

/*
  -----------------------------------------------------------------------------
  DIAGNOSTIC SERIAL COMMAND REFERENCE (ControlOneChannel)
  -----------------------------------------------------------------------------

  This sketch is a single-channel HV diagnostic controller.
  It controls:
    - 1 relay output  (pin 42)
    - 1 HV DAC path   (MCP4728 channel A)

  All commands must be framed with angle brackets:
    <...>

  Supported commands:

  1) Set relay + HV target
     Syntax:
       <SET,relay,hv>
     Parameters:
       relay: 0 or 1
       hv   : float in volts (clamped to 0..6000)
     Example:
       <SET,1,1500>
     Effect:
       - Updates relay state and HV setpoint
       - Applies outputs immediately
       - Refreshes watchdog timer
     Response:
       ACT:<relay>,<hv>

  2) Read current state (also keep-alive)
     Syntax:
       <GET>
     Effect:
       - Returns current relay/HV state
       - Refreshes watchdog timer (keep-alive)
     Response:
       ACT:<relay>,<hv>

  3) Force safe state
     Syntax:
       <ZERO>
     Effect:
       - relay = 0
       - hvSetpoint = 0.0
       - Applies outputs immediately
       - Refreshes watchdog timer
     Response:
       ACT:<relay>,<hv>

  4) Toggle watchdog at runtime
     Syntax:
       <WDT,0|1>
     Parameters:
       0 = disable watchdog
       1 = enable watchdog
     Examples:
       <WDT,0>   // hold a single SET indefinitely during bench tests
       <WDT,1>   // restore timeout safety
     Response:
       WDT:<0|1>
       ACT:<relay>,<hv>

  Asynchronous/error outputs:
    - ERR:MALFORMED   -> invalid frame or invalid token values
    - ERR:WATCHDOG    -> watchdog timeout occurred (when enabled)
    - ERR:DAC_INIT    -> DAC initialization failed during setup

  Notes:
    - Watchdog timeout is 1000 ms when enabled.
    - If watchdog is enabled, send GET or SET periodically to keep outputs alive.
    - hv is interpreted in volts and mapped linearly to DAC counts.
  -----------------------------------------------------------------------------
*/

Adafruit_MCP4728 dac;

const uint8_t RELAY_PIN = 42;
const float HV_MIN_VOLTAGE = 0.0f;
const float HV_COMMAND_MAX_VOLTAGE = 6000.0f;
const float DAC_REFERENCE_VOLTAGE = 5.0f;
const float HV_AMPLIFIER_GAIN = 1333.33f;
const float MAX_PHYSICAL_HV_VOLTAGE = DAC_REFERENCE_VOLTAGE * HV_AMPLIFIER_GAIN;
const uint16_t DAC_MAX_VALUE = 4095;
const unsigned long WATCHDOG_TIMEOUT_MS = 1000;
const bool WATCHDOG_ENABLED_DEFAULT = true;

const byte RX_BUFFER_SIZE = 96;
char rxBuffer[RX_BUFFER_SIZE];
bool frameReady = false;

uint8_t relayState = 0;
float hvSetpoint = 0.0f;
unsigned long lastCommandMs = 0;
bool watchdogTripped = false;
bool watchdogEnabled = WATCHDOG_ENABLED_DEFAULT;

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

void sendError(const char *code) {
  Serial.print("ERR:");
  Serial.println(code);
}

void applyOutputs() {
  digitalWrite(RELAY_PIN, relayState ? HIGH : LOW);
  dac.setChannelValue(MCP4728_CHANNEL_A, hvToDac(hvSetpoint));
  dac.setChannelValue(MCP4728_CHANNEL_B, 0);
}

void applySafeOutputs() {
  relayState = 0;
  hvSetpoint = 0.0f;
  applyOutputs();
}

void sendActEcho() {
  Serial.print("ACT:");
  Serial.print(relayState);
  Serial.print(",");
  Serial.println(hvSetpoint, 1);
}

void sendWatchdogEcho() {
  Serial.print("WDT:");
  Serial.println(watchdogEnabled ? "1" : "0");
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

void processFrame() {
  char *tokens[4];
  uint8_t tokenCount = 0;

  char *token = strtok(rxBuffer, ",");
  while (token != NULL && tokenCount < 4) {
    tokens[tokenCount++] = trimToken(token);
    token = strtok(NULL, ",");
  }

  if (token != NULL || tokenCount == 0) {
    sendError("MALFORMED");
    return;
  }

  if (tokenCount == 1) {
    if (strcmp(tokens[0], "GET") == 0) {
      // Treat GET as a keep-alive for bench sessions using readback polling.
      lastCommandMs = millis();
      watchdogTripped = false;
      sendActEcho();
      return;
    }
    if (strcmp(tokens[0], "ZERO") == 0) {
      applySafeOutputs();
      lastCommandMs = millis();
      watchdogTripped = false;
      sendActEcho();
      return;
    }
    sendError("MALFORMED");
    return;
  }

  if (tokenCount == 2 && strcmp(tokens[0], "WDT") == 0) {
    uint8_t requestedWatchdog = 0;
    if (!parseUIntToken(tokens[1], 0, 1, requestedWatchdog)) {
      sendError("MALFORMED");
      return;
    }

    watchdogEnabled = (requestedWatchdog == 1);
    lastCommandMs = millis();
    watchdogTripped = false;
    sendWatchdogEcho();
    sendActEcho();
    return;
  }

  if (tokenCount != 3 || strcmp(tokens[0], "SET") != 0) {
    sendError("MALFORMED");
    return;
  }

  uint8_t requestedRelay = 0;
  float requestedHv = 0.0f;
  if (!parseUIntToken(tokens[1], 0, 1, requestedRelay) || !parseFloatToken(tokens[2], requestedHv)) {
    sendError("MALFORMED");
    return;
  }

  relayState = requestedRelay;
  hvSetpoint = clampHv(requestedHv);
  applyOutputs();
  lastCommandMs = millis();
  watchdogTripped = false;
  sendActEcho();
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10);

  Wire.begin();
  Wire.setClock(100000);

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  if (!dac.begin(0x60)) {
    sendError("DAC_INIT");
    while (1) {
      delay(100);
    }
  }

  applySafeOutputs();
  lastCommandMs = millis();
  watchdogEnabled = WATCHDOG_ENABLED_DEFAULT;
  watchdogTripped = false;

  Serial.println("<HV_DIAG_READY>");
  Serial.println("CMD:<SET,relay(0|1),hv(0..6000)> | <ZERO> | <GET> | <WDT,0|1>");
  sendWatchdogEcho();
  sendActEcho();
}

void loop() {
  receiveFrame();
  if (frameReady) {
    processFrame();
    frameReady = false;
  }

  if (watchdogEnabled && (millis() - lastCommandMs) > WATCHDOG_TIMEOUT_MS) {
    if (!watchdogTripped) {
      applySafeOutputs();
      watchdogTripped = true;
      sendError("WATCHDOG");
      sendActEcho();
    }
  }
}