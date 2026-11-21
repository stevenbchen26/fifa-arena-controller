/********************************************************
 * Robot Soccer Game Controller v2
 * Goal Sensors + Audio + Web Control Dashboard
 *
 * Target:
 *   LoR Core v3 / ESP32 Arduino
 *
 * What this includes:
 *   - 20 second test match
 *   - Blue / Orange score tracking
 *   - Two E18-D80NK goal sensors
 *   - DY-HV20T / DY1900A UART audio module
 *   - Professional web control dashboard over WiFi AP
 *   - Serial command fallback
 *   - Event log
 *   - Sensor debounce
 *   - Goal lockout
 *   - Reset cooldown
 *   - Countdown events
 *
 * LoR Core v3 physical IO mapping used:
 *
 *   Audio:
 *     DY-HV20T TXD / IO0  -> LoR IO_01 signal -> ESP32 GPIO32
 *     DY-HV20T RXD / IO1  -> LoR IO_02 signal -> ESP32 GPIO25
 *     DY-HV20T BUSY       -> LoR IO_03 signal -> ESP32 GPIO26
 *
 *   Goal sensors:
 *     BLUE goal sensor    -> LoR IO_11 signal -> ESP32 GPIO22
 *     ORANGE goal sensor  -> LoR IO_12 signal -> ESP32 GPIO21
 *
 * Important:
 *   Arduino code uses ESP32 GPIO numbers, not LoR physical IO numbers.
 *
 * E18-D80NK wiring:
 *   Brown = VCC
 *   Blue  = GND
 *   Black = OUT / signal
 *
 * E18-D80NK logic:
 *   This version uses plain INPUT, not INPUT_PULLUP.
 *   Sensor logic is active LOW by default.
 *
 * Electrical warning:
 *   ESP32 GPIO is 3.3V logic and NOT 5V tolerant.
 *   If a sensor or module output is 5V, use a divider/level shifter.
 *
 * Audio track mapping:
 *   00001.mp3 = match start
 *   00002.mp3 = blue goal
 *   00003.mp3 = orange goal
 *   00004.mp3 = ball hit
 *   00005.mp3 = big ball hit
 *   00006.mp3 = countdown tick
 *   00007.mp3 = match end
 *   00008.mp3 = reset
 *
 * WiFi:
 *   SSID: RobotSoccer_CTRL
 *   PASS: fifabot123
 *
 * Open in browser:
 *   http://192.168.4.1
 ********************************************************/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

/* ======================================================
   USER CONFIG
   ====================================================== */

#define SERIAL_BAUD 115200

// Short match for testing.
// Change to 90000UL for a real 90 second match.
static const uint32_t MATCH_LENGTH_MS = 20000UL;

// Short pre-game state before active scoring begins.
static const uint32_t GAME_START_HOLD_MS = 1000UL;

// Countdown starts at this many seconds left.
static const uint8_t COUNTDOWN_START_SEC = 5;

// Scoring.
static const uint16_t POINTS_PER_GOAL = 1;

// Reset spam protection.
static const uint32_t RESET_COOLDOWN_MS = 750;

// Goal sensor debounce and lockout.
static const uint32_t GOAL_DEBOUNCE_MS = 75;
static const uint32_t GOAL_LOCKOUT_MS = 1500;

// Serial live status print interval.
static const uint32_t STATUS_PRINT_MS = 1000;

// Sensor status print interval.
static const uint32_t SENSOR_PRINT_MS = 500;

// Web update friendliness.
static const uint16_t WEB_PORT = 80;

/* ======================================================
   WIFI AP CONFIG
   ====================================================== */

const char* AP_SSID = "RobotSoccer_CTRL";
const char* AP_PASS = "fifabot123";

/* ======================================================
   PIN CONFIG
   ====================================================== */

// Audio module pins.
#define MP3_RX_PIN       32   // ESP32 RX, connect to module TXD / IO0
#define MP3_TX_PIN       25   // ESP32 TX, connect to module RXD / IO1
#define MP3_BUSY_PIN     26   // Module BUSY output

#define MP3_BAUD         9600

// Project test confirmed:
// BUSY HIGH = playing
// BUSY LOW  = idle
#define BUSY_ACTIVE_HIGH 1

// Goal sensors.
// LoR IO_11 -> GPIO22.
// LoR IO_12 -> GPIO21.
#define BLUE_GOAL_SENSOR_PIN    22
#define ORANGE_GOAL_SENSOR_PIN  21

// Your working setup uses inverted sensor logic.
// LOW = detected / active.
#define SENSOR_ACTIVE_LOW 1

/* ======================================================
   SYSTEM STATES AND EVENTS
   Plain constants avoid Arduino auto-prototype weirdness.
   ====================================================== */

static const uint8_t SYS_STANDBY    = 0;
static const uint8_t SYS_GAME_START = 1;
static const uint8_t SYS_IN_GAME    = 2;
static const uint8_t SYS_END_GAME   = 3;
static const uint8_t SYS_RESET      = 4;

static const uint8_t EVT_NONE           = 0;
static const uint8_t EVT_MATCH_START    = 1;
static const uint8_t EVT_MATCH_RESET    = 2;
static const uint8_t EVT_MATCH_END      = 3;
static const uint8_t EVT_BALL_HIT       = 4;
static const uint8_t EVT_BALL_BIG_HIT   = 5;
static const uint8_t EVT_GOAL_BLUE      = 6;
static const uint8_t EVT_GOAL_ORANGE    = 7;
static const uint8_t EVT_COUNTDOWN_TICK = 8;
static const uint8_t EVT_FAULT          = 9;

static const char* stateName(uint8_t s)
{
  switch (s)
  {
    case SYS_STANDBY:    return "STANDBY";
    case SYS_GAME_START: return "GAME_START";
    case SYS_IN_GAME:    return "IN_GAME";
    case SYS_END_GAME:   return "END_GAME";
    case SYS_RESET:      return "RESET";
    default:             return "UNKNOWN";
  }
}

static const char* eventName(uint8_t e)
{
  switch (e)
  {
    case EVT_MATCH_START:    return "MATCH_START";
    case EVT_MATCH_RESET:    return "MATCH_RESET";
    case EVT_MATCH_END:      return "MATCH_END";
    case EVT_BALL_HIT:       return "BALL_HIT";
    case EVT_BALL_BIG_HIT:   return "BALL_BIG_HIT";
    case EVT_GOAL_BLUE:      return "GOAL_BLUE";
    case EVT_GOAL_ORANGE:    return "GOAL_ORANGE";
    case EVT_COUNTDOWN_TICK: return "COUNTDOWN_TICK";
    case EVT_FAULT:          return "FAULT";
    default:                 return "NONE";
  }
}

/* ======================================================
   AUDIO MANAGER
   ====================================================== */

HardwareSerial MP3Serial(2);

static const uint8_t DY_START_BYTE = 0xAA;
static const uint8_t DY_CMD_PLAY_TRACK = 0x07;
static const uint8_t DY_CMD_STOP_COMMON = 0x04;

static uint16_t lastAudioTrack = 0;
static uint32_t lastAudioCommandMs = 0;
static uint32_t lastHitAudioMs = 0;
static uint32_t lastCountdownAudioMs = 0;

static const uint32_t HIT_AUDIO_COOLDOWN_MS = 300;
static const uint32_t COUNTDOWN_AUDIO_COOLDOWN_MS = 700;
static const uint32_t SAME_TRACK_COOLDOWN_MS = 120;

static void printHexByte(uint8_t b)
{
  if (b < 16) Serial.print("0");
  Serial.print(b, HEX);
}

static bool audioBusyPlaying()
{
  const int level = digitalRead(MP3_BUSY_PIN);

#if BUSY_ACTIVE_HIGH
  return level == HIGH;
#else
  return level == LOW;
#endif
}

static void sendAudioCommand(uint8_t cmd, const uint8_t* data, uint8_t len)
{
  uint8_t sum = 0;

  MP3Serial.write(DY_START_BYTE);
  sum += DY_START_BYTE;

  MP3Serial.write(cmd);
  sum += cmd;

  MP3Serial.write(len);
  sum += len;

  for (uint8_t i = 0; i < len; i++)
  {
    MP3Serial.write(data[i]);
    sum += data[i];
  }

  const uint8_t checksum = sum & 0xFF;

  MP3Serial.write(checksum);
  MP3Serial.flush();

  Serial.print("[AUDIO_TX] ");
  printHexByte(DY_START_BYTE);
  Serial.print(" ");
  printHexByte(cmd);
  Serial.print(" ");
  printHexByte(len);

  for (uint8_t i = 0; i < len; i++)
  {
    Serial.print(" ");
    printHexByte(data[i]);
  }

  Serial.print(" ");
  printHexByte(checksum);
  Serial.println();
}

static void audioPlayTrack(uint16_t track)
{
  if (track == 0) return;

  const uint32_t now = millis();

  // Avoid accidental duplicate hammering of the same sound.
  if (track == lastAudioTrack && (uint32_t)(now - lastAudioCommandMs) < SAME_TRACK_COOLDOWN_MS)
  {
    Serial.println("[AUDIO] Duplicate track ignored due to cooldown.");
    return;
  }

  uint8_t data[2];
  data[0] = (uint8_t)((track >> 8) & 0xFF);
  data[1] = (uint8_t)(track & 0xFF);

  Serial.print("[AUDIO] Play track ");
  Serial.println(track);

  sendAudioCommand(DY_CMD_PLAY_TRACK, data, 2);

  lastAudioTrack = track;
  lastAudioCommandMs = now;
}

static void audioStopCommon()
{
  Serial.println("[AUDIO] Common stop command.");
  sendAudioCommand(DY_CMD_STOP_COMMON, nullptr, 0);
}

static void audioForEvent(uint8_t event, uint16_t value)
{
  (void)value;

  const uint32_t now = millis();

  switch (event)
  {
    case EVT_MATCH_START:
      audioPlayTrack(1);
      break;

    case EVT_GOAL_BLUE:
      audioPlayTrack(2);
      break;

    case EVT_GOAL_ORANGE:
      audioPlayTrack(3);
      break;

    case EVT_BALL_HIT:
      if ((uint32_t)(now - lastHitAudioMs) >= HIT_AUDIO_COOLDOWN_MS)
      {
        lastHitAudioMs = now;
        audioPlayTrack(4);
      }
      else
      {
        Serial.println("[AUDIO] Hit sound ignored due to cooldown.");
      }
      break;

    case EVT_BALL_BIG_HIT:
      lastHitAudioMs = now;
      audioPlayTrack(5);
      break;

    case EVT_COUNTDOWN_TICK:
      if ((uint32_t)(now - lastCountdownAudioMs) >= COUNTDOWN_AUDIO_COOLDOWN_MS)
      {
        lastCountdownAudioMs = now;
        audioPlayTrack(6);
      }
      break;

    case EVT_MATCH_END:
      audioPlayTrack(7);
      break;

    case EVT_MATCH_RESET:
      audioPlayTrack(8);
      break;

    default:
      break;
  }
}

static void drainAudioRx()
{
  bool any = false;

  while (MP3Serial.available() > 0)
  {
    if (!any)
    {
      Serial.print("[AUDIO_RX] ");
      any = true;
    }

    uint8_t b = (uint8_t)MP3Serial.read();
    printHexByte(b);
    Serial.print(" ");
  }

  if (any)
  {
    Serial.println();
  }
}

static void beginAudio()
{
  pinMode(MP3_BUSY_PIN, INPUT);

  MP3Serial.begin(
    MP3_BAUD,
    SERIAL_8N1,
    MP3_RX_PIN,
    MP3_TX_PIN
  );

  delay(100);

  Serial.println("[INIT] Audio UART started.");
  Serial.print("[INIT] Audio RX GPIO=");
  Serial.println(MP3_RX_PIN);
  Serial.print("[INIT] Audio TX GPIO=");
  Serial.println(MP3_TX_PIN);
  Serial.print("[INIT] Audio BUSY GPIO=");
  Serial.println(MP3_BUSY_PIN);
}

/* ======================================================
   EVENT LOG
   ====================================================== */

static const uint8_t EVENT_LOG_SIZE = 24;

static uint32_t logMs[EVENT_LOG_SIZE];
static uint8_t  logEvent[EVENT_LOG_SIZE];
static uint16_t logValue[EVENT_LOG_SIZE];
static uint16_t logBlueScore[EVENT_LOG_SIZE];
static uint16_t logOrangeScore[EVENT_LOG_SIZE];

static uint8_t eventLogHead = 0;
static uint32_t eventCounter = 0;

/* ======================================================
   GAME STATE
   ====================================================== */

static uint8_t gameState = SYS_STANDBY;

static bool matchRunning = false;
static uint32_t matchStartMs = 0;
static uint32_t stateEnterMs = 0;

static uint16_t blueScore = 0;
static uint16_t orangeScore = 0;

// Bitmask used to prevent duplicate countdown events.
static uint16_t countdownFiredMask = 0;

static uint8_t lastEvent = EVT_NONE;
static uint16_t lastEventValue = 0;
static uint32_t lastEventMs = 0;

static uint32_t lastResetRequestMs = 0;

/* ======================================================
   GOAL SENSOR ARRAYS
   ====================================================== */

static const uint8_t SENSOR_BLUE = 0;
static const uint8_t SENSOR_ORANGE = 1;
static const uint8_t SENSOR_COUNT = 2;

static uint8_t sensorPins[SENSOR_COUNT] = {
  BLUE_GOAL_SENSOR_PIN,
  ORANGE_GOAL_SENSOR_PIN
};

static const char* sensorNames[SENSOR_COUNT] = {
  "BLUE_GOAL",
  "ORANGE_GOAL"
};

static bool sensorRawActive[SENSOR_COUNT] = {false, false};
static bool sensorDebouncedActive[SENSOR_COUNT] = {false, false};
static bool sensorLastDebouncedActive[SENSOR_COUNT] = {false, false};
static uint32_t sensorRawChangedMs[SENSOR_COUNT] = {0, 0};
static uint32_t sensorLastGoalCountMs[SENSOR_COUNT] = {0, 0};

/* ======================================================
   WEB SERVER
   ====================================================== */

WebServer server(WEB_PORT);

/* ======================================================
   UTILITY
   ====================================================== */

static bool sensorDetectedFromPin(uint8_t pin)
{
  const bool rawHigh = (digitalRead(pin) == HIGH);

#if SENSOR_ACTIVE_LOW
  return !rawHigh;
#else
  return rawHigh;
#endif
}

static void clearEventLog()
{
  for (uint8_t i = 0; i < EVENT_LOG_SIZE; i++)
  {
    logMs[i] = 0;
    logEvent[i] = EVT_NONE;
    logValue[i] = 0;
    logBlueScore[i] = 0;
    logOrangeScore[i] = 0;
  }

  eventLogHead = 0;
  eventCounter = 0;
}

static uint32_t elapsedMatchMs()
{
  if (!matchRunning)
  {
    return 0;
  }

  uint32_t t = millis() - matchStartMs;

  if (t > MATCH_LENGTH_MS)
  {
    t = MATCH_LENGTH_MS;
  }

  return t;
}

static uint32_t remainingMatchMs()
{
  if (gameState == SYS_STANDBY || gameState == SYS_RESET)
  {
    return MATCH_LENGTH_MS;
  }

  if (gameState == SYS_END_GAME)
  {
    return 0;
  }

  const uint32_t elapsed = elapsedMatchMs();

  if (elapsed >= MATCH_LENGTH_MS)
  {
    return 0;
  }

  return MATCH_LENGTH_MS - elapsed;
}

static uint8_t remainingSecondsCeil()
{
  const uint32_t rem = remainingMatchMs();

  if (rem == 0)
  {
    return 0;
  }

  return (uint8_t)((rem + 999UL) / 1000UL);
}

static void addEventLog(uint8_t event, uint16_t value)
{
  logMs[eventLogHead] = millis();
  logEvent[eventLogHead] = event;
  logValue[eventLogHead] = value;
  logBlueScore[eventLogHead] = blueScore;
  logOrangeScore[eventLogHead] = orangeScore;

  eventLogHead++;

  if (eventLogHead >= EVENT_LOG_SIZE)
  {
    eventLogHead = 0;
  }

  eventCounter++;
}

static void emitEvent(uint8_t event, uint16_t value)
{
  lastEvent = event;
  lastEventValue = value;
  lastEventMs = millis();

  addEventLog(event, value);

  Serial.print("[EVENT] ");
  Serial.print(eventName(event));

  Serial.print(" value=");
  Serial.print(value);

  Serial.print(" state=");
  Serial.print(stateName(gameState));

  Serial.print(" time_left=");
  Serial.print(remainingSecondsCeil());
  Serial.print("s");

  Serial.print(" score BLUE=");
  Serial.print(blueScore);

  Serial.print(" ORANGE=");
  Serial.println(orangeScore);

  audioForEvent(event, value);
}

/* ======================================================
   GAME CONTROL
   ====================================================== */

static void enterState(uint8_t next)
{
  gameState = next;
  stateEnterMs = millis();

  Serial.print("[STATE] ");
  Serial.println(stateName(gameState));
}

static bool alreadyCleanStandby()
{
  return
    gameState == SYS_STANDBY &&
    !matchRunning &&
    blueScore == 0 &&
    orangeScore == 0;
}

static void resetMatch()
{
  const uint32_t now = millis();

  if ((uint32_t)(now - lastResetRequestMs) < RESET_COOLDOWN_MS)
  {
    Serial.println("[IGNORED] Reset ignored due to cooldown.");
    return;
  }

  lastResetRequestMs = now;

  if (alreadyCleanStandby())
  {
    Serial.println("[IGNORED] Reset ignored because controller is already clean standby.");
    return;
  }

  matchRunning = false;
  matchStartMs = 0;

  blueScore = 0;
  orangeScore = 0;

  countdownFiredMask = 0;

  clearEventLog();

  enterState(SYS_RESET);
  emitEvent(EVT_MATCH_RESET, 0);

  delay(100);
  enterState(SYS_STANDBY);
}

static void startMatch()
{
  clearEventLog();

  blueScore = 0;
  orangeScore = 0;

  matchRunning = true;
  matchStartMs = millis();
  countdownFiredMask = 0;

  // Re-arm sensor edge state so a blocked sensor at start does not instantly score.
  for (uint8_t i = 0; i < SENSOR_COUNT; i++)
  {
    sensorRawActive[i] = sensorDetectedFromPin(sensorPins[i]);
    sensorDebouncedActive[i] = sensorRawActive[i];
    sensorLastDebouncedActive[i] = sensorDebouncedActive[i];
    sensorRawChangedMs[i] = millis();
    sensorLastGoalCountMs[i] = 0;
  }

  enterState(SYS_GAME_START);
  emitEvent(EVT_MATCH_START, 0);
}

static void endMatch()
{
  if (gameState == SYS_END_GAME)
  {
    return;
  }

  matchRunning = false;
  enterState(SYS_END_GAME);
  emitEvent(EVT_MATCH_END, 0);
}

static bool scoringAllowed()
{
  return matchRunning && gameState == SYS_IN_GAME;
}

static void scoreBlueGoal(uint16_t sourceValue)
{
  if (!scoringAllowed())
  {
    Serial.println("[IGNORED] Blue goal ignored because match is not actively in-game.");
    return;
  }

  blueScore += POINTS_PER_GOAL;
  emitEvent(EVT_GOAL_BLUE, sourceValue);
}

static void scoreOrangeGoal(uint16_t sourceValue)
{
  if (!scoringAllowed())
  {
    Serial.println("[IGNORED] Orange goal ignored because match is not actively in-game.");
    return;
  }

  orangeScore += POINTS_PER_GOAL;
  emitEvent(EVT_GOAL_ORANGE, sourceValue);
}

static void fakeBallHit(bool big)
{
  if (!matchRunning)
  {
    Serial.println("[IGNORED] Ball hit ignored because match is not running.");
    return;
  }

  if (big)
  {
    emitEvent(EVT_BALL_BIG_HIT, 1500);
  }
  else
  {
    emitEvent(EVT_BALL_HIT, 500);
  }
}

/* ======================================================
   SENSOR HANDLING
   ====================================================== */

static void beginGoalSensors()
{
  for (uint8_t i = 0; i < SENSOR_COUNT; i++)
  {
    pinMode(sensorPins[i], INPUT);

    sensorRawActive[i] = sensorDetectedFromPin(sensorPins[i]);
    sensorDebouncedActive[i] = sensorRawActive[i];
    sensorLastDebouncedActive[i] = sensorDebouncedActive[i];
    sensorRawChangedMs[i] = millis();
    sensorLastGoalCountMs[i] = 0;
  }

  Serial.println("[INIT] Goal sensors enabled.");
  Serial.print("[INIT] BLUE sensor: LoR IO_11 / ESP32 GPIO ");
  Serial.println(BLUE_GOAL_SENSOR_PIN);
  Serial.print("[INIT] ORANGE sensor: LoR IO_12 / ESP32 GPIO ");
  Serial.println(ORANGE_GOAL_SENSOR_PIN);

#if SENSOR_ACTIVE_LOW
  Serial.println("[INIT] Sensor logic: ACTIVE LOW / inverted");
#else
  Serial.println("[INIT] Sensor logic: ACTIVE HIGH / non-inverted");
#endif

  Serial.println("[INIT] Sensor pinMode: INPUT");
}

static bool updateGoalSensor(uint8_t idx, uint32_t now)
{
  if (idx >= SENSOR_COUNT)
  {
    return false;
  }

  const bool rawNow = sensorDetectedFromPin(sensorPins[idx]);

  if (rawNow != sensorRawActive[idx])
  {
    sensorRawActive[idx] = rawNow;
    sensorRawChangedMs[idx] = now;

    Serial.print("[SENSOR_RAW] ");
    Serial.print(sensorNames[idx]);
    Serial.print(" raw=");
    Serial.println(rawNow ? "ACTIVE" : "CLEAR");
  }

  if ((uint32_t)(now - sensorRawChangedMs[idx]) >= GOAL_DEBOUNCE_MS)
  {
    sensorDebouncedActive[idx] = sensorRawActive[idx];
  }

  const bool risingEdge = sensorDebouncedActive[idx] && !sensorLastDebouncedActive[idx];
  sensorLastDebouncedActive[idx] = sensorDebouncedActive[idx];

  return risingEdge;
}

static void serviceGoalSensors(uint32_t now)
{
  if (updateGoalSensor(SENSOR_BLUE, now))
  {
    if ((uint32_t)(now - sensorLastGoalCountMs[SENSOR_BLUE]) >= GOAL_LOCKOUT_MS)
    {
      sensorLastGoalCountMs[SENSOR_BLUE] = now;
      Serial.print("[SENSOR] ");
      Serial.print(sensorNames[SENSOR_BLUE]);
      Serial.println(" debounced trigger.");
      scoreBlueGoal(1);
    }
    else
    {
      Serial.println("[SENSOR] Blue goal ignored due to lockout.");
    }
  }

  if (updateGoalSensor(SENSOR_ORANGE, now))
  {
    if ((uint32_t)(now - sensorLastGoalCountMs[SENSOR_ORANGE]) >= GOAL_LOCKOUT_MS)
    {
      sensorLastGoalCountMs[SENSOR_ORANGE] = now;
      Serial.print("[SENSOR] ");
      Serial.print(sensorNames[SENSOR_ORANGE]);
      Serial.println(" debounced trigger.");
      scoreOrangeGoal(1);
    }
    else
    {
      Serial.println("[SENSOR] Orange goal ignored due to lockout.");
    }
  }
}

/* ======================================================
   TIMELINE
   ====================================================== */

static void serviceTimeline(uint32_t now)
{
  if (gameState == SYS_GAME_START)
  {
    if ((uint32_t)(now - stateEnterMs) >= GAME_START_HOLD_MS)
    {
      enterState(SYS_IN_GAME);
    }
  }

  if (matchRunning)
  {
    const uint32_t elapsed = elapsedMatchMs();

    if (elapsed >= MATCH_LENGTH_MS)
    {
      endMatch();
      return;
    }

    if (gameState == SYS_IN_GAME)
    {
      const uint8_t secLeft = remainingSecondsCeil();

      if (secLeft > 0 && secLeft <= COUNTDOWN_START_SEC)
      {
        const uint16_t bit = (uint16_t)(1u << secLeft);

        if ((countdownFiredMask & bit) == 0)
        {
          countdownFiredMask |= bit;
          emitEvent(EVT_COUNTDOWN_TICK, secLeft);
        }
      }
    }
  }
}

/* ======================================================
   JSON HELPERS
   ====================================================== */

static String eventLogJson()
{
  String out = "[";

  bool first = true;

  for (uint8_t k = 0; k < EVENT_LOG_SIZE; k++)
  {
    uint8_t idx = (eventLogHead + k) % EVENT_LOG_SIZE;

    if (logEvent[idx] == EVT_NONE)
    {
      continue;
    }

    if (!first) out += ",";
    first = false;

    out += "{";
    out += "\"t\":";
    out += String(logMs[idx]);
    out += ",\"event\":\"";
    out += eventName(logEvent[idx]);
    out += "\",\"value\":";
    out += String(logValue[idx]);
    out += ",\"blue\":";
    out += String(logBlueScore[idx]);
    out += ",\"orange\":";
    out += String(logOrangeScore[idx]);
    out += "}";
  }

  out += "]";
  return out;
}

static String stateJson()
{
  String out = "{";

  out += "\"state\":\"";
  out += stateName(gameState);
  out += "\",";

  out += "\"running\":";
  out += matchRunning ? "true" : "false";
  out += ",";

  out += "\"timeLeft\":";
  out += String(remainingSecondsCeil());
  out += ",";

  out += "\"elapsedMs\":";
  out += String(elapsedMatchMs());
  out += ",";

  out += "\"matchLengthSec\":";
  out += String(MATCH_LENGTH_MS / 1000UL);
  out += ",";

  out += "\"blue\":";
  out += String(blueScore);
  out += ",";

  out += "\"orange\":";
  out += String(orangeScore);
  out += ",";

  out += "\"lastEvent\":\"";
  out += eventName(lastEvent);
  out += "\",";

  out += "\"lastValue\":";
  out += String(lastEventValue);
  out += ",";

  out += "\"sensorBlue\":";
  out += sensorDebouncedActive[SENSOR_BLUE] ? "true" : "false";
  out += ",";

  out += "\"sensorOrange\":";
  out += sensorDebouncedActive[SENSOR_ORANGE] ? "true" : "false";
  out += ",";

  out += "\"audioBusy\":";
  out += audioBusyPlaying() ? "true" : "false";
  out += ",";

  out += "\"lastAudioTrack\":";
  out += String(lastAudioTrack);
  out += ",";

  out += "\"eventCount\":";
  out += String(eventCounter);
  out += ",";

  out += "\"log\":";
  out += eventLogJson();

  out += "}";
  return out;
}

/* ======================================================
   WEB UI
   ====================================================== */

static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
  <title>Robot Soccer Controller</title>
  <style>
    :root{
      --bg:#08111f;
      --panel:#101b2f;
      --panel2:#0d1728;
      --text:#eaf2ff;
      --muted:#8da2c0;
      --blue:#2f80ff;
      --orange:#ff8a1f;
      --green:#2bd576;
      --red:#ff4d5e;
      --yellow:#ffd166;
      --border:rgba(255,255,255,.11);
      --shadow:0 18px 50px rgba(0,0,0,.35);
    }
    *{box-sizing:border-box}
    body{
      margin:0;
      font-family:system-ui,-apple-system,Segoe UI,Roboto,Arial,sans-serif;
      color:var(--text);
      background:
        radial-gradient(circle at 15% 10%, rgba(47,128,255,.22), transparent 30%),
        radial-gradient(circle at 85% 5%, rgba(255,138,31,.20), transparent 28%),
        linear-gradient(180deg,#07101d,#050914 80%);
      min-height:100vh;
    }
    .wrap{max-width:1180px;margin:0 auto;padding:22px}
    .top{
      display:flex;align-items:center;justify-content:space-between;gap:16px;
      margin-bottom:18px;
    }
    .brand h1{font-size:24px;margin:0 0 4px;font-weight:800;letter-spacing:.2px}
    .brand p{margin:0;color:var(--muted);font-size:14px}
    .pill{
      border:1px solid var(--border);
      background:rgba(255,255,255,.06);
      border-radius:999px;
      padding:10px 14px;
      color:var(--muted);
      font-weight:700;
      min-width:130px;
      text-align:center;
    }
    .grid{
      display:grid;
      grid-template-columns: 1.2fr .8fr;
      gap:16px;
    }
    .card{
      background:linear-gradient(180deg,rgba(255,255,255,.07),rgba(255,255,255,.035));
      border:1px solid var(--border);
      border-radius:24px;
      box-shadow:var(--shadow);
      padding:18px;
      backdrop-filter: blur(8px);
    }
    .scoreboard{
      display:grid;
      grid-template-columns:1fr auto 1fr;
      align-items:center;
      gap:16px;
      min-height:220px;
    }
    .team{
      border-radius:22px;
      padding:22px;
      background:rgba(255,255,255,.045);
      border:1px solid var(--border);
      text-align:center;
    }
    .team.blue{box-shadow:inset 0 0 0 1px rgba(47,128,255,.35)}
    .team.orange{box-shadow:inset 0 0 0 1px rgba(255,138,31,.35)}
    .label{font-size:14px;color:var(--muted);font-weight:800;letter-spacing:.12em;text-transform:uppercase}
    .score{
      font-size:88px;
      font-weight:900;
      line-height:.9;
      margin-top:10px;
    }
    .blue .score{color:var(--blue)}
    .orange .score{color:var(--orange)}
    .timer{
      width:160px;height:160px;border-radius:50%;
      display:flex;flex-direction:column;align-items:center;justify-content:center;
      background:radial-gradient(circle,rgba(255,255,255,.10),rgba(255,255,255,.03));
      border:1px solid var(--border);
    }
    .time{font-size:54px;font-weight:900}
    .state{font-size:13px;color:var(--muted);font-weight:800;letter-spacing:.08em}
    .controls{
      display:grid;
      grid-template-columns:repeat(2,1fr);
      gap:10px;
      margin-top:14px;
    }
    button{
      border:0;
      border-radius:16px;
      padding:14px 14px;
      color:white;
      font-weight:850;
      background:#1e2b44;
      box-shadow:0 8px 20px rgba(0,0,0,.20);
      cursor:pointer;
      font-size:15px;
    }
    button:active{transform:translateY(1px)}
    .start{background:linear-gradient(135deg,#18b866,#2bd576)}
    .reset{background:linear-gradient(135deg,#7b4dff,#a066ff)}
    .end{background:linear-gradient(135deg,#e43b4e,#ff6876)}
    .blueBtn{background:linear-gradient(135deg,#1f63d6,#2f80ff)}
    .orangeBtn{background:linear-gradient(135deg,#d76a12,#ff8a1f)}
    .hit{background:linear-gradient(135deg,#39465f,#58667f)}
    .big{background:linear-gradient(135deg,#ad2636,#ff4d5e)}
    .sound{background:linear-gradient(135deg,#1c516e,#2b97d5)}
    .miniGrid{
      display:grid;
      grid-template-columns:repeat(2,1fr);
      gap:10px;
      margin-top:14px;
    }
    .metric{
      background:rgba(255,255,255,.045);
      border:1px solid var(--border);
      border-radius:18px;
      padding:14px;
    }
    .metric .name{color:var(--muted);font-size:12px;font-weight:800;text-transform:uppercase;letter-spacing:.08em}
    .metric .val{font-size:22px;font-weight:900;margin-top:4px}
    .ok{color:var(--green)}
    .bad{color:var(--red)}
    .warn{color:var(--yellow)}
    .log{
      margin-top:12px;
      max-height:300px;
      overflow:auto;
      border-radius:18px;
      border:1px solid var(--border);
      background:rgba(0,0,0,.18);
    }
    .row{
      display:grid;
      grid-template-columns:90px 1fr 90px;
      gap:8px;
      padding:10px 12px;
      border-bottom:1px solid rgba(255,255,255,.07);
      font-size:13px;
      color:#cbd8eb;
    }
    .row:last-child{border-bottom:0}
    .muted{color:var(--muted)}
    .footer{
      margin-top:16px;
      color:var(--muted);
      font-size:12px;
      text-align:center;
    }
    @media (max-width:850px){
      .grid{grid-template-columns:1fr}
      .scoreboard{grid-template-columns:1fr;gap:12px}
      .timer{width:100%;height:auto;border-radius:22px;padding:18px}
      .score{font-size:72px}
    }
  </style>
</head>
<body>
  <div class="wrap">
    <div class="top">
      <div class="brand">
        <h1>Robot Soccer Controller</h1>
        <p>FMS test dashboard: goals, audio, match flow, sensors</p>
      </div>
      <div class="pill" id="conn">connecting</div>
    </div>

    <div class="grid">
      <div class="card">
        <div class="scoreboard">
          <div class="team blue">
            <div class="label">Blue</div>
            <div class="score" id="blue">0</div>
          </div>

          <div class="timer">
            <div class="time" id="time">20</div>
            <div class="state" id="state">STANDBY</div>
          </div>

          <div class="team orange">
            <div class="label">Orange</div>
            <div class="score" id="orange">0</div>
          </div>
        </div>

        <div class="controls">
          <button class="start" onclick="cmd('start')">Start Match</button>
          <button class="reset" onclick="cmd('reset')">Reset</button>
          <button class="end" onclick="cmd('end')">End Match</button>
          <button class="hit" onclick="cmd('hit')">Fake Hit</button>
          <button class="big" onclick="cmd('bigHit')">Fake Big Hit</button>
          <button class="sound" onclick="playTrack()">Play Track</button>
          <button class="blueBtn" onclick="cmd('goalBlue')">Fake Blue Goal</button>
          <button class="orangeBtn" onclick="cmd('goalOrange')">Fake Orange Goal</button>
        </div>
      </div>

      <div class="card">
        <div class="label">Live System Status</div>
        <div class="miniGrid">
          <div class="metric">
            <div class="name">Blue Sensor</div>
            <div class="val" id="blueSensor">clear</div>
          </div>
          <div class="metric">
            <div class="name">Orange Sensor</div>
            <div class="val" id="orangeSensor">clear</div>
          </div>
          <div class="metric">
            <div class="name">Audio</div>
            <div class="val" id="audio">idle</div>
          </div>
          <div class="metric">
            <div class="name">Last Event</div>
            <div class="val" id="last">NONE</div>
          </div>
        </div>

        <div class="metric" style="margin-top:10px">
          <div class="name">Manual Track Test</div>
          <input id="track" type="number" min="1" max="9999" value="1"
                 style="width:100%;margin-top:8px;padding:12px;border-radius:12px;border:1px solid rgba(255,255,255,.15);background:#0b1424;color:white;font-weight:800">
        </div>

        <div class="label" style="margin-top:16px">Event Log</div>
        <div class="log" id="log"></div>
      </div>
    </div>

    <div class="footer">
      AP: RobotSoccer_CTRL | Test match length: <span id="matchLength">20</span>s | Refresh: 500 ms
    </div>
  </div>

<script>
async function api(path){
  try{
    const r = await fetch(path, {method:'POST'});
    await update();
  }catch(e){
    document.getElementById('conn').textContent = 'offline';
  }
}
function cmd(name){ api('/api/' + name); }
function playTrack(){
  const t = document.getElementById('track').value || 1;
  api('/api/play?track=' + encodeURIComponent(t));
}
function cls(el, status){
  el.className = 'val ' + status;
}
async function update(){
  try{
    const r = await fetch('/api/state');
    const s = await r.json();

    document.getElementById('conn').textContent = 'online';
    document.getElementById('blue').textContent = s.blue;
    document.getElementById('orange').textContent = s.orange;
    document.getElementById('time').textContent = s.timeLeft;
    document.getElementById('state').textContent = s.state;
    document.getElementById('last').textContent = s.lastEvent;
    document.getElementById('matchLength').textContent = s.matchLengthSec;

    const bs = document.getElementById('blueSensor');
    bs.textContent = s.sensorBlue ? 'ACTIVE' : 'clear';
    cls(bs, s.sensorBlue ? 'bad' : 'ok');

    const os = document.getElementById('orangeSensor');
    os.textContent = s.sensorOrange ? 'ACTIVE' : 'clear';
    cls(os, s.sensorOrange ? 'bad' : 'ok');

    const au = document.getElementById('audio');
    au.textContent = s.audioBusy ? 'PLAYING' : 'idle';
    cls(au, s.audioBusy ? 'warn' : 'ok');

    const log = document.getElementById('log');
    if(!s.log || s.log.length === 0){
      log.innerHTML = '<div class="row"><span class="muted">empty</span><span>No events yet</span><span></span></div>';
    }else{
      log.innerHTML = s.log.slice().reverse().map(e => `
        <div class="row">
          <span class="muted">${e.t}</span>
          <span>${e.event} <span class="muted">(${e.value})</span></span>
          <span>B${e.blue} O${e.orange}</span>
        </div>
      `).join('');
    }
  }catch(e){
    document.getElementById('conn').textContent = 'offline';
  }
}
setInterval(update, 500);
update();
</script>
</body>
</html>
)rawliteral";

/* ======================================================
   WEB HANDLERS
   ====================================================== */

static void sendNoCache()
{
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  server.sendHeader("Pragma", "no-cache");
  server.sendHeader("Expires", "0");
}

static void handleRoot()
{
  sendNoCache();
  server.send_P(200, "text/html", INDEX_HTML);
}

static void handleState()
{
  sendNoCache();
  server.send(200, "application/json", stateJson());
}

static void handleStart()
{
  startMatch();
  server.send(200, "application/json", stateJson());
}

static void handleReset()
{
  resetMatch();
  server.send(200, "application/json", stateJson());
}

static void handleEnd()
{
  endMatch();
  server.send(200, "application/json", stateJson());
}

static void handleGoalBlue()
{
  scoreBlueGoal(99);
  server.send(200, "application/json", stateJson());
}

static void handleGoalOrange()
{
  scoreOrangeGoal(99);
  server.send(200, "application/json", stateJson());
}

static void handleHit()
{
  fakeBallHit(false);
  server.send(200, "application/json", stateJson());
}

static void handleBigHit()
{
  fakeBallHit(true);
  server.send(200, "application/json", stateJson());
}

static void handlePlay()
{
  if (server.hasArg("track"))
  {
    uint16_t track = (uint16_t)server.arg("track").toInt();
    audioPlayTrack(track);
  }

  server.send(200, "application/json", stateJson());
}

static void handleNotFound()
{
  server.send(404, "text/plain", "Not found");
}

static void beginWeb()
{
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);

  IPAddress ip = WiFi.softAPIP();

  Serial.println("[INIT] WiFi AP started.");
  Serial.print("[INIT] SSID: ");
  Serial.println(AP_SSID);
  Serial.print("[INIT] PASS: ");
  Serial.println(AP_PASS);
  Serial.print("[INIT] URL: http://");
  Serial.println(ip);

  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/state", HTTP_GET, handleState);

  server.on("/api/start", HTTP_POST, handleStart);
  server.on("/api/reset", HTTP_POST, handleReset);
  server.on("/api/end", HTTP_POST, handleEnd);
  server.on("/api/goalBlue", HTTP_POST, handleGoalBlue);
  server.on("/api/goalOrange", HTTP_POST, handleGoalOrange);
  server.on("/api/hit", HTTP_POST, handleHit);
  server.on("/api/bigHit", HTTP_POST, handleBigHit);
  server.on("/api/play", HTTP_POST, handlePlay);

  server.onNotFound(handleNotFound);

  server.begin();
  Serial.println("[INIT] Web server started.");
}

/* ======================================================
   SERIAL COMMANDS
   ====================================================== */

static void printEventLog()
{
  Serial.println();
  Serial.println("Recent event log:");

  bool any = false;

  for (uint8_t k = 0; k < EVENT_LOG_SIZE; k++)
  {
    uint8_t idx = (eventLogHead + k) % EVENT_LOG_SIZE;

    if (logEvent[idx] == EVT_NONE)
    {
      continue;
    }

    any = true;

    Serial.print("  t=");
    Serial.print(logMs[idx]);

    Serial.print(" event=");
    Serial.print(eventName(logEvent[idx]));

    Serial.print(" value=");
    Serial.print(logValue[idx]);

    Serial.print(" BLUE=");
    Serial.print(logBlueScore[idx]);

    Serial.print(" ORANGE=");
    Serial.println(logOrangeScore[idx]);
  }

  if (!any)
  {
    Serial.println("  <empty>");
  }

  Serial.println();
}

static void printStatus()
{
  Serial.println();
  Serial.println("========== STATUS ==========");

  Serial.print("State: ");
  Serial.println(stateName(gameState));

  Serial.print("Match running: ");
  Serial.println(matchRunning ? "YES" : "NO");

  Serial.print("Time left: ");
  Serial.print(remainingSecondsCeil());
  Serial.println(" s");

  Serial.print("Elapsed: ");
  Serial.print(elapsedMatchMs());
  Serial.println(" ms");

  Serial.print("Score BLUE: ");
  Serial.println(blueScore);

  Serial.print("Score ORANGE: ");
  Serial.println(orangeScore);

  Serial.print("Last event: ");
  Serial.print(eventName(lastEvent));
  Serial.print(" value=");
  Serial.println(lastEventValue);

  Serial.print("Blue goal sensor: ");
  Serial.println(sensorDebouncedActive[SENSOR_BLUE] ? "ACTIVE" : "clear");

  Serial.print("Orange goal sensor: ");
  Serial.println(sensorDebouncedActive[SENSOR_ORANGE] ? "ACTIVE" : "clear");

  Serial.print("Audio BUSY: ");
  Serial.println(audioBusyPlaying() ? "PLAYING" : "idle");

  Serial.print("Last audio track: ");
  Serial.println(lastAudioTrack);

  Serial.print("Web URL: http://");
  Serial.println(WiFi.softAPIP());

  Serial.print("Event count: ");
  Serial.println(eventCounter);

  Serial.println("============================");
  Serial.println();
}

static void printHelp()
{
  Serial.println();
  Serial.println("========== Robot Soccer Game Controller v2 ==========");
  Serial.println("Serial commands:");
  Serial.println("  ? = help");
  Serial.println("  s = start match");
  Serial.println("  r = reset match");
  Serial.println("  e = force end match");
  Serial.println("  b = fake BLUE goal");
  Serial.println("  o = fake ORANGE goal");
  Serial.println("  h = fake ball hit");
  Serial.println("  H = fake big ball hit");
  Serial.println("  p = print status");
  Serial.println("  l = print event log");
  Serial.println("  1-8 = manually play audio track 1 to 8");
  Serial.println("  q = send common audio stop command");
  Serial.println();
  Serial.print("Web dashboard: http://");
  Serial.println(WiFi.softAPIP());
  Serial.print("AP SSID: ");
  Serial.println(AP_SSID);
  Serial.print("AP PASS: ");
  Serial.println(AP_PASS);
  Serial.println("=====================================================");
  Serial.println();
}

static void handleSerialCommand(char c)
{
  if (c >= '1' && c <= '8')
  {
    audioPlayTrack((uint16_t)(c - '0'));
    return;
  }

  switch (c)
  {
    case '?':
      printHelp();
      break;

    case 's':
      startMatch();
      break;

    case 'r':
    case 'R':
      resetMatch();
      break;

    case 'e':
      endMatch();
      break;

    case 'b':
      scoreBlueGoal(99);
      break;

    case 'o':
      scoreOrangeGoal(99);
      break;

    case 'h':
      fakeBallHit(false);
      break;

    case 'H':
      fakeBallHit(true);
      break;

    case 'p':
      printStatus();
      break;

    case 'l':
      printEventLog();
      break;

    case 'q':
    case 'Q':
      audioStopCommon();
      break;

    case '\n':
    case '\r':
    case ' ':
      break;

    default:
      Serial.print("[UNKNOWN COMMAND] ");
      Serial.println(c);
      Serial.println("Type ? for help.");
      break;
  }
}

static void serviceSerial()
{
  while (Serial.available() > 0)
  {
    char c = (char)Serial.read();
    handleSerialCommand(c);
  }
}

/* ======================================================
   PERIODIC STATUS
   ====================================================== */

static void periodicStatus(uint32_t now)
{
  static uint32_t lastPrintMs = 0;

  if ((uint32_t)(now - lastPrintMs) < STATUS_PRINT_MS)
  {
    return;
  }

  lastPrintMs = now;

  Serial.print("[LIVE] state=");
  Serial.print(stateName(gameState));

  Serial.print(" time_left=");
  Serial.print(remainingSecondsCeil());
  Serial.print("s");

  Serial.print(" BLUE=");
  Serial.print(blueScore);

  Serial.print(" ORANGE=");
  Serial.print(orangeScore);

  Serial.print(" blueSensor=");
  Serial.print(sensorDebouncedActive[SENSOR_BLUE] ? "ACTIVE" : "clear");

  Serial.print(" orangeSensor=");
  Serial.print(sensorDebouncedActive[SENSOR_ORANGE] ? "ACTIVE" : "clear");

  Serial.print(" audio=");
  Serial.print(audioBusyPlaying() ? "PLAYING" : "idle");

  Serial.print(" last=");
  Serial.println(eventName(lastEvent));
}

static void periodicSensorStatus(uint32_t now)
{
  static uint32_t lastSensorPrintMs = 0;

  if ((uint32_t)(now - lastSensorPrintMs) < SENSOR_PRINT_MS)
  {
    return;
  }

  lastSensorPrintMs = now;

  // Keep this quiet-ish but still useful.
}

/* ======================================================
   SETUP / LOOP
   ====================================================== */

void setup()
{
  Serial.begin(SERIAL_BAUD);
  delay(300);

  Serial.println();
  Serial.println("Robot Soccer Game Controller v2 - Sensors + Audio + Web");

  clearEventLog();

  beginAudio();
  beginGoalSensors();
  beginWeb();

  matchRunning = false;
  matchStartMs = 0;
  blueScore = 0;
  orangeScore = 0;
  countdownFiredMask = 0;
  gameState = SYS_STANDBY;
  stateEnterMs = millis();
  lastEvent = EVT_NONE;
  lastEventValue = 0;
  lastEventMs = 0;

  Serial.println("[STATE] STANDBY");
  printHelp();
}

void loop()
{
  const uint32_t now = millis();

  server.handleClient();

  serviceSerial();
  drainAudioRx();
  serviceGoalSensors(now);
  serviceTimeline(now);
  periodicStatus(now);
  periodicSensorStatus(now);
}
