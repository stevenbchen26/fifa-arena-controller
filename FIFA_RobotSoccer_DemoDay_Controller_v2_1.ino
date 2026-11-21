/*
  FIFA Robot Soccer Demo Controller
  LoR Core V3 / ESP32

  Stage:
  - Demo day integrated controller v2.0
  - Web color picker for goal/countdown/start LED animation colors
  - Countdown animation uses four hex quadrants plus the physical center line
  - Countdown starts with all hex/center zones lit, then turns one zone off per step
  - Web-tunable countdown surrounding color, effect mode, pulse/blink speed, and brightness
  - Web-tunable goal wave timing, travel style, countdown timing, match length, and winner finale speed
  - Game start flashes the entire field green before normal in-game mode
  - Match End splits the two physical center strips: one Blue, one Orange
  - Goal scoring animation uses multiple directional waves from scoring side to opponent side
  - Keeps dual-sensor goal validation for each goal
  - Match score and match timer
  - Four E18-D80NK goal sensors
  - DY-HV20T / DY1900A UART audio module
  - 827 LED spatial field effects
  - Professional iPad scoreboard/control dashboard with winner animation
  - Hard LED safety:
      1. Brightness cannot exceed 50%
      2. Full-field white frames are blocked
      3. Demo effects avoid all-white colors

  Web UI:
  - Clean control dashboard with scoreboard mode toggle
  - AP mode default:
      SSID: RobotSoccer_CTRL
      Password: fifabot123
      URL: http://192.168.4.1

  Important LoR Core V3 pin mapping:
  - Left goal sensor A:
      LoR IO_09 signal = ESP32 GPIO2
  - Left goal sensor B:
      LoR IO_10 signal = ESP32 GPIO4
  - Right goal sensor A:
      LoR IO_11 signal = ESP32 GPIO22
  - Right goal sensor B:
      LoR IO_12 signal = ESP32 GPIO21
  - Audio:
      LoR IO_01 signal = ESP32 GPIO32 = ESP32 RX from module TXD
      LoR IO_02 signal = ESP32 GPIO25 = ESP32 TX to module RXD
      LoR IO_03 signal = ESP32 GPIO26 = BUSY
  - Field LED strip:
      LoR IO_05 signal = ESP32 GPIO14

  E18-D80NK sensors:
  - ACTIVE LOW
  - Use pinMode INPUT, not INPUT_PULLUP
  - If signal is 5V, use level shifter or voltage divider before ESP32 GPIO
  - Goal only counts when both sensors on the same goal confirm within the validation window

  Audio SD card files:
  - 00001.mp3 = match start
  - 00002.mp3 = blue goal
  - 00003.mp3 = orange goal
  - 00004.mp3 = ball hit
  - 00005.mp3 = big ball hit
  - 00006.mp3 = countdown tick
  - 00007.mp3 = match end
  - 00008.mp3 = reset
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoOTA.h>
#include <FastLED.h>
#include <math.h>

// ============================================================
// DEMO CONFIG
// ============================================================

#define DEMO_USE_ACCESS_POINT 1

#define AP_SSID       "RobotSoccer_CTRL"
#define AP_PASSWORD   "fifabot123"

#define STA_SSID      "minibot_fms"
#define STA_PASSWORD  "abcdefgh"

IPAddress sta_local_IP(192, 168, 8, 246);
IPAddress sta_gateway(192, 168, 8, 1);
IPAddress sta_subnet(255, 255, 255, 0);

#define OTA_HOSTNAME  "fifa-robot-soccer-demo"

// Default match length. Change this if you want a longer or shorter demo.
#define DEFAULT_MATCH_DURATION_SECONDS  20UL
#define QUICK_MATCH_DURATION_SECONDS    20UL
#define COUNTDOWN_SECONDS                5UL

// ============================================================
// PIN CONFIG
// ============================================================

// LoR IO slot to ESP32 GPIO mapping used here:
// IO_09 = GPIO2, IO_10 = GPIO4, IO_11 = GPIO22, IO_12 = GPIO21
// Left goal uses IO_09 + IO_10. Right goal uses IO_11 + IO_12.
#define LEFT_GOAL_SENSOR_A_PIN   2
#define LEFT_GOAL_SENSOR_B_PIN   4
#define RIGHT_GOAL_SENSOR_A_PIN  22
#define RIGHT_GOAL_SENSOR_B_PIN  21

#define MP3_RX_PIN              32
#define MP3_TX_PIN              25
#define MP3_BUSY_PIN            26
#define MP3_BAUD                9600

#define LED_PIN                 14
#define LED_COUNT               827
#define CHIPSET                 WS2812B
#define COLOR_ORDER             GRB

// ============================================================
// LED SAFETY CONFIG
// ============================================================

#define LED_BRIGHTNESS_MAX_PERCENT 50
#define LED_BRIGHTNESS_MAX_VALUE   127

// Hard block full-field white or near-white frames.
#define LED_BLOCK_FULL_FIELD_WHITE 1

// If at least this many LEDs are near white, the frame is blocked.
// This catches full-field white without blocking tiny calibration highlights.
#define LED_FULL_WHITE_BLOCK_PERCENT 80

// Center calibration.
// X shift now moves the virtual midfield cutline used by effects/tests.
// It does NOT shift raw LED indexes across neighboring map sections.
// Positive X moves the virtual center line right. Negative X moves it left.
#define CENTER_X_SHIFT_DEFAULT -1.5f
#define CENTER_Y_SHIFT_DEFAULT -0.5f
#define CENTER_SHIFT_LIMIT     10.0f
#define VIRTUAL_CENTER_X_BASE  50.0f

// Index shifting is deliberately disabled. The old index-shift method leaked into
// neighboring sections: negative stole blue-perimeter LEDs and positive stole
// yellow/right-arc LEDs. That is exactly what caused side strips to light up.
#define CENTER_LED_INDEX_SHIFT_DEFAULT 0
#define CENTER_LED_INDEX_SHIFT_LIMIT   0

#define CENTER_BASE_A_START 422
#define CENTER_BASE_A_END   507
#define CENTER_BASE_B_START 507
#define CENTER_BASE_B_END   509
#define CENTER_BASE_C_START 509
#define CENTER_BASE_C_END   594

// Center Test mode is deliberately simple for physical verification:
// only the two real center strips light up. No dim background, no purple overlay,
// no virtual X cutline marker. This prevents false visual contamination while
// you identify the true physical midfield LED strips.
#define CENTER_TEST_STRIP_A_COLOR CRGB(0, 170, 220)
#define CENTER_TEST_STRIP_C_COLOR CRGB(0, 170, 220)
#define CENTER_TEST_CONNECTOR_ENABLED 0

// Center hex perimeter LED range from the field map. Used for countdown.
#define HEX_CENTER_START 650
#define HEX_CENTER_END   771

// The green field flash shown immediately after countdown reaches zero.
#define GAME_START_GREEN_MS 1200UL

// Countdown customization defaults.
// The countdown still has 5 steps by default: 4 hex quadrants, then center line last.
#define COUNTDOWN_STEP_MS_DEFAULT          1000UL
#define COUNTDOWN_STEP_MS_MIN             400UL
#define COUNTDOWN_STEP_MS_MAX             2000UL
#define COUNTDOWN_FIELD_EFFECT_PERIOD_DEFAULT 1000UL
#define COUNTDOWN_FIELD_EFFECT_PERIOD_MIN     80UL
#define COUNTDOWN_FIELD_EFFECT_PERIOD_MAX     2500UL
#define COUNTDOWN_FIELD_MIN_BRIGHTNESS_DEFAULT 45
#define COUNTDOWN_FIELD_MAX_BRIGHTNESS_DEFAULT 125
#define COUNTDOWN_HEX_BRIGHTNESS_DEFAULT       230

// Center line portion used during the hex countdown.
// This lights the two physical center strips only where they pass through the hex.
#define COUNTDOWN_CENTER_LINE_Y_MIN 18.0f
#define COUNTDOWN_CENTER_LINE_Y_MAX 43.0f
#define COUNTDOWN_ZONE_COUNT 5

// User-adjustable demo colors. These are intentionally not white.
#define DEFAULT_BLUE_GOAL_COLOR     CRGB(0, 80, 255)
#define DEFAULT_ORANGE_GOAL_COLOR   CRGB(255, 80, 0)
#define DEFAULT_COUNTDOWN_HEX_COLOR   CRGB(0, 255, 0)
#define DEFAULT_COUNTDOWN_FIELD_COLOR CRGB(255, 0, 0)
#define DEFAULT_GAME_START_COLOR      CRGB(0, 255, 0)

// Blocks white/near-white user-selected colors. Manager rule: no all-white LED use.
#define USER_COLOR_WHITE_COMPONENT_LIMIT 185

// ============================================================
// SENSOR CONFIG
// ============================================================

#define SENSOR_ACTIVE_LOW       1
#define SENSOR_DEBOUNCE_MS     60UL

// Dual-sensor validation:
// A goal only counts when BOTH sensors on the same goal detect within this window.
// Raise this if the two sensors are physically far apart along the ball path.
// Lower this if noise is still causing false validations.
#define DUAL_SENSOR_CONFIRM_WINDOW_MS 450UL

// Both sensors must be clear for this long before that goal can score again.
// This prevents a stuck/noisy sensor from becoming half of a false goal forever.
#define DUAL_SENSOR_REARM_CLEAR_MS    180UL

#define GOAL_LOCKOUT_MS        1600UL
#define RESET_COOLDOWN_MS      900UL

// ============================================================
// AUDIO CONFIG
// ============================================================

#define BUSY_ACTIVE_HIGH 1

static const uint8_t DY_START_BYTE     = 0xAA;
static const uint8_t DY_CMD_PLAY_TRACK = 0x07;
static const uint8_t DY_CMD_STOP       = 0x04;
static const uint8_t DY_CMD_SET_VOLUME = 0x13;

const uint16_t MAX_TRACK = 8;

// Estimated lengths are only for the web progress bar.
// BUSY pin still shows the module's real playback state.
const unsigned long TRACK_LENGTH_MS[MAX_TRACK + 1] = {
  0,
  4500,   // 1 match start
  4000,   // 2 blue goal
  4000,   // 3 orange goal
  900,    // 4 ball hit
  1300,   // 5 big hit
  600,    // 6 countdown tick
  5000,   // 7 match end
  1800    // 8 reset
};

HardwareSerial MP3Serial(2);

// ============================================================
// WEB SERVER
// ============================================================

WebServer server(80);

// ============================================================
// LED STATE
// ============================================================

CRGB leds[LED_COUNT];

uint8_t currentLedBrightnessPercent = 25;
uint8_t currentLedBrightnessValue = 64;

#define FIELD_WIDTH   100.0f
#define FIELD_HEIGHT  60.0f

float pixelX[LED_COUNT];
float pixelY[LED_COUNT];

bool spatialMapReady = false;

float centerXShift = CENTER_X_SHIFT_DEFAULT;
float centerYShift = CENTER_Y_SHIFT_DEFAULT;
int16_t centerLedIndexShift = CENTER_LED_INDEX_SHIFT_DEFAULT;

uint16_t rainbowHueOffset = 0;
uint8_t rainbowSpeed = 7;
unsigned long lastLedUpdateMs = 0;

const unsigned long LED_UPDATE_MS = 30;

enum LedMode {
  LED_MODE_ATTRACT = 0,
  LED_MODE_IN_GAME,
  LED_MODE_COUNTDOWN,
  LED_MODE_GAME_START_GREEN,
  LED_MODE_BLUE_GOAL,
  LED_MODE_ORANGE_GOAL,
  LED_MODE_HIT,
  LED_MODE_BIG_HIT,
  LED_MODE_MATCH_END,
  LED_MODE_CENTER_TEST,
  LED_MODE_BLACKOUT
};

LedMode ledMode = LED_MODE_ATTRACT;
unsigned long ledModeStartMs = 0;

CRGB blueGoalColor = DEFAULT_BLUE_GOAL_COLOR;
CRGB orangeGoalColor = DEFAULT_ORANGE_GOAL_COLOR;
CRGB countdownHexColor = DEFAULT_COUNTDOWN_HEX_COLOR;
CRGB countdownFieldColor = DEFAULT_COUNTDOWN_FIELD_COLOR;
CRGB gameStartColor = DEFAULT_GAME_START_COLOR;

enum CountdownFieldEffectMode {
  COUNTDOWN_FIELD_SOLID = 0,
  COUNTDOWN_FIELD_PULSE,
  COUNTDOWN_FIELD_BLINK,
  COUNTDOWN_FIELD_BREATHE,
  COUNTDOWN_FIELD_HEARTBEAT,
  COUNTDOWN_FIELD_STROBE,
  COUNTDOWN_FIELD_CHASING_WAVE
};

CountdownFieldEffectMode countdownFieldEffectMode = COUNTDOWN_FIELD_PULSE;
unsigned long countdownStepMs = COUNTDOWN_STEP_MS_DEFAULT;
unsigned long countdownFieldEffectPeriodMs = COUNTDOWN_FIELD_EFFECT_PERIOD_DEFAULT;
uint8_t countdownFieldMinBrightness = COUNTDOWN_FIELD_MIN_BRIGHTNESS_DEFAULT;
uint8_t countdownFieldMaxBrightness = COUNTDOWN_FIELD_MAX_BRIGHTNESS_DEFAULT;
uint8_t countdownHexBrightness = COUNTDOWN_HEX_BRIGHTNESS_DEFAULT;

// Goal animation timing:
// Directional scoring animation:
// Blue goal  = repeated blue waves travel from Blue side to Orange side.
// Orange goal = repeated orange waves travel from Orange side to Blue side.
// This avoids the older center-origin shockwave and gives a longer, more dramatic scoring sweep.
const unsigned long GOAL_EFFECT_MS_DEFAULT = 4200UL;
const unsigned long GOAL_WAVE_TRAVEL_MS_DEFAULT = 950UL;
const unsigned long GOAL_WAVE_EMIT_INTERVAL_MS_DEFAULT = 260UL;
const unsigned long GOAL_WAVE_EMIT_END_MS_DEFAULT = 2600UL;
const unsigned long GOAL_SOURCE_POP_MS_DEFAULT = 700UL;
const unsigned long GOAL_SOURCE_STROBE_MS = 70UL;
const float GOAL_DIRECTIONAL_BAND_WIDTH = 0.055f;
const float GOAL_DIRECTIONAL_TRAIL_LENGTH = 0.20f;
const float GOAL_SOURCE_ZONE_WIDTH = 0.14f;
const float GOAL_DESTINATION_ZONE_WIDTH = 0.12f;

unsigned long goalEffectMs = GOAL_EFFECT_MS_DEFAULT;
unsigned long goalWaveTravelMs = GOAL_WAVE_TRAVEL_MS_DEFAULT;
unsigned long goalWaveEmitIntervalMs = GOAL_WAVE_EMIT_INTERVAL_MS_DEFAULT;
unsigned long goalWaveEmitEndMs = GOAL_WAVE_EMIT_END_MS_DEFAULT;
unsigned long goalSourcePopMs = GOAL_SOURCE_POP_MS_DEFAULT;

enum GoalTravelMode {
  GOAL_TRAVEL_STRAIGHT = 0,
  GOAL_TRAVEL_DIAGONAL_UP,
  GOAL_TRAVEL_DIAGONAL_DOWN,
  GOAL_TRAVEL_CENTER_FAN,
  GOAL_TRAVEL_EDGE_FAN
};

GoalTravelMode goalTravelMode = GOAL_TRAVEL_STRAIGHT;

const unsigned long HIT_EFFECT_MS = 700UL;
const unsigned long BIG_HIT_EFFECT_MS = 1200UL;
const unsigned long MATCH_END_EFFECT_MS_DEFAULT = 13500UL;
const unsigned long MATCH_END_EFFECT_MS_MIN = 3000UL;
const unsigned long MATCH_END_EFFECT_MS_MAX = 20000UL;

// Winner finale tuning. These are web-adjustable.
// winnerSpeed controls how fast the winner-colored rings move.
// rainbowSpeed controls how fast the center hex rainbow gradient spins.
// ringSpacing controls how close together the winner rings are.
unsigned long matchEndEffectMs = MATCH_END_EFFECT_MS_DEFAULT;
uint8_t matchEndWinnerSpeed = 255;
uint8_t matchEndRainbowSpeed = 50;
uint8_t matchEndRingSpacing = 37;

// Match-end center split:
// Strip A = LEDs 422-507, Strip C = LEDs 509-594.
// If the physical left/right colors are backwards, change this to 0.
#define MATCH_END_CENTER_STRIP_A_IS_BLUE 1

// ============================================================
// GAME STATE
// ============================================================

enum GameState {
  STATE_STANDBY = 0,
  STATE_COUNTDOWN,
  STATE_IN_GAME,
  STATE_END_GAME
};

GameState gameState = STATE_STANDBY;

uint16_t blueScore = 0;
uint16_t orangeScore = 0;

unsigned long matchDurationMs = DEFAULT_MATCH_DURATION_SECONDS * 1000UL;
unsigned long countdownStartMs = 0;
unsigned long matchStartMs = 0;
unsigned long matchEndMs = 0;

int lastCountdownAnnounced = -1;

unsigned long lastBlueGoalMs = 0;
unsigned long lastOrangeGoalMs = 0;
unsigned long lastResetMs = 0;

String lastEvent = "Boot";

// ============================================================
// EVENT LOG
// ============================================================

#define EVENT_LOG_SIZE 14

String eventLog[EVENT_LOG_SIZE];
uint8_t eventLogIndex = 0;
uint8_t eventLogCount = 0;

// ============================================================
// AUDIO STATE
// ============================================================

uint16_t currentTrack = 0;
uint16_t lastAudioTrack = 0;

unsigned long currentTrackStartTime = 0;
unsigned long lastAudioCommandMs = 0;

bool trackActive = false;

uint32_t audioCommandCount = 0;

uint8_t currentVolumePercent = 80;
uint8_t currentModuleVolume = 24;

static const uint32_t SAME_TRACK_COOLDOWN_MS = 120;

// ============================================================
// SENSOR STATE
// ============================================================

struct DebouncedSensor {
  uint8_t pin;
  bool stableDetected;
  bool previousStableDetected;
  bool lastRawDetected;
  unsigned long lastRawChangeMs;
};

struct DualGoalValidator {
  DebouncedSensor sensorA;
  DebouncedSensor sensorB;
  bool armed;
  unsigned long bothClearStartMs;
  unsigned long sensorASeenMs;
  unsigned long sensorBSeenMs;
  unsigned long validGoalCount;
  unsigned long rejectedSingleCount;
};

// Left goal = Blue scoring event in this demo file.
// Right goal = Orange scoring event in this demo file.
// If the score sides are backwards on the physical field, swap the handler calls in updateSensorsAndGoals().
DualGoalValidator leftGoal = {
  { LEFT_GOAL_SENSOR_A_PIN, false, false, false, 0 },
  { LEFT_GOAL_SENSOR_B_PIN, false, false, false, 0 },
  false,
  0,
  0,
  0,
  0,
  0
};

DualGoalValidator rightGoal = {
  { RIGHT_GOAL_SENSOR_A_PIN, false, false, false, 0 },
  { RIGHT_GOAL_SENSOR_B_PIN, false, false, false, 0 },
  false,
  0,
  0,
  0,
  0,
  0
};

// ============================================================
// SYSTEM FLAGS
// ============================================================

bool otaInProgress = false;
bool otaReady = false;
bool wifiReady = false;

// ============================================================
// SMALL HELPERS
// ============================================================

static String gameStateName() {
  if (gameState == STATE_STANDBY) return "STANDBY";
  if (gameState == STATE_COUNTDOWN) return "COUNTDOWN";
  if (gameState == STATE_IN_GAME) return "IN_GAME";
  if (gameState == STATE_END_GAME) return "END_GAME";
  return "UNKNOWN";
}

static String ledModeName() {
  if (ledMode == LED_MODE_ATTRACT) return "ATTRACT";
  if (ledMode == LED_MODE_IN_GAME) return "IN_GAME";
  if (ledMode == LED_MODE_COUNTDOWN) return "COUNTDOWN";
  if (ledMode == LED_MODE_GAME_START_GREEN) return "GAME_START_GREEN";
  if (ledMode == LED_MODE_BLUE_GOAL) return "BLUE_GOAL";
  if (ledMode == LED_MODE_ORANGE_GOAL) return "ORANGE_GOAL";
  if (ledMode == LED_MODE_HIT) return "HIT";
  if (ledMode == LED_MODE_BIG_HIT) return "BIG_HIT";
  if (ledMode == LED_MODE_MATCH_END) return "MATCH_END";
  if (ledMode == LED_MODE_CENTER_TEST) return "CENTER_TEST";
  if (ledMode == LED_MODE_BLACKOUT) return "BLACKOUT";
  return "UNKNOWN";
}

static String countdownFieldEffectName() {
  if (countdownFieldEffectMode == COUNTDOWN_FIELD_SOLID) return "SOLID";
  if (countdownFieldEffectMode == COUNTDOWN_FIELD_PULSE) return "PULSE";
  if (countdownFieldEffectMode == COUNTDOWN_FIELD_BLINK) return "BLINK";
  if (countdownFieldEffectMode == COUNTDOWN_FIELD_BREATHE) return "BREATHE";
  if (countdownFieldEffectMode == COUNTDOWN_FIELD_HEARTBEAT) return "HEARTBEAT";
  if (countdownFieldEffectMode == COUNTDOWN_FIELD_STROBE) return "STROBE";
  if (countdownFieldEffectMode == COUNTDOWN_FIELD_CHASING_WAVE) return "CHASING_WAVE";
  return "UNKNOWN";
}

static CountdownFieldEffectMode parseCountdownFieldEffectMode(String mode) {
  mode.trim();
  mode.toLowerCase();

  if (mode == "0" || mode == "solid") return COUNTDOWN_FIELD_SOLID;
  if (mode == "1" || mode == "pulse") return COUNTDOWN_FIELD_PULSE;
  if (mode == "2" || mode == "blink") return COUNTDOWN_FIELD_BLINK;
  if (mode == "3" || mode == "breathe") return COUNTDOWN_FIELD_BREATHE;
  if (mode == "4" || mode == "heartbeat") return COUNTDOWN_FIELD_HEARTBEAT;
  if (mode == "5" || mode == "strobe") return COUNTDOWN_FIELD_STROBE;
  if (mode == "6" || mode == "wave" || mode == "chasing_wave") return COUNTDOWN_FIELD_CHASING_WAVE;

  return COUNTDOWN_FIELD_PULSE;
}

static String goalTravelModeName() {
  if (goalTravelMode == GOAL_TRAVEL_STRAIGHT) return "STRAIGHT";
  if (goalTravelMode == GOAL_TRAVEL_DIAGONAL_UP) return "DIAGONAL_UP";
  if (goalTravelMode == GOAL_TRAVEL_DIAGONAL_DOWN) return "DIAGONAL_DOWN";
  if (goalTravelMode == GOAL_TRAVEL_CENTER_FAN) return "CENTER_FAN";
  if (goalTravelMode == GOAL_TRAVEL_EDGE_FAN) return "EDGE_FAN";
  return "STRAIGHT";
}

static GoalTravelMode parseGoalTravelMode(String mode) {
  mode.trim();
  mode.toLowerCase();

  if (mode == "0" || mode == "straight") return GOAL_TRAVEL_STRAIGHT;
  if (mode == "1" || mode == "diagonal_up" || mode == "diagup") return GOAL_TRAVEL_DIAGONAL_UP;
  if (mode == "2" || mode == "diagonal_down" || mode == "diagdown") return GOAL_TRAVEL_DIAGONAL_DOWN;
  if (mode == "3" || mode == "center_fan" || mode == "centerfan") return GOAL_TRAVEL_CENTER_FAN;
  if (mode == "4" || mode == "edge_fan" || mode == "edgefan") return GOAL_TRAVEL_EDGE_FAN;

  return GOAL_TRAVEL_STRAIGHT;
}

static String goalTravelModeValue() {
  if (goalTravelMode == GOAL_TRAVEL_STRAIGHT) return "straight";
  if (goalTravelMode == GOAL_TRAVEL_DIAGONAL_UP) return "diagonal_up";
  if (goalTravelMode == GOAL_TRAVEL_DIAGONAL_DOWN) return "diagonal_down";
  if (goalTravelMode == GOAL_TRAVEL_CENTER_FAN) return "center_fan";
  if (goalTravelMode == GOAL_TRAVEL_EDGE_FAN) return "edge_fan";
  return "straight";
}

static float goalTravelCoordinate(float pathX, float yNorm) {
  float coord = pathX;

  if (goalTravelMode == GOAL_TRAVEL_DIAGONAL_UP) {
    coord = pathX + ((0.5f - yNorm) * 0.26f);
  } else if (goalTravelMode == GOAL_TRAVEL_DIAGONAL_DOWN) {
    coord = pathX + ((yNorm - 0.5f) * 0.26f);
  } else if (goalTravelMode == GOAL_TRAVEL_CENTER_FAN) {
    coord = pathX + (fabs(yNorm - 0.5f) * 0.20f);
  } else if (goalTravelMode == GOAL_TRAVEL_EDGE_FAN) {
    coord = pathX - (fabs(yNorm - 0.5f) * 0.16f);
  }

  if (coord < 0.0f) coord = 0.0f;
  if (coord > 1.0f) coord = 1.0f;
  return coord;
}

static float clampFloat(float value, float low, float high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

static uint8_t clampU8(int value) {
  if (value < 0) return 0;
  if (value > 255) return 255;
  return (uint8_t)value;
}

static unsigned long clampUL(unsigned long value, unsigned long low, unsigned long high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

static uint8_t clampBrightnessValue(int value) {
  return clampU8(value);
}

static int16_t clampCenterIndexShift(int16_t value) {
  (void)value;
  return 0;
}

static uint16_t shiftedCenterIndex(uint16_t baseIndex) {
  int32_t shifted = (int32_t)baseIndex + (int32_t)centerLedIndexShift;

  if (shifted < 0) shifted = 0;
  if (shifted >= LED_COUNT) shifted = LED_COUNT - 1;

  return (uint16_t)shifted;
}

static uint16_t centerAStart() { return shiftedCenterIndex(CENTER_BASE_A_START); }
static uint16_t centerAEnd()   { return shiftedCenterIndex(CENTER_BASE_A_END); }
static uint16_t centerBStart() { return shiftedCenterIndex(CENTER_BASE_B_START); }
static uint16_t centerBEnd()   { return shiftedCenterIndex(CENTER_BASE_B_END); }
static uint16_t centerCStart() { return shiftedCenterIndex(CENTER_BASE_C_START); }
static uint16_t centerCEnd()   { return shiftedCenterIndex(CENTER_BASE_C_END); }
static uint16_t centerStripStart() { return shiftedCenterIndex(CENTER_BASE_A_START); }
static uint16_t centerStripEnd()   { return shiftedCenterIndex(CENTER_BASE_C_END); }

static float virtualCenterX() {
  return VIRTUAL_CENTER_X_BASE + centerXShift;
}

static CRGB scaledColor(CRGB color, uint8_t brightness) {
  CRGB c = color;
  c.nscale8_video(brightness);
  return c;
}

static bool colorIsUnsafeWhite(CRGB c) {
  return c.r >= USER_COLOR_WHITE_COMPONENT_LIMIT &&
         c.g >= USER_COLOR_WHITE_COMPONENT_LIMIT &&
         c.b >= USER_COLOR_WHITE_COMPONENT_LIMIT;
}

static CRGB sanitizeUserColor(CRGB requested, CRGB fallback) {
  // Prevent accidental white/near-white animation colors from the web color picker.
  // This is separate from the full-field white frame block. This stops bad input
  // before it becomes a tempting demo-day disaster.
  if (colorIsUnsafeWhite(requested)) {
    return fallback;
  }

  return requested;
}

static String colorToHex(CRGB c) {
  char buf[8];
  snprintf(buf, sizeof(buf), "#%02X%02X%02X", c.r, c.g, c.b);
  return String(buf);
}

static uint8_t hexNibble(char ch) {
  if (ch >= '0' && ch <= '9') return (uint8_t)(ch - '0');
  if (ch >= 'a' && ch <= 'f') return (uint8_t)(ch - 'a' + 10);
  if (ch >= 'A' && ch <= 'F') return (uint8_t)(ch - 'A' + 10);
  return 0;
}

static uint8_t hexPairToByte(char high, char low) {
  return (uint8_t)((hexNibble(high) << 4) | hexNibble(low));
}

static CRGB parseWebHexColor(String hex, CRGB fallback) {
  hex.trim();

  if (hex.length() == 0) {
    return fallback;
  }

  if (hex.charAt(0) == '#') {
    hex.remove(0, 1);
  }

  if (hex.length() != 6) {
    return fallback;
  }

  for (uint8_t i = 0; i < 6; i++) {
    char ch = hex.charAt(i);
    bool valid = (ch >= '0' && ch <= '9') ||
                 (ch >= 'a' && ch <= 'f') ||
                 (ch >= 'A' && ch <= 'F');

    if (!valid) {
      return fallback;
    }
  }

  CRGB parsed(
    hexPairToByte(hex.charAt(0), hex.charAt(1)),
    hexPairToByte(hex.charAt(2), hex.charAt(3)),
    hexPairToByte(hex.charAt(4), hex.charAt(5))
  );

  return sanitizeUserColor(parsed, fallback);
}

static void setLedMode(LedMode mode) {
  ledMode = mode;
  ledModeStartMs = millis();

  Serial.print("[LED] Mode -> ");
  Serial.println(ledModeName());
}

static void addEvent(String text) {
  unsigned long seconds = millis() / 1000UL;

  String line = "[";
  line += String(seconds);
  line += "s] ";
  line += text;

  eventLog[eventLogIndex] = line;
  eventLogIndex = (eventLogIndex + 1) % EVENT_LOG_SIZE;

  if (eventLogCount < EVENT_LOG_SIZE) {
    eventLogCount++;
  }

  lastEvent = text;

  Serial.print("[EVENT] ");
  Serial.println(line);
}

static void printHexByte(uint8_t b) {
  if (b < 16) Serial.print("0");
  Serial.print(b, HEX);
}

// ============================================================
// SPATIAL FIELD MAP
// ============================================================

static float lerpFloat(float a, float b, float t) {
  return a + ((b - a) * t);
}

static void mapLinearSegment(
  uint16_t startIndex,
  uint16_t endIndex,
  float x0,
  float y0,
  float x1,
  float y1
) {
  if (startIndex >= LED_COUNT) return;
  if (endIndex >= LED_COUNT) endIndex = LED_COUNT - 1;
  if (endIndex < startIndex) return;

  uint16_t count = endIndex - startIndex + 1;

  for (uint16_t offset = 0; offset < count; offset++) {
    float t = 0.0f;

    if (count > 1) {
      t = (float)offset / (float)(count - 1);
    }

    uint16_t ledIndex = startIndex + offset;

    pixelX[ledIndex] = lerpFloat(x0, x1, t);
    pixelY[ledIndex] = lerpFloat(y0, y1, t);
  }
}

static void mapBezierSegment(
  uint16_t startIndex,
  uint16_t endIndex,
  float x0,
  float y0,
  float cx,
  float cy,
  float x1,
  float y1
) {
  if (startIndex >= LED_COUNT) return;
  if (endIndex >= LED_COUNT) endIndex = LED_COUNT - 1;
  if (endIndex < startIndex) return;

  uint16_t count = endIndex - startIndex + 1;

  for (uint16_t offset = 0; offset < count; offset++) {
    float t = 0.0f;

    if (count > 1) {
      t = (float)offset / (float)(count - 1);
    }

    float u = 1.0f - t;

    float x = (u * u * x0) + (2.0f * u * t * cx) + (t * t * x1);
    float y = (u * u * y0) + (2.0f * u * t * cy) + (t * t * y1);

    uint16_t ledIndex = startIndex + offset;

    pixelX[ledIndex] = x;
    pixelY[ledIndex] = y;
  }
}

static void buildXYMap() {
  /*
    Coordinate system:
    X = 0 to 100, left to right
    Y = 0 to 60, bottom to top

    LED ranges:
    Blue perimeter      0 - 421
    Green center      422 - 594
    Yellow right arc  595 - 649
    Purple hex center 650 - 771
    Red left arc      772 - 826

    Center X shift adjusts the virtual midfield cutline used by effects/tests.
    Center Y shift is still applied to the green center strip alignment.
    Raw LED index shifting is disabled because it crosses into neighboring physical sections.
  */

  // Blue perimeter: 0 - 421
  mapLinearSegment(0,   53,  48.0f,  1.5f, 84.0f,  1.5f);
  mapLinearSegment(53,  79,  84.0f,  1.5f, 95.25f, 13.5f);
  mapLinearSegment(79,  131, 95.25f, 13.5f, 95.25f, 47.0f);
  mapLinearSegment(131, 157, 95.25f, 47.0f, 84.0f,  59.5f);
  mapLinearSegment(157, 264, 84.0f,  59.5f, 13.5f,  59.5f);
  mapLinearSegment(264, 290, 13.5f,  59.5f, 1.5f,   47.0f);
  mapLinearSegment(290, 342, 1.5f,   47.0f, 1.5f,   13.5f);
  mapLinearSegment(342, 368, 1.5f,   13.5f, 13.5f,  1.5f);
  mapLinearSegment(368, 421, 13.5f,  1.5f, 48.0f,  1.5f);

  // Green center base map: 422 - 594.
  // This prevents unmapped gaps if the physical center index is shifted.
  mapLinearSegment(
    CENTER_BASE_A_START,
    CENTER_BASE_A_END,
    48.0f,
    2.0f + centerYShift,
    48.0f,
    59.25f + centerYShift
  );

  mapLinearSegment(
    CENTER_BASE_B_START,
    CENTER_BASE_B_END,
    48.0f,
    59.25f + centerYShift,
    47.5f,
    59.25f + centerYShift
  );

  mapLinearSegment(
    CENTER_BASE_C_START,
    CENTER_BASE_C_END,
    47.5f,
    59.25f + centerYShift,
    47.0f,
    2.0f + centerYShift
  );

  // Yellow right arc: 595 - 649
  mapLinearSegment(595, 602, 95.25f, 17.5f, 89.0f, 17.5f);
  mapLinearSegment(602, 608, 89.0f,  17.5f, 86.0f, 21.0f);

  mapBezierSegment(
    608,
    636,
    86.0f, 21.0f,
    91.0f, 30.0f,
    86.0f, 39.0f
  );

  mapLinearSegment(636, 642, 86.0f,  39.0f, 89.0f, 42.5f);
  mapLinearSegment(642, 649, 89.0f,  42.5f, 95.25f, 42.5f);

  // Purple hex center: 650 - 771
  mapLinearSegment(650, 659, 48.0f,  19.0f, 55.5f,  19.0f);
  mapLinearSegment(659, 680, 55.5f,  19.0f, 61.75f, 30.5f);
  mapLinearSegment(680, 701, 61.75f, 30.5f, 55.5f,  42.0f);
  mapLinearSegment(701, 720, 55.5f,  42.0f, 41.75f, 42.0f);
  mapLinearSegment(720, 741, 41.75f, 42.0f, 35.25f, 30.5f);
  mapLinearSegment(741, 762, 35.25f, 30.5f, 39.5f,  19.0f);
  mapLinearSegment(762, 771, 39.5f,  19.0f, 47.0f,  19.0f);

  // Red left arc: 772 - 826
  mapLinearSegment(772, 779, 1.5f, 17.5f, 7.0f, 21.0f);
  mapLinearSegment(779, 785, 7.0f, 21.0f, 10.0f, 23.5f);

  mapBezierSegment(
    785,
    813,
    10.0f, 23.5f,
    5.0f, 30.0f,
    10.0f, 39.0f
  );

  mapLinearSegment(813, 819, 10.0f, 39.0f, 7.0f, 42.5f);
  mapLinearSegment(819, 826, 7.0f, 42.5f, 1.5f, 42.5f);

  // Re-map the active physical center strip after all other geometry.
  // This is the real calibration knob when the center line is off by LED count.
  mapLinearSegment(
    centerAStart(),
    centerAEnd(),
    48.0f,
    2.0f + centerYShift,
    48.0f,
    59.25f + centerYShift
  );

  mapLinearSegment(
    centerBStart(),
    centerBEnd(),
    48.0f,
    59.25f + centerYShift,
    47.5f,
    59.25f + centerYShift
  );

  mapLinearSegment(
    centerCStart(),
    centerCEnd(),
    47.5f,
    59.25f + centerYShift,
    47.0f,
    2.0f + centerYShift
  );

  spatialMapReady = true;

  Serial.print("[SPATIAL] XY map built. Virtual center X = ");
  Serial.print(virtualCenterX(), 2);
  Serial.print(", Center Y shift = ");
  Serial.print(centerYShift, 2);
  Serial.print(", Center LED index shift = ");
  Serial.println(centerLedIndexShift);
}

static void setCenterShift(float xShift, float yShift, int16_t ledIndexShift) {
  centerXShift = clampFloat(xShift, -CENTER_SHIFT_LIMIT, CENTER_SHIFT_LIMIT);
  centerYShift = clampFloat(yShift, -CENTER_SHIFT_LIMIT, CENTER_SHIFT_LIMIT);
  centerLedIndexShift = clampCenterIndexShift(ledIndexShift);

  buildXYMap();

  Serial.print("[CENTER] Shift set. Virtual X = ");
  Serial.print(virtualCenterX(), 2);
  Serial.print(", Y = ");
  Serial.print(centerYShift, 2);
  Serial.print(", LED index = ");
  Serial.println(centerLedIndexShift);
}

// ============================================================
// LED SAFETY
// ============================================================

static uint8_t ledPercentToFastLedValue(uint8_t percent) {
  if (percent > LED_BRIGHTNESS_MAX_PERCENT) {
    percent = LED_BRIGHTNESS_MAX_PERCENT;
  }

  return map(percent, 0, LED_BRIGHTNESS_MAX_PERCENT, 0, LED_BRIGHTNESS_MAX_VALUE);
}

static void enforceLedBrightnessLimit() {
  if (currentLedBrightnessPercent > LED_BRIGHTNESS_MAX_PERCENT) {
    currentLedBrightnessPercent = LED_BRIGHTNESS_MAX_PERCENT;
  }

  currentLedBrightnessValue = ledPercentToFastLedValue(currentLedBrightnessPercent);
  FastLED.setBrightness(currentLedBrightnessValue);
}

static bool isNearWhite(const CRGB &c) {
  int r = c.r;
  int g = c.g;
  int b = c.b;

  if (r < 210 || g < 210 || b < 210) {
    return false;
  }

  if (abs(r - g) > 35) return false;
  if (abs(g - b) > 35) return false;
  if (abs(r - b) > 35) return false;

  return true;
}

static bool frameLooksLikeFullWhite() {
#if LED_BLOCK_FULL_FIELD_WHITE
  uint16_t nearWhiteCount = 0;
  const uint16_t blockThreshold = (uint16_t)(((uint32_t)LED_COUNT * (uint32_t)LED_FULL_WHITE_BLOCK_PERCENT) / 100UL);

  for (uint16_t i = 0; i < LED_COUNT; i++) {
    if (isNearWhite(leds[i])) {
      nearWhiteCount++;
      if (nearWhiteCount >= blockThreshold) {
        return true;
      }
    }
  }
#endif

  return false;
}

static void replaceUnsafeWhiteFrame() {
  CRGB safe = CRGB(0, 120, 180);

  for (uint16_t i = 0; i < LED_COUNT; i++) {
    leds[i] = scaledColor(safe, 120);
  }
}

static void safeFastLedShow() {
  enforceLedBrightnessLimit();

  if (frameLooksLikeFullWhite()) {
    replaceUnsafeWhiteFrame();
    Serial.println("[LED_SAFETY] Blocked full-field white frame. Replaced with safe cyan.");
  }

  FastLED.show();
}

static void setLedBrightnessPercent(uint8_t percent) {
  if (percent > LED_BRIGHTNESS_MAX_PERCENT) {
    percent = LED_BRIGHTNESS_MAX_PERCENT;
  }

  currentLedBrightnessPercent = percent;
  currentLedBrightnessValue = ledPercentToFastLedValue(percent);

  FastLED.setBrightness(currentLedBrightnessValue);
  safeFastLedShow();

  Serial.print("[LED] Brightness set to ");
  Serial.print(currentLedBrightnessPercent);
  Serial.print("%, FastLED value ");
  Serial.println(currentLedBrightnessValue);
}

static void clearFieldLeds() {
  FastLED.clear();
  safeFastLedShow();
}

static void showSolidFieldColor(CRGB color) {
  fill_solid(leds, LED_COUNT, color);
  safeFastLedShow();
}

// ============================================================
// LED RENDERERS
// ============================================================

static void renderAttractOrInGame(bool inGame) {
  unsigned long now = millis();

  if (now - lastLedUpdateMs < LED_UPDATE_MS) return;
  lastLedUpdateMs = now;

  rainbowHueOffset += rainbowSpeed;

  for (uint16_t i = 0; i < LED_COUNT; i++) {
    float xNorm = pixelX[i] / FIELD_WIDTH;
    float yNorm = pixelY[i] / FIELD_HEIGHT;

    xNorm = clampFloat(xNorm, 0.0f, 1.0f);
    yNorm = clampFloat(yNorm, 0.0f, 1.0f);

    uint8_t hue = (uint8_t)(
      rainbowHueOffset +
      (uint16_t)(xNorm * 150.0f) +
      (uint16_t)(yNorm * 70.0f)
    );

    uint8_t wave = sin8(
      (uint8_t)(
        rainbowHueOffset +
        (uint16_t)(xNorm * 120.0f) +
        (uint16_t)(yNorm * 90.0f)
      )
    );

    uint8_t value;

    if (inGame) {
      value = map(wave, 0, 255, 40, 150);
    } else {
      value = map(wave, 0, 255, 80, 220);
    }

    leds[i] = CHSV(hue, 230, value);
  }

  safeFastLedShow();
}

static bool ledIsHexCenter(uint16_t i) {
  return i >= HEX_CENTER_START && i <= HEX_CENTER_END;
}


static bool ledIsCountdownCenterLine(uint16_t i) {
  if (!((i >= centerAStart() && i <= centerAEnd()) || (i >= centerCStart() && i <= centerCEnd()))) {
    return false;
  }

  float y = pixelY[i];
  return y >= COUNTDOWN_CENTER_LINE_Y_MIN && y <= COUNTDOWN_CENTER_LINE_Y_MAX;
}

// Countdown zones:
// 0 = hex top-left perimeter
// 1 = hex top-right perimeter
// 2 = hex bottom-right perimeter
// 3 = hex bottom-left perimeter
// 4 = physical center line through the hex, last to turn off
static int countdownZoneForLed(uint16_t i) {
  if (ledIsCountdownCenterLine(i)) {
    return 4;
  }

  if (!ledIsHexCenter(i)) {
    return -1;
  }

  float x = pixelX[i];
  float y = pixelY[i];

  bool leftHalf = x < virtualCenterX();
  bool topHalf = y >= (FIELD_HEIGHT / 2.0f);

  if (leftHalf && topHalf) return 0;
  if (!leftHalf && topHalf) return 1;
  if (!leftHalf && !topHalf) return 2;
  return 3;
}

static bool countdownZoneActive(int zone, unsigned long elapsed) {
  if (zone < 0) return false;

  unsigned long stepIndex = elapsed / countdownStepMs;

  if (stepIndex >= COUNTDOWN_ZONE_COUNT) {
    return false;
  }

  // Start with everything lit. Each step turns one zone off.
  // zone 4 is the center line and is therefore the last zone lit.
  return (unsigned long)zone >= stepIndex;
}

static uint8_t countdownFieldBrightnessForLed(uint16_t i, unsigned long elapsed) {
  uint8_t low = countdownFieldMinBrightness;
  uint8_t high = countdownFieldMaxBrightness;

  if (high < low) {
    uint8_t tmp = high;
    high = low;
    low = tmp;
  }

  if (countdownFieldEffectMode == COUNTDOWN_FIELD_SOLID) {
    return high;
  }

  unsigned long period = countdownFieldEffectPeriodMs;
  if (period < 1UL) period = 1UL;

  unsigned long phaseMs = elapsed % period;
  float phase = (float)phaseMs / (float)period;
  float intensity = 1.0f;

  if (countdownFieldEffectMode == COUNTDOWN_FIELD_PULSE) {
    uint8_t wave = sin8((uint8_t)(phase * 255.0f));
    intensity = (float)wave / 255.0f;
  } else if (countdownFieldEffectMode == COUNTDOWN_FIELD_BLINK) {
    intensity = phase < 0.50f ? 1.0f : 0.0f;
  } else if (countdownFieldEffectMode == COUNTDOWN_FIELD_BREATHE) {
    uint8_t wave = sin8((uint8_t)(phase * 255.0f));
    float normalized = (float)wave / 255.0f;
    intensity = normalized * normalized;
  } else if (countdownFieldEffectMode == COUNTDOWN_FIELD_HEARTBEAT) {
    // Two short punches at the start of each countdown step.
    if (phase < 0.13f) {
      intensity = 1.0f - (phase / 0.13f) * 0.35f;
    } else if (phase < 0.26f) {
      intensity = 0.25f;
    } else if (phase < 0.39f) {
      intensity = 0.95f - ((phase - 0.26f) / 0.13f) * 0.45f;
    } else {
      intensity = 0.18f;
    }
  } else if (countdownFieldEffectMode == COUNTDOWN_FIELD_STROBE) {
    // Faster than blink, still controlled by the same period slider.
    uint8_t flashes = 6;
    uint8_t slot = (uint8_t)(phase * flashes);
    intensity = (slot % 2 == 0) ? 1.0f : 0.0f;
  } else if (countdownFieldEffectMode == COUNTDOWN_FIELD_CHASING_WAVE) {
    float xNorm = clampFloat(pixelX[i] / FIELD_WIDTH, 0.0f, 1.0f);
    float yNorm = clampFloat(pixelY[i] / FIELD_HEIGHT, 0.0f, 1.0f);
    float moving = phase;
    float d = fabs(xNorm - moving);
    if (d > 0.5f) d = 1.0f - d;
    float band = 1.0f - clampFloat(d / 0.18f, 0.0f, 1.0f);
    float shimmer = ((float)sin8((uint8_t)((yNorm * 120.0f) + (phase * 255.0f))) / 255.0f) * 0.22f;
    intensity = clampFloat(0.20f + band * 0.80f + shimmer, 0.0f, 1.0f);
  }

  int brightness = (int)((float)low + ((float)(high - low) * intensity));
  return clampU8(brightness);
}

static void renderCountdown() {
  unsigned long now = millis();

  if (now - lastLedUpdateMs < LED_UPDATE_MS) return;
  lastLedUpdateMs = now;

  unsigned long elapsed = now - ledModeStartMs;

  for (uint16_t i = 0; i < LED_COUNT; i++) {
    int zone = countdownZoneForLed(i);

    if (zone >= 0) {
      if (countdownZoneActive(zone, elapsed)) {
        leds[i] = scaledColor(countdownHexColor, countdownHexBrightness);
      } else {
        leds[i] = CRGB::Black;
      }
    } else {
      uint8_t fieldBrightness = countdownFieldBrightnessForLed(i, elapsed);
      leds[i] = scaledColor(countdownFieldColor, fieldBrightness);
    }
  }

  safeFastLedShow();
}

static void renderGameStartGreen() {
  unsigned long now = millis();

  if (now - lastLedUpdateMs < LED_UPDATE_MS) return;
  lastLedUpdateMs = now;

  unsigned long elapsed = now - ledModeStartMs;

  if (elapsed >= GAME_START_GREEN_MS) {
    if (gameState == STATE_IN_GAME) {
      setLedMode(LED_MODE_IN_GAME);
    } else {
      setLedMode(LED_MODE_ATTRACT);
    }
    return;
  }

  uint8_t pulse = sin8((uint8_t)(elapsed / 3));
  uint8_t brightness = map(pulse, 0, 255, 130, 230);

  for (uint16_t i = 0; i < LED_COUNT; i++) {
    leds[i] = scaledColor(gameStartColor, brightness);
  }

  safeFastLedShow();
}

static float waveIntensity(float distance, float radius, float bandWidth) {
  float band = fabs(distance - radius);

  if (band >= bandWidth) {
    return 0.0f;
  }

  return 1.0f - (band / bandWidth);
}

static void paintCenterStripsSingleColor(CRGB color, uint8_t brightness) {
  CRGB c = scaledColor(color, brightness);

  for (uint16_t i = CENTER_BASE_A_START; i <= CENTER_BASE_A_END && i < LED_COUNT; i++) {
    leds[i] = c;
  }

  for (uint16_t i = CENTER_BASE_C_START; i <= CENTER_BASE_C_END && i < LED_COUNT; i++) {
    leds[i] = c;
  }
}

static void paintCenterConnectorGap() {
  // Keep the tiny connector between the two physical center strips from becoming
  // an accidental third color during split effects. Only LED 508 should remain
  // as a small dark divider based on the current map constants.
  if (CENTER_BASE_B_END > CENTER_BASE_B_START + 1) {
    for (uint16_t i = CENTER_BASE_B_START + 1; i < CENTER_BASE_B_END && i < LED_COUNT; i++) {
      leds[i] = CRGB::Black;
    }
  }
}

static void paintCenterStripsSplit(CRGB blueColor, CRGB orangeColor, uint8_t brightness) {
#if MATCH_END_CENTER_STRIP_A_IS_BLUE
  CRGB stripA = scaledColor(blueColor, brightness);
  CRGB stripC = scaledColor(orangeColor, brightness);
#else
  CRGB stripA = scaledColor(orangeColor, brightness);
  CRGB stripC = scaledColor(blueColor, brightness);
#endif

  for (uint16_t i = CENTER_BASE_A_START; i <= CENTER_BASE_A_END && i < LED_COUNT; i++) {
    leds[i] = stripA;
  }

  for (uint16_t i = CENTER_BASE_C_START; i <= CENTER_BASE_C_END && i < LED_COUNT; i++) {
    leds[i] = stripC;
  }

  paintCenterConnectorGap();
}

static void renderGoalEffect(CRGB teamColor, bool blueTeam) {
  unsigned long now = millis();

  if (now - lastLedUpdateMs < LED_UPDATE_MS) return;
  lastLedUpdateMs = now;

  unsigned long elapsed = now - ledModeStartMs;

  if (elapsed >= goalEffectMs) {
    if (gameState == STATE_IN_GAME) {
      setLedMode(LED_MODE_IN_GAME);
    } else {
      setLedMode(LED_MODE_ATTRACT);
    }
    return;
  }

  // Directional scoring wave:
  // Blue scoring sends blue waves from the Blue side toward the Orange side.
  // Orange scoring sends orange waves from the Orange side toward the Blue side.
  // We use pathX as a 0.0 to 1.0 coordinate in the travel direction:
  //   pathX = 0.0  source/scoring team's side
  //   pathX = 1.0  opponent/goal side
  for (uint16_t i = 0; i < LED_COUNT; i++) {
    float xNorm = pixelX[i] / FIELD_WIDTH;
    xNorm = clampFloat(xNorm, 0.0f, 1.0f);

    float yNorm = pixelY[i] / FIELD_HEIGHT;
    yNorm = clampFloat(yNorm, 0.0f, 1.0f);

    float pathX;
    if (blueTeam) {
      // Blue side is treated as the left side of the field.
      pathX = xNorm;
    } else {
      // Orange side is treated as the right side of the field.
      pathX = 1.0f - xNorm;
    }

    // travelCoord controls the moving wave head. pathX still controls source
    // and destination impact zones so the animation remains team-directional.
    float travelCoord = goalTravelCoordinate(pathX, yNorm);

    float intensity = 0.0f;

    // Low background glow so the wave reads clearly without turning the whole field into a floodlight.
    float baseGlow = 0.035f;

    // Source-side pop: the scoring team's side flashes hard at the start.
    if (elapsed < goalSourcePopMs && pathX <= GOAL_SOURCE_ZONE_WIDTH) {
      bool sourceFlashOn = ((elapsed / GOAL_SOURCE_STROBE_MS) % 2UL) == 0UL;
      float flashFade = 1.0f - ((float)elapsed / (float)goalSourcePopMs);
      float sourceFalloff = 1.0f - (pathX / GOAL_SOURCE_ZONE_WIDTH);
      if (sourceFalloff < 0.0f) sourceFalloff = 0.0f;

      float sourcePop = (sourceFlashOn ? 1.0f : 0.38f) * flashFade * sourceFalloff;
      if (sourcePop > intensity) intensity = sourcePop;
    }

    // Repeated traveling waves. Each emitted wave moves from source side to opponent side.
    for (unsigned long pulseStart = 0; pulseStart <= goalWaveEmitEndMs; pulseStart += goalWaveEmitIntervalMs) {
      if (elapsed < pulseStart) {
        continue;
      }

      unsigned long pulseAge = elapsed - pulseStart;

      // Allow a small tail after the head reaches the far side.
      if (pulseAge > goalWaveTravelMs + 260UL) {
        continue;
      }

      float wavePos = (float)pulseAge / (float)goalWaveTravelMs;
      if (wavePos > 1.16f) {
        continue;
      }

      // Main bright wave head.
      float headDistance = fabs(travelCoord - wavePos);
      if (headDistance < GOAL_DIRECTIONAL_BAND_WIDTH) {
        float head = 1.0f - (headDistance / GOAL_DIRECTIONAL_BAND_WIDTH);
        head = head * head;  // sharper leading edge, more pop
        if (head > intensity) intensity = head;
      }

      // Trailing glow behind each wave head.
      float behindDistance = wavePos - travelCoord;
      if (behindDistance > 0.0f && behindDistance < GOAL_DIRECTIONAL_TRAIL_LENGTH) {
        float trail = 1.0f - (behindDistance / GOAL_DIRECTIONAL_TRAIL_LENGTH);
        trail *= 0.36f;
        if (trail > intensity) intensity = trail;
      }

      // Opponent-side impact flash as each wave reaches the far side.
      if (pulseAge > goalWaveTravelMs - 100UL && pulseAge < goalWaveTravelMs + 150UL && pathX >= (1.0f - GOAL_DESTINATION_ZONE_WIDTH)) {
        float impactTime = 1.0f - (fabs((float)pulseAge - (float)goalWaveTravelMs) / 150.0f);
        if (impactTime < 0.0f) impactTime = 0.0f;

        float destinationFalloff = (pathX - (1.0f - GOAL_DESTINATION_ZONE_WIDTH)) / GOAL_DESTINATION_ZONE_WIDTH;
        destinationFalloff = clampFloat(destinationFalloff, 0.0f, 1.0f);

        float impact = impactTime * destinationFalloff;
        if (impact > intensity) intensity = impact;
      }
    }

    // End-of-effect fade down after the final waves have crossed.
    unsigned long fadeStart = goalWaveEmitEndMs + goalWaveTravelMs;
    if (elapsed > fadeStart) {
      float fade = 1.0f - ((float)(elapsed - fadeStart) / (float)(goalEffectMs - fadeStart));
      fade = clampFloat(fade, 0.0f, 1.0f);
      intensity *= fade;
      baseGlow *= fade;
    }

    uint8_t brightness = clampU8((int)((baseGlow + intensity) * 255.0f));
    leds[i] = scaledColor(teamColor, brightness);
  }

  // Make the two physical center strips participate in the passing wave naturally,
  // instead of flashing from the middle. This keeps the scoring animation directional.
  paintCenterConnectorGap();

  safeFastLedShow();
}

static void renderHitEffect(bool bigHit) {
  unsigned long now = millis();

  if (now - lastLedUpdateMs < LED_UPDATE_MS) return;
  lastLedUpdateMs = now;

  unsigned long duration = bigHit ? BIG_HIT_EFFECT_MS : HIT_EFFECT_MS;
  unsigned long elapsed = now - ledModeStartMs;

  if (elapsed >= duration) {
    if (gameState == STATE_IN_GAME) {
      setLedMode(LED_MODE_IN_GAME);
    } else {
      setLedMode(LED_MODE_ATTRACT);
    }
    return;
  }

  float progress = (float)elapsed / (float)duration;
  float radius = progress * 70.0f;
  float bandWidth = bigHit ? 18.0f : 11.0f;

  CRGB hitColor = bigHit ? CRGB(180, 0, 255) : CRGB(0, 180, 120);

  for (uint16_t i = 0; i < LED_COUNT; i++) {
    float dx = pixelX[i] - 50.0f;
    float dy = pixelY[i] - 30.0f;
    float d = sqrtf((dx * dx) + (dy * dy));

    float band = fabs(d - radius);

    if (band < bandWidth) {
      float intensity = 1.0f - (band / bandWidth);
      leds[i] = scaledColor(hitColor, clampU8((int)(255.0f * intensity)));
    } else {
      leds[i] = scaledColor(hitColor, bigHit ? 20 : 10);
    }
  }

  safeFastLedShow();
}

static bool ledIsHex(uint16_t i) {
  return i >= HEX_CENTER_START && i <= HEX_CENTER_END;
}

static void renderMatchEndEffect() {
  unsigned long now = millis();

  if (now - lastLedUpdateMs < LED_UPDATE_MS) return;
  lastLedUpdateMs = now;

  unsigned long elapsed = now - ledModeStartMs;

  if (elapsed >= matchEndEffectMs) {
    setLedMode(LED_MODE_ATTRACT);
    return;
  }

  CRGB blue = blueGoalColor;
  CRGB orange = orangeGoalColor;

  bool blueWinner = blueScore > orangeScore;
  bool orangeWinner = orangeScore > blueScore;
  bool tied = blueScore == orangeScore;

  CRGB winnerColor = blueWinner ? blue : orange;

  // Match-end winner animation:
  // - Winner field: moving concentric rings in the winning team's color.
  // - Center hex: fast moving rainbow gradient.
  // - Tie: clean blue/orange split with a neutral fast rainbow hex.
  float speedScale = (float)matchEndWinnerSpeed / 100.0f;
  if (speedScale < 0.05f) speedScale = 0.05f;

  float ringSpacing = (float)matchEndRingSpacing;
  if (ringSpacing < 6.0f) ringSpacing = 6.0f;

  uint8_t hexHueBase = (uint8_t)((elapsed * (unsigned long)matchEndRainbowSpeed) / 7UL);
  uint8_t tiePulse = sin8((uint8_t)(elapsed / 4));

  for (uint16_t i = 0; i < LED_COUNT; i++) {
    float x = pixelX[i];
    float y = pixelY[i];
    float xNorm = clampFloat(x / FIELD_WIDTH, 0.0f, 1.0f);
    float yNorm = clampFloat(y / FIELD_HEIGHT, 0.0f, 1.0f);

    if (ledIsHex(i)) {
      // Fast moving rainbow gradient through the hex. This is not white,
      // and the global full-field white block still protects the output.
      uint8_t hue = (uint8_t)(hexHueBase + (uint8_t)(xNorm * 96.0f) + (uint8_t)(yNorm * 128.0f));
      uint8_t valueWave = sin8((uint8_t)(hexHueBase + (uint8_t)(xNorm * 140.0f) + (uint8_t)(yNorm * 90.0f)));
      uint8_t value = map(valueWave, 0, 255, 155, 255);
      leds[i] = CHSV(hue, 255, value);
      continue;
    }

    if (tied) {
      bool leftSide = x < virtualCenterX();
      CRGB sideColor = leftSide ? blue : orange;
      uint8_t b = map(tiePulse, 0, 255, 75, 225);
      leds[i] = scaledColor(sideColor, b);
      continue;
    }

    // Winner rings. Origin is on the winner's side, so the field feels like
    // it is being claimed by the winner instead of just blinking randomly.
    float originX = blueWinner ? 0.0f : FIELD_WIDTH;
    float originY = FIELD_HEIGHT / 2.0f;
    float dx = x - originX;
    float dy = y - originY;
    float d = sqrtf((dx * dx) + (dy * dy));

    float moving = ((float)elapsed * speedScale) / 38.0f;
    float ring = fmodf((d - moving), ringSpacing);
    if (ring < 0.0f) ring += ringSpacing;

    float ringDistance = (ring < (ringSpacing - ring)) ? ring : (ringSpacing - ring);
    float ringStrength = 1.0f - clampFloat(ringDistance / (ringSpacing * 0.34f), 0.0f, 1.0f);
    ringStrength = ringStrength * ringStrength;

    // Winner side gets more base energy. Opposite side still lights, but less.
    float fieldBias = blueWinner ? (1.0f - xNorm) : xNorm;
    float base = 0.13f + (fieldBias * 0.28f);
    float sparkle = (sin8((uint8_t)(elapsed / 3 + i * 7)) / 255.0f) * 0.13f;
    float intensity = clampFloat(base + (ringStrength * 0.78f) + sparkle, 0.0f, 1.0f);

    leds[i] = scaledColor(winnerColor, clampU8((int)(intensity * 255.0f)));
  }

  // Hard override for the two physical center strips:
  // one strip belongs to Blue, the other belongs to Orange. This keeps the
  // midfield split honest even while the winner/rainbow finale runs.
  if (!tied) {
    uint8_t stripBrightness = map(sin8((uint8_t)(elapsed / 4)), 0, 255, 125, 235);
    paintCenterStripsSplit(blue, orange, stripBrightness);
  }

  safeFastLedShow();
}

static void renderCenterTest() {
  unsigned long now = millis();

  if (now - lastLedUpdateMs < LED_UPDATE_MS) return;
  lastLedUpdateMs = now;

  // Hard calibration mode:
  // Turn EVERYTHING off first. No dim background, no virtual X overlay,
  // no purple/magenta reference band. The only LEDs allowed on are the
  // two physical center strips. This is for visual field verification.
  FastLED.clear();

  // Gentle pulse makes the center strips easy to see without going near
  // all-white or high-current conditions.
  uint8_t pulse = sin8((uint8_t)(now / 8));
  uint8_t brightness = map(pulse, 0, 255, 90, 210);

  CRGB stripA = CENTER_TEST_STRIP_A_COLOR;
  stripA.nscale8_video(brightness);

  CRGB stripC = CENTER_TEST_STRIP_C_COLOR;
  stripC.nscale8_video(brightness);

  // First physical center strip: LED 422 to 507.
  for (uint16_t i = CENTER_BASE_A_START; i <= CENTER_BASE_A_END && i < LED_COUNT; i++) {
    leds[i] = stripA;
  }

#if CENTER_TEST_CONNECTOR_ENABLED
  // Optional tiny connector section: LED 507 to 509.
  // Kept disabled by default because you said the true center is two strips.
  for (uint16_t i = CENTER_BASE_B_START; i <= CENTER_BASE_B_END && i < LED_COUNT; i++) {
    leds[i] = CRGB(0, 90, 130);
  }
#endif

  // Second physical center strip: LED 509 to 594.
  for (uint16_t i = CENTER_BASE_C_START; i <= CENTER_BASE_C_END && i < LED_COUNT; i++) {
    leds[i] = stripC;
  }

  safeFastLedShow();
}

static void updateLeds() {
  if (!spatialMapReady) return;
  if (otaInProgress) return;

  if (ledMode == LED_MODE_BLACKOUT) {
    return;
  }

  if (ledMode == LED_MODE_ATTRACT) {
    renderAttractOrInGame(false);
  } else if (ledMode == LED_MODE_IN_GAME) {
    renderAttractOrInGame(true);
  } else if (ledMode == LED_MODE_COUNTDOWN) {
    renderCountdown();
  } else if (ledMode == LED_MODE_GAME_START_GREEN) {
    renderGameStartGreen();
  } else if (ledMode == LED_MODE_BLUE_GOAL) {
    renderGoalEffect(blueGoalColor, true);
  } else if (ledMode == LED_MODE_ORANGE_GOAL) {
    renderGoalEffect(orangeGoalColor, false);
  } else if (ledMode == LED_MODE_HIT) {
    renderHitEffect(false);
  } else if (ledMode == LED_MODE_BIG_HIT) {
    renderHitEffect(true);
  } else if (ledMode == LED_MODE_MATCH_END) {
    renderMatchEndEffect();
  } else if (ledMode == LED_MODE_CENTER_TEST) {
    renderCenterTest();
  }
}

// ============================================================
// AUDIO HELPERS
// ============================================================

static bool audioBusyPlaying() {
  const int level = digitalRead(MP3_BUSY_PIN);

#if BUSY_ACTIVE_HIGH
  return level == HIGH;
#else
  return level == LOW;
#endif
}

static unsigned long getCurrentTrackLengthMs() {
  if (currentTrack == 0 || currentTrack > MAX_TRACK) return 0;
  return TRACK_LENGTH_MS[currentTrack];
}

static unsigned long getAudioElapsedMs() {
  if (!trackActive) return 0;
  return millis() - currentTrackStartTime;
}

static unsigned long getAudioRemainingMs() {
  if (!trackActive) return 0;

  unsigned long lengthMs = getCurrentTrackLengthMs();
  unsigned long elapsedMs = getAudioElapsedMs();

  if (elapsedMs >= lengthMs) return 0;
  return lengthMs - elapsedMs;
}

static int getAudioProgressPercent() {
  if (!trackActive) return 0;

  unsigned long lengthMs = getCurrentTrackLengthMs();
  if (lengthMs == 0) return 0;

  unsigned long elapsedMs = getAudioElapsedMs();
  if (elapsedMs >= lengthMs) return 100;

  return (int)((elapsedMs * 100UL) / lengthMs);
}

static void sendAudioCommand(uint8_t cmd, const uint8_t* data, uint8_t len) {
  uint8_t sum = 0;

  MP3Serial.write(DY_START_BYTE);
  sum += DY_START_BYTE;

  MP3Serial.write(cmd);
  sum += cmd;

  MP3Serial.write(len);
  sum += len;

  for (uint8_t i = 0; i < len; i++) {
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

  for (uint8_t i = 0; i < len; i++) {
    Serial.print(" ");
    printHexByte(data[i]);
  }

  Serial.print(" ");
  printHexByte(checksum);
  Serial.println();
}

static void audioSetVolumePercent(uint8_t percent) {
  if (percent > 100) percent = 100;

  uint8_t moduleVolume = map(percent, 0, 100, 0, 30);

  uint8_t data[1];
  data[0] = moduleVolume;

  sendAudioCommand(DY_CMD_SET_VOLUME, data, 1);

  currentVolumePercent = percent;
  currentModuleVolume = moduleVolume;

  Serial.print("[AUDIO] Volume set to ");
  Serial.print(currentVolumePercent);
  Serial.print("%, module volume ");
  Serial.println(currentModuleVolume);
}

static void audioPlayTrack(uint16_t track) {
  if (track == 0 || track > MAX_TRACK) {
    Serial.println("[AUDIO] Invalid track.");
    return;
  }

  const uint32_t now = millis();

  if (track == lastAudioTrack && (uint32_t)(now - lastAudioCommandMs) < SAME_TRACK_COOLDOWN_MS) {
    Serial.println("[AUDIO] Duplicate track ignored due to cooldown.");
    return;
  }

  uint8_t data[2];
  data[0] = (uint8_t)((track >> 8) & 0xFF);
  data[1] = (uint8_t)(track & 0xFF);

  Serial.print("[AUDIO] Play track ");
  Serial.println(track);

  sendAudioCommand(DY_CMD_PLAY_TRACK, data, 2);

  currentTrack = track;
  lastAudioTrack = track;
  currentTrackStartTime = millis();
  lastAudioCommandMs = millis();
  trackActive = true;
  audioCommandCount++;
}

static void audioStopCommon() {
  Serial.println("[AUDIO] Stop command.");
  sendAudioCommand(DY_CMD_STOP, nullptr, 0);

  trackActive = false;
  currentTrack = 0;
}

static void drainAudioRx() {
  bool any = false;

  while (MP3Serial.available() > 0) {
    if (!any) {
      Serial.print("[AUDIO_RX] ");
      any = true;
    }

    uint8_t b = (uint8_t)MP3Serial.read();
    printHexByte(b);
    Serial.print(" ");
  }

  if (any) Serial.println();
}

static void beginAudio() {
  pinMode(MP3_BUSY_PIN, INPUT);

  MP3Serial.begin(
    MP3_BAUD,
    SERIAL_8N1,
    MP3_RX_PIN,
    MP3_TX_PIN
  );

  delay(100);

  Serial.println("[INIT] Audio UART started.");
}

static void updateAudioState() {
  if (!trackActive) return;

  if (getAudioElapsedMs() >= getCurrentTrackLengthMs()) {
    trackActive = false;
    currentTrack = 0;
    Serial.println("[AUDIO] Estimated track duration elapsed. Track marked inactive.");
  }
}

// ============================================================
// SENSOR HANDLING
// ============================================================

static bool readRawDetected(uint8_t pin) {
  int level = digitalRead(pin);

#if SENSOR_ACTIVE_LOW
  return level == LOW;
#else
  return level == HIGH;
#endif
}

static void initializeDebouncedSensor(DebouncedSensor &s) {
  pinMode(s.pin, INPUT);
  s.lastRawDetected = readRawDetected(s.pin);
  s.stableDetected = s.lastRawDetected;
  s.previousStableDetected = s.stableDetected;
  s.lastRawChangeMs = millis();
}

static void initializeDualGoal(DualGoalValidator &g) {
  initializeDebouncedSensor(g.sensorA);
  initializeDebouncedSensor(g.sensorB);

  bool bothClear = !g.sensorA.stableDetected && !g.sensorB.stableDetected;
  g.armed = bothClear;
  g.bothClearStartMs = millis();
  g.sensorASeenMs = 0;
  g.sensorBSeenMs = 0;
}

static void setupSensors() {
  initializeDualGoal(leftGoal);
  initializeDualGoal(rightGoal);

  Serial.println("[INIT] Four goal sensors started with plain INPUT mode.");
  Serial.println("[INIT] Left goal:  IO_09/GPIO2 + IO_10/GPIO4");
  Serial.println("[INIT] Right goal: IO_11/GPIO22 + IO_12/GPIO21");
  Serial.println("[INIT] Dual-sensor validation enabled: both sensors must confirm within window.");
}

static void updateDebouncedSensor(DebouncedSensor &s) {
  bool rawDetected = readRawDetected(s.pin);
  unsigned long now = millis();

  if (rawDetected != s.lastRawDetected) {
    s.lastRawDetected = rawDetected;
    s.lastRawChangeMs = now;
  }

  if (now - s.lastRawChangeMs >= SENSOR_DEBOUNCE_MS) {
    s.stableDetected = rawDetected;
  }
}

static bool sensorRisingEdge(const DebouncedSensor &s) {
  return s.stableDetected && !s.previousStableDetected;
}

static bool dualGoalBothClear(const DualGoalValidator &g) {
  return !g.sensorA.stableDetected && !g.sensorB.stableDetected;
}

static bool dualGoalAnyDetected(const DualGoalValidator &g) {
  return g.sensorA.stableDetected || g.sensorB.stableDetected;
}

static bool updateDualGoalValidator(DualGoalValidator &g, const char *goalName) {
  updateDebouncedSensor(g.sensorA);
  updateDebouncedSensor(g.sensorB);

  unsigned long now = millis();

  bool edgeA = sensorRisingEdge(g.sensorA);
  bool edgeB = sensorRisingEdge(g.sensorB);
  bool bothClear = dualGoalBothClear(g);

  // Rearm only after both sensors are clear for a short stable period.
  // This is what stops one stuck sensor from permanently acting as "half a goal".
  if (bothClear) {
    if (g.bothClearStartMs == 0) {
      g.bothClearStartMs = now;
    }

    if (!g.armed && (now - g.bothClearStartMs >= DUAL_SENSOR_REARM_CLEAR_MS)) {
      g.armed = true;
      g.sensorASeenMs = 0;
      g.sensorBSeenMs = 0;

      Serial.print("[SENSOR] ");
      Serial.print(goalName);
      Serial.println(" rearmed after both sensors cleared.");
    }
  } else {
    g.bothClearStartMs = 0;
  }

  if (!g.armed) {
    g.sensorA.previousStableDetected = g.sensorA.stableDetected;
    g.sensorB.previousStableDetected = g.sensorB.stableDetected;
    return false;
  }

  if (edgeA) {
    g.sensorASeenMs = now;

    Serial.print("[SENSOR] ");
    Serial.print(goalName);
    Serial.println(" sensor A confirmed.");
  }

  if (edgeB) {
    g.sensorBSeenMs = now;

    Serial.print("[SENSOR] ");
    Serial.print(goalName);
    Serial.println(" sensor B confirmed.");
  }

  bool validPair = false;

  if (g.sensorASeenMs > 0 && g.sensorBSeenMs > 0) {
    unsigned long diff;

    if (g.sensorASeenMs >= g.sensorBSeenMs) {
      diff = g.sensorASeenMs - g.sensorBSeenMs;
    } else {
      diff = g.sensorBSeenMs - g.sensorASeenMs;
    }

    if (diff <= DUAL_SENSOR_CONFIRM_WINDOW_MS) {
      validPair = true;
    }
  }

  if (validPair) {
    g.armed = false;
    g.sensorASeenMs = 0;
    g.sensorBSeenMs = 0;
    g.validGoalCount++;

    Serial.print("[SENSOR] ");
    Serial.print(goalName);
    Serial.println(" VALID dual-sensor goal.");
  }

  // If only one sensor fired and the other did not confirm in time, reject that partial event.
  if (!validPair) {
    if (g.sensorASeenMs > 0 && now - g.sensorASeenMs > DUAL_SENSOR_CONFIRM_WINDOW_MS) {
      g.sensorASeenMs = 0;
      g.rejectedSingleCount++;

      Serial.print("[SENSOR] ");
      Serial.print(goalName);
      Serial.println(" rejected: only sensor A fired.");
    }

    if (g.sensorBSeenMs > 0 && now - g.sensorBSeenMs > DUAL_SENSOR_CONFIRM_WINDOW_MS) {
      g.sensorBSeenMs = 0;
      g.rejectedSingleCount++;

      Serial.print("[SENSOR] ");
      Serial.print(goalName);
      Serial.println(" rejected: only sensor B fired.");
    }
  }

  g.sensorA.previousStableDetected = g.sensorA.stableDetected;
  g.sensorB.previousStableDetected = g.sensorB.stableDetected;

  return validPair;
}

// ============================================================
// GAME LOGIC
// ============================================================

static unsigned long getMatchRemainingMs() {
  if (gameState == STATE_IN_GAME) {
    unsigned long elapsed = millis() - matchStartMs;

    if (elapsed >= matchDurationMs) return 0;
    return matchDurationMs - elapsed;
  }

  if (gameState == STATE_COUNTDOWN) {
    return matchDurationMs;
  }

  if (gameState == STATE_END_GAME) {
    return 0;
  }

  return matchDurationMs;
}

static unsigned long getCountdownTotalMs() {
  return countdownStepMs * COUNTDOWN_SECONDS;
}

static int getCountdownValue() {
  if (gameState != STATE_COUNTDOWN) return 0;

  unsigned long elapsed = millis() - countdownStartMs;
  unsigned long elapsedSteps = elapsed / countdownStepMs;

  if (elapsedSteps >= COUNTDOWN_SECONDS) return 0;

  return (int)(COUNTDOWN_SECONDS - elapsedSteps);
}

static void startMatchWithDuration(unsigned long seconds) {
  if (seconds < 5UL) seconds = 5UL;
  if (seconds > 1800UL) seconds = 1800UL;

  matchDurationMs = seconds * 1000UL;

  // Demo-day behavior: Start Match always starts a clean scoreboard.
  blueScore = 0;
  orangeScore = 0;
  lastBlueGoalMs = 0;
  lastOrangeGoalMs = 0;

  gameState = STATE_COUNTDOWN;
  countdownStartMs = millis();
  lastCountdownAnnounced = -1;

  setLedMode(LED_MODE_COUNTDOWN);

  addEvent("Countdown started");
}

static void completeCountdownAndStartGame() {
  gameState = STATE_IN_GAME;
  matchStartMs = millis();

  setLedMode(LED_MODE_GAME_START_GREEN);
  audioPlayTrack(1);

  addEvent("Match started");
}

static void endMatch(bool manualEnd) {
  if (gameState == STATE_END_GAME) return;

  gameState = STATE_END_GAME;
  matchEndMs = millis();

  setLedMode(LED_MODE_MATCH_END);
  audioPlayTrack(7);

  String result = "Match ended";

  if (manualEnd) {
    result += " manually";
  }

  result += ". ";

  if (blueScore > orangeScore) {
    result += "Blue wins";
  } else if (orangeScore > blueScore) {
    result += "Orange wins";
  } else {
    result += "Tie game";
  }

  addEvent(result);
}

static void resetGame() {
  unsigned long now = millis();

  if (now - lastResetMs < RESET_COOLDOWN_MS) {
    addEvent("Reset ignored due to cooldown");
    return;
  }

  lastResetMs = now;

  blueScore = 0;
  orangeScore = 0;

  gameState = STATE_STANDBY;
  matchStartMs = 0;
  countdownStartMs = 0;
  lastCountdownAnnounced = -1;

  setLedMode(LED_MODE_ATTRACT);
  audioPlayTrack(8);

  addEvent("Game reset");
}

static void handleBlueGoal(bool fake) {
  unsigned long now = millis();

  if (gameState != STATE_IN_GAME) {
    addEvent(fake ? "Fake Blue goal ignored outside IN_GAME" : "Blue sensor ignored outside IN_GAME");
    return;
  }

  if (now - lastBlueGoalMs < GOAL_LOCKOUT_MS) {
    addEvent("Blue goal ignored due to lockout");
    return;
  }

  lastBlueGoalMs = now;
  blueScore++;

  setLedMode(LED_MODE_BLUE_GOAL);
  audioPlayTrack(2);

  addEvent(fake ? "Fake Blue goal scored" : "Blue goal scored");
}

static void handleOrangeGoal(bool fake) {
  unsigned long now = millis();

  if (gameState != STATE_IN_GAME) {
    addEvent(fake ? "Fake Orange goal ignored outside IN_GAME" : "Orange sensor ignored outside IN_GAME");
    return;
  }

  if (now - lastOrangeGoalMs < GOAL_LOCKOUT_MS) {
    addEvent("Orange goal ignored due to lockout");
    return;
  }

  lastOrangeGoalMs = now;
  orangeScore++;

  setLedMode(LED_MODE_ORANGE_GOAL);
  audioPlayTrack(3);

  addEvent(fake ? "Fake Orange goal scored" : "Orange goal scored");
}

static void handleBallHit(bool bigHit) {
  if (bigHit) {
    setLedMode(LED_MODE_BIG_HIT);
    audioPlayTrack(5);
    addEvent("Big ball hit");
  } else {
    setLedMode(LED_MODE_HIT);
    audioPlayTrack(4);
    addEvent("Ball hit");
  }
}

static void updateGame() {
  unsigned long now = millis();

  if (gameState == STATE_COUNTDOWN) {
    int count = getCountdownValue();

    if (count > 0 && count != lastCountdownAnnounced) {
      lastCountdownAnnounced = count;
      audioPlayTrack(6);

      String msg = "Countdown ";
      msg += String(count);
      addEvent(msg);
    }

    if (now - countdownStartMs >= getCountdownTotalMs()) {
      completeCountdownAndStartGame();
    }
  }

  if (gameState == STATE_IN_GAME) {
    if (getMatchRemainingMs() == 0) {
      endMatch(false);
    }
  }
}

static void updateSensorsAndGoals() {
  bool leftGoalValid = updateDualGoalValidator(leftGoal, "LEFT GOAL");
  bool rightGoalValid = updateDualGoalValidator(rightGoal, "RIGHT GOAL");

  if (leftGoalValid) {
    // Left goal is mapped to Blue scoring for this demo.
    // Swap this to handleOrangeGoal(false) if the physical field scoring is reversed.
    handleBlueGoal(false);
  }

  if (rightGoalValid) {
    // Right goal is mapped to Orange scoring for this demo.
    // Swap this to handleBlueGoal(false) if the physical field scoring is reversed.
    handleOrangeGoal(false);
  }
}

// ============================================================
// JSON STATE
// ============================================================

static String jsonEscape(String s) {
  s.replace("\\", "\\\\");
  s.replace("\"", "\\\"");
  s.replace("\n", " ");
  s.replace("\r", " ");
  return s;
}

static String stateJson() {
  String out = "{";

  out += "\"wifi\":";
  out += wifiReady ? "true" : "false";
  out += ",";

  out += "\"ip\":\"";
#if DEMO_USE_ACCESS_POINT
  out += WiFi.softAPIP().toString();
#else
  out += WiFi.localIP().toString();
#endif
  out += "\",";

  out += "\"gameState\":\"";
  out += gameStateName();
  out += "\",";

  out += "\"ledMode\":\"";
  out += ledModeName();
  out += "\",";

  out += "\"blueScore\":";
  out += String(blueScore);
  out += ",";

  out += "\"orangeScore\":";
  out += String(orangeScore);
  out += ",";

  out += "\"matchDurationMs\":";
  out += String(matchDurationMs);
  out += ",";

  out += "\"matchRemainingMs\":";
  out += String(getMatchRemainingMs());
  out += ",";

  out += "\"countdownValue\":";
  out += String(getCountdownValue());
  out += ",";

  out += "\"leftSensorA\":";
  out += leftGoal.sensorA.stableDetected ? "true" : "false";
  out += ",";

  out += "\"leftSensorB\":";
  out += leftGoal.sensorB.stableDetected ? "true" : "false";
  out += ",";

  out += "\"rightSensorA\":";
  out += rightGoal.sensorA.stableDetected ? "true" : "false";
  out += ",";

  out += "\"rightSensorB\":";
  out += rightGoal.sensorB.stableDetected ? "true" : "false";
  out += ",";

  out += "\"leftGoalArmed\":";
  out += leftGoal.armed ? "true" : "false";
  out += ",";

  out += "\"rightGoalArmed\":";
  out += rightGoal.armed ? "true" : "false";
  out += ",";

  out += "\"leftGoalValidCount\":";
  out += String(leftGoal.validGoalCount);
  out += ",";

  out += "\"rightGoalValidCount\":";
  out += String(rightGoal.validGoalCount);
  out += ",";

  out += "\"leftGoalRejectedSingles\":";
  out += String(leftGoal.rejectedSingleCount);
  out += ",";

  out += "\"rightGoalRejectedSingles\":";
  out += String(rightGoal.rejectedSingleCount);
  out += ",";

  // Backward-compatible combined status for the old dashboard labels.
  out += "\"blueSensorDetected\":";
  out += dualGoalAnyDetected(leftGoal) ? "true" : "false";
  out += ",";

  out += "\"orangeSensorDetected\":";
  out += dualGoalAnyDetected(rightGoal) ? "true" : "false";
  out += ",";

  out += "\"audioBusy\":";
  out += audioBusyPlaying() ? "true" : "false";
  out += ",";

  out += "\"trackActive\":";
  out += trackActive ? "true" : "false";
  out += ",";

  out += "\"currentTrack\":";
  out += String(currentTrack);
  out += ",";

  out += "\"lastAudioTrack\":";
  out += String(lastAudioTrack);
  out += ",";

  out += "\"audioRemainingMs\":";
  out += String(getAudioRemainingMs());
  out += ",";

  out += "\"audioProgressPercent\":";
  out += String(getAudioProgressPercent());
  out += ",";

  out += "\"audioCommandCount\":";
  out += String(audioCommandCount);
  out += ",";

  out += "\"volume\":";
  out += String(currentVolumePercent);
  out += ",";

  out += "\"moduleVolume\":";
  out += String(currentModuleVolume);
  out += ",";

  out += "\"ledBrightness\":";
  out += String(currentLedBrightnessPercent);
  out += ",";

  out += "\"ledBrightnessRaw\":";
  out += String(currentLedBrightnessValue);
  out += ",";

  out += "\"ledSafetyMaxPercent\":";
  out += String(LED_BRIGHTNESS_MAX_PERCENT);
  out += ",";

  out += "\"fullWhiteBlocked\":true,";

  out += "\"centerXShift\":";
  out += String(centerXShift, 2);
  out += ",";

  out += "\"virtualCenterX\":";
  out += String(virtualCenterX(), 2);
  out += ",";

  out += "\"centerYShift\":";
  out += String(centerYShift, 2);
  out += ",";

  out += "\"centerLedIndexShift\":";
  out += String(centerLedIndexShift);
  out += ",";

  out += "\"centerStripStart\":";
  out += String(centerStripStart());
  out += ",";

  out += "\"centerStripEnd\":";
  out += String(centerStripEnd());
  out += ",";

  out += "\"spatialMapReady\":";
  out += spatialMapReady ? "true" : "false";
  out += ",";

  out += "\"ledCount\":";
  out += String(LED_COUNT);
  out += ",";

  out += "\"blueGoalColor\":\"";
  out += colorToHex(blueGoalColor);
  out += "\",";

  out += "\"orangeGoalColor\":\"";
  out += colorToHex(orangeGoalColor);
  out += "\",";

  out += "\"countdownHexColor\":\"";
  out += colorToHex(countdownHexColor);
  out += "\",";

  out += "\"countdownFieldColor\":\"";
  out += colorToHex(countdownFieldColor);
  out += "\",";

  out += "\"gameStartColor\":\"";
  out += colorToHex(gameStartColor);
  out += "\",";

  out += "\"countdownEffectMode\":\"";
  out += countdownFieldEffectName();
  out += "\",";

  out += "\"countdownStepMs\":";
  out += String(countdownStepMs);
  out += ",";

  out += "\"countdownFieldEffectPeriodMs\":";
  out += String(countdownFieldEffectPeriodMs);
  out += ",";

  out += "\"countdownFieldMinBrightness\":";
  out += String(countdownFieldMinBrightness);
  out += ",";

  out += "\"countdownFieldMaxBrightness\":";
  out += String(countdownFieldMaxBrightness);
  out += ",";

  out += "\"countdownHexBrightness\":";
  out += String(countdownHexBrightness);
  out += ",";

  out += "\"goalEffectMs\":";
  out += String(goalEffectMs);
  out += ",";

  out += "\"goalWaveTravelMs\":";
  out += String(goalWaveTravelMs);
  out += ",";

  out += "\"goalWaveEmitIntervalMs\":";
  out += String(goalWaveEmitIntervalMs);
  out += ",";

  out += "\"goalWaveEmitEndMs\":";
  out += String(goalWaveEmitEndMs);
  out += ",";

  out += "\"goalSourcePopMs\":";
  out += String(goalSourcePopMs);
  out += ",";

  out += "\"goalTravelMode\":\"";
  out += goalTravelModeValue();
  out += "\",";

  out += "\"matchEndEffectMs\":";
  out += String(matchEndEffectMs);
  out += ",";

  out += "\"matchEndWinnerSpeed\":";
  out += String(matchEndWinnerSpeed);
  out += ",";

  out += "\"matchEndRainbowSpeed\":";
  out += String(matchEndRainbowSpeed);
  out += ",";

  out += "\"matchEndRingSpacing\":";
  out += String(matchEndRingSpacing);
  out += ",";

  out += "\"winner\":\"";
  if (gameState == STATE_END_GAME) {
    if (blueScore > orangeScore) out += "blue";
    else if (orangeScore > blueScore) out += "orange";
    else out += "tie";
  } else {
    out += "none";
  }
  out += "\",";

  out += "\"lastEvent\":\"";
  out += jsonEscape(lastEvent);
  out += "\",";

  out += "\"eventLog\":[";
  for (uint8_t n = 0; n < eventLogCount; n++) {
    uint8_t idx = (eventLogIndex + EVENT_LOG_SIZE - eventLogCount + n) % EVENT_LOG_SIZE;

    out += "\"";
    out += jsonEscape(eventLog[idx]);
    out += "\"";

    if (n < eventLogCount - 1) out += ",";
  }
  out += "],";

  out += "\"uptimeMs\":";
  out += String(millis());

  out += "}";

  return out;
}

// ============================================================
// WEB UI
// ============================================================

static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
  <meta name="apple-mobile-web-app-capable" content="yes">
  <meta name="apple-mobile-web-app-status-bar-style" content="black-translucent">
  <title>FIFA Robot Soccer Controller</title>

  <style>
    :root{
      --bg:#020617;--panel:rgba(15,23,42,.76);--panel2:rgba(2,6,23,.72);
      --line:rgba(255,255,255,.11);--line2:rgba(255,255,255,.19);
      --text:#f8fafc;--muted:#94a3b8;--blue:#2563eb;--blue2:#60a5fa;
      --orange:#f97316;--orange2:#fdba74;--green:#22c55e;--red:#ef4444;
      --purple:#a855f7;--shadow:0 24px 80px rgba(0,0,0,.56);--radius:26px;
    }
    *{box-sizing:border-box}
    body{margin:0;min-height:100vh;color:var(--text);font-family:Inter,ui-sans-serif,system-ui,-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,Arial,sans-serif;background:radial-gradient(circle at 18% -10%,rgba(37,99,235,.30),transparent 34%),radial-gradient(circle at 82% -8%,rgba(249,115,22,.27),transparent 32%),radial-gradient(circle at 50% 110%,rgba(34,197,94,.13),transparent 34%),linear-gradient(180deg,#020617,#030712 72%,#000)}
    button,input,select,textarea{font:inherit}.app{width:min(1320px,100%);margin:0 auto;padding:24px}
    .topbar{display:flex;align-items:center;justify-content:space-between;gap:18px;margin-bottom:18px}.brand{display:flex;align-items:center;gap:14px;min-width:0}.badgeLogo{width:56px;height:56px;border-radius:18px;display:grid;place-items:center;color:#020617;font-weight:1000;letter-spacing:-.08em;background:linear-gradient(135deg,#fff,#38bdf8,#fb923c);box-shadow:0 18px 40px rgba(56,189,248,.18);flex:0 0 auto}h1{margin:0;font-size:30px;letter-spacing:-.04em;line-height:1.05}.subtitle{margin-top:5px;color:var(--muted);font-size:14px;line-height:1.4}.pillbar{display:flex;gap:10px;flex-wrap:wrap;justify-content:flex-end}.pill{border:1px solid var(--line);background:rgba(255,255,255,.055);border-radius:999px;padding:9px 12px;color:var(--muted);font-weight:850;font-size:13px;backdrop-filter:blur(12px)}.pill strong{color:#fff}
    .scoreHero{position:relative;overflow:hidden;border:1px solid var(--line);background:linear-gradient(90deg,rgba(37,99,235,.24),transparent 38%,transparent 62%,rgba(249,115,22,.24)),linear-gradient(180deg,rgba(255,255,255,.08),rgba(255,255,255,.025));border-radius:32px;box-shadow:var(--shadow);padding:22px;margin-bottom:18px}.scoreGrid{display:grid;grid-template-columns:1fr minmax(230px,.62fr) 1fr;gap:14px;align-items:stretch}.teamCard,.clockCard{border:1px solid var(--line);background:rgba(2,6,23,.52);border-radius:26px;padding:20px;min-height:160px;display:flex;flex-direction:column;justify-content:center;box-shadow:inset 0 1px 0 rgba(255,255,255,.04)}.teamCard.blue{background:linear-gradient(180deg,rgba(37,99,235,.25),rgba(2,6,23,.48))}.teamCard.orange{background:linear-gradient(180deg,rgba(249,115,22,.26),rgba(2,6,23,.48))}.teamLabel{color:var(--muted);font-size:14px;font-weight:1000;text-transform:uppercase;letter-spacing:.18em}.scoreNum{margin-top:6px;font-size:104px;line-height:.85;font-weight:1000;letter-spacing:-.09em}.blue .scoreNum{color:#60a5fa;text-shadow:0 0 42px rgba(37,99,235,.36)}.orange .scoreNum{color:#fb923c;text-shadow:0 0 42px rgba(249,115,22,.32)}.clockCard{text-align:center;background:rgba(2,6,23,.64)}.clockLabel{color:var(--muted);font-size:13px;font-weight:1000;text-transform:uppercase;letter-spacing:.2em}.clock{font-size:64px;line-height:.92;font-weight:1000;letter-spacing:-.065em;margin-top:10px}.stateChip{align-self:center;margin-top:14px;border-radius:999px;padding:8px 13px;border:1px solid var(--line2);font-size:13px;font-weight:1000;letter-spacing:.08em;color:#e2e8f0;background:rgba(255,255,255,.06)}.stateChip.live{color:#86efac;border-color:rgba(34,197,94,.35);background:rgba(34,197,94,.12)}.stateChip.countdown{color:#fde68a;border-color:rgba(250,204,21,.35);background:rgba(250,204,21,.12)}.stateChip.end{color:#fca5a5;border-color:rgba(239,68,68,.35);background:rgba(239,68,68,.12)}
    .matchSetup{display:grid;grid-template-columns:1fr auto auto;gap:12px;align-items:center;margin-top:14px;border:1px solid var(--line);border-radius:22px;padding:14px;background:rgba(2,6,23,.38)}.setupTitle{font-size:12px;text-transform:uppercase;letter-spacing:.14em;color:var(--muted);font-weight:1000}.setupValue{font-weight:1000;font-size:22px;white-space:nowrap}.actionBar{display:grid;grid-template-columns:repeat(6,1fr);gap:10px;margin-top:14px}
    button{border:0;border-radius:18px;min-height:58px;padding:14px 12px;color:#fff;font-weight:950;cursor:pointer;box-shadow:0 14px 30px rgba(0,0,0,.32);transition:transform .08s ease,filter .15s ease,box-shadow .15s ease;background:#1e293b}button:hover{filter:brightness(1.08);box-shadow:0 18px 42px rgba(0,0,0,.42)}button:active{transform:translateY(1px) scale(.99)}.smallText{display:block;margin-top:3px;font-size:11px;opacity:.76;font-weight:850}.primary{background:linear-gradient(135deg,#15803d,#22c55e);color:#02130a}.quick{background:linear-gradient(135deg,#0f766e,#22d3ee);color:#02131a}.end{background:linear-gradient(135deg,#7f1d1d,#ef4444)}.reset{background:linear-gradient(135deg,#334155,#0f172a);border:1px solid rgba(255,255,255,.12)}.blueBtn{background:linear-gradient(135deg,#1e3a8a,#2563eb)}.orangeBtn{background:linear-gradient(135deg,#9a3412,#f97316);color:#111827}.scoreboardBtn{background:linear-gradient(135deg,#7e22ce,#38bdf8);color:#020617}
    .layout{display:grid;grid-template-columns:1.02fr .98fr;gap:18px;align-items:start}.card{border:1px solid var(--line);background:linear-gradient(180deg,rgba(255,255,255,.065),rgba(255,255,255,.025));border-radius:var(--radius);box-shadow:var(--shadow);overflow:hidden;backdrop-filter:blur(12px)}.cardBody{padding:18px}.sectionTitle{display:flex;justify-content:space-between;gap:12px;align-items:end;margin-bottom:14px}.sectionTitle h2{margin:0;font-size:16px;letter-spacing:.02em}.hint{color:var(--muted);font-size:12px;line-height:1.45}.metrics{display:grid;grid-template-columns:repeat(2,1fr);gap:10px}.metric{border:1px solid var(--line);border-radius:20px;padding:14px;background:rgba(2,6,23,.42)}.metricName{color:var(--muted);font-size:11px;font-weight:1000;text-transform:uppercase;letter-spacing:.12em}.metricValue{margin-top:6px;font-size:18px;font-weight:1000;word-break:break-word}.ok{color:#86efac}.bad{color:#fca5a5}.warn{color:#fde68a}.blueText{color:#7dd3fc}.orangeText{color:#fdba74}.eventLog{display:flex;flex-direction:column;gap:8px;max-height:250px;overflow:auto;padding-right:4px}.logItem{border:1px solid var(--line);border-radius:16px;padding:10px 12px;background:rgba(2,6,23,.38);color:#dbeafe;font-size:13px;line-height:1.35}.studioGrid{display:grid;grid-template-columns:repeat(2,1fr);gap:12px}.controlPanel{border:1px solid var(--line);border-radius:22px;padding:15px;background:rgba(2,6,23,.38)}.panelTop{display:flex;align-items:center;justify-content:space-between;gap:12px;margin-bottom:10px}.panelTitle{font-size:12px;text-transform:uppercase;letter-spacing:.12em;color:var(--muted);font-weight:1000}.panelValue{font-weight:1000;font-size:18px;white-space:nowrap}input[type=range]{width:100%;accent-color:#38bdf8;cursor:pointer}input[type=color]{width:54px;height:38px;border:0;border-radius:12px;background:transparent;cursor:pointer}select{width:100%;padding:12px;border-radius:14px;background:#050816;color:#f8fafc;border:1px solid var(--line2);font-weight:900;outline:none}.colorRow{display:flex;align-items:center;justify-content:space-between;gap:10px;padding:10px 0;border-bottom:1px solid rgba(255,255,255,.07)}.colorRow:last-child{border-bottom:0}.colorRow label{color:#e5e7eb;font-size:14px;font-weight:900}.controlActions{display:grid;grid-template-columns:repeat(4,1fr);gap:10px;margin-top:12px}details{border:1px solid var(--line);border-radius:22px;background:rgba(2,6,23,.34);overflow:hidden}summary{cursor:pointer;list-style:none;padding:15px 16px;font-weight:1000;color:#e2e8f0;display:flex;justify-content:space-between;align-items:center}summary::-webkit-details-marker{display:none}details .inner{padding:0 16px 16px}.recipeBox{width:100%;min-height:130px;border:1px solid var(--line2);border-radius:18px;background:#050816;color:#dbeafe;padding:12px;font-size:12px;line-height:1.45;resize:vertical}.footer{margin-top:16px;text-align:center;color:#64748b;font-size:12px}
    .scoreboardOverlay{display:none;position:fixed;inset:0;z-index:50;padding:34px;background:radial-gradient(circle at 20% 0%,rgba(37,99,235,.35),transparent 34%),radial-gradient(circle at 80% 0%,rgba(249,115,22,.31),transparent 34%),linear-gradient(180deg,#020617,#020617 65%,#000);color:white;overflow:hidden}body.scoreboardMode .app{display:none}body.scoreboardMode .scoreboardOverlay{display:grid;grid-template-rows:auto 1fr auto;gap:22px}.scoreboardOverlay::before{content:"";position:absolute;inset:-30%;opacity:0;pointer-events:none;background:conic-gradient(from 90deg,transparent,rgba(255,255,255,.10),transparent,rgba(255,255,255,.16),transparent);animation:spinGlow 4s linear infinite}.scoreboardHeader,.scoreboardMain,.scoreboardFooter{position:relative;z-index:1}.scoreboardHeader{display:flex;align-items:center;justify-content:space-between;gap:18px}.scoreboardTitle{font-size:28px;font-weight:1000;letter-spacing:-.04em;color:#e2e8f0}.exitBtn{min-height:46px;padding:10px 14px;border-radius:999px;background:rgba(255,255,255,.09);border:1px solid rgba(255,255,255,.16)}.winnerBanner{display:none;font-size:clamp(30px,5vw,84px);font-weight:1000;letter-spacing:.08em;text-transform:uppercase;text-align:center;text-shadow:0 0 42px rgba(255,255,255,.35)}.scoreboardMain{display:grid;grid-template-columns:1fr .82fr 1fr;gap:24px;align-items:stretch;min-height:0}.scoreboardTeam,.scoreboardClock{border:1px solid rgba(255,255,255,.14);border-radius:34px;display:flex;flex-direction:column;align-items:center;justify-content:center;box-shadow:0 28px 90px rgba(0,0,0,.45),inset 0 1px 0 rgba(255,255,255,.06)}.scoreboardTeam.blue{background:linear-gradient(180deg,rgba(37,99,235,.35),rgba(2,6,23,.58))}.scoreboardTeam.orange{background:linear-gradient(180deg,rgba(249,115,22,.36),rgba(2,6,23,.58))}.scoreboardTeamName{font-size:clamp(28px,4vw,62px);font-weight:1000;letter-spacing:.1em;text-transform:uppercase;color:#cbd5e1}.scoreboardScore{font-size:clamp(150px,20vw,310px);font-weight:1000;letter-spacing:-.12em;line-height:.78;margin-top:16px}.scoreboardTeam.blue .scoreboardScore{color:#60a5fa;text-shadow:0 0 70px rgba(37,99,235,.5)}.scoreboardTeam.orange .scoreboardScore{color:#fb923c;text-shadow:0 0 70px rgba(249,115,22,.5)}.scoreboardClock{background:rgba(2,6,23,.62);text-align:center;padding:20px}.scoreboardClockLabel{font-size:clamp(18px,2vw,28px);font-weight:1000;color:#94a3b8;text-transform:uppercase;letter-spacing:.22em}.scoreboardTime{font-size:clamp(86px,12vw,190px);font-weight:1000;letter-spacing:-.08em;line-height:.88;margin-top:18px}.scoreboardState{margin-top:22px;border-radius:999px;padding:13px 20px;border:1px solid rgba(255,255,255,.16);font-size:clamp(18px,2.2vw,32px);font-weight:1000;letter-spacing:.12em;text-transform:uppercase;background:rgba(255,255,255,.07)}.scoreboardFooter{display:flex;justify-content:space-between;align-items:center;gap:20px;color:#cbd5e1;font-size:clamp(16px,2vw,24px);font-weight:900}body.scoreboardMode.winnerBlue .scoreboardOverlay{background:radial-gradient(circle at 18% 10%,rgba(37,99,235,.70),transparent 36%),radial-gradient(circle at 70% 88%,rgba(56,189,248,.30),transparent 34%),linear-gradient(180deg,#020617,#06163d 70%,#000)}body.scoreboardMode.winnerOrange .scoreboardOverlay{background:radial-gradient(circle at 82% 10%,rgba(249,115,22,.74),transparent 36%),radial-gradient(circle at 30% 88%,rgba(250,204,21,.26),transparent 34%),linear-gradient(180deg,#170901,#3a1605 72%,#000)}body.scoreboardMode.winnerTie .scoreboardOverlay{background:linear-gradient(90deg,rgba(37,99,235,.40),rgba(2,6,23,.92),rgba(249,115,22,.40)),#020617}body.scoreboardMode.winnerBlue .scoreboardOverlay::before,body.scoreboardMode.winnerOrange .scoreboardOverlay::before,body.scoreboardMode.winnerTie .scoreboardOverlay::before{opacity:1}body.scoreboardMode.winnerBlue .winnerBanner,body.scoreboardMode.winnerOrange .winnerBanner,body.scoreboardMode.winnerTie .winnerBanner{display:block;animation:winnerPop 1.1s ease-in-out infinite alternate}body.scoreboardMode.winnerBlue .scoreboardTeam.blue,body.scoreboardMode.winnerOrange .scoreboardTeam.orange{animation:winnerCardPulse .85s ease-in-out infinite alternate}body.scoreboardMode.winnerBlue .scoreboardTeam.orange,body.scoreboardMode.winnerOrange .scoreboardTeam.blue{filter:saturate(.55) brightness(.55)}@keyframes winnerCardPulse{from{transform:scale(1);box-shadow:0 28px 90px rgba(0,0,0,.45),0 0 30px rgba(255,255,255,.10)}to{transform:scale(1.025);box-shadow:0 32px 110px rgba(0,0,0,.55),0 0 70px rgba(255,255,255,.22)}}@keyframes winnerPop{from{transform:scale(1);filter:brightness(1)}to{transform:scale(1.04);filter:brightness(1.28)}}@keyframes spinGlow{to{transform:rotate(360deg)}}
    @media(max-width:980px){.app{padding:16px}.topbar{flex-direction:column;align-items:flex-start}.pillbar{justify-content:flex-start}.scoreGrid{grid-template-columns:1fr}.scoreNum{font-size:82px}.clock{font-size:54px}.matchSetup{grid-template-columns:1fr}.actionBar{grid-template-columns:repeat(2,1fr)}.layout{grid-template-columns:1fr}.studioGrid{grid-template-columns:1fr}.controlActions{grid-template-columns:repeat(2,1fr)}.scoreboardMain{grid-template-columns:1fr;grid-template-rows:1fr .75fr 1fr}.scoreboardOverlay{padding:18px}}
    @media(max-width:560px){h1{font-size:24px}.metrics{grid-template-columns:1fr}.scoreboardFooter{flex-direction:column;align-items:flex-start}.scoreboardHeader{align-items:flex-start;flex-direction:column}}
  </style>
</head>
<body>
  <div class="scoreboardOverlay" id="scoreboardOverlay">
    <div class="scoreboardHeader"><div><div class="scoreboardTitle">FIFA Robot Soccer</div><div class="winnerBanner" id="boardWinnerBanner">Winner</div></div><button class="exitBtn" onclick="exitScoreboardMode()">Exit Scoreboard</button></div>
    <div class="scoreboardMain"><div class="scoreboardTeam blue"><div class="scoreboardTeamName">Blue</div><div class="scoreboardScore" id="boardBlueScore">0</div></div><div class="scoreboardClock"><div class="scoreboardClockLabel" id="boardClockLabel">Match Timer</div><div class="scoreboardTime" id="boardTimer">2:00</div><div class="scoreboardState" id="boardState">Standby</div></div><div class="scoreboardTeam orange"><div class="scoreboardTeamName">Orange</div><div class="scoreboardScore" id="boardOrangeScore">0</div></div></div>
    <div class="scoreboardFooter"><div id="boardLastEvent">Ready</div><div id="boardSafety">LED limit 50% · No full-field white</div></div>
  </div>
  <div class="app">
    <div class="topbar"><div class="brand"><div class="badgeLogo">FC</div><div><h1>Robot Soccer Match Controller</h1><div class="subtitle">LoR Core V3 · dual-sensor goals · DY-HV20T audio · safe field LEDs</div></div></div><div class="pillbar"><div class="pill">IP <strong id="topIp">---</strong></div><div class="pill">Wi‑Fi <strong id="topWifi">---</strong></div><div class="pill">LED Max <strong id="topLedMax">50%</strong></div></div></div>
    <section class="scoreHero"><div class="scoreGrid"><div class="teamCard blue"><div class="teamLabel">Blue Team</div><div class="scoreNum" id="blueScore">0</div></div><div class="clockCard"><div class="clockLabel" id="clockLabel">Match Timer</div><div class="clock" id="timer">2:00</div><div class="stateChip" id="stateChip">STANDBY</div></div><div class="teamCard orange"><div class="teamLabel">Orange Team</div><div class="scoreNum" id="orangeScore">0</div></div></div>
      <div class="matchSetup"><div><div class="setupTitle">Custom Match Length</div><div class="hint">Slide to test length. It now stays where you put it. Range: 10 s to 10 min.</div></div><div class="setupValue" id="matchLengthValue">0:20</div><input id="matchLengthSlider" type="range" min="10" max="600" step="5" value="20" oninput="previewMatchLength(this.value)" onchange="applyMatchLength(this.value)"></div>
      <div class="actionBar"><button class="primary" onclick="startCustomMatch()">Start Custom<span class="smallText">uses slider</span></button><button class="quick" onclick="quickMatch()">Quick Match<span class="smallText">20 second test</span></button><button class="end" onclick="api('/api/end')">End Match<span class="smallText">show finale</span></button><button class="reset" onclick="api('/api/reset')">Reset<span class="smallText">score + state</span></button><button class="scoreboardBtn" onclick="enterScoreboardMode()">Scoreboard Mode<span class="smallText">iPad display</span></button><button class="reset" onclick="api('/api/stop-audio')">Stop Audio<span class="smallText">audio only</span></button></div></section>
    <div class="layout"><div class="card"><div class="cardBody"><div class="sectionTitle"><div><h2>Live System Status</h2><div class="hint">Use this before the crowd sees it. During the demo, use Scoreboard Mode.</div></div></div><div class="metrics"><div class="metric"><div class="metricName">Left Goal Pair</div><div class="metricValue" id="leftPair">---</div></div><div class="metric"><div class="metricName">Right Goal Pair</div><div class="metricValue" id="rightPair">---</div></div><div class="metric"><div class="metricName">Left Sensors A/B</div><div class="metricValue" id="leftSensors">---</div></div><div class="metric"><div class="metricName">Right Sensors A/B</div><div class="metricValue" id="rightSensors">---</div></div><div class="metric"><div class="metricName">Audio BUSY</div><div class="metricValue" id="busy">---</div></div><div class="metric"><div class="metricName">LED Mode</div><div class="metricValue blueText" id="ledMode">---</div></div><div class="metric"><div class="metricName">Rejected Singles</div><div class="metricValue" id="rejects">0</div></div><div class="metric"><div class="metricName">Last Event</div><div class="metricValue" id="lastEvent">Boot</div></div></div><details style="margin-top:14px;" open><summary>Testing Controls <span class="hint">fake events</span></summary><div class="inner"><div class="controlActions"><button class="blueBtn" onclick="api('/api/fake-blue-goal')">Fake Blue Goal</button><button class="orangeBtn" onclick="api('/api/fake-orange-goal')">Fake Orange Goal</button><button class="quick" onclick="api('/api/fake-hit')">Fake Hit</button><button class="scoreboardBtn" onclick="api('/api/fake-big-hit')">Fake Big Hit</button></div></div></details><details style="margin-top:14px;"><summary>Event Log <span class="hint">latest actions</span></summary><div class="inner"><div class="eventLog" id="eventLog"></div></div></details></div></div>
      <div class="card"><div class="cardBody"><div class="sectionTitle"><div><h2>Animation Studio</h2><div class="hint">Tune the show without reflashing. Defaults: countdown hex green, countdown field red, game-start flash green.</div></div></div>
        <details open><summary>Color Themes <span class="hint">live update</span></summary><div class="inner"><div class="colorRow"><label>Blue Goal Waves</label><input id="blueGoalColorInput" type="color" value="#0050ff" onchange="setAnimationColors()"></div><div class="colorRow"><label>Orange Goal Waves</label><input id="orangeGoalColorInput" type="color" value="#ff5000" onchange="setAnimationColors()"></div><div class="colorRow"><label>Countdown Hex</label><input id="countdownHexColorInput" type="color" value="#00ff00" onchange="setAnimationColors()"></div><div class="colorRow"><label>Countdown Field</label><input id="countdownFieldColorInput" type="color" value="#ff0000" onchange="setAnimationColors()"></div><div class="colorRow"><label>Game Start Flash</label><input id="gameStartColorInput" type="color" value="#00ff00" onchange="setAnimationColors()"></div></div></details>
        <details style="margin-top:12px;" open><summary>Countdown Look <span class="hint">hex + field effect</span></summary><div class="inner"><div class="controlPanel"><div class="panelTop"><div class="panelTitle">Field Effect Mode</div><div class="panelValue" id="countdownEffectValue">PULSE</div></div><select id="countdownEffectSelect" onchange="setCountdownTuning()"><option value="pulse">Pulse</option><option value="blink">Blink</option><option value="solid">Solid</option><option value="breathe">Breathe</option><option value="heartbeat">Heartbeat</option><option value="strobe">Strobe</option><option value="wave">Chasing Wave</option></select></div><div class="studioGrid" style="margin-top:12px;"><div class="controlPanel"><div class="panelTop"><div class="panelTitle">Countdown Step Speed</div><div class="panelValue" id="countdownStepValue">1000 ms</div></div><input id="countdownStepSlider" type="range" min="400" max="2000" step="50" value="1000" oninput="previewCountdownStep(this.value)" onchange="setCountdownTuning()"></div><div class="controlPanel"><div class="panelTop"><div class="panelTitle">Field Effect Speed</div><div class="panelValue" id="countdownPeriodValue">1000 ms</div></div><input id="countdownPeriodSlider" type="range" min="80" max="2500" step="20" value="1000" oninput="previewCountdownPeriod(this.value)" onchange="setCountdownTuning()"></div></div><div class="controlPanel" style="margin-top:12px;"><div class="panelTop"><div class="panelTitle">Countdown Brightness</div><div class="panelValue" id="countdownBrightnessValue">45 / 125 / 230</div></div><div class="hint">Field min / field max / hex brightness</div><input id="countdownMinSlider" type="range" min="0" max="220" step="5" value="45" oninput="previewCountdownBrightness()" onchange="setCountdownTuning()"><input id="countdownMaxSlider" type="range" min="20" max="255" step="5" value="125" oninput="previewCountdownBrightness()" onchange="setCountdownTuning()"><input id="countdownHexBrightnessSlider" type="range" min="20" max="255" step="5" value="230" oninput="previewCountdownBrightness()" onchange="setCountdownTuning()"></div></div></details>
        <details style="margin-top:12px;"><summary>Goal Wave Engine <span class="hint">direction + speed</span></summary><div class="inner"><div class="controlPanel"><div class="panelTop"><div class="panelTitle">Goal Travel Style</div><div class="panelValue" id="goalTravelModeValue">STRAIGHT</div></div><select id="goalTravelModeSelect" onchange="setGoalTiming()"><option value="straight">Straight Side-to-Side</option><option value="diagonal_up">Diagonal Up Sweep</option><option value="diagonal_down">Diagonal Down Sweep</option><option value="center_fan">Center Fan</option><option value="edge_fan">Edge Fan</option></select></div><div class="studioGrid" style="margin-top:12px;"><div class="controlPanel"><div class="panelTop"><div class="panelTitle">Total Duration</div><div class="panelValue" id="goalDurationValue">4200 ms</div></div><input id="goalDurationSlider" type="range" min="1200" max="9000" step="100" value="4200" oninput="previewGoalTiming()" onchange="setGoalTiming()"></div><div class="controlPanel"><div class="panelTop"><div class="panelTitle">Wave Travel</div><div class="panelValue" id="goalTravelValue">950 ms</div></div><input id="goalTravelSlider" type="range" min="250" max="2500" step="50" value="950" oninput="previewGoalTiming()" onchange="setGoalTiming()"></div><div class="controlPanel"><div class="panelTop"><div class="panelTitle">Wave Spacing</div><div class="panelValue" id="goalIntervalValue">260 ms</div></div><input id="goalIntervalSlider" type="range" min="80" max="900" step="20" value="260" oninput="previewGoalTiming()" onchange="setGoalTiming()"></div><div class="controlPanel"><div class="panelTop"><div class="panelTitle">Source Pop</div><div class="panelValue" id="goalPopValue">700 ms</div></div><input id="goalPopSlider" type="range" min="0" max="2000" step="50" value="700" oninput="previewGoalTiming()" onchange="setGoalTiming()"></div></div><div class="controlPanel" style="margin-top:12px;"><div class="panelTop"><div class="panelTitle">Wave Emit Length</div><div class="panelValue" id="goalEmitValue">2600 ms</div></div><input id="goalEmitSlider" type="range" min="400" max="7000" step="100" value="2600" oninput="previewGoalTiming()" onchange="setGoalTiming()"></div></div></details>
        <details style="margin-top:12px;"><summary>Finale / Winner Animation <span class="hint">rings + rainbow hex</span></summary><div class="inner"><div class="studioGrid"><div class="controlPanel"><div class="panelTop"><div class="panelTitle">Finale Duration</div><div class="panelValue" id="endDurationValue">9000 ms</div></div><input id="endDurationSlider" type="range" min="3000" max="20000" step="500" value="9000" oninput="previewEndTuning()" onchange="setEndTuning()"></div><div class="controlPanel"><div class="panelTop"><div class="panelTitle">Winner Ring Speed</div><div class="panelValue" id="endWinnerSpeedValue">95</div></div><input id="endWinnerSpeedSlider" type="range" min="5" max="255" step="5" value="95" oninput="previewEndTuning()" onchange="setEndTuning()"></div><div class="controlPanel"><div class="panelTop"><div class="panelTitle">Hex Rainbow Speed</div><div class="panelValue" id="endRainbowSpeedValue">60</div></div><input id="endRainbowSpeedSlider" type="range" min="5" max="255" step="5" value="60" oninput="previewEndTuning()" onchange="setEndTuning()"></div><div class="controlPanel"><div class="panelTop"><div class="panelTitle">Ring Spacing</div><div class="panelValue" id="endRingSpacingValue">14</div></div><input id="endRingSpacingSlider" type="range" min="6" max="40" step="1" value="14" oninput="previewEndTuning()" onchange="setEndTuning()"></div></div></div></details>
        <details style="margin-top:12px;"><summary>Settings Recipe <span class="hint">copy once it looks good</span></summary><div class="inner"><textarea id="settingsRecipe" class="recipeBox" readonly></textarea><div class="controlActions"><button class="scoreboardBtn" onclick="copySettingsRecipe()">Copy Settings</button><button onclick="buildSettingsRecipe(latestState)">Refresh Recipe</button></div></div></details>
        <details style="margin-top:12px;"><summary>Audio & LED Output <span class="hint">safe limits</span></summary><div class="inner"><div class="studioGrid"><div class="controlPanel"><div class="panelTop"><div class="panelTitle">Audio Volume</div><div class="panelValue" id="volumeValue">80%</div></div><input id="volumeSlider" type="range" min="0" max="100" value="80" oninput="previewVolume(this.value)" onchange="setVolume(this.value)"></div><div class="controlPanel"><div class="panelTop"><div class="panelTitle">LED Brightness</div><div class="panelValue" id="ledBrightnessValue">25%</div></div><input id="ledBrightnessSlider" type="range" min="0" max="50" value="25" oninput="previewLedBrightness(this.value)" onchange="setLedBrightness(this.value)"><div class="hint">Hard limit: 50%</div></div></div><div class="controlPanel" style="margin-top:12px;"><div class="panelTop"><div class="panelTitle">Manual Audio Check</div><div class="panelValue" id="currentTrack">Track 0</div></div><div class="controlActions"><button onclick="api('/api/play?track=1')">Start</button><button onclick="api('/api/play?track=2')">Blue Goal</button><button onclick="api('/api/play?track=3')">Orange Goal</button><button onclick="api('/api/play?track=7')">End</button></div></div></div></details>
      </div></div></div><div class="footer">RobotSoccer_CTRL · http://192.168.4.1 · LED safety: max 50%, no full-field white · UI refreshes every 350 ms</div></div>
<script>
let latestState=null;let scoreboardMode=false;function fmt(ms){ms=Math.max(0,Number(ms||0));const total=Math.ceil(ms/1000);const m=Math.floor(total/60);const s=total%60;return m+':'+String(s).padStart(2,'0')}function displayTimer(s){if(!s)return'0:00';if(s.gameState==='COUNTDOWN'&&Number(s.countdownValue||0)>0)return String(s.countdownValue);return fmt(s.matchRemainingMs)}function timerLabel(s){if(!s)return'Match Timer';if(s.gameState==='COUNTDOWN')return'Countdown';if(s.gameState==='END_GAME')return'Final';if(s.gameState==='IN_GAME')return'Match Timer';return'Ready'}function stateClass(state){if(state==='IN_GAME')return'stateChip live';if(state==='COUNTDOWN')return'stateChip countdown';if(state==='END_GAME')return'stateChip end';return'stateChip'}async function api(path){try{await fetch(path,{method:'POST'});await update()}catch(e){setOffline()}}function setMetric(id,text,cls){const el=document.getElementById(id);if(!el)return;el.textContent=text;el.className='metricValue '+(cls||'')}function pairText(a,b,armed){if(a&&b)return['BOTH DETECTED','warn'];if(a||b)return['PARTIAL','warn'];return[armed?'ARMED':'LOCKED','ok']}function sensorText(a,b){return(a?'A DETECTED':'A clear')+' / '+(b?'B DETECTED':'B clear')}function setOffline(){document.getElementById('topWifi').textContent='OFFLINE';setMetric('ledMode','OFFLINE','bad')}function enterScoreboardMode(){scoreboardMode=true;document.body.classList.add('scoreboardMode');if(document.documentElement.requestFullscreen){document.documentElement.requestFullscreen().catch(()=>{})}updateScoreboard(latestState)}function exitScoreboardMode(){scoreboardMode=false;document.body.classList.remove('scoreboardMode','winnerBlue','winnerOrange','winnerTie');if(document.fullscreenElement&&document.exitFullscreen){document.exitFullscreen().catch(()=>{})}}
function updateScoreboard(s){if(!s)return;document.getElementById('boardBlueScore').textContent=s.blueScore;document.getElementById('boardOrangeScore').textContent=s.orangeScore;document.getElementById('boardTimer').textContent=displayTimer(s);document.getElementById('boardClockLabel').textContent=timerLabel(s);document.getElementById('boardState').textContent=s.gameState;document.getElementById('boardLastEvent').textContent=s.lastEvent||'Ready';document.body.classList.remove('winnerBlue','winnerOrange','winnerTie');const banner=document.getElementById('boardWinnerBanner');if(s.gameState==='END_GAME'){if(s.winner==='blue'){document.body.classList.add('winnerBlue');banner.textContent='BLUE WINS'}else if(s.winner==='orange'){document.body.classList.add('winnerOrange');banner.textContent='ORANGE WINS'}else{document.body.classList.add('winnerTie');banner.textContent='TIE GAME'}}else{banner.textContent=''}}
let matchLengthDirty=false;function previewMatchLength(v){matchLengthDirty=true;document.getElementById('matchLengthValue').textContent=fmt(Number(v||0)*1000)}async function applyMatchLength(v){const sec=Math.trunc(Number(v||20));matchLengthDirty=false;await api('/api/match-length?seconds='+encodeURIComponent(sec))}function startCustomMatch(){const sec=Math.trunc(Number(document.getElementById('matchLengthSlider').value||20));matchLengthDirty=false;api('/api/start?seconds='+encodeURIComponent(sec))}function quickMatch(){matchLengthDirty=false;document.getElementById('matchLengthSlider').value=20;previewMatchLength(20);matchLengthDirty=false;api('/api/start?seconds=20')}
function setAnimationColors(){const blue=document.getElementById('blueGoalColorInput').value;const orange=document.getElementById('orangeGoalColorInput').value;const hex=document.getElementById('countdownHexColorInput').value;const field=document.getElementById('countdownFieldColorInput').value;const start=document.getElementById('gameStartColorInput').value;api('/api/colors?blue='+encodeURIComponent(blue)+'&orange='+encodeURIComponent(orange)+'&hex='+encodeURIComponent(hex)+'&field='+encodeURIComponent(field)+'&start='+encodeURIComponent(start))}
function previewCountdownStep(v){document.getElementById('countdownStepValue').textContent=Math.trunc(Number(v||0))+' ms'}function previewCountdownPeriod(v){document.getElementById('countdownPeriodValue').textContent=Math.trunc(Number(v||0))+' ms'}function previewCountdownBrightness(){const minB=Math.trunc(Number(document.getElementById('countdownMinSlider').value||0));const maxB=Math.trunc(Number(document.getElementById('countdownMaxSlider').value||0));const hexB=Math.trunc(Number(document.getElementById('countdownHexBrightnessSlider').value||0));document.getElementById('countdownBrightnessValue').textContent=minB+' / '+maxB+' / '+hexB}function setCountdownTuning(){const mode=document.getElementById('countdownEffectSelect').value;const step=document.getElementById('countdownStepSlider').value;const period=document.getElementById('countdownPeriodSlider').value;const minB=document.getElementById('countdownMinSlider').value;const maxB=document.getElementById('countdownMaxSlider').value;const hexB=document.getElementById('countdownHexBrightnessSlider').value;api('/api/countdown-tuning?mode='+encodeURIComponent(mode)+'&step='+encodeURIComponent(step)+'&period='+encodeURIComponent(period)+'&min='+encodeURIComponent(minB)+'&max='+encodeURIComponent(maxB)+'&hexBrightness='+encodeURIComponent(hexB))}
function previewGoalTiming(){document.getElementById('goalDurationValue').textContent=Math.trunc(Number(document.getElementById('goalDurationSlider').value||0))+' ms';document.getElementById('goalTravelValue').textContent=Math.trunc(Number(document.getElementById('goalTravelSlider').value||0))+' ms';document.getElementById('goalIntervalValue').textContent=Math.trunc(Number(document.getElementById('goalIntervalSlider').value||0))+' ms';document.getElementById('goalEmitValue').textContent=Math.trunc(Number(document.getElementById('goalEmitSlider').value||0))+' ms';document.getElementById('goalPopValue').textContent=Math.trunc(Number(document.getElementById('goalPopSlider').value||0))+' ms'}function setGoalTiming(){const mode=document.getElementById('goalTravelModeSelect').value;const duration=document.getElementById('goalDurationSlider').value;const travel=document.getElementById('goalTravelSlider').value;const interval=document.getElementById('goalIntervalSlider').value;const emit=document.getElementById('goalEmitSlider').value;const pop=document.getElementById('goalPopSlider').value;api('/api/goal-timing?mode='+encodeURIComponent(mode)+'&duration='+encodeURIComponent(duration)+'&travel='+encodeURIComponent(travel)+'&interval='+encodeURIComponent(interval)+'&emit='+encodeURIComponent(emit)+'&pop='+encodeURIComponent(pop))}
function previewEndTuning(){document.getElementById('endDurationValue').textContent=Math.trunc(Number(document.getElementById('endDurationSlider').value||0))+' ms';document.getElementById('endWinnerSpeedValue').textContent=Math.trunc(Number(document.getElementById('endWinnerSpeedSlider').value||0));document.getElementById('endRainbowSpeedValue').textContent=Math.trunc(Number(document.getElementById('endRainbowSpeedSlider').value||0));document.getElementById('endRingSpacingValue').textContent=Math.trunc(Number(document.getElementById('endRingSpacingSlider').value||0))}function setEndTuning(){const duration=document.getElementById('endDurationSlider').value;const winnerSpeed=document.getElementById('endWinnerSpeedSlider').value;const rainbowSpeed=document.getElementById('endRainbowSpeedSlider').value;const spacing=document.getElementById('endRingSpacingSlider').value;api('/api/end-tuning?duration='+encodeURIComponent(duration)+'&winnerSpeed='+encodeURIComponent(winnerSpeed)+'&rainbowSpeed='+encodeURIComponent(rainbowSpeed)+'&spacing='+encodeURIComponent(spacing))}
function previewVolume(v){document.getElementById('volumeValue').textContent=Math.trunc(Number(v||0))+'%'}function setVolume(v){v=Math.max(0,Math.min(100,Number(v||0)));api('/api/volume?level='+encodeURIComponent(v))}function previewLedBrightness(v){document.getElementById('ledBrightnessValue').textContent=Math.trunc(Number(v||0))+'%'}function setLedBrightness(v){v=Math.max(0,Math.min(50,Number(v||0)));api('/api/led-brightness?level='+encodeURIComponent(v))}
function buildSettingsRecipe(s){if(!s)return;const txt=['FIFA ROBOT SOCCER SETTINGS','matchSeconds='+Math.round((s.matchDurationMs||20000)/1000),'colors: blue='+s.blueGoalColor+' orange='+s.orangeGoalColor+' countdownHex='+s.countdownHexColor+' countdownField='+s.countdownFieldColor+' gameStart='+s.gameStartColor,'countdown: mode='+s.countdownEffectMode+' stepMs='+s.countdownStepMs+' fieldEffectMs='+s.countdownFieldEffectPeriodMs+' brightness='+s.countdownFieldMinBrightness+'/'+s.countdownFieldMaxBrightness+'/'+s.countdownHexBrightness,'goal: mode='+s.goalTravelMode+' durationMs='+s.goalEffectMs+' travelMs='+s.goalWaveTravelMs+' spacingMs='+s.goalWaveEmitIntervalMs+' emitMs='+s.goalWaveEmitEndMs+' popMs='+s.goalSourcePopMs,'finale: durationMs='+s.matchEndEffectMs+' winnerSpeed='+s.matchEndWinnerSpeed+' hexRainbowSpeed='+s.matchEndRainbowSpeed+' ringSpacing='+s.matchEndRingSpacing,'output: ledBrightnessPercent='+s.ledBrightness+' audioVolumePercent='+s.volume].join('\n');document.getElementById('settingsRecipe').value=txt}function copySettingsRecipe(){const box=document.getElementById('settingsRecipe');box.select();document.execCommand('copy')}
async function update(){try{const r=await fetch('/api/state',{cache:'no-store'});const s=await r.json();latestState=s;document.getElementById('topIp').textContent=s.ip;document.getElementById('topWifi').textContent=s.wifi?'READY':'OFF';document.getElementById('topLedMax').textContent=s.ledSafetyMaxPercent+'%';document.getElementById('blueScore').textContent=s.blueScore;document.getElementById('orangeScore').textContent=s.orangeScore;document.getElementById('timer').textContent=displayTimer(s);document.getElementById('clockLabel').textContent=timerLabel(s);const chip=document.getElementById('stateChip');chip.textContent=s.gameState;chip.className=stateClass(s.gameState);const left=pairText(s.leftSensorA,s.leftSensorB,s.leftGoalArmed);const right=pairText(s.rightSensorA,s.rightSensorB,s.rightGoalArmed);setMetric('leftPair',left[0],left[1]);setMetric('rightPair',right[0],right[1]);setMetric('leftSensors',sensorText(s.leftSensorA,s.leftSensorB),(s.leftSensorA||s.leftSensorB)?'warn':'ok');setMetric('rightSensors',sensorText(s.rightSensorA,s.rightSensorB),(s.rightSensorA||s.rightSensorB)?'warn':'ok');setMetric('busy',s.audioBusy?'PLAYING':'IDLE',s.audioBusy?'warn':'ok');setMetric('ledMode',s.ledMode||'---','blueText');setMetric('rejects',String((s.leftGoalRejectedSingles||0)+(s.rightGoalRejectedSingles||0)),'warn');setMetric('lastEvent',s.lastEvent||'Ready','');document.getElementById('currentTrack').textContent='Track '+s.currentTrack;
if(typeof s.matchDurationMs!=='undefined'&&!matchLengthDirty&&document.activeElement!==document.getElementById('matchLengthSlider')){const sec=Math.round(s.matchDurationMs/1000);document.getElementById('matchLengthSlider').value=sec;document.getElementById('matchLengthValue').textContent=fmt(sec*1000)}if(typeof s.volume!=='undefined'){document.getElementById('volumeSlider').value=s.volume;previewVolume(s.volume)}if(typeof s.ledBrightness!=='undefined'){document.getElementById('ledBrightnessSlider').value=s.ledBrightness;previewLedBrightness(s.ledBrightness)}if(s.blueGoalColor)document.getElementById('blueGoalColorInput').value=s.blueGoalColor;if(s.orangeGoalColor)document.getElementById('orangeGoalColorInput').value=s.orangeGoalColor;if(s.countdownHexColor)document.getElementById('countdownHexColorInput').value=s.countdownHexColor;if(s.countdownFieldColor)document.getElementById('countdownFieldColorInput').value=s.countdownFieldColor;if(s.gameStartColor)document.getElementById('gameStartColorInput').value=s.gameStartColor;if(typeof s.countdownEffectMode!=='undefined'){const mode=String(s.countdownEffectMode).toLowerCase();document.getElementById('countdownEffectSelect').value=mode==='chasing_wave'?'wave':mode;document.getElementById('countdownEffectValue').textContent=s.countdownEffectMode}if(typeof s.countdownStepMs!=='undefined'){document.getElementById('countdownStepSlider').value=s.countdownStepMs;previewCountdownStep(s.countdownStepMs)}if(typeof s.countdownFieldEffectPeriodMs!=='undefined'){document.getElementById('countdownPeriodSlider').value=s.countdownFieldEffectPeriodMs;previewCountdownPeriod(s.countdownFieldEffectPeriodMs)}if(typeof s.countdownFieldMinBrightness!=='undefined')document.getElementById('countdownMinSlider').value=s.countdownFieldMinBrightness;if(typeof s.countdownFieldMaxBrightness!=='undefined')document.getElementById('countdownMaxSlider').value=s.countdownFieldMaxBrightness;if(typeof s.countdownHexBrightness!=='undefined')document.getElementById('countdownHexBrightnessSlider').value=s.countdownHexBrightness;previewCountdownBrightness();if(typeof s.goalTravelMode!=='undefined'){document.getElementById('goalTravelModeSelect').value=s.goalTravelMode;document.getElementById('goalTravelModeValue').textContent=String(s.goalTravelMode).toUpperCase()}if(typeof s.goalEffectMs!=='undefined')document.getElementById('goalDurationSlider').value=s.goalEffectMs;if(typeof s.goalWaveTravelMs!=='undefined')document.getElementById('goalTravelSlider').value=s.goalWaveTravelMs;if(typeof s.goalWaveEmitIntervalMs!=='undefined')document.getElementById('goalIntervalSlider').value=s.goalWaveEmitIntervalMs;if(typeof s.goalWaveEmitEndMs!=='undefined')document.getElementById('goalEmitSlider').value=s.goalWaveEmitEndMs;if(typeof s.goalSourcePopMs!=='undefined')document.getElementById('goalPopSlider').value=s.goalSourcePopMs;previewGoalTiming();if(typeof s.matchEndEffectMs!=='undefined')document.getElementById('endDurationSlider').value=s.matchEndEffectMs;if(typeof s.matchEndWinnerSpeed!=='undefined')document.getElementById('endWinnerSpeedSlider').value=s.matchEndWinnerSpeed;if(typeof s.matchEndRainbowSpeed!=='undefined')document.getElementById('endRainbowSpeedSlider').value=s.matchEndRainbowSpeed;if(typeof s.matchEndRingSpacing!=='undefined')document.getElementById('endRingSpacingSlider').value=s.matchEndRingSpacing;previewEndTuning();const log=document.getElementById('eventLog');if(Array.isArray(s.eventLog)){log.innerHTML=s.eventLog.slice().reverse().map(e=>'<div class="logItem">'+String(e).replace(/[&<>]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;'}[c]))+'</div>').join('')}buildSettingsRecipe(s);updateScoreboard(s)}catch(e){setOffline()}}
setInterval(update,350);update();
</script>
</body>
</html>
)rawliteral";

// ============================================================
// WEB HANDLERS
// ============================================================

static void sendNoCache() {
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  server.sendHeader("Pragma", "no-cache");
  server.sendHeader("Expires", "0");
}

static void handleRoot() {
  sendNoCache();
  server.send_P(200, "text/html", INDEX_HTML);
}

static void handleState() {
  updateAudioState();
  sendNoCache();
  server.send(200, "application/json", stateJson());
}

static unsigned long clampMatchSeconds(unsigned long seconds);

static void handleStart() {
  unsigned long seconds = DEFAULT_MATCH_DURATION_SECONDS;

  if (server.hasArg("seconds")) {
    seconds = (unsigned long)server.arg("seconds").toInt();
  }

  seconds = clampMatchSeconds(seconds);
  startMatchWithDuration(seconds);
  server.send(200, "application/json", stateJson());
}


static unsigned long clampMatchSeconds(unsigned long seconds) {
  if (seconds < 10UL) seconds = 10UL;
  if (seconds > 600UL) seconds = 600UL;
  return seconds;
}

static void handleMatchLength() {
  unsigned long seconds = DEFAULT_MATCH_DURATION_SECONDS;

  if (server.hasArg("seconds")) {
    seconds = (unsigned long)server.arg("seconds").toInt();
  }

  seconds = clampMatchSeconds(seconds);
  matchDurationMs = seconds * 1000UL;

  String msg = "Match length set to ";
  msg += String(seconds);
  msg += " seconds";
  addEvent(msg);

  // If a live match is shortened below elapsed time, end it cleanly.
  if (gameState == STATE_IN_GAME && getMatchRemainingMs() == 0) {
    endMatch(false);
  }

  server.send(200, "application/json", stateJson());
}

static void handleReset() {
  resetGame();
  server.send(200, "application/json", stateJson());
}

static void handleEnd() {
  endMatch(true);
  server.send(200, "application/json", stateJson());
}

static void handleFakeBlueGoal() {
  handleBlueGoal(true);
  server.send(200, "application/json", stateJson());
}

static void handleFakeOrangeGoal() {
  handleOrangeGoal(true);
  server.send(200, "application/json", stateJson());
}

static void handleFakeHit() {
  handleBallHit(false);
  server.send(200, "application/json", stateJson());
}

static void handleFakeBigHit() {
  handleBallHit(true);
  server.send(200, "application/json", stateJson());
}

static void handlePlay() {
  if (server.hasArg("track")) {
    uint16_t track = (uint16_t)server.arg("track").toInt();

    if (track > 0) {
      audioPlayTrack(track);
      String msg = "Manual audio track ";
      msg += String(track);
      addEvent(msg);
    }
  }

  server.send(200, "application/json", stateJson());
}

static void handleStopAudio() {
  audioStopCommon();
  addEvent("Audio stopped");
  server.send(200, "application/json", stateJson());
}

static void handleVolume() {
  if (server.hasArg("level")) {
    int requestedVolume = server.arg("level").toInt();

    if (requestedVolume < 0) requestedVolume = 0;
    if (requestedVolume > 100) requestedVolume = 100;

    audioSetVolumePercent((uint8_t)requestedVolume);
  }

  server.send(200, "application/json", stateJson());
}

static void handleLedBrightness() {
  if (server.hasArg("level")) {
    int requestedBrightness = server.arg("level").toInt();

    if (requestedBrightness < 0) requestedBrightness = 0;
    if (requestedBrightness > LED_BRIGHTNESS_MAX_PERCENT) {
      requestedBrightness = LED_BRIGHTNESS_MAX_PERCENT;
    }

    setLedBrightnessPercent((uint8_t)requestedBrightness);
  }

  server.send(200, "application/json", stateJson());
}

static void handleColors() {
  if (server.hasArg("blue")) {
    blueGoalColor = parseWebHexColor(server.arg("blue"), DEFAULT_BLUE_GOAL_COLOR);
  }

  if (server.hasArg("orange")) {
    orangeGoalColor = parseWebHexColor(server.arg("orange"), DEFAULT_ORANGE_GOAL_COLOR);
  }

  if (server.hasArg("hex")) {
    countdownHexColor = parseWebHexColor(server.arg("hex"), DEFAULT_COUNTDOWN_HEX_COLOR);
  }

  if (server.hasArg("field")) {
    countdownFieldColor = parseWebHexColor(server.arg("field"), DEFAULT_COUNTDOWN_FIELD_COLOR);
  }

  if (server.hasArg("start")) {
    gameStartColor = parseWebHexColor(server.arg("start"), DEFAULT_GAME_START_COLOR);
  }

  addEvent("Animation colors updated");
  server.send(200, "application/json", stateJson());
}

static void handleCountdownTuning() {
  if (server.hasArg("mode")) {
    countdownFieldEffectMode = parseCountdownFieldEffectMode(server.arg("mode"));
  }

  if (server.hasArg("step")) {
    countdownStepMs = clampUL((unsigned long)server.arg("step").toInt(), COUNTDOWN_STEP_MS_MIN, COUNTDOWN_STEP_MS_MAX);
  }

  if (server.hasArg("period")) {
    countdownFieldEffectPeriodMs = clampUL((unsigned long)server.arg("period").toInt(), COUNTDOWN_FIELD_EFFECT_PERIOD_MIN, COUNTDOWN_FIELD_EFFECT_PERIOD_MAX);
  }

  if (server.hasArg("min")) {
    countdownFieldMinBrightness = clampBrightnessValue(server.arg("min").toInt());
  }

  if (server.hasArg("max")) {
    countdownFieldMaxBrightness = clampBrightnessValue(server.arg("max").toInt());
  }

  if (server.hasArg("hexBrightness")) {
    countdownHexBrightness = clampBrightnessValue(server.arg("hexBrightness").toInt());
  }

  addEvent("Countdown tuning updated");
  server.send(200, "application/json", stateJson());
}

static void handleGoalTiming() {
  if (server.hasArg("mode")) {
    goalTravelMode = parseGoalTravelMode(server.arg("mode"));
  }

  if (server.hasArg("duration")) {
    goalEffectMs = clampUL((unsigned long)server.arg("duration").toInt(), 1200UL, 9000UL);
  }

  if (server.hasArg("travel")) {
    goalWaveTravelMs = clampUL((unsigned long)server.arg("travel").toInt(), 250UL, 2500UL);
  }

  if (server.hasArg("interval")) {
    goalWaveEmitIntervalMs = clampUL((unsigned long)server.arg("interval").toInt(), 80UL, 900UL);
  }

  if (server.hasArg("emit")) {
    goalWaveEmitEndMs = clampUL((unsigned long)server.arg("emit").toInt(), 400UL, 7000UL);
  }

  if (server.hasArg("pop")) {
    goalSourcePopMs = clampUL((unsigned long)server.arg("pop").toInt(), 0UL, 2000UL);
  }

  // Keep total duration long enough for the emitted waves to finish.
  unsigned long minimumDuration = goalWaveEmitEndMs + goalWaveTravelMs + 300UL;
  if (goalEffectMs < minimumDuration) {
    goalEffectMs = minimumDuration;
  }

  addEvent("Goal timing updated");
  server.send(200, "application/json", stateJson());
}

static void handleEndTuning() {
  if (server.hasArg("duration")) {
    matchEndEffectMs = clampUL((unsigned long)server.arg("duration").toInt(), MATCH_END_EFFECT_MS_MIN, MATCH_END_EFFECT_MS_MAX);
  }

  if (server.hasArg("winnerSpeed")) {
    matchEndWinnerSpeed = clampU8(server.arg("winnerSpeed").toInt());
  }

  if (server.hasArg("rainbowSpeed")) {
    matchEndRainbowSpeed = clampU8(server.arg("rainbowSpeed").toInt());
  }

  if (server.hasArg("spacing")) {
    int requestedSpacing = server.arg("spacing").toInt();
    if (requestedSpacing < 6) requestedSpacing = 6;
    if (requestedSpacing > 40) requestedSpacing = 40;
    matchEndRingSpacing = (uint8_t)requestedSpacing;
  }

  addEvent("Finale tuning updated");
  server.send(200, "application/json", stateJson());
}

static void handleCenterShift() {
  float x = centerXShift;
  float y = centerYShift;
  int16_t idx = centerLedIndexShift;

  if (server.hasArg("x")) x = server.arg("x").toFloat();
  if (server.hasArg("y")) y = server.arg("y").toFloat();
  if (server.hasArg("index")) idx = (int16_t)server.arg("index").toInt();

  setCenterShift(x, y, idx);
  addEvent("Center shift updated");

  server.send(200, "application/json", stateJson());
}

static void handleCenterTest() {
  setLedMode(LED_MODE_CENTER_TEST);
  addEvent("Center test enabled");
  server.send(200, "application/json", stateJson());
}

static void handleBlackout() {
  setLedMode(LED_MODE_BLACKOUT);
  clearFieldLeds();
  addEvent("LED blackout");
  server.send(200, "application/json", stateJson());
}

static void handleNotFound() {
  server.send(404, "text/plain", "Not found");
}

static void beginWeb() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/state", HTTP_GET, handleState);

  server.on("/api/start", HTTP_POST, handleStart);
  server.on("/api/match-length", HTTP_POST, handleMatchLength);
  server.on("/api/reset", HTTP_POST, handleReset);
  server.on("/api/end", HTTP_POST, handleEnd);

  server.on("/api/fake-blue-goal", HTTP_POST, handleFakeBlueGoal);
  server.on("/api/fake-orange-goal", HTTP_POST, handleFakeOrangeGoal);
  server.on("/api/fake-hit", HTTP_POST, handleFakeHit);
  server.on("/api/fake-big-hit", HTTP_POST, handleFakeBigHit);

  server.on("/api/play", HTTP_POST, handlePlay);
  server.on("/api/stop-audio", HTTP_POST, handleStopAudio);
  server.on("/api/volume", HTTP_POST, handleVolume);

  server.on("/api/led-brightness", HTTP_POST, handleLedBrightness);
  server.on("/api/colors", HTTP_POST, handleColors);
  server.on("/api/countdown-tuning", HTTP_POST, handleCountdownTuning);
  server.on("/api/goal-timing", HTTP_POST, handleGoalTiming);
  server.on("/api/end-tuning", HTTP_POST, handleEndTuning);
  server.on("/api/center-shift", HTTP_POST, handleCenterShift);
  server.on("/api/center-test", HTTP_POST, handleCenterTest);
  server.on("/api/blackout", HTTP_POST, handleBlackout);

  server.onNotFound(handleNotFound);

  server.begin();

  Serial.println("[INIT] Web server started.");
}

// ============================================================
// WIFI AND OTA
// ============================================================

static void setupWiFi() {
  Serial.println();

#if DEMO_USE_ACCESS_POINT
  Serial.println("[INIT] Starting Wi-Fi access point...");
  Serial.print("[INIT] SSID: ");
  Serial.println(AP_SSID);

  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);

  bool ok = WiFi.softAP(AP_SSID, AP_PASSWORD);

  if (!ok) {
    Serial.println("[ERROR] Failed to start access point.");
    wifiReady = false;
    return;
  }

  wifiReady = true;

  Serial.print("[INIT] AP IP address: ");
  Serial.println(WiFi.softAPIP());

#else
  Serial.println("[INIT] Connecting to Wi-Fi...");
  Serial.print("[INIT] SSID: ");
  Serial.println(STA_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  if (!WiFi.config(sta_local_IP, sta_gateway, sta_subnet)) {
    Serial.println("[WARN] Static IP configuration failed.");
  }

  WiFi.begin(STA_SSID, STA_PASSWORD);

  int attempts = 0;

  while (WiFi.status() != WL_CONNECTED && attempts < 60) {
    Serial.print(".");
    delay(500);
    attempts++;
  }

  Serial.println();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[ERROR] Wi-Fi failed to connect.");
    wifiReady = false;
    return;
  }

  wifiReady = true;

  Serial.println("[INIT] Wi-Fi connected.");
  Serial.print("[INIT] IP address: ");
  Serial.println(WiFi.localIP());
#endif
}

static void setupOTA() {
  if (!wifiReady) {
    Serial.println("[OTA] Skipped because Wi-Fi is not ready.");
    otaReady = false;
    return;
  }

  ArduinoOTA.setHostname(OTA_HOSTNAME);

  ArduinoOTA.onStart([]() {
    otaInProgress = true;
    Serial.println("[OTA] Update started.");
    showSolidFieldColor(CRGB(255, 90, 0));
  });

  ArduinoOTA.onEnd([]() {
    Serial.println();
    Serial.println("[OTA] Update finished.");
    otaInProgress = false;
    clearFieldLeds();
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    unsigned int percent = (progress * 100) / total;

    static unsigned int lastPrintedPercent = 999;

    if (percent != lastPrintedPercent && percent % 10 == 0) {
      Serial.printf("[OTA] Progress: %u%%\n", percent);
      lastPrintedPercent = percent;
    }
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("\n[OTA] Error [%u]\n", error);
    otaInProgress = false;
    showSolidFieldColor(CRGB(180, 0, 0));
  });

  ArduinoOTA.begin();
  otaReady = true;

  Serial.println("[OTA] Ready.");
  Serial.print("[OTA] Hostname: ");
  Serial.println(OTA_HOSTNAME);
}

// ============================================================
// SERIAL FALLBACK
// ============================================================

static void serviceSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c >= '1' && c <= '8') {
      audioPlayTrack((uint16_t)(c - '0'));
    } else if (c == 's' || c == 'S') {
      startMatchWithDuration(DEFAULT_MATCH_DURATION_SECONDS);
    } else if (c == 't' || c == 'T') {
      startMatchWithDuration(QUICK_MATCH_DURATION_SECONDS);
    } else if (c == 'e' || c == 'E') {
      endMatch(true);
    } else if (c == '0') {
      resetGame();
    } else if (c == 'b') {
      handleBlueGoal(true);
    } else if (c == 'o') {
      handleOrangeGoal(true);
    } else if (c == 'h') {
      handleBallHit(false);
    } else if (c == 'H') {
      handleBallHit(true);
    } else if (c == 'q' || c == 'Q') {
      audioStopCommon();
    } else if (c == '+') {
      if (currentVolumePercent <= 95) {
        audioSetVolumePercent(currentVolumePercent + 5);
      } else {
        audioSetVolumePercent(100);
      }
    } else if (c == '-') {
      if (currentVolumePercent >= 5) {
        audioSetVolumePercent(currentVolumePercent - 5);
      } else {
        audioSetVolumePercent(0);
      }
    } else if (c == 'c' || c == 'C') {
      setLedMode(LED_MODE_CENTER_TEST);
    } else if (c == 'x') {
      setCenterShift(centerXShift - 0.5f, centerYShift, centerLedIndexShift);
    } else if (c == 'X') {
      setCenterShift(centerXShift + 0.5f, centerYShift, centerLedIndexShift);
    } else if (c == 'y') {
      setCenterShift(centerXShift, centerYShift - 0.5f, centerLedIndexShift);
    } else if (c == 'Y') {
      setCenterShift(centerXShift, centerYShift + 0.5f, centerLedIndexShift);
    } else if (c == 'i' || c == 'I') {
      Serial.println("[CENTER] Index shift is disabled. Use X/x for virtual center alignment.");
    } else if (c == 'z' || c == 'Z') {
      setCenterShift(0.0f, CENTER_Y_SHIFT_DEFAULT, CENTER_LED_INDEX_SHIFT_DEFAULT);
    }
  }
}

// ============================================================
// SETUP / LOOP
// ============================================================

void setup() {
  Serial.begin(115200);
  delay(300);

  randomSeed((uint32_t)micros());

  Serial.println();
  Serial.println("==================================================");
  Serial.println("FIFA Robot Soccer Demo Controller");
  Serial.println("Scoreboard + 4 Sensors + DY-HV20T + Spatial LEDs");
  Serial.println("LED SAFETY: 50% max brightness, full-white blocked");
  Serial.println("==================================================");

  FastLED.addLeds<CHIPSET, LED_PIN, COLOR_ORDER>(leds, LED_COUNT);

  enforceLedBrightnessLimit();
  FastLED.clear();
  safeFastLedShow();

  setLedBrightnessPercent(currentLedBrightnessPercent);

  buildXYMap();

  setupSensors();

  beginAudio();
  audioSetVolumePercent(currentVolumePercent);
  delay(100);

  setupWiFi();

  if (wifiReady) {
    beginWeb();
    setupOTA();

    Serial.println();
    Serial.println("[READY] Open this in your browser:");
#if DEMO_USE_ACCESS_POINT
    Serial.println("http://192.168.4.1");
#else
    Serial.print("http://");
    Serial.println(WiFi.localIP());
#endif
  }

  setLedMode(LED_MODE_ATTRACT);

  addEvent("System booted");

  Serial.println("[READY] Serial fallback:");
  Serial.println("  s   = start default 20 second match");
  Serial.println("  t   = quick 20 second test match");
  Serial.println("  e   = end match");
  Serial.println("  0   = reset");
  Serial.println("  b   = fake Blue goal");
  Serial.println("  o   = fake Orange goal");
  Serial.println("  h/H = hit / big hit");
  Serial.println("  1-8 = play audio track");
  Serial.println("  q   = stop audio");
  Serial.println("  c   = center test");
  Serial.println("  x/X = center X shift down/up");
  Serial.println("  y/Y = center Y coordinate shift down/up");
  Serial.println("  i/I = physical center LED index shift back/forward");
  Serial.println("  z   = reset center shift to demo defaults");
}

void loop() {
  if (otaReady) {
    ArduinoOTA.handle();
  }

  if (wifiReady) {
    server.handleClient();
  }

  updateAudioState();
  updateGame();
  updateSensorsAndGoals();
  updateLeds();

  serviceSerial();
  drainAudioRx();

  delay(2);
}
