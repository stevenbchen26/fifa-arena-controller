/*
  FIFA Robot Soccer Demo Controller v4.32
  LoR Core V3 / ESP32

  Stage:
  - Demo day integrated controller v3.3
  - Web color picker for goal/countdown/start LED animation colors
  - Countdown hex animation uses 3-second team-split fill plus white comet styles
  - Web UI supports effect test mode, last-10-second warning, and countdown/idle tuning
  - Goal/hit effects transition smoothly back into in-game idle instead of briefly blacking out
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
  - Router/STATION mode default:
      SSID: Soccerbots_Devices
      Password: soccerabcdefgh
      URL: shown in Serial Monitor and dashboard IP field after connection
      Optional mDNS hostname: http://soccerbots.local

  Important LoR Core V3 pin mapping:
  - Right goal sensor A:
      LoR IO_07 signal = ESP32 GPIO13
  - Right goal sensor B:
      LoR IO_10 signal = ESP32 GPIO4
  - Left goal sensor A:
      LoR IO_11 signal = ESP32 GPIO22
  - Left goal sensor B:
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
  - 00001.mp3 = optional pre-match / manual launch stinger
  - 00002.mp3 = full countdown sequence: 3 beeps plus higher GO tone
  - 00003.mp3 = first-half regular gameplay background music
  - 00004.mp3 = resume-after-goal cue
  - 00005.mp3 = second-half regular gameplay background music
  - 00006.mp3 = clean halftime / match-end horn cue
  - 00007.mp3 = clean loud goal celebration sound
  - 00008.mp3 = reset cue
  - 00009.mp3 = standby / idle attract ambience
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <FastLED.h>
#include <math.h>

// v4.32 compile fix:
// Arduino's auto-prototype generator can emit function prototypes before the
// interrupt latch struct definition. Forward-declare the struct so prototypes
// like "SensorInterruptLatch *sensorLatchForPin(...)" compile cleanly.
struct SensorInterruptLatch;

// ============================================================
// DEMO CONFIG
// ============================================================

// Demo network mode:
// 0 = connect to external router as a Wi-Fi client.
// 1 = create ESP32 access point. Current requested default.
#define DEMO_USE_ACCESS_POINT 1

#define AP_SSID       "RobotSoccer_BACKUP"
#define AP_PASSWORD   "fifabot123"

#define STA_SSID      "Soccerbots_Devices"
#define STA_PASSWORD  "soccerabcdefgh"

// Use DHCP by default so this works with any router subnet.
// Set to 1 only if you intentionally configure the router to use 192.168.8.x.
#define STA_USE_STATIC_IP 0
IPAddress sta_local_IP(192, 168, 8, 246);
IPAddress sta_gateway(192, 168, 8, 1);
IPAddress sta_subnet(255, 255, 255, 0);

#define OTA_HOSTNAME  "fifa-robot-soccer-demo"
#define MDNS_HOSTNAME "soccerbots"

// Default match length. Change this if you want a longer or shorter demo.
#define DEFAULT_MATCH_DURATION_SECONDS  90UL
#define QUICK_MATCH_DURATION_SECONDS    20UL
#define COUNTDOWN_SECONDS                3UL

// ============================================================
// PIN CONFIG
// ============================================================

// LoR IO slot to ESP32 GPIO mapping used here:
// IO_07 = GPIO13, IO_10 = GPIO4, IO_11 = GPIO22, IO_12 = GPIO21
// Right goal uses IO_07 + IO_10. Left goal uses IO_11 + IO_12.
#define LEFT_GOAL_SENSOR_A_PIN   22
#define LEFT_GOAL_SENSOR_B_PIN   21
#define RIGHT_GOAL_SENSOR_A_PIN  13
#define RIGHT_GOAL_SENSOR_B_PIN  4

#define MP3_RX_PIN              32
#define MP3_TX_PIN              25
#define MP3_BUSY_PIN            26
#define MP3_BAUD                9600

#define LED_PIN                 27  // LoR IO_04 signal / GPIO27, external field LEDs
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
#define GAME_START_GREEN_MS 850UL

// Countdown customization defaults.
// Countdown now uses a 3-second team-split hex fill with an optional white comet.
#define COUNTDOWN_STEP_MS_DEFAULT          1000UL
#define COUNTDOWN_STEP_MS_MIN             400UL
#define COUNTDOWN_STEP_MS_MAX             2000UL
#define COUNTDOWN_FIELD_EFFECT_PERIOD_DEFAULT 1000UL
#define COUNTDOWN_FIELD_EFFECT_PERIOD_MIN     80UL
#define COUNTDOWN_FIELD_EFFECT_PERIOD_MAX     2500UL
#define COUNTDOWN_FIELD_MIN_BRIGHTNESS_DEFAULT 5
#define COUNTDOWN_FIELD_MAX_BRIGHTNESS_DEFAULT 125
#define COUNTDOWN_HEX_BRIGHTNESS_DEFAULT       255

// Smooth hex countdown tuning. Default: one full rotation per countdown second.
#define COUNTDOWN_HEX_ROTATION_MS_DEFAULT      1000UL
#define COUNTDOWN_HEX_ROTATION_MS_MIN          150UL
#define COUNTDOWN_HEX_ROTATION_MS_MAX          2500UL
#define COUNTDOWN_HEX_TRAIL_PERCENT_DEFAULT    14
#define COUNTDOWN_HEX_TRAIL_PERCENT_MIN        8
#define COUNTDOWN_HEX_TRAIL_PERCENT_MAX        90
#define COUNTDOWN_COLOR_COUNT                  3

// Center line portion used during the hex countdown.
// This lights the two physical center strips only where they pass through the hex.
#define COUNTDOWN_CENTER_LINE_Y_MIN 18.0f
#define COUNTDOWN_CENTER_LINE_Y_MAX 43.0f
#define COUNTDOWN_ZONE_COUNT 5

// User-adjustable demo colors. These are intentionally not white.
#define DEFAULT_BLUE_GOAL_COLOR     CRGB(0, 80, 255)
#define DEFAULT_ORANGE_GOAL_COLOR   CRGB(255, 80, 0)
#define DEFAULT_COUNTDOWN_HEX_COLOR   CRGB(0, 255, 0)
#define DEFAULT_COUNTDOWN_FIELD_COLOR CRGB(71, 71, 71)
#define DEFAULT_GAME_START_COLOR      CRGB(0, 255, 0)

// Optional per-second accent colors for countdown 3, 2, 1.
// Team fill uses Blue/Orange team colors. These are used by alternate hex styles.
#define DEFAULT_COUNTDOWN_SECOND_3_COLOR CRGB(0, 255, 0)
#define DEFAULT_COUNTDOWN_SECOND_2_COLOR CRGB(255, 190, 0)
#define DEFAULT_COUNTDOWN_SECOND_1_COLOR CRGB(255, 80, 0)

// Blocks white/near-white user-selected colors. Manager rule: no all-white LED use.
#define USER_COLOR_WHITE_COMPONENT_LIMIT 185

// Last-10-second warning effect.
#define LAST_TEN_WARNING_ENABLED_DEFAULT 1
#define LAST_TEN_PULSE_COLOR_DEFAULT CRGB(152, 42, 188)
#define LAST_TEN_PULSE_PERIOD_DEFAULT 1000UL
#define LAST_TEN_PULSE_PERIOD_MIN     120UL
#define LAST_TEN_PULSE_PERIOD_MAX     2000UL
#define LAST_TEN_MIN_BRIGHTNESS_DEFAULT 75
#define LAST_TEN_MAX_BRIGHTNESS_DEFAULT 135

// In-game idle animation. This runs while no goal/hit/end effect is active.
#define IN_GAME_IDLE_SPEED_DEFAULT      37
#define IN_GAME_IDLE_SPEED_MIN          1
#define IN_GAME_IDLE_SPEED_MAX          40
#define IN_GAME_IDLE_MIN_BRIGHTNESS_DEFAULT 10
#define IN_GAME_IDLE_MAX_BRIGHTNESS_DEFAULT 255


// ============================================================
// SENSOR CONFIG
// ============================================================

// v4.28 wiring note:
// Right goal sensors are on IO_07 + IO_10.
// Left goal sensors are on IO_11 + IO_12.
// On this LoR Core mapping that means GPIO13, GPIO4, GPIO22, GPIO21.
// GPIO4 is an ESP32 boot-strapping pin, so if the board ever fails
// to boot with sensors connected, check whether the sensor on IO_10 is forcing
// that pin to the wrong level during reset/power-up.

#define SENSOR_ACTIVE_LOW       1

// v4.31:
// Interrupts catch the pin change immediately.
// Debounce is now shorter because the ISR is only a wake-up/latch,
// while dual-sensor validation still protects against single-sensor false goals.
#define SENSOR_DEBOUNCE_MS     15UL

// Set to 0 if you want to fall back to loop-only polling.
#define USE_SENSOR_INTERRUPTS  1

// Tiny ISR noise filter. This is not the main debounce.
// It only prevents extremely fast electrical chatter from flooding the latch.
#define SENSOR_INTERRUPT_MIN_GAP_US 250UL

// If enabled, any interrupt that sees the goal-trigger state is allowed to
// update the debounced state immediately in the main loop.
// With GOAL_TRIGGER_ON_CLEAR = 1, that means a no-detect pulse can score fast.
#define SENSOR_INTERRUPT_FAST_EDGE 1

// Goal trigger mode:
// 0 = normal mode: goal counts when sensors DETECT an object.
// 1 = mounted-reflection mode: goal counts when sensors STOP detecting.
// v4.31 keeps mode 1 because the mounted sensors are normally ON with no ball.
#define GOAL_TRIGGER_ON_CLEAR  1

// Dual-sensor validation:
// A goal only counts when BOTH sensors on the same goal confirm within this window.
// In v4.31, confirm means both sensors go from detected to clear / no-detect.
// Raise this if the two sensors are physically far apart along the ball path.
// Lower this if noise is still causing false validations.
#define DUAL_SENSOR_CONFIRM_WINDOW_MS 450UL

// Rearm delay:
// In normal mode, both sensors must be clear before rearming.
// In v4.31 mounted-reflection mode, both sensors must return to detected before rearming.
// This prevents one stuck/noisy sensor from false-scoring forever.
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

const uint16_t MAX_TRACK = 9;

// Audio track assignment for the SD card:
//   00001.mp3 = optional pre-match / manual launch stinger
//   00002.mp3 = full countdown sequence: 3 beeps plus higher GO tone
//   00003.mp3 = first-half regular gameplay background music
//   00004.mp3 = resume-after-goal cue
//   00005.mp3 = second-half regular gameplay background music
//   00006.mp3 = clean halftime / match-end horn cue
//   00007.mp3 = clean loud goal celebration sound
//   00008.mp3 = reset / system clear cue
//   00009.mp3 = standby / idle attract ambience
//
// Note: DY-HV20T is a single-track module, so true BGM ducking/mixing is not
// possible in firmware. This version uses clean interrupt priority and delayed
// BGM restart. Audio fades are baked into the MP3 files, not performed by firmware.
static const uint16_t AUDIO_TRACK_GAME_START       = 1;
static const uint16_t AUDIO_TRACK_COUNTDOWN        = 2;
static const uint16_t AUDIO_TRACK_FIRST_HALF_BGM   = 3;
static const uint16_t AUDIO_TRACK_RESUME_PLAY      = 4;
static const uint16_t AUDIO_TRACK_SECOND_HALF_BGM  = 5;
static const uint16_t AUDIO_TRACK_HALFTIME_END     = 6;
static const uint16_t AUDIO_TRACK_GOAL             = 7;
static const uint16_t AUDIO_TRACK_RESET            = 8;
static const uint16_t AUDIO_TRACK_STANDBY_IDLE     = 9;

// Estimated lengths are only for the web progress bar and background-music resume timing.
// BUSY pin still shows the module's real playback state.
const unsigned long TRACK_LENGTH_MS[MAX_TRACK + 1] = {
  0,
  1600,     // 1 optional pre-match / manual launch stinger
  3400,     // 2 full countdown sequence: 3 beeps plus GO tone
  90000,    // 3 first-half gameplay background music
  1250,     // 4 resume-after-goal cue
  90000,    // 5 second-half gameplay background music
  2800,     // 6 clean halftime / match-end horn cue
  5200,     // 7 clean loud goal celebration sound
  900,      // 8 reset / system clear cue
  60000     // 9 standby / idle attract ambience
};

// Audio command timing guards. These small delays make the DY-HV20T less likely
// to ignore rapid STOP -> PLAY changes during countdown and goal events.
static const unsigned long AUDIO_COMMAND_SETTLE_MS = 150UL;
static const unsigned long AUDIO_HARD_STOP_GAP_MS = 90UL;
static const unsigned long GAME_START_TO_BGM_DELAY_MS = 650UL;
static const unsigned long GOAL_RESUME_TO_BGM_DELAY_MS = 1500UL;

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
  LED_MODE_LAST_TEN_TEST,
  LED_MODE_LED_SELF_TEST,
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

enum CountdownHexAnimationMode {
  COUNTDOWN_HEX_ORBIT_GRADIENT = 0,
  COUNTDOWN_HEX_COMET,
  COUNTDOWN_HEX_DUAL_COMET,
  COUNTDOWN_HEX_FULL_GRADIENT,
  COUNTDOWN_HEX_THEATER_CHASE,
  COUNTDOWN_HEX_SPARKLE_ORBIT,
  COUNTDOWN_HEX_SOLID_STEP,
  COUNTDOWN_HEX_TEAM_FILL_COMET,
  COUNTDOWN_HEX_TEAM_FILL_ONLY,
  COUNTDOWN_HEX_TEAM_SPLIT_PULSE,
  COUNTDOWN_HEX_COMET_ONLY
};

CountdownHexAnimationMode countdownHexAnimationMode = COUNTDOWN_HEX_TEAM_FILL_COMET;
unsigned long countdownHexRotationMs = COUNTDOWN_HEX_ROTATION_MS_DEFAULT;
uint8_t countdownHexTrailPercent = COUNTDOWN_HEX_TRAIL_PERCENT_DEFAULT;

CRGB countdownStepColors[COUNTDOWN_COLOR_COUNT] = {
  DEFAULT_COUNTDOWN_SECOND_3_COLOR,
  DEFAULT_COUNTDOWN_SECOND_2_COLOR,
  DEFAULT_COUNTDOWN_SECOND_1_COLOR
};

CRGB lastTenPulseColor = LAST_TEN_PULSE_COLOR_DEFAULT;
bool lastTenWarningEnabled = LAST_TEN_WARNING_ENABLED_DEFAULT;
unsigned long lastTenPulsePeriodMs = LAST_TEN_PULSE_PERIOD_DEFAULT;
uint8_t lastTenMinBrightness = LAST_TEN_MIN_BRIGHTNESS_DEFAULT;
uint8_t lastTenMaxBrightness = LAST_TEN_MAX_BRIGHTNESS_DEFAULT;

uint8_t inGameIdleSpeed = IN_GAME_IDLE_SPEED_DEFAULT;
uint8_t inGameIdleMinBrightness = IN_GAME_IDLE_MIN_BRIGHTNESS_DEFAULT;
uint8_t inGameIdleMaxBrightness = IN_GAME_IDLE_MAX_BRIGHTNESS_DEFAULT;

bool effectTestMode = false;

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
  STATE_GOAL_PAUSE,
  STATE_HALFTIME,
  STATE_END_GAME
};

GameState gameState = STATE_STANDBY;

uint16_t blueScore = 0;
uint16_t orangeScore = 0;

// Two-half match flow:
// Half 1 runs for matchDurationMs and uses 00003.mp3 as gameplay music.
// After half 1 ends, the controller pauses at HALFTIME and waits for
// Resume Second Half. Half 2 then uses 00005.mp3 as gameplay music.
uint8_t currentHalf = 1;
bool firstHalfComplete = false;
bool secondHalfComplete = false;

unsigned long matchDurationMs = DEFAULT_MATCH_DURATION_SECONDS * 1000UL;
unsigned long countdownStartMs = 0;
unsigned long matchStartMs = 0;
unsigned long matchEndMs = 0;
unsigned long pausedMatchRemainingMs = DEFAULT_MATCH_DURATION_SECONDS * 1000UL;
unsigned long goalPauseStartMs = 0;

// Goal pause behavior:
// Manual mode keeps the match paused until the operator presses Resume Match After Goal.
// Auto mode resumes after the configured delay while still allowing the operator to resume early.
bool autoResumeAfterGoalEnabled = false;
unsigned long autoResumeAfterGoalDelayMs = 5000UL;

int lastCountdownAnnounced = -1;
unsigned long lastCountdownBeepMs = 0;

// v4.24: 00002.mp3 contains the complete countdown:
// beep, beep, beep, then a higher-pitched GO tone.
// The firmware starts track 2 once at countdown start.
bool countdownSequenceStarted = false;

// Background music is intentionally gated so it cannot sneak into countdown,
// goal pause, or one-shot sound effects.
unsigned long bgmAllowedAfterMs = 0;
uint16_t lastRequestedBgmTrack = 0;

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
  if (gameState == STATE_GOAL_PAUSE) return "GOAL_PAUSE";
  if (gameState == STATE_HALFTIME) return "HALFTIME";
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
  if (ledMode == LED_MODE_LAST_TEN_TEST) return "LAST_TEN_TEST";
  if (ledMode == LED_MODE_LED_SELF_TEST) return "LED_SELF_TEST";
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

static String countdownHexStyleName() {
  if (countdownHexAnimationMode == COUNTDOWN_HEX_ORBIT_GRADIENT) return "ORBIT_GRADIENT";
  if (countdownHexAnimationMode == COUNTDOWN_HEX_COMET) return "COMET";
  if (countdownHexAnimationMode == COUNTDOWN_HEX_DUAL_COMET) return "DUAL_COMET";
  if (countdownHexAnimationMode == COUNTDOWN_HEX_FULL_GRADIENT) return "FULL_GRADIENT";
  if (countdownHexAnimationMode == COUNTDOWN_HEX_THEATER_CHASE) return "THEATER_CHASE";
  if (countdownHexAnimationMode == COUNTDOWN_HEX_SPARKLE_ORBIT) return "SPARKLE_ORBIT";
  if (countdownHexAnimationMode == COUNTDOWN_HEX_SOLID_STEP) return "SOLID_STEP";
  if (countdownHexAnimationMode == COUNTDOWN_HEX_TEAM_FILL_COMET) return "TEAM_FILL_COMET";
  if (countdownHexAnimationMode == COUNTDOWN_HEX_TEAM_FILL_ONLY) return "TEAM_FILL_ONLY";
  if (countdownHexAnimationMode == COUNTDOWN_HEX_TEAM_SPLIT_PULSE) return "TEAM_SPLIT_PULSE";
  if (countdownHexAnimationMode == COUNTDOWN_HEX_COMET_ONLY) return "COMET_ONLY";
  return "TEAM_FILL_COMET";
}

static CountdownHexAnimationMode parseCountdownHexStyle(String mode) {
  mode.trim();
  mode.toLowerCase();

  if (mode == "0" || mode == "orbit" || mode == "orbit_gradient") return COUNTDOWN_HEX_ORBIT_GRADIENT;
  if (mode == "1" || mode == "comet") return COUNTDOWN_HEX_COMET;
  if (mode == "2" || mode == "dual" || mode == "dual_comet") return COUNTDOWN_HEX_DUAL_COMET;
  if (mode == "3" || mode == "gradient" || mode == "full_gradient") return COUNTDOWN_HEX_FULL_GRADIENT;
  if (mode == "4" || mode == "theater" || mode == "theater_chase") return COUNTDOWN_HEX_THEATER_CHASE;
  if (mode == "5" || mode == "sparkle" || mode == "sparkle_orbit") return COUNTDOWN_HEX_SPARKLE_ORBIT;
  if (mode == "6" || mode == "solid" || mode == "solid_step") return COUNTDOWN_HEX_SOLID_STEP;
  if (mode == "team" || mode == "team_fill" || mode == "team_fill_comet") return COUNTDOWN_HEX_TEAM_FILL_COMET;
  if (mode == "team_fill_only") return COUNTDOWN_HEX_TEAM_FILL_ONLY;
  if (mode == "team_split_pulse") return COUNTDOWN_HEX_TEAM_SPLIT_PULSE;
  if (mode == "comet_only") return COUNTDOWN_HEX_COMET_ONLY;

  return COUNTDOWN_HEX_TEAM_FILL_COMET;
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

static void paintCenterStripsSplit(CRGB blueColor, CRGB orangeColor, uint8_t brightness);
static unsigned long getCountdownTotalMs();

static uint8_t aggressiveOneSecondPulse(unsigned long elapsed, unsigned long period, uint8_t lowB, uint8_t highB) {
  if (period < 1UL) period = 1UL;

  float phase = (float)(elapsed % period) / (float)period;

  // Hard punch at the start of every second, then a fast falloff.
  float hit = 1.0f - phase;
  hit = hit * hit;

  // Add a small secondary kick so the pulse feels alive, not like a fading lamp.
  float secondary = 0.0f;
  if (phase > 0.42f && phase < 0.58f) {
    float local = fabsf(phase - 0.50f) / 0.08f;
    secondary = 1.0f - clampFloat(local, 0.0f, 1.0f);
    secondary *= 0.35f;
  }

  float intensity = clampFloat(hit + secondary, 0.0f, 1.0f);
  int b = (int)((float)lowB + ((float)(highB - lowB) * intensity));
  return clampU8(b);
}

static unsigned long currentInGameRemainingMsForRender() {
  if (gameState != STATE_IN_GAME || matchStartMs == 0) return matchDurationMs;

  unsigned long elapsed = millis() - matchStartMs;
  if (elapsed >= matchDurationMs) return 0;
  return matchDurationMs - elapsed;
}

static void renderLastTenPulseFrame(unsigned long remainingMs, unsigned long now) {
  unsigned long elapsedInWindow = 10000UL - remainingMs;
  if (elapsedInWindow > 10000UL) elapsedInWindow = 10000UL;

  uint8_t brightness = aggressiveOneSecondPulse(elapsedInWindow, lastTenPulsePeriodMs, lastTenMinBrightness, lastTenMaxBrightness);

  // Fade the entire warning slightly as it approaches zero, but keep each second punch dramatic.
  uint8_t urgency = map((int)remainingMs, 0, 10000, 255, 160);
  brightness = scale8_video(brightness, urgency);

  for (uint16_t i = 0; i < LED_COUNT; i++) {
    float xNorm = clampFloat(pixelX[i] / FIELD_WIDTH, 0.0f, 1.0f);
    float yNorm = clampFloat(pixelY[i] / FIELD_HEIGHT, 0.0f, 1.0f);
    uint8_t shimmer = sin8((uint8_t)((now / 3UL) + (uint8_t)(xNorm * 110.0f) + (uint8_t)(yNorm * 70.0f)));
    uint8_t localB = clampU8((int)brightness + map(shimmer, 0, 255, -20, 28));
    leds[i] = scaledColor(lastTenPulseColor, localB);
  }

  // Keep the center hex extra readable during the final warning.
  if (remainingMs <= 3000UL) {
    uint8_t hexB = map((int)remainingMs, 0, 3000, 245, 175);
    for (uint16_t i = HEX_CENTER_START; i <= HEX_CENTER_END && i < LED_COUNT; i++) {
      leds[i] += scaledColor(lastTenPulseColor, hexB);
    }
  }
}

static void renderAttractOrInGame(bool inGame) {
  unsigned long now = millis();

  if (now - lastLedUpdateMs < LED_UPDATE_MS) return;
  lastLedUpdateMs = now;

  rainbowHueOffset += rainbowSpeed;

  if (inGame) {
    unsigned long remaining = currentInGameRemainingMsForRender();

    if (lastTenWarningEnabled && remaining > 0UL && remaining <= 10000UL) {
      renderLastTenPulseFrame(remaining, now);
      safeFastLedShow();
      return;
    }

    // Default in-game idle: blue side and orange side stay subtly alive.
    // This is intentionally calm so goals/countdown/finale still feel special.
    for (uint16_t i = 0; i < LED_COUNT; i++) {
      float xNorm = clampFloat(pixelX[i] / FIELD_WIDTH, 0.0f, 1.0f);
      float yNorm = clampFloat(pixelY[i] / FIELD_HEIGHT, 0.0f, 1.0f);

      bool blueSide = pixelX[i] < virtualCenterX();
      CRGB baseColor = blueSide ? blueGoalColor : orangeGoalColor;

      uint8_t wave = sin8((uint8_t)(rainbowHueOffset * inGameIdleSpeed + (uint8_t)(xNorm * 130.0f) + (uint8_t)(yNorm * 70.0f)));
      uint8_t brightness = map(wave, 0, 255, inGameIdleMinBrightness, inGameIdleMaxBrightness);

      // Slight midfield breathing so both center strips read as active but not distracting.
      float centerDistance = fabsf(pixelX[i] - virtualCenterX()) / 10.0f;
      if (centerDistance < 1.0f) {
        uint8_t centerBoost = (uint8_t)((1.0f - centerDistance) * 24.0f);
        brightness = clampU8((int)brightness + centerBoost);
      }

      leds[i] = scaledColor(baseColor, brightness);
    }

    paintCenterStripsSplit(blueGoalColor, orangeGoalColor, inGameIdleMaxBrightness);
    safeFastLedShow();
    return;
  }

  // Attract mode remains a colorful moving field wash.
  for (uint16_t i = 0; i < LED_COUNT; i++) {
    float xNorm = clampFloat(pixelX[i] / FIELD_WIDTH, 0.0f, 1.0f);
    float yNorm = clampFloat(pixelY[i] / FIELD_HEIGHT, 0.0f, 1.0f);

    uint8_t hue = (uint8_t)(rainbowHueOffset + (uint16_t)(xNorm * 150.0f) + (uint16_t)(yNorm * 70.0f));
    uint8_t wave = sin8((uint8_t)(rainbowHueOffset + (uint16_t)(xNorm * 120.0f) + (uint16_t)(yNorm * 90.0f)));
    uint8_t value = map(wave, 0, 255, 80, 220);

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

static float circularDistance01(float a, float b) {
  float d = fabsf(a - b);
  if (d > 0.5f) d = 1.0f - d;
  return d;
}

static float hexRingPosition01(uint16_t i) {
  // Physical hex center based on the mapped purple hex geometry.
  const float cx = 48.5f + centerXShift;
  const float cy = 30.5f + centerYShift;

  float angle = atan2f(pixelY[i] - cy, pixelX[i] - cx);
  float normalized = (angle + PI) / (2.0f * PI);

  if (normalized < 0.0f) normalized += 1.0f;
  if (normalized >= 1.0f) normalized -= 1.0f;

  return normalized;
}

static CRGB countdownColorAt(unsigned long elapsed, float ringPos) {
  unsigned long stepIndex = elapsed / countdownStepMs;
  if (stepIndex >= COUNTDOWN_COLOR_COUNT) stepIndex = COUNTDOWN_COLOR_COUNT - 1;

  unsigned long nextIndex = stepIndex + 1;
  if (nextIndex >= COUNTDOWN_COLOR_COUNT) nextIndex = COUNTDOWN_COLOR_COUNT - 1;

  unsigned long stepElapsed = elapsed % countdownStepMs;
  float stepPhase = (float)stepElapsed / (float)countdownStepMs;

  // Color changes gradually with time and around the moving ring, not as a hard jump.
  float mixAmount = clampFloat((stepPhase * 0.70f) + (ringPos * 0.30f), 0.0f, 1.0f);
  return blend(countdownStepColors[stepIndex], countdownStepColors[nextIndex], (uint8_t)(mixAmount * 255.0f));
}

static uint8_t countdownHexBrightnessFor(uint16_t i, unsigned long elapsed, CRGB &outColor) {
  unsigned long period = countdownHexRotationMs;
  if (period < 1UL) period = 1UL;

  float pos = hexRingPosition01(i);
  float head = (float)(elapsed % period) / (float)period;
  float dist = circularDistance01(pos, head);
  float trail = clampFloat((float)countdownHexTrailPercent / 100.0f, 0.08f, 0.90f);

  // Rotating gradient position. This makes the color feel like it travels around the hex.
  float gradientPos = pos + head;
  while (gradientPos >= 1.0f) gradientPos -= 1.0f;

  outColor = countdownColorAt(elapsed, gradientPos);

  float intensity = 1.0f;

  if (countdownHexAnimationMode == COUNTDOWN_HEX_SOLID_STEP) {
    // Full hex uses the current second color with a very gentle spatial gradient.
    intensity = 1.0f;
  } else if (countdownHexAnimationMode == COUNTDOWN_HEX_FULL_GRADIENT) {
    // Entire hex stays on while the color gradient spins.
    uint8_t wave = sin8((uint8_t)(gradientPos * 255.0f));
    intensity = 0.55f + ((float)wave / 255.0f) * 0.45f;
  } else if (countdownHexAnimationMode == COUNTDOWN_HEX_COMET) {
    float tail = 1.0f - clampFloat(dist / trail, 0.0f, 1.0f);
    intensity = tail * tail;
  } else if (countdownHexAnimationMode == COUNTDOWN_HEX_DUAL_COMET) {
    float d2 = circularDistance01(pos, head + 0.5f);
    float tail1 = 1.0f - clampFloat(dist / trail, 0.0f, 1.0f);
    float tail2 = 1.0f - clampFloat(d2 / trail, 0.0f, 1.0f);
    intensity = max(tail1 * tail1, tail2 * tail2);
  } else if (countdownHexAnimationMode == COUNTDOWN_HEX_THEATER_CHASE) {
    float moving = pos - head;
    while (moving < 0.0f) moving += 1.0f;
    while (moving >= 1.0f) moving -= 1.0f;
    int slot = (int)(moving * 16.0f);
    intensity = (slot % 2 == 0) ? 1.0f : 0.18f;
  } else if (countdownHexAnimationMode == COUNTDOWN_HEX_SPARKLE_ORBIT) {
    float tail = 1.0f - clampFloat(dist / trail, 0.0f, 1.0f);
    uint8_t pseudo = (uint8_t)(((uint32_t)i * 37UL + (elapsed / 24UL) * 19UL) & 0xFF);
    float sparkle = pseudo > 220 ? 1.0f : 0.0f;
    intensity = clampFloat((tail * tail) + (sparkle * 0.80f), 0.0f, 1.0f);
  } else {
    // Default ORBIT_GRADIENT: visible base gradient plus a smooth rotating highlight.
    float tail = 1.0f - clampFloat(dist / trail, 0.0f, 1.0f);
    intensity = clampFloat(0.22f + (tail * tail * 0.88f), 0.0f, 1.0f);
  }

  int brightness = (int)((float)countdownHexBrightness * intensity);
  return clampU8(brightness);
}

static bool countdownHexTeamStyleActive() {
  return countdownHexAnimationMode == COUNTDOWN_HEX_TEAM_FILL_COMET ||
         countdownHexAnimationMode == COUNTDOWN_HEX_TEAM_FILL_ONLY ||
         countdownHexAnimationMode == COUNTDOWN_HEX_TEAM_SPLIT_PULSE ||
         countdownHexAnimationMode == COUNTDOWN_HEX_COMET_ONLY;
}

static float hexSideFillCoordinate(uint16_t i, bool blueSide) {
  float cx = 48.5f + centerXShift;

  if (blueSide) {
    float leftEdge = 35.0f;
    float denom = cx - leftEdge;
    if (denom < 1.0f) denom = 1.0f;
    return clampFloat((cx - pixelX[i]) / denom, 0.0f, 1.0f);
  }

  float rightEdge = 62.0f;
  float denom = rightEdge - cx;
  if (denom < 1.0f) denom = 1.0f;
  return clampFloat((pixelX[i] - cx) / denom, 0.0f, 1.0f);
}

static void renderTeamFillHexCountdown(unsigned long elapsed) {
  unsigned long totalMs = getCountdownTotalMs();
  if (totalMs < 1UL) totalMs = 1UL;

  float fillProgress = clampFloat((float)elapsed / (float)totalMs, 0.0f, 1.0f);
  unsigned long stepElapsed = elapsed % countdownStepMs;
  uint8_t fillBrightness = aggressiveOneSecondPulse(stepElapsed, countdownStepMs, 80, countdownHexBrightness);

  // A little minimum fill makes the hex readable at the very start.
  float visibleFill = clampFloat(fillProgress + 0.08f, 0.0f, 1.0f);

  unsigned long cometPeriod = countdownHexRotationMs;
  if (cometPeriod < 1UL) cometPeriod = 1UL;
  float cometHead = (float)(elapsed % cometPeriod) / (float)cometPeriod;
  float cometTail = clampFloat((float)countdownHexTrailPercent / 100.0f, 0.06f, 0.92f);

  for (uint16_t i = 0; i < LED_COUNT; i++) {
    if (!ledIsHexCenter(i)) {
      uint8_t fieldBrightness = countdownFieldBrightnessForLed(i, elapsed);
      leds[i] = scaledColor(countdownFieldColor, fieldBrightness);
      continue;
    }

    bool blueSide = pixelX[i] < (48.5f + centerXShift);
    CRGB sideColor = blueSide ? blueGoalColor : orangeGoalColor;

    float sideFillCoord = hexSideFillCoordinate(i, blueSide);
    bool filled = sideFillCoord <= visibleFill;

    CRGB c = CRGB::Black;

    if (countdownHexAnimationMode == COUNTDOWN_HEX_TEAM_SPLIT_PULSE) {
      c = scaledColor(sideColor, fillBrightness);
    } else if (countdownHexAnimationMode != COUNTDOWN_HEX_COMET_ONLY && filled) {
      // Filled pixels get team color. The boundary feather prevents it from looking blocky.
      float edge = clampFloat((visibleFill - sideFillCoord) / 0.14f, 0.0f, 1.0f);
      uint8_t localB = clampU8((int)((float)fillBrightness * (0.55f + edge * 0.45f)));
      c = scaledColor(sideColor, localB);
    } else {
      c = scaledColor(sideColor, 10);
    }

    // White comet around the hex. It is intentionally small, so it does not violate the full-field white rule.
    if (countdownHexAnimationMode == COUNTDOWN_HEX_TEAM_FILL_COMET || countdownHexAnimationMode == COUNTDOWN_HEX_COMET_ONLY) {
      float pos = hexRingPosition01(i);
      float d = circularDistance01(pos, cometHead);

      if (d < cometTail) {
        float comet = 1.0f - (d / cometTail);
        comet = comet * comet;
        uint8_t cometB = clampU8((int)(255.0f * comet));
        c += scaledColor(CRGB::White, cometB);
      }
    }

    leds[i] = c;
  }
}

static void renderCountdown() {
  unsigned long now = millis();

  if (now - lastLedUpdateMs < LED_UPDATE_MS) return;
  lastLedUpdateMs = now;

  unsigned long elapsed = now - ledModeStartMs;

  if (countdownHexTeamStyleActive()) {
    renderTeamFillHexCountdown(elapsed);
    safeFastLedShow();
    return;
  }

  // Legacy / alternate rotating styles remain available for experimenting.
  for (uint16_t i = 0; i < LED_COUNT; i++) {
    if (ledIsHexCenter(i)) {
      CRGB hexColor = countdownHexColor;
      uint8_t hexB = countdownHexBrightnessFor(i, elapsed, hexColor);
      leds[i] = scaledColor(hexColor, hexB);
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
    if (gameState == STATE_IN_GAME || gameState == STATE_GOAL_PAUSE) {
      setLedMode(LED_MODE_IN_GAME);
      lastLedUpdateMs = 0;
      renderAttractOrInGame(true);
    } else {
      setLedMode(LED_MODE_ATTRACT);
      lastLedUpdateMs = 0;
      renderAttractOrInGame(false);
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

    // End-of-effect transition.
    // Old behavior faded the goal frame toward black, which caused a brief blackout
    // before the normal in-game idle frame returned. That looked cheap.
    // New behavior crossfades from the goal wave into the normal idle/attract look.
    unsigned long fadeStart = goalWaveEmitEndMs + goalWaveTravelMs;
    float goalMix = 1.0f;

    if (elapsed > fadeStart && goalEffectMs > fadeStart) {
      goalMix = 1.0f - ((float)(elapsed - fadeStart) / (float)(goalEffectMs - fadeStart));
      goalMix = clampFloat(goalMix, 0.0f, 1.0f);
    }

    uint8_t goalBrightness = clampU8((int)((baseGlow + intensity) * 255.0f));
    CRGB goalPixel = scaledColor(teamColor, goalBrightness);

    if (goalMix < 0.999f) {
      CRGB backgroundPixel;

      if (gameState == STATE_IN_GAME || gameState == STATE_GOAL_PAUSE) {
        bool blueSideIdle = pixelX[i] < virtualCenterX();
        CRGB idleBase = blueSideIdle ? blueGoalColor : orangeGoalColor;
        uint8_t idleWave = sin8((uint8_t)(rainbowHueOffset * inGameIdleSpeed + (uint8_t)(xNorm * 130.0f) + (uint8_t)(yNorm * 70.0f)));
        uint8_t idleB = map(idleWave, 0, 255, inGameIdleMinBrightness, inGameIdleMaxBrightness);
        backgroundPixel = scaledColor(idleBase, idleB);
      } else {
        uint8_t hue = (uint8_t)(rainbowHueOffset + (uint16_t)(xNorm * 150.0f) + (uint16_t)(yNorm * 70.0f));
        uint8_t valueWave = sin8((uint8_t)(rainbowHueOffset + (uint16_t)(xNorm * 120.0f) + (uint16_t)(yNorm * 90.0f)));
        uint8_t value = map(valueWave, 0, 255, 80, 220);
        backgroundPixel = CHSV(hue, 230, value);
      }

      leds[i] = blend(backgroundPixel, goalPixel, (uint8_t)(goalMix * 255.0f));
    } else {
      leds[i] = goalPixel;
    }
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
    if (gameState == STATE_IN_GAME || gameState == STATE_GOAL_PAUSE) {
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


static void renderLedSelfTest() {
  unsigned long now = millis();

  if (now - lastLedUpdateMs < 35UL) {
    return;
  }

  lastLedUpdateMs = now;

  unsigned long phase = ((now - ledModeStartMs) / 700UL) % 4UL;
  CRGB color = CRGB::Black;

  if (phase == 0) {
    color = CRGB(255, 0, 0);
  } else if (phase == 1) {
    color = CRGB(0, 255, 0);
  } else if (phase == 2) {
    color = CRGB(0, 0, 255);
  } else {
    color = CRGB(255, 90, 0);
  }

  fill_solid(leds, LED_COUNT, color);
  safeFastLedShow();
}

static void updateLeds() {
  if (otaInProgress) return;

  // Self-test bypasses the spatial map. If this does not light the strip,
  // the problem is pin/power/data wiring, not the animation math.
  if (ledMode == LED_MODE_LED_SELF_TEST) {
    renderLedSelfTest();
    return;
  }

  if (!spatialMapReady) return;

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
  } else if (ledMode == LED_MODE_LAST_TEN_TEST) {
    unsigned long elapsed = (millis() - ledModeStartMs) % 10000UL;
    renderLastTenPulseFrame(10000UL - elapsed, millis());
    safeFastLedShow();
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

static void audioStopHard(const char* reason) {
  Serial.print("[AUDIO] Hard stop reason=");
  Serial.println(reason);

  sendAudioCommand(DY_CMD_STOP, nullptr, 0);
  delay(AUDIO_HARD_STOP_GAP_MS);
  sendAudioCommand(DY_CMD_STOP, nullptr, 0);
  delay(AUDIO_COMMAND_SETTLE_MS);

  trackActive = false;
  currentTrack = 0;
  lastAudioTrack = 0;
  lastAudioCommandMs = 0;
}

// Plays an important one-shot sound by first stopping whatever the DY module
// is currently doing. This avoids BGM continuing underneath or blocking the
// goal/countdown/start command.
static void audioPlayTrackInterrupting(uint16_t track, const char* reason) {
  if (track == 0 || track > MAX_TRACK) {
    Serial.println("[AUDIO] Invalid interrupting track.");
    return;
  }

  Serial.print("[AUDIO] Interrupt play track ");
  Serial.print(track);
  Serial.print(" reason=");
  Serial.println(reason);

  audioStopHard(reason);
  audioPlayTrack(track);
}

// Used for manual tests where the same track may be replayed quickly.
static void audioPlayTrackForced(uint16_t track) {
  audioPlayTrackInterrupting(track, "manual-forced");
}

// Countdown-specific helper.
// v4.24 requirement:
//   00002.mp3 is a complete countdown sequence:
//   3 beeps plus a higher-pitched GO tone at the end.
//   Firmware plays it once at countdown start.
//   Do not replay it at 3, 2, and 1 or the audio becomes messy.
static void audioPlayCountdownSequenceOnce() {
  if (countdownSequenceStarted) {
    return;
  }

  Serial.println("[AUDIO] Countdown sequence track 2 started once");

  audioPlayTrackInterrupting(AUDIO_TRACK_COUNTDOWN, "countdown-sequence");

  countdownSequenceStarted = true;
  lastCountdownBeepMs = millis();

  // Block BGM through the countdown and the short GO-tone tail.
  bgmAllowedAfterMs = millis() + getCountdownTotalMs() + 1000UL;
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

static bool isGameplayBgmTrack(uint16_t track) {
  return track == AUDIO_TRACK_FIRST_HALF_BGM ||
         track == AUDIO_TRACK_SECOND_HALF_BGM ||
         track == AUDIO_TRACK_STANDBY_IDLE;
}

static void updateAudioState() {
  if (!trackActive) return;

  unsigned long elapsed = getAudioElapsedMs();
  unsigned long estimatedLength = getCurrentTrackLengthMs();

  // Important demo fix:
  // Do NOT use the BUSY pin to terminate the long gameplay background tracks.
  // Some DY-HV20T/DY1900A modules report BUSY inconsistently on longer MP3 files,
  // which caused 00003.mp3 / 00005.mp3 to restart every ~1 second.
  // For background music, trust the estimated 90-second track length instead.
  if (!isGameplayBgmTrack(currentTrack)) {
    // For short one-shot sounds, BUSY is still useful so the controller can return
    // to gameplay music once the sound has actually finished.
    if (elapsed > 800UL && !audioBusyPlaying()) {
      trackActive = false;
      currentTrack = 0;
      Serial.println("[AUDIO] BUSY pin idle. One-shot track marked inactive.");
      return;
    }
  }

  if (estimatedLength > 0UL && elapsed >= estimatedLength) {
    trackActive = false;
    currentTrack = 0;
    Serial.println("[AUDIO] Estimated track duration elapsed. Track marked inactive.");
  }
}

static void serviceGameplayBackgroundAudio() {
  // Background music should only run during normal gameplay / idle field animation.
  // It is deliberately blocked during countdown, goal pause, goal animation, halftime,
  // match end, and immediately after the game-start sound.
  bool normalGameplayVisual = (ledMode == LED_MODE_IN_GAME);
  bool normalGameplayState = (gameState == STATE_IN_GAME) || (effectTestMode && ledMode == LED_MODE_IN_GAME);

  if (!normalGameplayState || !normalGameplayVisual) {
    return;
  }

  unsigned long now = millis();

  if (now < bgmAllowedAfterMs) {
    return;
  }

  uint16_t bgmTrack = (currentHalf == 2) ? AUDIO_TRACK_SECOND_HALF_BGM : AUDIO_TRACK_FIRST_HALF_BGM;

  // If the correct BGM is already considered active, leave it alone.
  // Do not restart it. This is the fix for the "first second keeps looping" problem.
  if (trackActive && currentTrack == bgmTrack) {
    return;
  }

  // If a higher-priority one-shot is still active, do not stomp it.
  if (trackActive && !isGameplayBgmTrack(currentTrack)) {
    return;
  }

  // Start the correct half BGM once. Use a direct play command without an
  // unnecessary STOP if no track is active. If the module is confused from a
  // previous command, the start/countdown/goal handlers already stopped it.
  if (!trackActive || currentTrack == 0 || currentTrack != bgmTrack) {
    Serial.print("[AUDIO] Starting gameplay BGM track ");
    Serial.println(bgmTrack);
    audioPlayTrack(bgmTrack);
    lastRequestedBgmTrack = bgmTrack;
  }
}


static void serviceStandbyIdleAudio() {
  // Standby / Idle is the safe downtime state:
  // - no scoring
  // - attract LEDs stay alive
  // - track 00009 can loop as idle ambience
  if (gameState != STATE_STANDBY) {
    return;
  }

  if (ledMode != LED_MODE_ATTRACT) {
    return;
  }

  if (effectTestMode) {
    return;
  }

  if (trackActive && currentTrack == AUDIO_TRACK_STANDBY_IDLE) {
    return;
  }

  // Do not stomp reset/end/manual test audio. Idle ambience will start after those finish.
  if (trackActive && currentTrack != 0 && currentTrack != AUDIO_TRACK_STANDBY_IDLE) {
    return;
  }

  Serial.println("[AUDIO] Starting standby idle ambience track 9");
  audioPlayTrack(AUDIO_TRACK_STANDBY_IDLE);
}


// ============================================================
// SENSOR HANDLING
// ============================================================

#if USE_SENSOR_INTERRUPTS
struct SensorInterruptLatch {
  volatile bool pending;
  volatile bool sawDetected;
  volatile bool sawClear;
  volatile bool lastDetected;
  volatile uint32_t count;
  volatile uint32_t lastUs;
};

static SensorInterruptLatch isrLeftA  = { false, false, false, false, 0, 0 };
static SensorInterruptLatch isrLeftB  = { false, false, false, false, 0, 0 };
static SensorInterruptLatch isrRightA = { false, false, false, false, 0, 0 };
static SensorInterruptLatch isrRightB = { false, false, false, false, 0, 0 };

static void IRAM_ATTR latchSensorInterrupt(SensorInterruptLatch *latch, uint8_t pin) {
  uint32_t nowUs = micros();

  if ((uint32_t)(nowUs - latch->lastUs) < SENSOR_INTERRUPT_MIN_GAP_US) {
    return;
  }

  latch->lastUs = nowUs;

  int level = digitalRead(pin);

#if SENSOR_ACTIVE_LOW
  bool detected = (level == LOW);
#else
  bool detected = (level == HIGH);
#endif

  latch->pending = true;
  latch->lastDetected = detected;

  if (detected) {
    latch->sawDetected = true;
  } else {
    latch->sawClear = true;
  }

  latch->count++;
}

static void IRAM_ATTR onLeftSensorAInterrupt() {
  latchSensorInterrupt(&isrLeftA, LEFT_GOAL_SENSOR_A_PIN);
}

static void IRAM_ATTR onLeftSensorBInterrupt() {
  latchSensorInterrupt(&isrLeftB, LEFT_GOAL_SENSOR_B_PIN);
}

static void IRAM_ATTR onRightSensorAInterrupt() {
  latchSensorInterrupt(&isrRightA, RIGHT_GOAL_SENSOR_A_PIN);
}

static void IRAM_ATTR onRightSensorBInterrupt() {
  latchSensorInterrupt(&isrRightB, RIGHT_GOAL_SENSOR_B_PIN);
}

static SensorInterruptLatch *sensorLatchForPin(uint8_t pin) {
  if (pin == LEFT_GOAL_SENSOR_A_PIN) return &isrLeftA;
  if (pin == LEFT_GOAL_SENSOR_B_PIN) return &isrLeftB;
  if (pin == RIGHT_GOAL_SENSOR_A_PIN) return &isrRightA;
  if (pin == RIGHT_GOAL_SENSOR_B_PIN) return &isrRightB;
  return nullptr;
}

static bool consumeSensorInterruptForPin(uint8_t pin, bool &sawDetected, bool &sawClear, bool &lastDetected, uint32_t &countSnapshot) {
  SensorInterruptLatch *latch = sensorLatchForPin(pin);

  if (latch == nullptr) {
    return false;
  }

  noInterrupts();
  bool pending = latch->pending;
  sawDetected = latch->sawDetected;
  sawClear = latch->sawClear;
  lastDetected = latch->lastDetected;
  countSnapshot = latch->count;

  latch->pending = false;
  latch->sawDetected = false;
  latch->sawClear = false;
  interrupts();

  return pending;
}

static unsigned long totalSensorInterruptCount() {
  noInterrupts();
  unsigned long total =
    (unsigned long)isrLeftA.count +
    (unsigned long)isrLeftB.count +
    (unsigned long)isrRightA.count +
    (unsigned long)isrRightB.count;
  interrupts();

  return total;
}

static void setupSensorInterrupts() {
  attachInterrupt(digitalPinToInterrupt(LEFT_GOAL_SENSOR_A_PIN), onLeftSensorAInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_GOAL_SENSOR_B_PIN), onLeftSensorBInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_GOAL_SENSOR_A_PIN), onRightSensorAInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_GOAL_SENSOR_B_PIN), onRightSensorBInterrupt, CHANGE);

  Serial.println("[INIT] v4.31 sensor interrupts attached on CHANGE for all four goal sensors.");
  Serial.println("[INIT] ISR only latches events. Scoring still happens in the main loop after dual-sensor validation.");
}
#endif

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

#if GOAL_TRIGGER_ON_CLEAR
  // v4.29 mounted-reflection mode:
  // Default/ready state is both sensors already detecting the mounted background.
  bool readyState = g.sensorA.stableDetected && g.sensorB.stableDetected;
#else
  // Normal mode:
  // Default/ready state is both sensors clear.
  bool readyState = !g.sensorA.stableDetected && !g.sensorB.stableDetected;
#endif

  g.armed = readyState;
  g.bothClearStartMs = millis();
  g.sensorASeenMs = 0;
  g.sensorBSeenMs = 0;
}

static void setupSensors() {
  initializeDualGoal(leftGoal);
  initializeDualGoal(rightGoal);

#if USE_SENSOR_INTERRUPTS
  setupSensorInterrupts();
#endif

  #if USE_SENSOR_INTERRUPTS
  Serial.println("[INIT] Four goal sensors started with INPUT mode plus interrupt latches.");
#else
  Serial.println("[INIT] Four goal sensors started with plain INPUT polling mode.");
#endif
  Serial.println("[INIT] Left goal:  IO_11/GPIO22 + IO_12/GPIO21");
  Serial.println("[INIT] Right goal: IO_07/GPIO13 + IO_10/GPIO4");
  #if GOAL_TRIGGER_ON_CLEAR
  Serial.println("[INIT] Dual-sensor validation enabled: goal triggers when both sensors become CLEAR / NO-DETECT.");
#else
  Serial.println("[INIT] Dual-sensor validation enabled: goal triggers when both sensors DETECT.");
#endif
}

static void updateDebouncedSensor(DebouncedSensor &s) {
  bool rawDetected = readRawDetected(s.pin);
  unsigned long now = millis();

#if USE_SENSOR_INTERRUPTS
  bool isrSawDetected = false;
  bool isrSawClear = false;
  bool isrLastDetected = rawDetected;
  uint32_t isrCountSnapshot = 0;

  bool hadInterrupt = consumeSensorInterruptForPin(
    s.pin,
    isrSawDetected,
    isrSawClear,
    isrLastDetected,
    isrCountSnapshot
  );

  if (hadInterrupt) {
#if GOAL_TRIGGER_ON_CLEAR
    // Mounted-reflection mode:
    // A clear/no-detect pulse is the scoring edge.
    // If the ISR saw clear at any point since the last loop pass, preserve that event
    // even if the sensor has already returned to detected by the time we poll it.
    if (isrSawClear) {
      rawDetected = false;
    } else {
      rawDetected = isrLastDetected;
    }
#else
    // Normal mode:
    // A detected pulse is the scoring edge.
    if (isrSawDetected) {
      rawDetected = true;
    } else {
      rawDetected = isrLastDetected;
    }
#endif
  }
#endif

  if (rawDetected != s.lastRawDetected) {
    s.lastRawDetected = rawDetected;
    s.lastRawChangeMs = now;

#if USE_SENSOR_INTERRUPTS && SENSOR_INTERRUPT_FAST_EDGE
    // Fast path:
    // The ISR has already proven a pin transition happened.
    // The main loop still performs dual-sensor validation and game-state gating.
    if (hadInterrupt) {
      s.stableDetected = rawDetected;
    }
#endif
  }

  if (now - s.lastRawChangeMs >= SENSOR_DEBOUNCE_MS) {
    s.stableDetected = rawDetected;
  }
}

static bool sensorRisingEdge(const DebouncedSensor &s) {
  return s.stableDetected && !s.previousStableDetected;
}

static bool sensorClearedEdge(const DebouncedSensor &s) {
  return !s.stableDetected && s.previousStableDetected;
}

static bool dualGoalBothClear(const DualGoalValidator &g) {
  return !g.sensorA.stableDetected && !g.sensorB.stableDetected;
}

static bool dualGoalBothDetected(const DualGoalValidator &g) {
  return g.sensorA.stableDetected && g.sensorB.stableDetected;
}

static bool dualGoalAnyDetected(const DualGoalValidator &g) {
  return g.sensorA.stableDetected || g.sensorB.stableDetected;
}

static bool dualGoalReadyState(const DualGoalValidator &g) {
#if GOAL_TRIGGER_ON_CLEAR
  // Mounted-reflection mode: ready means both sensors are detecting the background.
  return dualGoalBothDetected(g);
#else
  // Normal mode: ready means both sensors are clear.
  return dualGoalBothClear(g);
#endif
}

static bool sensorGoalTriggerEdge(const DebouncedSensor &s) {
#if GOAL_TRIGGER_ON_CLEAR
  return sensorClearedEdge(s);
#else
  return sensorRisingEdge(s);
#endif
}

static const char *sensorGoalTriggerWord() {
#if GOAL_TRIGGER_ON_CLEAR
  return "cleared / no-detect";
#else
  return "detected";
#endif
}

static bool updateDualGoalValidator(DualGoalValidator &g, const char *goalName) {
  updateDebouncedSensor(g.sensorA);
  updateDebouncedSensor(g.sensorB);

  unsigned long now = millis();

  bool edgeA = sensorGoalTriggerEdge(g.sensorA);
  bool edgeB = sensorGoalTriggerEdge(g.sensorB);
  bool readyState = dualGoalReadyState(g);

  // Rearm only after the goal returns to its default/ready state for a short stable period.
  // v4.31 mounted-reflection mode:
  //   default/ready = both sensors detecting the mounted background
  //   trigger       = both sensors become clear / no-detect
  //   interrupt ISR = near-instant event latch, not the final scorer
  if (readyState) {
    if (g.bothClearStartMs == 0) {
      g.bothClearStartMs = now;
    }

    if (!g.armed && (now - g.bothClearStartMs >= DUAL_SENSOR_REARM_CLEAR_MS)) {
      g.armed = true;
      g.sensorASeenMs = 0;
      g.sensorBSeenMs = 0;

      Serial.print("[SENSOR] ");
      Serial.print(goalName);
      Serial.print(" rearmed after both sensors returned to ready state: ");
      Serial.println(sensorGoalTriggerWord());
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
    Serial.print(" sensor A confirmed ");
    Serial.println(sensorGoalTriggerWord());
  }

  if (edgeB) {
    g.sensorBSeenMs = now;

    Serial.print("[SENSOR] ");
    Serial.print(goalName);
    Serial.print(" sensor B confirmed ");
    Serial.println(sensorGoalTriggerWord());
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
    Serial.print(" VALID dual-sensor goal on ");
    Serial.println(sensorGoalTriggerWord());
  }

  // If only one sensor fired and the other did not confirm in time, reject that partial event.
  if (!validPair) {
    if (g.sensorASeenMs > 0 && now - g.sensorASeenMs > DUAL_SENSOR_CONFIRM_WINDOW_MS) {
      g.sensorASeenMs = 0;
      g.rejectedSingleCount++;

      Serial.print("[SENSOR] ");
      Serial.print(goalName);
      Serial.print(" rejected: only sensor A ");
      Serial.print(sensorGoalTriggerWord());
      Serial.println(".");
    }

    if (g.sensorBSeenMs > 0 && now - g.sensorBSeenMs > DUAL_SENSOR_CONFIRM_WINDOW_MS) {
      g.sensorBSeenMs = 0;
      g.rejectedSingleCount++;

      Serial.print("[SENSOR] ");
      Serial.print(goalName);
      Serial.print(" rejected: only sensor B ");
      Serial.print(sensorGoalTriggerWord());
      Serial.println(".");
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

  if (gameState == STATE_GOAL_PAUSE) {
    return pausedMatchRemainingMs;
  }

  if (gameState == STATE_COUNTDOWN) {
    return matchDurationMs;
  }

  if (gameState == STATE_HALFTIME) {
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
  pausedMatchRemainingMs = matchDurationMs;
  goalPauseStartMs = 0;

  // Demo-day behavior: Start Match always starts a clean scoreboard.
  blueScore = 0;
  orangeScore = 0;
  lastBlueGoalMs = 0;
  lastOrangeGoalMs = 0;
  currentHalf = 1;
  firstHalfComplete = false;
  secondHalfComplete = false;

  gameState = STATE_COUNTDOWN;
  countdownStartMs = millis();
  lastCountdownAnnounced = -1;
  lastCountdownBeepMs = 0;
  countdownSequenceStarted = false;

  setLedMode(LED_MODE_COUNTDOWN);

  // Critical: kill any previous BGM or test audio before countdown.
  // Track 00002 now contains the whole countdown sequence including the GO tone.
  audioStopHard("start-countdown");
  bgmAllowedAfterMs = millis() + getCountdownTotalMs() + 1000UL;
  lastRequestedBgmTrack = 0;

  addEvent("First half countdown started");
}

static void completeCountdownAndStartGame() {
  gameState = STATE_IN_GAME;
  matchStartMs = millis();
  pausedMatchRemainingMs = matchDurationMs;
  goalPauseStartMs = 0;

  setLedMode(LED_MODE_GAME_START_GREEN);

  // v4.24 polish:
  // The countdown MP3 already ends with a higher-pitched GO tone.
  // Do not interrupt it with the old game-start stinger.
  // This lets the match feel like: 3, 2, 1, GO, then gameplay BGM.
  bgmAllowedAfterMs = millis() + GAME_START_TO_BGM_DELAY_MS;
  lastRequestedBgmTrack = 0;

  String msg = (currentHalf == 2) ? "Second half started after GO tone" : "First half started after GO tone";
  addEvent(msg);
}

static void enterHalftime() {
  firstHalfComplete = true;
  gameState = STATE_HALFTIME;
  matchEndMs = millis();

  setLedMode(LED_MODE_ATTRACT);
  audioPlayTrackInterrupting(AUDIO_TRACK_HALFTIME_END, "halftime");
  bgmAllowedAfterMs = 0;
  lastRequestedBgmTrack = 0;

  addEvent("First half complete. Ready to resume second half.");
}

static void startSecondHalf() {
  if (gameState != STATE_HALFTIME && !firstHalfComplete) {
    addEvent("Resume second half ignored before halftime");
    return;
  }

  if (secondHalfComplete || gameState == STATE_END_GAME) {
    addEvent("Second half already complete");
    return;
  }

  currentHalf = 2;
  gameState = STATE_COUNTDOWN;
  countdownStartMs = millis();
  lastCountdownAnnounced = -1;
  lastCountdownBeepMs = 0;
  countdownSequenceStarted = false;
  matchStartMs = 0;
  pausedMatchRemainingMs = matchDurationMs;
  goalPauseStartMs = 0;

  setLedMode(LED_MODE_COUNTDOWN);
  audioStopHard("second-half-countdown");
  bgmAllowedAfterMs = millis() + getCountdownTotalMs() + 1000UL;
  lastRequestedBgmTrack = 0;

  addEvent("Second half countdown started");
}

static void endMatch(bool manualEnd) {
  if (gameState == STATE_END_GAME) return;

  gameState = STATE_END_GAME;
  matchEndMs = millis();
  secondHalfComplete = (currentHalf == 2);

  setLedMode(LED_MODE_MATCH_END);
  audioPlayTrackInterrupting(AUDIO_TRACK_HALFTIME_END, "match-end");
  bgmAllowedAfterMs = 0;
  lastRequestedBgmTrack = 0;

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
  pausedMatchRemainingMs = matchDurationMs;
  goalPauseStartMs = 0;
  lastCountdownAnnounced = -1;
  lastCountdownBeepMs = 0;
  countdownSequenceStarted = false;
  currentHalf = 1;
  firstHalfComplete = false;
  secondHalfComplete = false;

  setLedMode(LED_MODE_ATTRACT);
  audioPlayTrackInterrupting(AUDIO_TRACK_RESET, "reset");
  bgmAllowedAfterMs = 0;
  lastRequestedBgmTrack = 0;

  addEvent("Game reset");
}


static void enterStandbyIdleMode() {
  gameState = STATE_STANDBY;
  matchStartMs = 0;
  countdownStartMs = 0;
  pausedMatchRemainingMs = matchDurationMs;
  goalPauseStartMs = 0;
  lastCountdownAnnounced = -1;
  lastCountdownBeepMs = 0;
  countdownSequenceStarted = false;
  currentHalf = 1;
  firstHalfComplete = false;
  secondHalfComplete = false;

  setLedMode(LED_MODE_ATTRACT);

  // Stop any gameplay/goal/reset audio. Track 00009 will start from serviceStandbyIdleAudio().
  audioStopHard("standby-idle");
  bgmAllowedAfterMs = 0;
  lastRequestedBgmTrack = 0;

  addEvent("Standby / Idle active. Scoring disabled.");
}

static void resumeAfterGoal() {
  if (gameState != STATE_GOAL_PAUSE) {
    addEvent("Resume after goal ignored: not paused for goal");
    return;
  }

  unsigned long remaining = pausedMatchRemainingMs;

  if (remaining == 0 || remaining > matchDurationMs) {
    remaining = matchDurationMs;
  }

  // Reconstruct matchStartMs so the countdown continues from the frozen time.
  matchStartMs = millis() - (matchDurationMs - remaining);
  gameState = STATE_IN_GAME;
  goalPauseStartMs = 0;

  setLedMode(LED_MODE_IN_GAME);

  // DY-HV20T cannot mix BGM under a one-shot. Play a short polished resume cue,
  // then allow the correct half BGM to restart cleanly after the cue.
  audioPlayTrackInterrupting(AUDIO_TRACK_RESUME_PLAY, "resume-after-goal");
  bgmAllowedAfterMs = millis() + GOAL_RESUME_TO_BGM_DELAY_MS;
  lastRequestedBgmTrack = 0;

  addEvent("Gameplay resumed after goal reset");
}

static void pauseMatchForGoal() {
  pausedMatchRemainingMs = getMatchRemainingMs();

  if (pausedMatchRemainingMs == 0 || pausedMatchRemainingMs > matchDurationMs) {
    pausedMatchRemainingMs = matchDurationMs;
  }

  goalPauseStartMs = millis();
  gameState = STATE_GOAL_PAUSE;
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
  pauseMatchForGoal();

  setLedMode(LED_MODE_BLUE_GOAL);
  audioPlayTrackInterrupting(AUDIO_TRACK_GOAL, "goal");

  if (autoResumeAfterGoalEnabled) {
    String msg = fake ? "Fake Blue goal scored. Auto resume armed for " : "Blue goal scored. Auto resume armed for ";
    msg += String(autoResumeAfterGoalDelayMs / 1000.0f, 1);
    msg += "s";
    addEvent(msg);
  } else {
    addEvent(fake ? "Fake Blue goal scored. Timer paused for ball reset." : "Blue goal scored. Timer paused for ball reset.");
  }
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
  pauseMatchForGoal();

  setLedMode(LED_MODE_ORANGE_GOAL);
  audioPlayTrackInterrupting(AUDIO_TRACK_GOAL, "goal");

  if (autoResumeAfterGoalEnabled) {
    String msg = fake ? "Fake Orange goal scored. Auto resume armed for " : "Orange goal scored. Auto resume armed for ";
    msg += String(autoResumeAfterGoalDelayMs / 1000.0f, 1);
    msg += "s";
    addEvent(msg);
  } else {
    addEvent(fake ? "Fake Orange goal scored. Timer paused for ball reset." : "Orange goal scored. Timer paused for ball reset.");
  }
}

static void handleBallHit(bool bigHit) {
  if (gameState != STATE_IN_GAME && !effectTestMode) {
    addEvent(bigHit ? "Big ball hit ignored outside IN_GAME" : "Ball hit ignored outside IN_GAME");
    return;
  }

  if (bigHit) {
    setLedMode(LED_MODE_BIG_HIT);
    addEvent("Big ball hit");
  } else {
    setLedMode(LED_MODE_HIT);
    addEvent("Ball hit");
  }
}

static void updateGame() {
  unsigned long now = millis();

  if (gameState == STATE_GOAL_PAUSE && autoResumeAfterGoalEnabled) {
    unsigned long elapsedPause = now - goalPauseStartMs;

    if (goalPauseStartMs > 0 && elapsedPause >= autoResumeAfterGoalDelayMs) {
      addEvent("Auto resume after goal delay");
      resumeAfterGoal();
      return;
    }
  }

  if (gameState == STATE_COUNTDOWN) {
    int count = getCountdownValue();

    // 00002.mp3 contains all countdown audio now:
    // 3 beeps plus the GO tone. Start it once and let the file play.
    audioPlayCountdownSequenceOnce();

    if (count > 0 && count != lastCountdownAnnounced) {
      lastCountdownAnnounced = count;

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
      if (currentHalf == 1) {
        enterHalftime();
      } else {
        endMatch(false);
      }
    }
  }
}

static void updateSensorsAndGoals() {
  bool leftGoalValid = updateDualGoalValidator(leftGoal, "LEFT GOAL");
  bool rightGoalValid = updateDualGoalValidator(rightGoal, "RIGHT GOAL");

  if (gameState != STATE_IN_GAME) {
    // Sensors are debounced and monitored, but they cannot score outside live gameplay.
    return;
  }

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

  out += "\"scoringEnabled\":";
  out += (gameState == STATE_IN_GAME) ? "true" : "false";
  out += ",";

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

  out += "\"currentHalf\":";
  out += String(currentHalf);
  out += ",";

  out += "\"firstHalfComplete\":";
  out += firstHalfComplete ? "true" : "false";
  out += ",";

  out += "\"secondHalfComplete\":";
  out += secondHalfComplete ? "true" : "false";
  out += ",";

  out += "\"matchRemainingMs\":";
  out += String(getMatchRemainingMs());
  out += ",";

  out += "\"goalPauseRemainingMs\":";
  out += String(pausedMatchRemainingMs);
  out += ",";

  out += "\"goalPauseActive\":";
  out += gameState == STATE_GOAL_PAUSE ? "true" : "false";
  out += ",";

  out += "\"autoResumeAfterGoal\":";
  out += autoResumeAfterGoalEnabled ? "true" : "false";
  out += ",";

  out += "\"autoResumeAfterGoalDelayMs\":";
  out += String(autoResumeAfterGoalDelayMs);
  out += ",";

  out += "\"autoResumeGoalRemainingMs\":";
  if (gameState == STATE_GOAL_PAUSE && autoResumeAfterGoalEnabled && goalPauseStartMs > 0) {
    unsigned long elapsedAutoResume = millis() - goalPauseStartMs;
    if (elapsedAutoResume >= autoResumeAfterGoalDelayMs) {
      out += "0";
    } else {
      out += String(autoResumeAfterGoalDelayMs - elapsedAutoResume);
    }
  } else {
    out += "0";
  }
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

  out += "\"countdownHexStyle\":\"";
  out += countdownHexStyleName();
  out += "\",";

  out += "\"countdownHexRotationMs\":";
  out += String(countdownHexRotationMs);
  out += ",";

  out += "\"countdownHexTrailPercent\":";
  out += String(countdownHexTrailPercent);
  out += ",";

  out += "\"lastTenColor\":\"";
  out += colorToHex(lastTenPulseColor);
  out += "\",";

  out += "\"lastTenEnabled\":";
  out += lastTenWarningEnabled ? "true" : "false";
  out += ",";

  out += "\"lastTenPeriodMs\":";
  out += String(lastTenPulsePeriodMs);
  out += ",";

  out += "\"lastTenBrightness\":\"";
  out += String(lastTenMinBrightness);
  out += "/";
  out += String(lastTenMaxBrightness);
  out += "\",";

  out += "\"inGameIdleSpeed\":";
  out += String(inGameIdleSpeed);
  out += ",";

  out += "\"inGameIdleBrightness\":\"";
  out += String(inGameIdleMinBrightness);
  out += "/";
  out += String(inGameIdleMaxBrightness);
  out += "\",";

  out += "\"effectTestMode\":";
  out += effectTestMode ? "true" : "false";
  out += ",";

  out += "\"countdownStepColors\":[";
  for (uint8_t ci = 0; ci < COUNTDOWN_COLOR_COUNT; ci++) {
    out += "\"";
    out += colorToHex(countdownStepColors[ci]);
    out += "\"";
    if (ci < COUNTDOWN_COLOR_COUNT - 1) out += ",";
  }
  out += "],";

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
    :root{--bg:#05070d;--panel:#0c1220;--panel2:#111a2e;--line:rgba(255,255,255,.12);--text:#f8fafc;--muted:#94a3b8;--blue:#0050ff;--orange:#ff5000;--green:#00ff72;--red:#ef4444;--yellow:#facc15;--shadow:0 26px 90px rgba(0,0,0,.55)}
    *{box-sizing:border-box}html,body{margin:0;min-height:100%;background:radial-gradient(circle at 20% 0%,rgba(0,80,255,.25),transparent 34%),radial-gradient(circle at 80% 0%,rgba(255,80,0,.22),transparent 34%),linear-gradient(180deg,#07111f,#02040a);color:var(--text);font-family:Inter,ui-sans-serif,system-ui,-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,Arial,sans-serif}button,input,select{font:inherit}.app{width:min(1420px,100%);margin:0 auto;padding:20px}.top{display:flex;justify-content:space-between;align-items:center;gap:16px;margin-bottom:16px}.brand{display:flex;align-items:center;gap:14px}.logo{width:54px;height:54px;border-radius:18px;background:linear-gradient(135deg,#fff,#3b82f6,#f97316);display:grid;place-items:center;color:#020617;font-weight:1000;letter-spacing:-.08em;box-shadow:0 18px 50px rgba(0,0,0,.45)}h1{margin:0;font-size:28px;letter-spacing:-.04em;line-height:1}.sub{color:var(--muted);font-size:13px;margin-top:6px;font-weight:750}.tabs{display:flex;gap:10px;flex-wrap:wrap;justify-content:flex-end}.tab{border:1px solid var(--line);background:rgba(255,255,255,.07);color:var(--text);border-radius:999px;padding:12px 16px;font-weight:950;cursor:pointer}.tab.active{background:#f8fafc;color:#020617}.tab.score{background:linear-gradient(135deg,var(--blue),var(--orange));border:0;color:white}.mode{display:none}.mode.active{display:block}.grid{display:grid;grid-template-columns:1.1fr .9fr;gap:16px}.studioGrid{display:grid;grid-template-columns:repeat(3,1fr);gap:16px}.card{background:linear-gradient(180deg,rgba(255,255,255,.085),rgba(255,255,255,.035));border:1px solid var(--line);border-radius:28px;box-shadow:var(--shadow);overflow:hidden}.card.pad{padding:20px}.span2{grid-column:span 2}.span3{grid-column:span 3}.controlTitle{display:flex;justify-content:space-between;align-items:center;gap:12px;margin-bottom:14px}.controlTitle h2{margin:0;font-size:22px;letter-spacing:-.03em}.badge{border:1px solid var(--line);background:rgba(255,255,255,.08);padding:8px 10px;border-radius:999px;color:var(--muted);font-size:12px;font-weight:900}.sectionNote{color:var(--muted);font-size:13px;margin:-4px 0 14px;line-height:1.35}.scoreRow{display:grid;grid-template-columns:1fr .9fr 1fr;gap:14px;align-items:stretch}.team{border-radius:26px;padding:20px;min-height:190px;display:flex;flex-direction:column;justify-content:space-between;position:relative;overflow:hidden}.team:before{content:"";position:absolute;inset:-35%;background:radial-gradient(circle,rgba(255,255,255,.2),transparent 50%);opacity:.28;animation:drift 7s linear infinite}.team.blue{background:linear-gradient(135deg,#001a70,var(--blue))}.team.orange{background:linear-gradient(135deg,#7a2100,var(--orange))}.teamName{font-size:17px;text-transform:uppercase;letter-spacing:.18em;font-weight:1000;opacity:.9}.score{font-size:110px;line-height:.8;font-weight:1000;letter-spacing:-.1em;z-index:1}.timerBox{border-radius:26px;background:rgba(2,6,23,.84);border:1px solid var(--line);padding:20px;text-align:center;display:flex;flex-direction:column;justify-content:center}.timer{font-size:78px;font-weight:1000;letter-spacing:-.08em;line-height:.9}.state{display:inline-flex;align-self:center;margin-top:12px;padding:10px 14px;border-radius:999px;background:rgba(255,255,255,.09);border:1px solid var(--line);font-size:13px;text-transform:uppercase;letter-spacing:.14em;font-weight:1000}.state.go{background:rgba(0,255,114,.18);color:#86efac}.state.pause{background:rgba(250,204,21,.18);color:#fde68a}.state.end{background:rgba(255,255,255,.18)}.actions{display:grid;grid-template-columns:repeat(2,1fr);gap:12px}.actions button{min-height:74px;border:0;border-radius:22px;color:white;font-weight:1000;font-size:16px;cursor:pointer;box-shadow:0 16px 34px rgba(0,0,0,.32);transition:.12s transform,.18s filter}.actions button:active{transform:translateY(1px) scale(.99)}.actions button:disabled{opacity:.45;filter:grayscale(.75);cursor:not-allowed;box-shadow:none}.primary{background:linear-gradient(135deg,#16a34a,#00ff72);color:#03130a!important}.resume{background:linear-gradient(135deg,#eab308,#f97316);color:#111827!important}.blueBtn{background:linear-gradient(135deg,#0031a8,var(--blue))}.orangeBtn{background:linear-gradient(135deg,#9a3412,var(--orange))}.dark{background:linear-gradient(135deg,#1f2937,#050505);border:1px solid var(--line)!important}.danger{background:linear-gradient(135deg,#7f1d1d,#ef4444)}.wide{grid-column:span 2}.resumeBig{grid-column:span 2;min-height:94px!important;font-size:22px!important}.resumeBig.attention{animation:softPulse 1s ease-in-out infinite;outline:2px solid rgba(255,255,255,.45)}.hidden{display:none!important}.miniGrid{display:grid;grid-template-columns:repeat(2,1fr);gap:12px}.metric{border:1px solid var(--line);background:rgba(255,255,255,.055);border-radius:20px;padding:14px}.metric .k{font-size:11px;color:var(--muted);letter-spacing:.12em;text-transform:uppercase;font-weight:1000}.metric .v{font-size:20px;font-weight:1000;margin-top:5px}.ok{color:#86efac}.warn{color:#fde68a}.bad{color:#fca5a5}.log{max-height:260px;overflow:auto;display:flex;flex-direction:column;gap:8px}.logItem{font-size:13px;color:#cbd5e1;border-bottom:1px solid rgba(255,255,255,.07);padding-bottom:8px}.field{display:grid;grid-template-columns:1fr;gap:7px;margin:12px 0}.field label{font-size:11px;color:var(--muted);letter-spacing:.12em;text-transform:uppercase;font-weight:1000}.select,.num,.color,.textBox{width:100%;border:1px solid var(--line);border-radius:16px;background:rgba(2,6,23,.82);color:var(--text);padding:12px 13px;font-weight:900;outline:none}.select:focus,.num:focus,.textBox:focus{border-color:rgba(255,255,255,.34);box-shadow:0 0 0 4px rgba(0,80,255,.12)}.color{height:48px;padding:5px}.rangeRow{display:grid;grid-template-columns:1fr 86px 62px;gap:8px;align-items:center}.rangeRow input[type=range]{width:100%;accent-color:#60a5fa}.rangeRow .num{padding:10px;text-align:center}.resetBtn{border:1px solid var(--line);background:rgba(255,255,255,.07);color:var(--text);border-radius:14px;padding:10px 8px;font-weight:1000;cursor:pointer}.applyRow{display:flex;gap:10px;flex-wrap:wrap;margin-top:12px}.applyRow button{border:0;border-radius:16px;padding:12px 14px;font-weight:1000;cursor:pointer}.apply{background:#f8fafc;color:#020617}.test{background:rgba(255,255,255,.08);border:1px solid var(--line)!important;color:var(--text)}.recipe{width:100%;min-height:116px;border:1px solid var(--line);border-radius:18px;background:rgba(2,6,23,.86);color:#dbeafe;padding:12px;font-family:ui-monospace,SFMono-Regular,Menlo,Monaco,Consolas,monospace;font-size:12px;line-height:1.45}.smallActions{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;margin-top:10px}.smallActions button{border:1px solid var(--line);border-radius:16px;background:rgba(255,255,255,.07);color:var(--text);padding:12px;font-weight:900;cursor:pointer}.diag{margin-top:12px;border:1px solid var(--line);border-radius:18px;padding:10px;background:rgba(255,255,255,.035)}.diag summary{cursor:pointer;color:var(--muted);font-weight:1000;text-transform:uppercase;letter-spacing:.12em;font-size:12px}.public{position:fixed;inset:0;background:radial-gradient(circle at 20% 5%,rgba(0,80,255,.34),transparent 36%),radial-gradient(circle at 80% 5%,rgba(255,80,0,.34),transparent 36%),#020617;z-index:20;padding:clamp(8px,1.4vw,18px);display:none;touch-action:manipulation}.public.active{display:grid;grid-template-rows:minmax(170px,30vh) 1fr;gap:clamp(8px,1.2vh,14px)}.publicTop{border:1px solid rgba(255,255,255,.09);border-radius:28px;background:rgba(255,255,255,.045);display:grid;place-items:center;position:relative;overflow:hidden;min-height:0}.publicTimer{font-size:clamp(82px,17vw,215px);font-weight:1000;letter-spacing:-.09em;line-height:.76}.publicState{position:absolute;top:clamp(10px,1.8vh,18px);left:50%;transform:translateX(-50%);text-align:center;font-size:clamp(12px,1.45vw,20px);font-weight:1000;letter-spacing:.20em;text-transform:uppercase;color:#e2e8f0;white-space:nowrap}.publicMeta{position:absolute;bottom:clamp(9px,1.7vh,17px);left:50%;transform:translateX(-50%);font-size:clamp(11px,1.25vw,16px);font-weight:950;letter-spacing:.16em;text-transform:uppercase;color:#94a3b8}.publicScores{display:grid;grid-template-columns:1fr 1fr;gap:clamp(8px,1.3vw,16px);min-height:0}.publicTeam{border-radius:28px;padding:clamp(16px,2.6vw,34px);display:flex;align-items:center;justify-content:space-between;overflow:hidden;position:relative;min-height:0}.publicTeam:before{content:"";position:absolute;inset:-40%;background:radial-gradient(circle,rgba(255,255,255,.22),transparent 52%);opacity:.23;animation:drift 9s linear infinite}.publicTeam.blue{background:linear-gradient(135deg,#001a70,var(--blue))}.publicTeam.orange{background:linear-gradient(135deg,#7a2100,var(--orange))}.publicTeam .name{font-size:clamp(26px,4.6vw,74px);font-weight:1000;text-transform:uppercase;letter-spacing:.08em;z-index:1}.publicTeam .num{font-size:clamp(150px,25vw,410px);font-weight:1000;line-height:.70;letter-spacing:-.11em;z-index:1}.winnerGlow{animation:champ 1.25s ease-in-out infinite}.tieGlow{animation:tiePulse 1.5s ease-in-out infinite}.exitPublic{position:fixed;right:clamp(12px,2vw,22px);top:clamp(12px,2vw,22px);z-index:25;border:1px solid rgba(255,255,255,.20);background:rgba(0,0,0,.42);backdrop-filter:blur(14px);color:white;border-radius:999px;padding:12px 16px;font-weight:1000;opacity:0;pointer-events:none;transform:translateY(-10px);transition:opacity .35s ease,transform .35s ease}.public.controlsVisible .exitPublic{opacity:1;pointer-events:auto;transform:translateY(0)}.publicWinner{position:fixed;left:50%;top:50%;transform:translate(-50%,-50%);font-size:clamp(34px,7vw,110px);font-weight:1000;letter-spacing:.08em;text-transform:uppercase;opacity:0;pointer-events:none;text-shadow:0 0 40px rgba(255,255,255,.35)}.public.active .publicWinner.show{animation:winnerText 2.4s ease-in-out infinite}@keyframes drift{from{transform:rotate(0)}to{transform:rotate(360deg)}}@keyframes softPulse{0%,100%{filter:brightness(1)}50%{filter:brightness(1.25)}}@keyframes champ{0%,100%{filter:brightness(1);transform:scale(1)}50%{filter:brightness(1.22);transform:scale(1.018)}}@keyframes tiePulse{0%,100%{filter:brightness(1)}50%{filter:brightness(1.16)}}@keyframes winnerText{0%,100%{opacity:.10;transform:translate(-50%,-50%) scale(.98) rotate(-1deg)}50%{opacity:.34;transform:translate(-50%,-50%) scale(1.03) rotate(1deg)}}


    /* v4.20 Audience Last-10 warning:
       Dramatic blinking/pulsing countdown for the final 10 seconds of either half.
       Uses Setup & Tuning: Last 10 color, period, enabled, and brightness controls. */
    .public.lastTenActive{
      --lastColor:#982ABC;
      --lastDur:1000ms;
    }
    .public.active.lastTenActive:not(.countdownActive):not(.goalBlueActive):not(.goalOrangeActive):not(.finalBlueActive):not(.finalOrangeActive):not(.finalTieActive){
      background:
        radial-gradient(circle at 50% 24%,var(--lastColor),transparent 30%),
        radial-gradient(circle at 50% 82%,rgba(255,255,255,.10),transparent 42%),
        linear-gradient(180deg,#020617,#070012 64%,#000);
      animation:scoreboardLastTenBackground var(--lastDur) cubic-bezier(.2,.9,.2,1) infinite;
    }
    .public.active.lastTenActive:not(.countdownActive):not(.goalBlueActive):not(.goalOrangeActive):not(.finalBlueActive):not(.finalOrangeActive):not(.finalTieActive):before{
      opacity:.98;
      background:conic-gradient(from 0deg,transparent,var(--lastColor),transparent,rgba(255,255,255,.25),transparent,var(--lastColor),transparent);
      animation:scoreboardLastTenSpin var(--lastDur) linear infinite;
      filter:blur(.35px);
    }
    .public.active.lastTenActive:not(.countdownActive):not(.goalBlueActive):not(.goalOrangeActive):not(.finalBlueActive):not(.finalOrangeActive):not(.finalTieActive):after{
      opacity:1;
      background:radial-gradient(circle,transparent 0 16%,rgba(255,255,255,.10) 17%,transparent 28%,var(--lastColor) 29%,transparent 46%,rgba(255,255,255,.08) 47%,transparent 62%);
      animation:scoreboardLastTenPulse var(--lastDur) cubic-bezier(.16,.9,.18,1) infinite;
    }
    .public.lastTenActive .publicState,
    .public.lastTenActive .publicMeta{
      opacity:0!important;
      visibility:hidden!important;
    }
    .public.active.lastTenActive .publicTop{
      border-color:rgba(255,255,255,.18);
      background:rgba(0,0,0,.18);
      box-shadow:inset 0 0 80px rgba(255,255,255,.06),0 0 90px var(--lastColor);
      animation:scoreboardLastTenPanel var(--lastDur) cubic-bezier(.2,.9,.2,1) infinite;
    }
    .public.active.lastTenActive .publicTimer{
      font-size:min(24vw,300px);
      line-height:.70;
      letter-spacing:-.08em;
      text-shadow:0 0 28px var(--lastColor),0 0 100px rgba(255,255,255,.25);
      animation:scoreboardLastTenNumber var(--lastDur) cubic-bezier(.16,.9,.18,1) infinite!important;
    }
    .public.active.lastTenActive .publicTeam{
      animation:scoreboardLastTenCard var(--lastDur) cubic-bezier(.2,.9,.2,1) infinite!important;
      outline:2px solid rgba(255,255,255,.18);
    }
    .public.active.lastTenActive .publicTeam .num{
      animation:scoreboardLastTenScore var(--lastDur) cubic-bezier(.2,.9,.2,1) infinite!important;
    }
    @keyframes scoreboardLastTenBackground{
      0%,100%{filter:brightness(1) saturate(1.05)}
      18%{filter:brightness(1.22) saturate(1.25)}
      52%{filter:brightness(.88) saturate(.92)}
    }
    @keyframes scoreboardLastTenSpin{to{transform:rotate(360deg)}}
    @keyframes scoreboardLastTenPulse{
      0%{transform:scale(.70);opacity:.10}
      18%{transform:scale(1.00);opacity:.95}
      100%{transform:scale(1.34);opacity:.04}
    }
    @keyframes scoreboardLastTenPanel{
      0%,100%{transform:scale(1)}
      18%{transform:scale(1.018)}
      52%{transform:scale(.995)}
    }
    @keyframes scoreboardLastTenNumber{
      0%{transform:scale(.92);filter:brightness(1.65)}
      18%{transform:scale(1.10);filter:brightness(1.45)}
      100%{transform:scale(1.00);filter:brightness(1)}
    }
    @keyframes scoreboardLastTenCard{
      0%,100%{filter:brightness(1)}
      18%{filter:brightness(1.28)}
      52%{filter:brightness(.92)}
    }
    @keyframes scoreboardLastTenScore{
      0%,100%{transform:scale(1)}
      18%{transform:scale(1.055)}
      52%{transform:scale(.995)}
    }

    /* v4.15 Audience Scoreboard motion polish:
       Subtle waves, pulses, shimmer, and score-change pop without clutter. */
    .public.active:not(.countdownActive):not(.goalBlueActive):not(.goalOrangeActive):not(.finalBlueActive):not(.finalOrangeActive):not(.finalTieActive){
      animation:scoreboardAmbientBreath 7.5s ease-in-out infinite;
    }
    .public.active:not(.countdownActive):not(.goalBlueActive):not(.goalOrangeActive):not(.finalBlueActive):not(.finalOrangeActive):not(.finalTieActive) .publicTop{
      box-shadow:inset 0 0 36px rgba(255,255,255,.035),0 18px 60px rgba(0,0,0,.26);
    }
    .public.active:not(.countdownActive):not(.goalBlueActive):not(.goalOrangeActive):not(.finalBlueActive):not(.finalOrangeActive):not(.finalTieActive) .publicTop:before{
      content:"";position:absolute;inset:-45%;pointer-events:none;
      background:conic-gradient(from 0deg,transparent,rgba(255,255,255,.08),transparent,rgba(255,255,255,.06),transparent);
      opacity:.72;animation:scoreboardTopDrift 18s linear infinite;
    }
    .public.active:not(.countdownActive):not(.goalBlueActive):not(.goalOrangeActive):not(.finalBlueActive):not(.finalOrangeActive):not(.finalTieActive) .publicTop:after{
      content:"";position:absolute;inset:0;pointer-events:none;
      background:linear-gradient(115deg,transparent 0 38%,rgba(255,255,255,.075) 46%,transparent 56% 100%);
      transform:translateX(-130%);animation:scoreboardTopSweep 6.2s ease-in-out infinite;
    }
    .public.active:not(.countdownActive):not(.goalBlueActive):not(.goalOrangeActive):not(.finalBlueActive):not(.finalOrangeActive):not(.finalTieActive) .publicTimer{
      position:relative;z-index:1;
      animation:scoreboardTimerBreath 3.2s ease-in-out infinite;
      text-shadow:0 0 24px rgba(255,255,255,.10),0 0 72px rgba(96,165,250,.08);
    }
    .publicScores{position:relative}
    .public.active:not(.countdownActive):not(.goalBlueActive):not(.goalOrangeActive):not(.finalBlueActive):not(.finalOrangeActive):not(.finalTieActive) .publicScores:before{
      content:"";position:absolute;inset:-8%;pointer-events:none;z-index:0;
      background:radial-gradient(circle at 20% 50%,rgba(0,80,255,.16),transparent 36%),
                 radial-gradient(circle at 80% 50%,rgba(255,80,0,.15),transparent 36%);
      filter:blur(20px);opacity:.75;animation:scoreboardScoreAura 8s ease-in-out infinite;
    }
    .public.active:not(.countdownActive):not(.goalBlueActive):not(.goalOrangeActive):not(.finalBlueActive):not(.finalOrangeActive):not(.finalTieActive) .publicTeam{
      isolation:isolate;
      box-shadow:inset 0 0 42px rgba(255,255,255,.045),0 26px 78px rgba(0,0,0,.42);
      animation:scoreboardCardFloat 5.5s ease-in-out infinite;
    }
    .public.active:not(.countdownActive):not(.goalBlueActive):not(.goalOrangeActive):not(.finalBlueActive):not(.finalOrangeActive):not(.finalTieActive) .publicTeam.orange{
      animation-delay:-2.15s;
    }
    .public.active:not(.countdownActive):not(.goalBlueActive):not(.goalOrangeActive):not(.finalBlueActive):not(.finalOrangeActive):not(.finalTieActive) .publicTeam:before{
      opacity:.30;
      background:
        radial-gradient(circle at 30% 30%,rgba(255,255,255,.24),transparent 36%),
        radial-gradient(circle at 72% 70%,rgba(255,255,255,.12),transparent 42%);
      animation:scoreboardCardOrb 9s linear infinite;
    }
    .publicTeam:after{
      content:"";position:absolute;inset:-25%;pointer-events:none;z-index:0;
      background:linear-gradient(112deg,transparent 0 38%,rgba(255,255,255,.18) 47%,transparent 57% 100%);
      transform:translateX(-135%) skewX(-8deg);
      opacity:0;
    }
    .public.active:not(.countdownActive):not(.goalBlueActive):not(.goalOrangeActive):not(.finalBlueActive):not(.finalOrangeActive):not(.finalTieActive) .publicTeam:after{
      opacity:.72;animation:scoreboardCardSweep 6.8s ease-in-out infinite;
    }
    .public.active:not(.countdownActive):not(.goalBlueActive):not(.goalOrangeActive):not(.finalBlueActive):not(.finalOrangeActive):not(.finalTieActive) .publicTeam.orange:after{
      animation-delay:-3.1s;
    }
    .public.active:not(.countdownActive):not(.goalBlueActive):not(.goalOrangeActive):not(.finalBlueActive):not(.finalOrangeActive):not(.finalTieActive) .publicTeam .name{
      animation:scoreboardLabelGlow 3.8s ease-in-out infinite;
      text-shadow:0 0 18px rgba(255,255,255,.16);
    }
    .public.active:not(.countdownActive):not(.goalBlueActive):not(.goalOrangeActive):not(.finalBlueActive):not(.finalOrangeActive):not(.finalTieActive) .publicTeam .num{
      animation:scoreboardNumberPulse 3.6s ease-in-out infinite;
      text-shadow:0 0 28px rgba(255,255,255,.16),0 0 70px rgba(255,255,255,.07);
    }
    .publicTeam.scorePop{
      animation:scoreboardScorePop .95s cubic-bezier(.16,.9,.18,1) both!important;
      outline:3px solid rgba(255,255,255,.30);
    }
    .publicTeam.scorePop .num{
      animation:scoreboardNumberPop .95s cubic-bezier(.16,.9,.18,1) both!important;
    }
    @keyframes scoreboardAmbientBreath{
      0%,100%{filter:brightness(1) saturate(1)}
      50%{filter:brightness(1.04) saturate(1.08)}
    }
    @keyframes scoreboardTopDrift{to{transform:rotate(360deg)}}
    @keyframes scoreboardTopSweep{
      0%,54%{transform:translateX(-135%) skewX(-10deg);opacity:0}
      64%{opacity:.85}
      100%{transform:translateX(135%) skewX(-10deg);opacity:0}
    }
    @keyframes scoreboardTimerBreath{
      0%,100%{transform:scale(1);filter:brightness(1)}
      50%{transform:scale(1.012);filter:brightness(1.10)}
    }
    @keyframes scoreboardScoreAura{
      0%,100%{transform:translate3d(0,0,0) scale(1);opacity:.65}
      50%{transform:translate3d(0,-1.2%,0) scale(1.035);opacity:.90}
    }
    @keyframes scoreboardCardFloat{
      0%,100%{transform:translateY(0) scale(1)}
      50%{transform:translateY(-5px) scale(1.006)}
    }
    @keyframes scoreboardCardOrb{to{transform:rotate(360deg)}}
    @keyframes scoreboardCardSweep{
      0%,62%{transform:translateX(-135%) skewX(-8deg);opacity:0}
      72%{opacity:.72}
      100%{transform:translateX(135%) skewX(-8deg);opacity:0}
    }
    @keyframes scoreboardLabelGlow{
      0%,100%{opacity:.88;filter:brightness(1)}
      50%{opacity:1;filter:brightness(1.18)}
    }
    @keyframes scoreboardNumberPulse{
      0%,100%{transform:scale(1);filter:brightness(1)}
      50%{transform:scale(1.018);filter:brightness(1.13)}
    }
    @keyframes scoreboardScorePop{
      0%{transform:scale(1);filter:brightness(1)}
      18%{transform:scale(1.045);filter:brightness(1.34)}
      100%{transform:scale(1);filter:brightness(1)}
    }
    @keyframes scoreboardNumberPop{
      0%{transform:scale(1);filter:brightness(1)}
      22%{transform:scale(1.12);filter:brightness(1.45)}
      100%{transform:scale(1);filter:brightness(1)}
    }

    
    /* v4.4 Audience Scoreboard polish:
       Restores the earlier v3.x subtle rotating winner-glow feel while keeping
       the newer timer-on-top / massive-score-card layout. */
    .public{overflow:hidden}
    .public:before{content:"";position:absolute;inset:-35%;opacity:0;pointer-events:none;z-index:0;background:conic-gradient(from 90deg,transparent,rgba(255,255,255,.10),transparent,rgba(255,255,255,.16),transparent);animation:scoreboardSpinGlow 5.2s linear infinite;mix-blend-mode:screen}
    .public:after{content:"";position:absolute;inset:-18%;opacity:0;pointer-events:none;z-index:0;background:radial-gradient(circle at 25% 25%,rgba(255,255,255,.10),transparent 34%),radial-gradient(circle at 78% 74%,rgba(255,255,255,.08),transparent 38%);animation:scoreboardWinnerDrift 6.5s ease-in-out infinite alternate}
    .publicTop,.publicScores,.publicWinner{position:relative;z-index:1}
    .public.winnerBlue{background:radial-gradient(circle at 18% 10%,rgba(0,80,255,.70),transparent 36%),radial-gradient(circle at 70% 88%,rgba(56,189,248,.30),transparent 34%),linear-gradient(180deg,#020617,#06163d 70%,#000)}
    .public.winnerOrange{background:radial-gradient(circle at 82% 10%,rgba(255,80,0,.76),transparent 36%),radial-gradient(circle at 30% 88%,rgba(250,204,21,.26),transparent 34%),linear-gradient(180deg,#170901,#3a1605 72%,#000)}
    .public.winnerTie{background:linear-gradient(90deg,rgba(0,80,255,.38),rgba(2,6,23,.94),rgba(255,80,0,.38)),#020617}
    .public.winnerBlue:before,.public.winnerOrange:before,.public.winnerTie:before,.public.winnerBlue:after,.public.winnerOrange:after,.public.winnerTie:after{opacity:1}
    .public.winnerBlue .publicTeam.blue,.public.winnerOrange .publicTeam.orange{animation:scoreboardChampionPulse .82s ease-in-out infinite alternate,scoreboardChampionTilt 4.8s ease-in-out infinite}
    .public.winnerBlue .publicTeam.orange,.public.winnerOrange .publicTeam.blue{filter:saturate(.58) brightness(.62)}
    .public.winnerTie .publicTeam.blue,.public.winnerTie .publicTeam.orange{animation:scoreboardTieBreath 1.45s ease-in-out infinite}
    .public.winnerBlue .publicTimer,.public.winnerOrange .publicTimer,.public.winnerTie .publicTimer{text-shadow:0 0 42px rgba(255,255,255,.24)}
    .publicWinner{letter-spacing:.10em;text-shadow:0 0 45px rgba(255,255,255,.34),0 0 95px rgba(255,255,255,.16)}
    .public.active .publicWinner.show{animation:scoreboardWinnerText 1.25s ease-in-out infinite alternate}
    @keyframes scoreboardSpinGlow{to{transform:rotate(360deg)}}
    @keyframes scoreboardWinnerDrift{from{transform:translate3d(-2%,1%,0) rotate(0deg) scale(1)}to{transform:translate3d(2%,-1%,0) rotate(8deg) scale(1.04)}}
    @keyframes scoreboardChampionPulse{from{transform:scale(1.005);box-shadow:0 28px 90px rgba(0,0,0,.45),0 0 30px rgba(255,255,255,.10)}to{transform:scale(1.026);box-shadow:0 32px 110px rgba(0,0,0,.55),0 0 76px rgba(255,255,255,.22)}}
    @keyframes scoreboardChampionTilt{0%,100%{rotate:-.20deg}50%{rotate:.20deg}}
    @keyframes scoreboardTieBreath{0%,100%{filter:brightness(1);transform:scale(1)}50%{filter:brightness(1.15);transform:scale(1.012)}}



    /* v4.10 Audience final/winner celebration:
       Full-screen match-end celebration, only visible inside Audience Scoreboard. */
    .public.active.finalBlueActive,.public.active.finalOrangeActive,.public.active.finalTieActive{display:grid;grid-template-rows:1fr;background:#020617}
    .public.finalBlueActive{--finalColor:var(--blue);--finalColorSoft:rgba(0,80,255,.46);background:radial-gradient(circle at 50% 18%,rgba(0,80,255,.88),transparent 31%),radial-gradient(circle at 18% 82%,rgba(56,189,248,.32),transparent 36%),linear-gradient(180deg,#020617,#06163d 64%,#000)}
    .public.finalOrangeActive{--finalColor:var(--orange);--finalColorSoft:rgba(255,80,0,.50);background:radial-gradient(circle at 50% 18%,rgba(255,80,0,.90),transparent 31%),radial-gradient(circle at 82% 82%,rgba(250,204,21,.28),transparent 36%),linear-gradient(180deg,#170901,#3a1605 66%,#000)}
    .public.finalTieActive{--finalColor:#ffffff;--finalColorSoft:rgba(255,255,255,.24);background:linear-gradient(90deg,rgba(0,80,255,.55),rgba(2,6,23,.96),rgba(255,80,0,.55)),radial-gradient(circle at 50% 20%,rgba(255,255,255,.16),transparent 34%),#020617}
    .public.finalBlueActive:before,.public.finalOrangeActive:before,.public.finalTieActive:before{opacity:.98;background:conic-gradient(from 0deg,transparent,var(--finalColor),transparent,rgba(255,255,255,.35),transparent,var(--finalColor),transparent);animation:scoreboardFinalSpin 2.25s linear infinite;filter:blur(.25px)}
    .public.finalBlueActive:after,.public.finalOrangeActive:after,.public.finalTieActive:after{opacity:1;background:radial-gradient(circle,transparent 0 14%,rgba(255,255,255,.12) 15%,transparent 27%,var(--finalColorSoft) 28%,transparent 44%,rgba(255,255,255,.10) 45%,transparent 63%);animation:scoreboardFinalPulse 1.65s cubic-bezier(.16,.9,.18,1) infinite}
    .public.finalBlueActive .publicTop,.public.finalOrangeActive .publicTop,.public.finalTieActive .publicTop{border-color:transparent;background:transparent;border-radius:0;min-height:100%;box-shadow:none}
    .public.finalBlueActive .publicTimer,.public.finalOrangeActive .publicTimer,.public.finalTieActive .publicTimer{font-size:min(18vw,230px);line-height:.80;letter-spacing:-.055em;text-transform:uppercase;text-align:center;text-shadow:0 0 42px var(--finalColor),0 0 125px rgba(255,255,255,.32);animation:scoreboardFinalTitlePop 1.65s cubic-bezier(.16,.9,.18,1) infinite}
    .public.finalBlueActive .publicState,.public.finalOrangeActive .publicState,.public.finalTieActive .publicState{top:clamp(22px,5vh,62px);font-size:min(4.8vw,62px);letter-spacing:.30em;color:#fff;text-shadow:0 0 30px var(--finalColor)}
    .public.finalBlueActive .publicMeta,.public.finalOrangeActive .publicMeta,.public.finalTieActive .publicMeta{bottom:clamp(26px,6vh,70px);font-size:min(3.2vw,38px);opacity:.94;color:#e2e8f0}
    .public.finalBlueActive .publicScores,.public.finalOrangeActive .publicScores,.public.finalTieActive .publicScores{position:absolute;left:2.4vw;right:2.4vw;bottom:2.4vh;height:min(30vh,255px);opacity:.92;transform:translateY(0) scale(.98);filter:saturate(1.08)}
    .public.finalBlueActive .publicTeam.blue,.public.finalOrangeActive .publicTeam.orange{animation:scoreboardFinalWinnerCard .95s ease-in-out infinite alternate;box-shadow:0 0 70px var(--finalColorSoft),0 24px 90px rgba(0,0,0,.50)}
    .public.finalBlueActive .publicTeam.orange,.public.finalOrangeActive .publicTeam.blue{opacity:.42;filter:saturate(.55) brightness(.68)}
    .public.finalTieActive .publicTeam.blue,.public.finalTieActive .publicTeam.orange{animation:scoreboardFinalTieCard 1.35s ease-in-out infinite alternate;box-shadow:0 0 48px rgba(255,255,255,.16),0 24px 80px rgba(0,0,0,.44)}
    .public.finalBlueActive .publicWinner,.public.finalOrangeActive .publicWinner,.public.finalTieActive .publicWinner{display:none}
    @keyframes scoreboardFinalSpin{to{transform:rotate(360deg)}}
    @keyframes scoreboardFinalPulse{0%{transform:scale(.70);opacity:.08}22%{transform:scale(1.02);opacity:.92}100%{transform:scale(1.42);opacity:.04}}
    @keyframes scoreboardFinalTitlePop{0%{transform:scale(.96);filter:brightness(1.50)}18%{transform:scale(1.045);filter:brightness(1.30)}100%{transform:scale(1.00);filter:brightness(1)}}
    @keyframes scoreboardFinalWinnerCard{from{transform:scale(1.000) rotate(-.15deg)}to{transform:scale(1.032) rotate(.15deg)}}
    @keyframes scoreboardFinalTieCard{from{transform:scale(1.000);filter:brightness(1)}to{transform:scale(1.022);filter:brightness(1.18)}}

    /* v4.12 subtle final confetti:
       Light celebratory texture for the audience winner page, intentionally not overwhelming. */
    .public.finalBlueActive .publicTop:before,
    .public.finalOrangeActive .publicTop:before,
    .public.finalTieActive .publicTop:before,
    .public.finalBlueActive .publicTop:after,
    .public.finalOrangeActive .publicTop:after,
    .public.finalTieActive .publicTop:after{
      content:"";position:absolute;inset:-18%;z-index:-1;pointer-events:none;
      background-repeat:repeat;mix-blend-mode:screen;border-radius:inherit
    }
    .public.finalBlueActive .publicTop:before{
      opacity:.24;background-image:
        radial-gradient(circle,rgba(255,255,255,.75) 0 2px,transparent 2.7px),
        radial-gradient(circle,rgba(56,189,248,.70) 0 2px,transparent 2.8px),
        radial-gradient(circle,rgba(0,80,255,.72) 0 2.3px,transparent 3px);
      background-size:112px 96px,156px 132px,198px 166px;
      animation:scoreboardConfettiDriftA 7.5s linear infinite
    }
    .public.finalOrangeActive .publicTop:before{
      opacity:.24;background-image:
        radial-gradient(circle,rgba(255,255,255,.72) 0 2px,transparent 2.7px),
        radial-gradient(circle,rgba(250,204,21,.66) 0 2px,transparent 2.8px),
        radial-gradient(circle,rgba(255,80,0,.72) 0 2.3px,transparent 3px);
      background-size:112px 96px,156px 132px,198px 166px;
      animation:scoreboardConfettiDriftA 7.5s linear infinite
    }
    .public.finalTieActive .publicTop:before{
      opacity:.21;background-image:
        radial-gradient(circle,rgba(255,255,255,.68) 0 2px,transparent 2.7px),
        radial-gradient(circle,rgba(0,80,255,.66) 0 2px,transparent 2.8px),
        radial-gradient(circle,rgba(255,80,0,.66) 0 2.3px,transparent 3px);
      background-size:112px 96px,156px 132px,198px 166px;
      animation:scoreboardConfettiDriftA 7.5s linear infinite
    }
    .public.finalBlueActive .publicTop:after,
    .public.finalOrangeActive .publicTop:after,
    .public.finalTieActive .publicTop:after{
      opacity:.16;background-image:
        linear-gradient(35deg,transparent 0 47%,rgba(255,255,255,.60) 48% 52%,transparent 53% 100%),
        linear-gradient(125deg,transparent 0 47%,var(--finalColor) 48% 52%,transparent 53% 100%);
      background-size:138px 118px,184px 152px;
      animation:scoreboardConfettiDriftB 10s linear infinite
    }
    @keyframes scoreboardConfettiDriftA{
      from{transform:translate3d(-1.5%,-9%,0) rotate(-2deg)}
      to{transform:translate3d(1.5%,9%,0) rotate(3deg)}
    }
    @keyframes scoreboardConfettiDriftB{
      from{transform:translate3d(2%,-12%,0) rotate(4deg)}
      to{transform:translate3d(-2%,10%,0) rotate(-5deg)}
    }


    /* v4.8 Audience goal celebration:
       Full-screen team-colored celebration while the operator resets the ball. */
    .public.active.goalBlueActive,.public.active.goalOrangeActive{display:grid;grid-template-rows:1fr;background:#020617}
    .public.goalBlueActive{--goalColor:var(--blue);--goalColorSoft:rgba(0,80,255,.42);background:radial-gradient(circle at 50% 18%,rgba(0,80,255,.82),transparent 30%),radial-gradient(circle at 18% 82%,rgba(56,189,248,.28),transparent 36%),linear-gradient(180deg,#020617,#06163d 64%,#000)}
    .public.goalOrangeActive{--goalColor:var(--orange);--goalColorSoft:rgba(255,80,0,.46);background:radial-gradient(circle at 50% 18%,rgba(255,80,0,.86),transparent 30%),radial-gradient(circle at 82% 82%,rgba(250,204,21,.24),transparent 36%),linear-gradient(180deg,#170901,#3a1605 66%,#000)}
    .public.goalBlueActive:before,.public.goalOrangeActive:before{opacity:.98;background:conic-gradient(from 0deg,transparent,var(--goalColor),transparent,rgba(255,255,255,.30),transparent,var(--goalColor),transparent);animation:scoreboardGoalSpin 1.15s linear infinite;filter:blur(.3px)}
    .public.goalBlueActive:after,.public.goalOrangeActive:after{opacity:1;background:radial-gradient(circle,transparent 0 16%,rgba(255,255,255,.12) 17%,transparent 29%,var(--goalColorSoft) 30%,transparent 45%,rgba(255,255,255,.10) 46%,transparent 62%);animation:scoreboardGoalPulse 1.05s cubic-bezier(.16,.9,.18,1) infinite}
    .public.goalBlueActive .publicTop,.public.goalOrangeActive .publicTop{border-color:transparent;background:transparent;border-radius:0;min-height:100%;box-shadow:none}
    .public.goalBlueActive .publicTimer,.public.goalOrangeActive .publicTimer{font-size:min(24vw,260px);line-height:.78;letter-spacing:-.07em;text-transform:uppercase;text-shadow:0 0 38px var(--goalColor),0 0 110px rgba(255,255,255,.30);animation:scoreboardGoalTextPop 1.05s cubic-bezier(.16,.9,.18,1) infinite}
    .public.goalBlueActive .publicState,.public.goalOrangeActive .publicState{top:clamp(22px,5vh,62px);font-size:min(5.2vw,68px);letter-spacing:.28em;color:#fff;text-shadow:0 0 30px var(--goalColor)}
    .public.goalBlueActive .publicMeta,.public.goalOrangeActive .publicMeta{bottom:clamp(26px,6vh,70px);font-size:min(3.4vw,40px);opacity:.92;color:#e2e8f0}
    .public.goalBlueActive .publicScores,.public.goalOrangeActive .publicScores{position:absolute;left:2.4vw;right:2.4vw;bottom:2.4vh;height:min(28vh,240px);opacity:.82;transform:translateY(0) scale(.96);filter:saturate(1.05)}
    .public.goalBlueActive .publicTeam.blue,.public.goalOrangeActive .publicTeam.orange{animation:scoreboardGoalCardPop .72s ease-in-out infinite alternate;box-shadow:0 0 60px var(--goalColorSoft),0 24px 80px rgba(0,0,0,.45)}
    .public.goalBlueActive .publicTeam.orange,.public.goalOrangeActive .publicTeam.blue{opacity:.44;filter:saturate(.55) brightness(.70)}
    .public.goalBlueActive .publicWinner,.public.goalOrangeActive .publicWinner{display:none}
    @keyframes scoreboardGoalSpin{to{transform:rotate(360deg)}}
    @keyframes scoreboardGoalPulse{0%{transform:scale(.62);opacity:.08}18%{transform:scale(1.00);opacity:.95}100%{transform:scale(1.34);opacity:.04}}
    @keyframes scoreboardGoalTextPop{0%{transform:scale(.90);filter:brightness(1.55)}16%{transform:scale(1.06);filter:brightness(1.35)}100%{transform:scale(1.00);filter:brightness(1)}}
    @keyframes scoreboardGoalCardPop{from{transform:scale(1.000)}to{transform:scale(1.025)}}

    /* v4.11 refined audience goal celebration:
       Cleaner spectator-facing celebration with no operator/reset language. */
    .public.goalBlueActive,.public.goalOrangeActive{--goalAccent:white}
    .public.goalBlueActive .publicTop,.public.goalOrangeActive .publicTop{isolation:isolate}
    .public.goalBlueActive .publicTop:before,.public.goalOrangeActive .publicTop:before{
      content:"";position:absolute;inset:-22%;z-index:-1;opacity:.55;
      background:linear-gradient(115deg,transparent 0 34%,rgba(255,255,255,.20) 42%,transparent 52% 100%);
      animation:scoreboardGoalLightSweep 1.65s cubic-bezier(.16,.9,.18,1) infinite;
      transform:translateX(-38%) rotate(0deg)
    }
    .public.goalBlueActive .publicTop:after,.public.goalOrangeActive .publicTop:after{
      content:"";position:absolute;inset:6%;z-index:-1;border-radius:50%;
      border:clamp(6px,1.1vw,14px) solid rgba(255,255,255,.08);
      box-shadow:0 0 50px var(--goalColor),inset 0 0 70px rgba(255,255,255,.10);
      animation:scoreboardGoalHalo 1.15s cubic-bezier(.16,.9,.18,1) infinite
    }
    .public.goalBlueActive .publicTimer,.public.goalOrangeActive .publicTimer{
      font-size:min(27vw,300px);
      letter-spacing:-.08em;
      text-shadow:0 0 28px var(--goalColor),0 0 80px var(--goalColorSoft),0 0 140px rgba(255,255,255,.28);
      filter:drop-shadow(0 22px 38px rgba(0,0,0,.42))
    }
    .public.goalBlueActive .publicState,.public.goalOrangeActive .publicState{
      background:rgba(0,0,0,.20);border:1px solid rgba(255,255,255,.14);
      border-radius:999px;padding:.28em .72em;backdrop-filter:blur(12px)
    }
    .public.goalBlueActive .publicMeta,.public.goalOrangeActive .publicMeta{
      opacity:.86;letter-spacing:.22em;background:rgba(0,0,0,.16);
      border:1px solid rgba(255,255,255,.10);border-radius:999px;padding:.32em .85em;
      backdrop-filter:blur(10px)
    }
    .public.goalBlueActive .publicScores,.public.goalOrangeActive .publicScores{
      height:min(31vh,270px);opacity:.92;gap:clamp(10px,1.6vw,22px)
    }
    .public.goalBlueActive .publicTeam.blue,.public.goalOrangeActive .publicTeam.orange{
      animation:scoreboardGoalCardPop .72s ease-in-out infinite alternate,scoreboardGoalScoreGlow 1.15s ease-in-out infinite;
      outline:2px solid rgba(255,255,255,.22)
    }
    @keyframes scoreboardGoalLightSweep{
      0%{transform:translateX(-46%) skewX(-10deg);opacity:0}
      22%{opacity:.68}
      100%{transform:translateX(46%) skewX(-10deg);opacity:0}
    }
    @keyframes scoreboardGoalHalo{
      0%{transform:scale(.82);opacity:.15}
      18%{transform:scale(1.00);opacity:.70}
      100%{transform:scale(1.18);opacity:.10}
    }



    /* v4.16 clean final celebration:
       Hide all lower meta/final-score text during the winner screen. */
    .public.finalBlueActive .publicMeta,
    .public.finalOrangeActive .publicMeta,
    .public.finalTieActive .publicMeta{
      display:none!important;
      opacity:0!important;
      visibility:hidden!important;
    }

    /* v4.14 final score placement:
       Keep final score visible above the bottom scorecards instead of overlapping them. */
    .public.finalBlueActive .publicMeta,
    .public.finalOrangeActive .publicMeta,
    .public.finalTieActive .publicMeta{
      bottom:calc(min(30vh,255px) + 5.8vh)!important;
      top:auto!important;
      z-index:3;
      white-space:nowrap;
      background:rgba(0,0,0,.28);
      border:1px solid rgba(255,255,255,.16);
      border-radius:999px;
      padding:.36em .95em;
      backdrop-filter:blur(12px);
      text-shadow:0 0 24px rgba(255,255,255,.16);
    }


    /* v4.17 GOAL text:
       During audience goal celebration, center text is the word GOAL only. */
    .public.goalBlueActive .publicTimer,
    .public.goalOrangeActive .publicTimer{
      font-size:min(34vw,390px)!important;
      letter-spacing:.02em!important;
      line-height:.78;
      text-transform:uppercase;
    }

    /* v4.13 clean goal scoreboard:
       During goal celebration, show only timer, scorecards, and animation. */
    .public.goalBlueActive .publicState,
    .public.goalOrangeActive .publicState,
    .public.goalBlueActive .publicMeta,
    .public.goalOrangeActive .publicMeta{
      opacity:0!important;
      visibility:hidden!important;
    }
    .public.goalBlueActive .publicTimer,
    .public.goalOrangeActive .publicTimer{
      font-size:min(24vw,260px);
      letter-spacing:-.085em;
    }

    @keyframes scoreboardGoalScoreGlow{
      0%,100%{filter:brightness(1.05)}
      50%{filter:brightness(1.24)}
    }


    /* v4.6 Audience countdown:
       Full-screen countdown number for spectators. CSS timing follows countdownStepMs exactly. */
    .public{--countDur:1000ms}
    .publicTop,.publicScores{transition:opacity .55s ease,transform .55s ease,filter .55s ease,border-radius .55s ease}
    .publicTimer,.publicState,.publicMeta{transition:font-size .45s ease,opacity .45s ease,text-shadow .45s ease,transform .45s ease}
    .public.countdownActive.active{display:grid;grid-template-rows:1fr}
    .public.countdownActive{background:radial-gradient(circle at 50% 18%,var(--countColor,#00FF00),transparent 28%),radial-gradient(circle at 50% 70%,rgba(255,255,255,.10),transparent 44%),linear-gradient(180deg,#020617,#050816 62%,#000)}
    .public.countdownActive:before{opacity:.96;background:conic-gradient(from 0deg,transparent,var(--countColor,#00FF00),transparent,rgba(255,255,255,.24),transparent);animation:scoreboardCountdownSpin var(--countDur) linear infinite;filter:blur(.2px)}
    .public.countdownActive:after{opacity:1;background:radial-gradient(circle,transparent 0 18%,rgba(255,255,255,.10) 19%,transparent 31%,var(--countColor,#00FF00) 32%,transparent 45%,rgba(255,255,255,.08) 46%,transparent 60%);animation:scoreboardCountdownPulse var(--countDur) cubic-bezier(.2,.9,.2,1) infinite}
    .public.countdownActive .publicTop{border-color:transparent;background:transparent;border-radius:0;min-height:100%;box-shadow:none}
    .public.countdownActive .publicTimer{font-size:min(58vw,620px);line-height:.72;letter-spacing:-.09em;text-shadow:0 0 36px var(--countColor,#00FF00),0 0 100px rgba(255,255,255,.34);animation:scoreboardCountdownNumber var(--countDur) cubic-bezier(.2,.9,.2,1) infinite}
    .public.countdownActive .publicState{top:clamp(22px,5vh,62px);font-size:min(5.6vw,72px);letter-spacing:.28em;color:#fff;text-shadow:0 0 30px var(--countColor,#00FF00)}
    .public.countdownActive .publicMeta{bottom:clamp(26px,6vh,70px);font-size:min(3.5vw,42px);opacity:.9;color:#e2e8f0}
    .public.countdownActive .publicScores{position:absolute;inset:auto 2.5vw 2.5vh 2.5vw;opacity:0;transform:translateY(32px) scale(.90);filter:blur(3px) saturate(.65);pointer-events:none}
    .public.countdownActive .publicWinner{display:none}
    @keyframes scoreboardCountdownSpin{to{transform:rotate(360deg)}}
    @keyframes scoreboardCountdownPulse{0%{transform:scale(.70);opacity:.10}18%{transform:scale(1.00);opacity:.92}100%{transform:scale(1.30);opacity:.05}}
    @keyframes scoreboardCountdownNumber{0%{transform:scale(.86);filter:brightness(1.7)}15%{transform:scale(1.08);filter:brightness(1.42)}100%{transform:scale(1.00);filter:brightness(1)}}

    @keyframes scoreboardWinnerText{from{opacity:.22;transform:translate(-50%,-50%) scale(.985) rotate(-.35deg);filter:brightness(1)}to{opacity:.48;transform:translate(-50%,-50%) scale(1.025) rotate(.35deg);filter:brightness(1.22)}}


    /* v4.23 GOAL ring burst:
       Two transparent rings expand from publicTop center when goal state is entered.
       .goalRingEl divs sit inside .publicTop; goalRingsActive class triggers them. */
    .goalRingEl{position:absolute;top:50%;left:50%;transform:translate(-50%,-50%);width:0;height:0;border-radius:50%;border-style:solid;border-color:currentColor;border-width:0;pointer-events:none;opacity:0;z-index:3;will-change:width,height,opacity}
    .public.goalBlueActive .goalRingEl{color:var(--blue)}
    .public.goalOrangeActive .goalRingEl{color:var(--orange)}
    @keyframes v423RingBurst{0%{width:0;height:0;border-width:clamp(4px,.8vw,12px);opacity:.88}65%{opacity:.26}100%{width:320vmax;height:320vmax;border-width:.3px;opacity:0}}
    .goalRingsActive .goalRingEl:first-child{animation:v423RingBurst 1.65s cubic-bezier(.08,.90,.16,1) both}
    .goalRingsActive .goalRingEl:nth-child(2){animation:v423RingBurst 1.65s cubic-bezier(.08,.90,.16,1) .44s both}

    /* v4.23 GOAL text entrance:
       Plays once on state entry (goalJustEntered class), then continuous pulse takes over. */
    @keyframes v423GoalEntrance{0%{transform:scale(3.8) rotate(-2deg);opacity:0;filter:blur(45px) brightness(2.8)}25%{transform:scale(.88) rotate(1deg);opacity:1;filter:blur(0) brightness(1.55)}48%{transform:scale(1.08) rotate(-.4deg);filter:brightness(1.20)}100%{transform:scale(1) rotate(0);filter:brightness(1)}}
    .goalJustEntered .publicTimer{animation:v423GoalEntrance .80s cubic-bezier(.16,1,.3,1) both !important}

    /* v4.23 score number roll-up:
       When a score changes, the number slides up from below with blur fade-in. */
    @keyframes v423ScoreRoll{0%{transform:translateY(75%) scale(.78);opacity:0;filter:blur(6px)}52%{transform:translateY(-9%) scale(1.07);opacity:1;filter:blur(0)}72%{transform:translateY(3%) scale(1.02)}100%{transform:translateY(0) scale(1)}}
    .v423RollIn{animation:v423ScoreRoll .68s cubic-bezier(.16,1,.3,1) both}

    /* v4.23 improved score card pop — bigger snap, cleaner falloff */
    @keyframes v423CardPop{0%{transform:scale(1);filter:brightness(1);outline-color:rgba(255,255,255,0)}14%{transform:scale(1.068);filter:brightness(1.58);outline-color:rgba(255,255,255,.55)}55%{transform:scale(1.022);filter:brightness(1.20)}100%{transform:scale(1);filter:brightness(1);outline-color:rgba(255,255,255,.30)}}
    @keyframes v423NumPop{0%{transform:scale(1) translateY(0);filter:brightness(1)}16%{transform:scale(1.20) translateY(-4%);filter:brightness(1.68)}60%{transform:scale(1.03) translateY(-1%)}100%{transform:scale(1) translateY(0);filter:brightness(1)}}
    .publicTeam.scorePop{animation:v423CardPop 1.15s cubic-bezier(.16,.9,.18,1) both!important;outline:3px solid rgba(255,255,255,.30)}
    .publicTeam.scorePop .num{animation:v423NumPop 1.15s cubic-bezier(.16,.9,.18,1) both!important}


    /* v4.30 Audience Scoreboard premium motion polish:
       Layout stays the same. This only adds visible, controlled motion to the
       normal audience scoreboard state. Goal, countdown, final, and last-10
       special screens keep their own existing behavior. */
    .public{
      --v430BlueSoft:rgba(0,80,255,.30);
      --v430OrangeSoft:rgba(255,80,0,.30);
      --v430Glass:rgba(255,255,255,.065);
    }
    .public.active:not(.countdownActive):not(.goalBlueActive):not(.goalOrangeActive):not(.finalBlueActive):not(.finalOrangeActive):not(.finalTieActive){
      background:
        radial-gradient(circle at 14% 8%,rgba(0,80,255,.42),transparent 34%),
        radial-gradient(circle at 86% 8%,rgba(255,80,0,.40),transparent 34%),
        radial-gradient(circle at 50% 112%,rgba(255,255,255,.075),transparent 42%),
        linear-gradient(180deg,#061121 0%,#020617 58%,#00040b 100%);
      animation:v430ArenaBreath 6.8s ease-in-out infinite;
    }
    .public.active:not(.countdownActive):not(.goalBlueActive):not(.goalOrangeActive):not(.finalBlueActive):not(.finalOrangeActive):not(.finalTieActive):before{
      opacity:.42;
      background:
        conic-gradient(from 0deg at 50% 50%,transparent 0 12%,rgba(255,255,255,.10) 18%,transparent 26%,rgba(255,255,255,.075) 40%,transparent 56%,rgba(255,255,255,.10) 72%,transparent 86% 100%);
      filter:blur(.4px);
      animation:v430SlowHalo 24s linear infinite;
    }
    .public.active:not(.countdownActive):not(.goalBlueActive):not(.goalOrangeActive):not(.finalBlueActive):not(.finalOrangeActive):not(.finalTieActive):after{
      opacity:.36;
      background:
        linear-gradient(90deg,transparent,rgba(255,255,255,.08),transparent),
        repeating-linear-gradient(0deg,rgba(255,255,255,.035) 0 1px,transparent 1px 13px);
      mix-blend-mode:screen;
      animation:v430ScreenSheen 7.8s ease-in-out infinite;
    }
    .public.active:not(.countdownActive):not(.goalBlueActive):not(.goalOrangeActive):not(.finalBlueActive):not(.finalOrangeActive):not(.finalTieActive) .publicTop{
      border-color:rgba(255,255,255,.16);
      background:
        radial-gradient(circle at 50% 0%,rgba(255,255,255,.12),transparent 42%),
        linear-gradient(180deg,rgba(255,255,255,.075),rgba(255,255,255,.028));
      box-shadow:
        inset 0 0 42px rgba(255,255,255,.045),
        0 22px 70px rgba(0,0,0,.34),
        0 0 45px rgba(96,165,250,.06);
      transform:translateZ(0);
      animation:v430TopFloat 5.8s ease-in-out infinite;
    }
    .public.active:not(.countdownActive):not(.goalBlueActive):not(.goalOrangeActive):not(.finalBlueActive):not(.finalOrangeActive):not(.finalTieActive) .publicState{
      opacity:.94;
      text-shadow:0 0 16px rgba(255,255,255,.18);
      animation:v430StateGlow 3.4s ease-in-out infinite;
    }
    .public.active:not(.countdownActive):not(.goalBlueActive):not(.goalOrangeActive):not(.finalBlueActive):not(.finalOrangeActive):not(.finalTieActive) .publicMeta{
      opacity:.92;
      animation:v430MetaSlide 5.2s ease-in-out infinite;
    }
    .publicTimer{
      will-change:transform,filter,text-shadow;
    }
    .public.active:not(.countdownActive):not(.goalBlueActive):not(.goalOrangeActive):not(.finalBlueActive):not(.finalOrangeActive):not(.finalTieActive) .publicTimer{
      text-shadow:
        0 0 22px rgba(255,255,255,.16),
        0 0 72px rgba(96,165,250,.12),
        0 0 120px rgba(255,80,0,.06);
      animation:v430TimerIdle 2.8s ease-in-out infinite;
    }
    .publicTimer.timerTick{
      animation:v430TimerTick .58s cubic-bezier(.16,1,.3,1) both!important;
    }
    .publicScores{
      gap:clamp(10px,1.45vw,20px);
    }
    .public.active:not(.countdownActive):not(.goalBlueActive):not(.goalOrangeActive):not(.finalBlueActive):not(.finalOrangeActive):not(.finalTieActive) .publicScores:after{
      content:"";
      position:absolute;
      left:50%;
      top:7%;
      bottom:7%;
      width:1px;
      transform:translateX(-50%);
      pointer-events:none;
      background:linear-gradient(180deg,transparent,rgba(255,255,255,.30),transparent);
      box-shadow:0 0 20px rgba(255,255,255,.18);
      opacity:.36;
      animation:v430CenterLine 4.8s ease-in-out infinite;
      z-index:2;
    }
    .publicTeam{
      border:1px solid rgba(255,255,255,.16);
      transform:translateZ(0);
      will-change:transform,filter,box-shadow;
    }
    .publicTeam .name,.publicTeam .num{
      position:relative;
      z-index:2;
    }
    .publicTeam .name{
      text-shadow:0 0 18px rgba(255,255,255,.20);
    }
    .publicTeam .num{
      text-shadow:
        0 0 24px rgba(255,255,255,.20),
        0 0 88px rgba(255,255,255,.10);
    }
    .publicTeam.blue{
      background:
        radial-gradient(circle at 18% 10%,rgba(255,255,255,.16),transparent 30%),
        radial-gradient(circle at 90% 95%,rgba(96,165,250,.24),transparent 36%),
        linear-gradient(135deg,#00155f 0%,var(--blue) 78%);
    }
    .publicTeam.orange{
      background:
        radial-gradient(circle at 82% 10%,rgba(255,255,255,.15),transparent 30%),
        radial-gradient(circle at 10% 95%,rgba(250,204,21,.22),transparent 36%),
        linear-gradient(135deg,#6f1d00 0%,var(--orange) 78%);
    }
    .public.active:not(.countdownActive):not(.goalBlueActive):not(.goalOrangeActive):not(.finalBlueActive):not(.finalOrangeActive):not(.finalTieActive) .publicTeam{
      animation:v430CardHover 4.8s ease-in-out infinite;
      box-shadow:
        inset 0 0 54px rgba(255,255,255,.055),
        0 28px 86px rgba(0,0,0,.48);
    }
    .public.active:not(.countdownActive):not(.goalBlueActive):not(.goalOrangeActive):not(.finalBlueActive):not(.finalOrangeActive):not(.finalTieActive) .publicTeam.orange{
      animation-delay:-2.35s;
    }
    .public.active:not(.countdownActive):not(.goalBlueActive):not(.goalOrangeActive):not(.finalBlueActive):not(.finalOrangeActive):not(.finalTieActive) .publicTeam:before{
      opacity:.34;
      background:
        radial-gradient(circle at 24% 28%,rgba(255,255,255,.28),transparent 32%),
        radial-gradient(circle at 78% 72%,rgba(255,255,255,.13),transparent 43%),
        conic-gradient(from 0deg,transparent,rgba(255,255,255,.08),transparent,rgba(255,255,255,.06),transparent);
      animation:v430CardEnergy 12s linear infinite;
    }
    .public.active:not(.countdownActive):not(.goalBlueActive):not(.goalOrangeActive):not(.finalBlueActive):not(.finalOrangeActive):not(.finalTieActive) .publicTeam:after{
      background:
        linear-gradient(112deg,transparent 0 36%,rgba(255,255,255,.24) 47%,transparent 58% 100%);
      opacity:.68;
      animation:v430CardSheen 5.9s ease-in-out infinite;
    }
    .public.active:not(.countdownActive):not(.goalBlueActive):not(.goalOrangeActive):not(.finalBlueActive):not(.finalOrangeActive):not(.finalTieActive) .publicTeam.orange:after{
      animation-delay:-2.95s;
    }
    .publicTeam.scoreLead{
      outline:2px solid rgba(255,255,255,.27);
      filter:brightness(1.08) saturate(1.12);
    }
    .publicTeam.scoreBehind{
      filter:brightness(.88) saturate(.84);
    }
    .publicTeam.scoreTied{
      outline:1px solid rgba(255,255,255,.18);
    }
    .public.active.scoreBlueLead:not(.countdownActive):not(.goalBlueActive):not(.goalOrangeActive):not(.finalBlueActive):not(.finalOrangeActive):not(.finalTieActive) .publicTeam.blue{
      box-shadow:
        inset 0 0 64px rgba(255,255,255,.075),
        0 30px 95px rgba(0,0,0,.50),
        0 0 52px rgba(0,80,255,.34);
      animation:v430LeaderPulse 2.4s ease-in-out infinite;
    }
    .public.active.scoreOrangeLead:not(.countdownActive):not(.goalBlueActive):not(.goalOrangeActive):not(.finalBlueActive):not(.finalOrangeActive):not(.finalTieActive) .publicTeam.orange{
      box-shadow:
        inset 0 0 64px rgba(255,255,255,.075),
        0 30px 95px rgba(0,0,0,.50),
        0 0 52px rgba(255,80,0,.34);
      animation:v430LeaderPulse 2.4s ease-in-out infinite;
    }
    .public.active.scoreTieLive:not(.countdownActive):not(.goalBlueActive):not(.goalOrangeActive):not(.finalBlueActive):not(.finalOrangeActive):not(.finalTieActive) .publicTeam.blue,
    .public.active.scoreTieLive:not(.countdownActive):not(.goalBlueActive):not(.goalOrangeActive):not(.finalBlueActive):not(.finalOrangeActive):not(.finalTieActive) .publicTeam.orange{
      animation:v430TieBalance 3s ease-in-out infinite;
    }
    .public.active:not(.countdownActive):not(.goalBlueActive):not(.goalOrangeActive):not(.finalBlueActive):not(.finalOrangeActive):not(.finalTieActive) .publicTeam .name:after{
      content:"";
      display:block;
      height:3px;
      width:42%;
      margin-top:clamp(8px,1vh,14px);
      border-radius:999px;
      background:rgba(255,255,255,.72);
      box-shadow:0 0 18px rgba(255,255,255,.24);
      transform-origin:left center;
      animation:v430NameUnderline 4.4s ease-in-out infinite;
    }
    .public.active:not(.countdownActive):not(.goalBlueActive):not(.goalOrangeActive):not(.finalBlueActive):not(.finalOrangeActive):not(.finalTieActive) .publicTeam.orange .name:after{
      animation-delay:-2.2s;
    }
    .publicTeam.scorePop{
      animation:v430ScoreCardPop 1.10s cubic-bezier(.16,.9,.18,1) both!important;
      outline:3px solid rgba(255,255,255,.42)!important;
    }
    .publicTeam.scorePop:after{
      opacity:1!important;
      animation:v430ScoreFlashSweep 1.05s cubic-bezier(.16,.9,.18,1) both!important;
    }
    .publicTeam.scorePop .num{
      animation:v430ScoreNumberPop 1.10s cubic-bezier(.16,.9,.18,1) both!important;
      text-shadow:
        0 0 36px rgba(255,255,255,.44),
        0 0 120px rgba(255,255,255,.20);
    }
    .v423RollIn{
      animation:v430ScoreRollPremium .72s cubic-bezier(.16,1,.3,1) both!important;
    }

    @keyframes v430ArenaBreath{
      0%,100%{filter:brightness(1) saturate(1.02)}
      50%{filter:brightness(1.055) saturate(1.11)}
    }
    @keyframes v430SlowHalo{to{transform:rotate(360deg)}}
    @keyframes v430ScreenSheen{
      0%,100%{transform:translateX(-2%) scale(1);opacity:.24}
      50%{transform:translateX(2%) scale(1.015);opacity:.42}
    }
    @keyframes v430TopFloat{
      0%,100%{transform:translateY(0) scale(1)}
      50%{transform:translateY(-3px) scale(1.004)}
    }
    @keyframes v430StateGlow{
      0%,100%{filter:brightness(1);letter-spacing:.20em}
      50%{filter:brightness(1.20);letter-spacing:.23em}
    }
    @keyframes v430MetaSlide{
      0%,100%{transform:translateX(-50%) translateY(0);opacity:.82}
      50%{transform:translateX(-50%) translateY(-2px);opacity:1}
    }
    @keyframes v430TimerIdle{
      0%,100%{transform:scale(1);filter:brightness(1)}
      50%{transform:scale(1.014);filter:brightness(1.12)}
    }
    @keyframes v430TimerTick{
      0%{transform:scale(.982);filter:brightness(1.05)}
      24%{transform:scale(1.055);filter:brightness(1.38)}
      62%{transform:scale(1.010);filter:brightness(1.14)}
      100%{transform:scale(1);filter:brightness(1)}
    }
    @keyframes v430CenterLine{
      0%,100%{opacity:.22;filter:brightness(1)}
      50%{opacity:.55;filter:brightness(1.35)}
    }
    @keyframes v430CardHover{
      0%,100%{transform:translateY(0) scale(1)}
      50%{transform:translateY(-6px) scale(1.007)}
    }
    @keyframes v430CardEnergy{to{transform:rotate(360deg)}}
    @keyframes v430CardSheen{
      0%,58%{transform:translateX(-145%) skewX(-9deg);opacity:0}
      69%{opacity:.70}
      100%{transform:translateX(145%) skewX(-9deg);opacity:0}
    }
    @keyframes v430LeaderPulse{
      0%,100%{transform:translateY(0) scale(1.006);filter:brightness(1.08) saturate(1.15)}
      50%{transform:translateY(-7px) scale(1.018);filter:brightness(1.20) saturate(1.25)}
    }
    @keyframes v430TieBalance{
      0%,100%{filter:brightness(1.02) saturate(1.06)}
      50%{filter:brightness(1.14) saturate(1.16)}
    }
    @keyframes v430NameUnderline{
      0%,100%{transform:scaleX(.72);opacity:.58}
      50%{transform:scaleX(1.12);opacity:.96}
    }
    @keyframes v430ScoreCardPop{
      0%{transform:scale(1);filter:brightness(1)}
      15%{transform:scale(1.075);filter:brightness(1.62)}
      46%{transform:scale(1.026);filter:brightness(1.24)}
      100%{transform:scale(1);filter:brightness(1.05)}
    }
    @keyframes v430ScoreNumberPop{
      0%{transform:scale(1) translateY(0);filter:brightness(1)}
      18%{transform:scale(1.24) translateY(-5%);filter:brightness(1.78)}
      56%{transform:scale(1.06) translateY(-1%);filter:brightness(1.24)}
      100%{transform:scale(1) translateY(0);filter:brightness(1)}
    }
    @keyframes v430ScoreRollPremium{
      0%{transform:translateY(72%) scale(.76);opacity:0;filter:blur(7px) brightness(1.4)}
      48%{transform:translateY(-10%) scale(1.10);opacity:1;filter:blur(0) brightness(1.34)}
      72%{transform:translateY(2%) scale(1.03);filter:brightness(1.12)}
      100%{transform:translateY(0) scale(1);filter:brightness(1)}
    }
    @keyframes v430ScoreFlashSweep{
      0%{transform:translateX(-145%) skewX(-9deg);opacity:0}
      20%{opacity:.95}
      100%{transform:translateX(145%) skewX(-9deg);opacity:0}
    }
    @media(max-width:740px){
      .public.active:not(.countdownActive):not(.goalBlueActive):not(.goalOrangeActive):not(.finalBlueActive):not(.finalOrangeActive):not(.finalTieActive) .publicScores:after{
        left:7%;right:7%;top:50%;bottom:auto;width:auto;height:1px;transform:translateY(-50%);
      }
    }


    @media(max-width:1100px){.studioGrid{grid-template-columns:1fr 1fr}.span3{grid-column:span 2}.grid{grid-template-columns:1fr}.scoreRow{grid-template-columns:1fr}.score{font-size:96px}.timer{font-size:72px}.top{align-items:flex-start;flex-direction:column}.tabs{justify-content:flex-start}}@media(max-width:740px){.studioGrid{grid-template-columns:1fr}.span2,.span3{grid-column:span 1}.actions{grid-template-columns:1fr}.wide,.resumeBig{grid-column:span 1}.rangeRow{grid-template-columns:1fr 78px 56px}.public.active{grid-template-rows:42vh 1fr}.publicScores{grid-template-columns:1fr}.publicTeam .num{font-size:clamp(94px,28vw,220px)}}
  </style>
</head>
<body>
<div class="app">
  <div class="top">
    <div class="brand"><div class="logo">FS</div><div><h1>FIFA Robot Soccer Controller</h1><div class="sub">Default view: full setup and animation tuning. Switch to Operator Mode for game-time controls.</div></div></div>
    <div class="tabs">
      <button class="tab active" id="setupTab" onclick="showSetup()">Setup & Tuning</button>
      <button class="tab" id="operatorTab" onclick="showOperator()">Operator Mode</button>
      <button class="tab score" onclick="showPublic()">Audience Scoreboard</button>
    </div>
  </div>

  <div id="setupMode" class="mode active">
    <div class="studioGrid">
      <div class="card pad span3">
        <div class="scoreRow">
          <div class="team blue"><div class="teamName">Blue</div><div class="score" id="blueScoreSetup">0</div></div>
          <div class="timerBox"><div class="timer" id="timerSetup">1:30</div><div class="state" id="stateChipSetup">Standby</div><div class="sub" id="halfLabelSetup">First Half</div></div>
          <div class="team orange"><div class="teamName">Orange</div><div class="score" id="orangeScoreSetup">0</div></div>
        </div>
      </div>

      <div class="card pad">
        <div class="controlTitle"><h2>Match & Output</h2><div class="badge" id="ipBadgeSetup">IP ---</div></div>
        <div class="sectionNote">These are safe demo controls. LED output remains hard-capped at 50% in firmware.</div>
        <div class="field"><label>Match Length Per Half</label><div class="rangeRow"><input type="range" id="matchSecs" min="10" max="600" step="5"><input class="num" type="number" id="matchSecsN" min="10" max="600" step="5"><button class="resetBtn" onclick="resetOne('matchSecs')">Reset</button></div></div>
        <div class="field"><label>LED Brightness Percent</label><div class="rangeRow"><input type="range" id="ledBrightness" min="1" max="50"><input class="num" type="number" id="ledBrightnessN" min="1" max="50"><button class="resetBtn" onclick="resetOne('ledBrightness')">Reset</button></div></div>
        <div class="field"><label>Audio Volume Percent</label><div class="rangeRow"><input type="range" id="audioVolume" min="0" max="100"><input class="num" type="number" id="audioVolumeN" min="0" max="100"><button class="resetBtn" onclick="resetOne('audioVolume')">Reset</button></div></div>
        <div class="applyRow"><button class="apply" onclick="applyOutput()">Apply Output</button><button class="test" onclick="copyRecipe()">Copy Settings Recipe</button></div>
      </div>

      <div class="card pad span2">
        <div class="controlTitle"><h2>Animation Colors</h2><div class="badge">live color pickers</div></div>
        <div class="miniGrid">
          <div class="field"><label>Blue Team / Blue Goal</label><input class="color" type="color" id="blueColor"><button class="resetBtn" onclick="resetOne('blueColor')">Reset</button></div>
          <div class="field"><label>Orange Team / Orange Goal</label><input class="color" type="color" id="orangeColor"><button class="resetBtn" onclick="resetOne('orangeColor')">Reset</button></div>
          <div class="field"><label>Countdown 3 Color</label><input class="color" type="color" id="hex1Color"><button class="resetBtn" onclick="resetOne('hex1Color')">Reset</button></div>
          <div class="field"><label>Countdown 2 Color</label><input class="color" type="color" id="hex2Color"><button class="resetBtn" onclick="resetOne('hex2Color')">Reset</button></div>
          <div class="field"><label>Countdown 1 Color</label><input class="color" type="color" id="hex3Color"><button class="resetBtn" onclick="resetOne('hex3Color')">Reset</button></div>
          <div class="field"><label>Countdown Field Color</label><input class="color" type="color" id="fieldColor"><button class="resetBtn" onclick="resetOne('fieldColor')">Reset</button></div>
          <div class="field"><label>Game Start Flash</label><input class="color" type="color" id="startColor"><button class="resetBtn" onclick="resetOne('startColor')">Reset</button></div>
          <div class="field"><label>Last 10 Seconds Pulse</label><input class="color" type="color" id="lastTenColor"><button class="resetBtn" onclick="resetOne('lastTenColor')">Reset</button></div>
        </div>
        <div class="applyRow"><button class="apply" onclick="applyColors()">Apply Colors</button><button class="test" onclick="resetGroup('colors')">Reset Colors</button></div>
      </div>

      <div class="card pad">
        <div class="controlTitle"><h2>Countdown Hex & Field</h2><div class="badge">3 second start</div></div>
        <div class="field"><label>Field Effect Mode</label><select class="select" id="countdownMode"><option value="pulse">Pulse</option><option value="blink">Blink</option><option value="solid">Solid</option><option value="breathe">Breathe</option><option value="heartbeat">Heartbeat</option><option value="strobe">Strobe</option><option value="chasing_wave">Chasing Wave</option></select></div>
        <div class="field"><label>Hex Style</label><select class="select" id="hexStyle"><option value="team_fill_comet">Team Fill + Comet</option><option value="team_fill_only">Team Fill Only</option><option value="team_split_pulse">Team Split Pulse</option><option value="comet_only">Comet Only</option><option value="orbit_gradient">Orbit Gradient</option><option value="comet">Single Comet</option><option value="dual_comet">Dual Comet</option><option value="full_gradient">Full Gradient</option><option value="theater_chase">Theater Chase</option><option value="sparkle_orbit">Sparkle Orbit</option><option value="solid_step">Solid Step</option></select></div>
        <div class="field"><label>Countdown Step Ms</label><div class="rangeRow"><input type="range" id="countStep" min="500" max="2000" step="50"><input class="num" type="number" id="countStepN"><button class="resetBtn" onclick="resetOne('countStep')">Reset</button></div></div>
        <div class="field"><label>Field Effect Period Ms</label><div class="rangeRow"><input type="range" id="fieldPeriod" min="150" max="2500" step="25"><input class="num" type="number" id="fieldPeriodN"><button class="resetBtn" onclick="resetOne('fieldPeriod')">Reset</button></div></div>
        <div class="field"><label>Hex Rotation Ms</label><div class="rangeRow"><input type="range" id="hexRotation" min="250" max="3000" step="25"><input class="num" type="number" id="hexRotationN"><button class="resetBtn" onclick="resetOne('hexRotation')">Reset</button></div></div>
        <div class="field"><label>Hex Trail Percent</label><div class="rangeRow"><input type="range" id="hexTrail" min="4" max="70" step="1"><input class="num" type="number" id="hexTrailN"><button class="resetBtn" onclick="resetOne('hexTrail')">Reset</button></div></div>
        <div class="field"><label>Brightness: Field Min / Field Max / Hex</label><div class="rangeRow"><input type="range" id="fieldMin" min="0" max="255"><input class="num" type="number" id="fieldMinN"><button class="resetBtn" onclick="resetOne('fieldMin')">Reset</button></div><div class="rangeRow"><input type="range" id="fieldMax" min="0" max="255"><input class="num" type="number" id="fieldMaxN"><button class="resetBtn" onclick="resetOne('fieldMax')">Reset</button></div><div class="rangeRow"><input type="range" id="hexBright" min="0" max="255"><input class="num" type="number" id="hexBrightN"><button class="resetBtn" onclick="resetOne('hexBright')">Reset</button></div></div>
        <div class="applyRow"><button class="apply" onclick="applyCountdown()">Apply Countdown</button><button class="test" onclick="api('/api/test-countdown')">Test Countdown</button><button class="test" onclick="resetGroup('countdown')">Reset Countdown</button></div>
      </div>

      <div class="card pad">
        <div class="controlTitle"><h2>Goal Animation</h2><div class="badge">directional waves</div></div>
        <div class="field"><label>Goal Travel Style</label><select class="select" id="goalMode"><option value="straight">Straight Side-to-Side</option><option value="diagonal_up">Diagonal Up Sweep</option><option value="diagonal_down">Diagonal Down Sweep</option><option value="center_fan">Center Fan</option><option value="edge_fan">Edge Fan</option></select></div>
        <div class="field"><label>Total Duration Ms</label><div class="rangeRow"><input type="range" id="goalDuration" min="1200" max="9000" step="50"><input class="num" type="number" id="goalDurationN"><button class="resetBtn" onclick="resetOne('goalDuration')">Reset</button></div></div>
        <div class="field"><label>Wave Travel Ms</label><div class="rangeRow"><input type="range" id="goalTravel" min="250" max="2500" step="25"><input class="num" type="number" id="goalTravelN"><button class="resetBtn" onclick="resetOne('goalTravel')">Reset</button></div></div>
        <div class="field"><label>Wave Spacing Ms</label><div class="rangeRow"><input type="range" id="goalInterval" min="80" max="900" step="10"><input class="num" type="number" id="goalIntervalN"><button class="resetBtn" onclick="resetOne('goalInterval')">Reset</button></div></div>
        <div class="field"><label>Emit Until Ms</label><div class="rangeRow"><input type="range" id="goalEmit" min="400" max="7000" step="50"><input class="num" type="number" id="goalEmitN"><button class="resetBtn" onclick="resetOne('goalEmit')">Reset</button></div></div>
        <div class="field"><label>Source Pop Ms</label><div class="rangeRow"><input type="range" id="goalPop" min="0" max="2000" step="25"><input class="num" type="number" id="goalPopN"><button class="resetBtn" onclick="resetOne('goalPop')">Reset</button></div></div>
        <div class="field"><label>Resume After Goal</label><select class="select" id="autoGoalResumeEnabled"><option value="0">Manual Operator Resume</option><option value="1">Auto Resume After Delay</option></select></div>
        <div class="field"><label>Auto Resume Delay Ms</label><div class="rangeRow"><input type="range" id="autoGoalResumeMs" min="1000" max="30000" step="250"><input class="num" type="number" id="autoGoalResumeMsN" min="1000" max="30000" step="250"><button class="resetBtn" onclick="resetOne('autoGoalResumeMs')">Reset</button></div></div>
        <div class="sectionNote">Manual mode is safest for live operation. Auto mode resumes after the delay but the operator can still resume early.</div>
        <div class="applyRow"><button class="apply" onclick="applyGoal();applyGoalResume()">Apply Goal</button><button class="test" onclick="api('/api/fake-blue-goal')">Test Blue</button><button class="test" onclick="api('/api/fake-orange-goal')">Test Orange</button><button class="test" onclick="resetGroup('goal')">Reset Goal</button></div>
      </div>

      <div class="card pad">
        <div class="controlTitle"><h2>Finale / Winner Animation</h2><div class="badge">iPad winner</div></div>
        <div class="field"><label>Finale Duration Ms</label><div class="rangeRow"><input type="range" id="finaleDuration" min="3000" max="20000" step="100"><input class="num" type="number" id="finaleDurationN"><button class="resetBtn" onclick="resetOne('finaleDuration')">Reset</button></div></div>
        <div class="field"><label>Winner Ring Speed</label><div class="rangeRow"><input type="range" id="winnerSpeed" min="0" max="255"><input class="num" type="number" id="winnerSpeedN"><button class="resetBtn" onclick="resetOne('winnerSpeed')">Reset</button></div></div>
        <div class="field"><label>Hex Rainbow Speed</label><div class="rangeRow"><input type="range" id="rainbowSpeed" min="0" max="255"><input class="num" type="number" id="rainbowSpeedN"><button class="resetBtn" onclick="resetOne('rainbowSpeed')">Reset</button></div></div>
        <div class="field"><label>Ring Spacing</label><div class="rangeRow"><input type="range" id="ringSpacing" min="6" max="40"><input class="num" type="number" id="ringSpacingN"><button class="resetBtn" onclick="resetOne('ringSpacing')">Reset</button></div></div>
        <div class="applyRow"><button class="apply" onclick="applyFinale()">Apply Finale</button><button class="test" onclick="api('/api/end')">Test End Match</button><button class="test" onclick="resetGroup('finale')">Reset Finale</button></div>
      </div>

      <div class="card pad">
        <div class="controlTitle"><h2>Last 10 Seconds</h2><div class="badge">field + scoreboard warning</div></div>
        <div class="field"><label>Enabled</label><select class="select" id="lastTenEnabled"><option value="1">Enabled</option><option value="0">Disabled</option></select></div>
        <div class="field"><label>Pulse / Blink Period Ms</label><div class="rangeRow"><input type="range" id="lastTenPeriod" min="250" max="2000" step="25"><input class="num" type="number" id="lastTenPeriodN"><button class="resetBtn" onclick="resetOne('lastTenPeriod')">Reset</button></div></div>
        <div class="field"><label>LED Brightness Min / Max</label><div class="rangeRow"><input type="range" id="lastTenMin" min="0" max="255"><input class="num" type="number" id="lastTenMinN"><button class="resetBtn" onclick="resetOne('lastTenMin')">Reset</button></div><div class="rangeRow"><input type="range" id="lastTenMax" min="0" max="255"><input class="num" type="number" id="lastTenMaxN"><button class="resetBtn" onclick="resetOne('lastTenMax')">Reset</button></div></div>
        <div class="applyRow"><button class="apply" onclick="applyCountdown()">Apply Last 10</button><button class="test" onclick="api('/api/test-last-ten')">Test Last 10</button></div>
      </div>

      <div class="card pad">
        <div class="controlTitle"><h2>In-Game Idle Field</h2><div class="badge">subtle motion</div></div>
        <div class="field"><label>Idle Movement Speed</label><div class="rangeRow"><input type="range" id="idleSpeed" min="1" max="255"><input class="num" type="number" id="idleSpeedN"><button class="resetBtn" onclick="resetOne('idleSpeed')">Reset</button></div></div>
        <div class="field"><label>Brightness Min / Max</label><div class="rangeRow"><input type="range" id="idleMin" min="0" max="255"><input class="num" type="number" id="idleMinN"><button class="resetBtn" onclick="resetOne('idleMin')">Reset</button></div><div class="rangeRow"><input type="range" id="idleMax" min="0" max="255"><input class="num" type="number" id="idleMaxN"><button class="resetBtn" onclick="resetOne('idleMax')">Reset</button></div></div>
        <div class="applyRow"><button class="apply" onclick="applyCountdown()">Apply Idle</button><button class="test" onclick="api('/api/test-idle')">Test Idle</button></div>
      </div>

      <div class="card pad">
        <div class="controlTitle"><h2>Effect Test Mode</h2><div class="badge">debug without match</div></div>
        <div class="field"><label>Effect Test Mode</label><select class="select" id="testMode" onchange="api('/api/test-mode?enabled='+this.value)"><option value="0">Off</option><option value="1">On</option></select></div>
        <div class="smallActions">
          <button onclick="api('/api/test-countdown')">Test Countdown</button><button onclick="api('/api/fake-blue-goal')">Blue Goal</button><button onclick="api('/api/fake-orange-goal')">Orange Goal</button><button onclick="api('/api/test-last-ten')">Last 10</button><button onclick="api('/api/test-idle')">Idle</button><button onclick="api('/api/test-leds')">LED Self Test</button><button onclick="showPublic()">Scoreboard</button>
        </div>
      </div>

      <div class="card pad span2">
        <div class="controlTitle"><h2>Settings Recipe</h2><div class="badge">copy after tuning</div></div>
        <textarea class="recipe" id="recipe" readonly></textarea>
        <div class="applyRow"><button class="apply" onclick="copyRecipe()">Copy Recipe</button><button class="test" onclick="resetAll()">Reset All Tuning Controls</button></div>
      </div>

      <div class="card pad">
        <div class="controlTitle"><h2>System Status</h2><div class="badge" id="audioBadgeSetup">Audio ---</div></div>
        <div class="miniGrid">
          <div class="metric"><div class="k">Left Goal Pair</div><div class="v" id="leftPairSetup">---</div></div>
          <div class="metric"><div class="k">Right Goal Pair</div><div class="v" id="rightPairSetup">---</div></div>
          <div class="metric"><div class="k">Audio Track</div><div class="v" id="trackInfoSetup">0</div></div>
          <div class="metric"><div class="k">LED Mode</div><div class="v" id="ledModeSetup">---</div></div><div class="metric"><div class="k">Scoring</div><div class="v" id="scoringSetup">---</div></div>
        </div>
        <details class="diag"><summary>Audio diagnostics</summary><div class="smallActions"><button onclick="api('/api/play?track=1')">Start</button><button onclick="api('/api/play?track=2')">Beep</button><button onclick="api('/api/play?track=3')">1st BGM</button><button onclick="api('/api/play?track=4')">Resume</button><button onclick="api('/api/play?track=5')">2nd BGM</button><button onclick="api('/api/play?track=6')">End</button><button onclick="api('/api/play?track=7')">Goal</button><button onclick="api('/api/play?track=8')">Reset</button><button onclick="api('/api/play?track=9')">Idle</button><button onclick="api('/api/stop-audio')">Stop</button></div></details>
      </div>
    </div>
  </div>

  <div id="operatorMode" class="mode">
    <div class="grid">
      <div class="card pad">
        <div class="scoreRow">
          <div class="team blue"><div class="teamName">Blue</div><div class="score" id="blueScoreOp">0</div></div>
          <div class="timerBox"><div class="timer" id="timerOp">1:30</div><div class="state" id="stateChipOp">Standby</div><div class="sub" id="halfLabelOp">First Half</div></div>
          <div class="team orange"><div class="teamName">Orange</div><div class="score" id="orangeScoreOp">0</div></div>
        </div>
      </div>
      <div class="card pad">
        <div class="controlTitle"><h2>Operator Controls</h2><div class="badge" id="ipBadgeOp">IP ---</div></div>
        <div class="sectionNote">Game-time controls only. Use the top Audience Scoreboard button for the iPad display.</div>
        <div class="actions"><button class="primary" onclick="startFirstHalf()">Start 1st Half</button><button class="primary" onclick="api('/api/resume-second-half')">Resume 2nd Half</button><button class="resume resumeBig" id="resumeGoalBtn" disabled onclick="api('/api/resume-after-goal')">Resume Match After Goal</button><button class="dark" onclick="api('/api/standby')">Standby / Idle</button><button class="dark" onclick="api('/api/reset')">Reset Match</button><button class="blueBtn" onclick="api('/api/fake-blue-goal')">Blue Goal / Test</button><button class="orangeBtn" onclick="api('/api/fake-orange-goal')">Orange Goal / Test</button><button class="danger wide" onclick="api('/api/end')">End Match</button></div>
      </div>
      <div class="card pad"><div class="controlTitle"><h2>Live Sensors</h2><div class="badge" id="audioBadgeOp">Audio ---</div></div><div class="miniGrid"><div class="metric"><div class="k">Left Goal Pair</div><div class="v" id="leftPairOp">---</div></div><div class="metric"><div class="k">Right Goal Pair</div><div class="v" id="rightPairOp">---</div></div><div class="metric"><div class="k">Audio Track</div><div class="v" id="trackInfoOp">0</div></div><div class="metric"><div class="k">LED Mode</div><div class="v" id="ledModeOp">---</div></div><div class="metric"><div class="k">Scoring</div><div class="v" id="scoringOp">---</div></div><div class="metric"><div class="k">Last Event</div><div class="v" id="lastEventOp">Ready</div></div><div class="metric"><div class="k">Rejected Singles</div><div class="v" id="rejectsOp">0</div></div></div></div>
      <div class="card pad"><div class="controlTitle"><h2>Event Log</h2><div class="badge">recent</div></div><div class="log" id="eventLogOp"></div></div>
    </div>
  </div>
</div>

<div class="public" id="publicMode">
  <div class="publicTop"><div class="goalRingEl"></div><div class="goalRingEl"></div><div class="publicState" id="publicState">Standby</div><div class="publicTimer" id="publicTimer">1:30</div><div class="publicMeta" id="publicMeta">First Half</div></div>
  <div class="publicScores"><div class="publicTeam blue" id="publicBlue"><div class="name">Blue</div><div class="num" id="publicBlueScore">0</div></div><div class="publicTeam orange" id="publicOrange"><div class="name">Orange</div><div class="num" id="publicOrangeScore">0</div></div></div>
  <div class="publicWinner" id="publicWinner">Winner</div>
  <button class="exitPublic" onclick="hidePublic()">Exit Scoreboard</button>
</div>

<script>
const DEFAULTS={matchSecs:90,blueColor:'#0050FF',orangeColor:'#FF5000',hex1Color:'#00FF00',hex2Color:'#FFBE00',hex3Color:'#FF5000',fieldColor:'#474747',startColor:'#00FF00',lastTenColor:'#982ABC',countdownMode:'pulse',hexStyle:'team_fill_comet',countStep:1000,fieldPeriod:1000,hexRotation:1000,hexTrail:14,fieldMin:5,fieldMax:125,hexBright:255,goalMode:'straight',goalDuration:4200,goalTravel:950,goalInterval:260,goalEmit:2600,goalPop:700,autoGoalResumeEnabled:0,autoGoalResumeMs:5000,finaleDuration:13500,winnerSpeed:255,rainbowSpeed:50,ringSpacing:37,lastTenEnabled:1,lastTenPeriod:1000,lastTenMin:75,lastTenMax:135,idleSpeed:37,idleMin:10,idleMax:255,ledBrightness:25,audioVolume:80};
let latest={};
let publicLastBlueScore=null;
let publicLastOrangeScore=null;
let publicLastTimerText=null;
let publicGoalTeam='none';
let prevGoalActive=false;
let goalRingTimeout=null;
let goalEntranceTimeout=null;
let timerTickTimeout=null;
function animateScoreRoll(numId){const node=el(numId);if(!node)return;node.classList.remove('v423RollIn');void node.offsetWidth;node.classList.add('v423RollIn');setTimeout(()=>node.classList.remove('v423RollIn'),760)}
function pulseClass(id,cls,ms){
  const node=el(id);
  if(!node)return;
  node.classList.remove(cls);
  void node.offsetWidth;
  node.classList.add(cls);
  setTimeout(()=>node.classList.remove(cls),ms);
}
function animatePublicTimerIfChanged(nextText){
  const node=el('publicTimer');
  if(!node)return;
  if(publicLastTimerText!==null && publicLastTimerText!==String(nextText)){
    clearTimeout(timerTickTimeout);
    node.classList.remove('timerTick');
    void node.offsetWidth;
    node.classList.add('timerTick');
    timerTickTimeout=setTimeout(()=>node.classList.remove('timerTick'),650);
  }
  publicLastTimerText=String(nextText);
}
function applyAudienceScoreMood(b,o){
  const board=el('publicMode');
  const blue=el('publicBlue');
  const orange=el('publicOrange');
  if(!board||!blue||!orange)return;
  board.classList.remove('scoreBlueLead','scoreOrangeLead','scoreTieLive');
  blue.classList.remove('scoreLead','scoreBehind','scoreTied');
  orange.classList.remove('scoreLead','scoreBehind','scoreTied');
  if(b>o){
    board.classList.add('scoreBlueLead');
    blue.classList.add('scoreLead');
    orange.classList.add('scoreBehind');
  }else if(o>b){
    board.classList.add('scoreOrangeLead');
    orange.classList.add('scoreLead');
    blue.classList.add('scoreBehind');
  }else{
    board.classList.add('scoreTieLive');
    blue.classList.add('scoreTied');
    orange.classList.add('scoreTied');
  }
}
function scoreFlash(id){
  const node=el(id);
  if(!node)return;
  node.classList.remove('scorePop');
  void node.offsetWidth;
  node.classList.add('scorePop');
  setTimeout(()=>node.classList.remove('scorePop'),1120);
}
function el(id){return document.getElementById(id)}
function exists(id){return !!el(id)}
function fmt(ms){ms=Math.max(0,Number(ms||0));return String(Math.ceil(ms/1000))}
function normState(x){return String(x||'STANDBY').replace(/^STATE_/,'')}
function prettyState(x){const k=normState(x);const m={STANDBY:'Standby / Idle',COUNTDOWN:'Countdown',IN_GAME:'In Game',GOAL_PAUSE:'Goal Pause',HALFTIME:'Halftime',END_GAME:'Match Ended'};return m[k]||k.replace(/_/g,' ').replace(/\b\w/g,c=>c.toUpperCase())}
function prettyMode(x){return String(x||'---').replace(/^LED_MODE_/,'').replace(/_/g,' ').replace(/\b\w/g,c=>c.toUpperCase())}
function esc(s){return String(s||'').replace(/[&<>]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;'}[c]))}
async function api(path){try{if(path.includes('/api/reset')||path.includes('/api/start')){publicGoalTeam='none';publicLastBlueScore=null;publicLastOrangeScore=null;publicLastTimerText=null;prevGoalActive=false;}await fetch(path,{method:'POST',cache:'no-store'});await update()}catch(e){console.log(e)}}
function enc(v){return encodeURIComponent(v)}
function halfText(s){const gs=normState(s.gameState);if(gs==='HALFTIME')return 'Halftime';if(gs==='END_GAME')return 'Final';if(gs==='COUNTDOWN')return s.currentHalf===2?'Second Half Starting':'First Half Starting';return s.currentHalf===2?'Second Half':'First Half'}
let publicControlsTimer=0;
function pulsePublicControls(){
  const board=el('publicMode');
  if(!board||!board.classList.contains('active'))return;
  board.classList.add('controlsVisible');
  clearTimeout(publicControlsTimer);
  publicControlsTimer=setTimeout(()=>board.classList.remove('controlsVisible'),3000);
}
function showSetup(){el('setupMode').classList.add('active');el('operatorMode').classList.remove('active');el('setupTab').classList.add('active');el('operatorTab').classList.remove('active');hidePublic(false)}
function showOperator(){el('operatorMode').classList.add('active');el('setupMode').classList.remove('active');el('operatorTab').classList.add('active');el('setupTab').classList.remove('active');hidePublic(false)}
function showPublic(){el('publicMode').classList.add('active');pulsePublicControls();if(document.documentElement.requestFullscreen){document.documentElement.requestFullscreen().catch(()=>{})}}
function hidePublic(exitFs=true){clearTimeout(publicControlsTimer);clearTimeout(goalRingTimeout);clearTimeout(goalEntranceTimeout);el('publicMode').classList.remove('active','winnerBlue','winnerOrange','winnerTie','countdownActive','goalBlueActive','goalOrangeActive','finalBlueActive','finalOrangeActive','finalTieActive','lastTenActive','goalRingsActive','goalJustEntered','controlsVisible');prevGoalActive=false;if(exitFs&&document.fullscreenElement){document.exitFullscreen().catch(()=>{})}}
function setVal(id,v){if(!exists(id))return;const e=el(id);if(document.activeElement===e)return;e.value=v}function setPair(id,v){setVal(id,v);setVal(id+'N',v)}function getNum(id){return Number(el(id)?.value||0)}function getVal(id){return el(id)?.value||''}function setSelect(id,v){setVal(id,String(v).toLowerCase())}function setColor(id,v){if(v)setVal(id,String(v).toUpperCase())}
function linkPair(id,applyFn){['input','change'].forEach(evt=>{el(id)?.addEventListener(evt,()=>{setVal(id+'N',el(id).value);if(evt==='change'){(applyFn||(()=>routeApply(id)))()}});el(id+'N')?.addEventListener(evt,()=>{setVal(id,el(id+'N').value);if(evt==='change'){(applyFn||(()=>routeApply(id)))()}})})}
function resetOne(id){const v=DEFAULTS[id];if(v===undefined)return;if(exists(id)&&el(id).type==='color')setColor(id,v);else if(exists(id)&&el(id).tagName==='SELECT')setSelect(id,v);else setPair(id,v);routeApply(id)}function resetGroup(g){const groups={colors:['blueColor','orangeColor','hex1Color','hex2Color','hex3Color','fieldColor','startColor','lastTenColor'],countdown:['countdownMode','hexStyle','countStep','fieldPeriod','hexRotation','hexTrail','fieldMin','fieldMax','hexBright','lastTenEnabled','lastTenPeriod','lastTenMin','lastTenMax','idleSpeed','idleMin','idleMax'],goal:['goalMode','goalDuration','goalTravel','goalInterval','goalEmit','goalPop','autoGoalResumeEnabled','autoGoalResumeMs'],finale:['finaleDuration','winnerSpeed','rainbowSpeed','ringSpacing'],output:['matchSecs','ledBrightness','audioVolume']};(groups[g]||[]).forEach(resetOne)}function resetAll(){Object.keys(DEFAULTS).forEach(resetOne);applyColors();applyCountdown();applyGoal();applyGoalResume();applyFinale();applyOutput()}
function routeApply(id){if(id==='matchSecs')applyMatchLength();else if(['blueColor','orangeColor','hex1Color','hex2Color','hex3Color','fieldColor','startColor','lastTenColor'].includes(id))applyColors();else if(['countdownMode','hexStyle','countStep','fieldPeriod','hexRotation','hexTrail','fieldMin','fieldMax','hexBright','lastTenEnabled','lastTenPeriod','lastTenMin','lastTenMax','idleSpeed','idleMin','idleMax'].includes(id))applyCountdown();else if(['goalMode','goalDuration','goalTravel','goalInterval','goalEmit','goalPop'].includes(id))applyGoal();else if(['autoGoalResumeEnabled','autoGoalResumeMs'].includes(id))applyGoalResume();else if(['finaleDuration','winnerSpeed','rainbowSpeed','ringSpacing'].includes(id))applyFinale();else if(['ledBrightness','audioVolume'].includes(id))applyOutput()}
function applyColors(){api('/api/colors?blue='+enc(getVal('blueColor'))+'&orange='+enc(getVal('orangeColor'))+'&hex1='+enc(getVal('hex1Color'))+'&hex2='+enc(getVal('hex2Color'))+'&hex3='+enc(getVal('hex3Color'))+'&field='+enc(getVal('fieldColor'))+'&start='+enc(getVal('startColor'))+'&lastTen='+enc(getVal('lastTenColor')))}
function applyCountdown(){api('/api/countdown-tuning?mode='+enc(getVal('countdownMode'))+'&hexStyle='+enc(getVal('hexStyle'))+'&step='+getNum('countStep')+'&period='+getNum('fieldPeriod')+'&hexRotation='+getNum('hexRotation')+'&hexTrail='+getNum('hexTrail')+'&min='+getNum('fieldMin')+'&max='+getNum('fieldMax')+'&hexBrightness='+getNum('hexBright')+'&lastTenEnabled='+getVal('lastTenEnabled')+'&lastTenPeriod='+getNum('lastTenPeriod')+'&lastTenMin='+getNum('lastTenMin')+'&lastTenMax='+getNum('lastTenMax')+'&idleSpeed='+getNum('idleSpeed')+'&idleMin='+getNum('idleMin')+'&idleMax='+getNum('idleMax'))}
function applyGoal(){api('/api/goal-timing?mode='+enc(getVal('goalMode'))+'&duration='+getNum('goalDuration')+'&travel='+getNum('goalTravel')+'&interval='+getNum('goalInterval')+'&emit='+getNum('goalEmit')+'&pop='+getNum('goalPop'))}function applyGoalResume(){api('/api/goal-resume?enabled='+getVal('autoGoalResumeEnabled')+'&delay='+getNum('autoGoalResumeMs'))}function applyFinale(){api('/api/end-tuning?duration='+getNum('finaleDuration')+'&winnerSpeed='+getNum('winnerSpeed')+'&rainbowSpeed='+getNum('rainbowSpeed')+'&spacing='+getNum('ringSpacing'))}function applyMatchLength(){api('/api/match-length?seconds='+getNum('matchSecs'))}function applyOutput(){api('/api/led-brightness?level='+getNum('ledBrightness'));api('/api/volume?level='+getNum('audioVolume'))}
function matchSeconds(){const v=exists('matchSecs')?Number(el('matchSecs').value||DEFAULTS.matchSecs):DEFAULTS.matchSecs;return Math.max(10,Math.min(600,v||DEFAULTS.matchSecs))}function startFirstHalf(){api('/api/start?seconds='+matchSeconds())}function syncMatchLength(s){const secs=Math.round(Number(s.matchDurationMs||90000)/1000);if(exists('matchSecs')&&document.activeElement!==el('matchSecs'))setVal('matchSecs',secs);if(exists('matchSecsN')&&document.activeElement!==el('matchSecsN'))setVal('matchSecsN',secs)}
function sensorPair(a,b,armed){if(a&&b)return ['Both Detected','warn'];if(a||b)return ['Partial','warn'];return [armed?'Armed':'Clearing',armed?'ok':'bad']}
function classState(s){const gs=normState(s);let cls='state';if(gs==='IN_GAME')cls+=' go';else if(gs==='GOAL_PAUSE'||gs==='COUNTDOWN'||gs==='HALFTIME')cls+=' pause';else if(gs==='END_GAME')cls+=' end';return cls}
function updatePublic(s){
  const gs=normState(s.gameState);
  const board=el('publicMode');
  const b=Number(s.blueScore??0);
  const o=Number(s.orangeScore??0);

  // Detect scoring from score changes. This keeps the audience celebration active
  // through GOAL_PAUSE even if the LED mode has already returned to IN_GAME.
  if(publicLastBlueScore===null || publicLastOrangeScore===null){
    publicLastBlueScore=b;
    publicLastOrangeScore=o;
  }else{
    if(b>publicLastBlueScore){ publicGoalTeam='blue'; scoreFlash('publicBlue'); animateScoreRoll('publicBlueScore'); }
    if(o>publicLastOrangeScore){ publicGoalTeam='orange'; scoreFlash('publicOrange'); animateScoreRoll('publicOrangeScore'); }
    if(b<publicLastBlueScore || o<publicLastOrangeScore) publicGoalTeam='none'; // reset/new match
    publicLastBlueScore=b;
    publicLastOrangeScore=o;
  }

  const led=String(s.ledMode||'');
  if(led==='BLUE_GOAL') publicGoalTeam='blue';
  if(led==='ORANGE_GOAL') publicGoalTeam='orange';
  if(gs!=='GOAL_PAUSE' && led!=='BLUE_GOAL' && led!=='ORANGE_GOAL'){
    if(gs==='IN_GAME' || gs==='COUNTDOWN' || gs==='HALFTIME' || gs==='END_GAME' || gs==='STANDBY') publicGoalTeam='none';
  }

  const goalActive=(gs==='GOAL_PAUSE') && (publicGoalTeam==='blue' || publicGoalTeam==='orange');
  const finalActive=(gs==='END_GAME') && (s.winner==='blue' || s.winner==='orange' || s.winner==='tie');
  const remainingMs=Number(s.matchRemainingMs||0);
  const testLastTen=(led==='LAST_TEN_TEST');
  const lastTenActive=(s.lastTenEnabled!==false) && ((gs==='IN_GAME' && remainingMs>0 && remainingMs<=10000) || testLastTen);
  const testLastTenDisplay=String(10-Math.floor((Number(s.uptimeMs||0)%10000)/1000));

  let mainText=(gs==='COUNTDOWN'&&s.countdownValue>0)?String(s.countdownValue):(goalActive?'GOAL':(testLastTen?testLastTenDisplay:fmt(s.matchRemainingMs)));
  let stateText=goalActive?'':prettyState(s.gameState);
  let metaText=goalActive?'':halfText(s);

  if(finalActive){
    if(s.winner==='blue'){
      mainText='BLUE WINS';
      stateText='MATCH COMPLETE';
    }else if(s.winner==='orange'){
      mainText='ORANGE WINS';
      stateText='MATCH COMPLETE';
    }else{
      mainText='TIE GAME';
      stateText='MATCH COMPLETE';
    }
    metaText=b+' \u2013 '+o;
  }

  animatePublicTimerIfChanged(mainText);
  el('publicTimer').textContent=mainText;
  el('publicState').textContent=stateText;
  el('publicMeta').textContent=metaText;
  el('publicBlueScore').textContent=b;
  el('publicOrangeScore').textContent=o;

  // These classes are safe to update even when the audience overlay is hidden.
  // CSS only displays full-screen audience celebrations when .public.active is also present.
  board.classList.remove('winnerBlue','winnerOrange','winnerTie','countdownActive','goalBlueActive','goalOrangeActive','finalBlueActive','finalOrangeActive','finalTieActive','lastTenActive','scoreBlueLead','scoreOrangeLead','scoreTieLive');
  el('publicBlue').classList.remove('winnerGlow','tieGlow','scoreLead','scoreBehind','scoreTied');
  el('publicOrange').classList.remove('winnerGlow','tieGlow','scoreLead','scoreBehind','scoreTied');

  const w=el('publicWinner');

  if(gs==='COUNTDOWN'){
    board.classList.add('countdownActive');
    const cs=s.countdownStepColors||[];
    const cv=Math.max(1,Math.min(3,Number(s.countdownValue||3)));
    const color=cs[3-cv]||DEFAULTS.hex1Color||'#00FF00';
    const countMs=Math.max(300,Number(s.countdownStepMs||DEFAULTS.countStep||1000));
    board.style.setProperty('--countColor',color);
    board.style.setProperty('--countDur',countMs+'ms');
    w.classList.remove('show');
    return;
  }

  if(goalActive){
    // Fire ring burst and dramatic entrance once on state entry
    if(!prevGoalActive){
      board.classList.remove('goalRingsActive','goalJustEntered');
      clearTimeout(goalRingTimeout);clearTimeout(goalEntranceTimeout);
      void board.offsetWidth;
      board.classList.add('goalRingsActive','goalJustEntered');
      goalEntranceTimeout=setTimeout(()=>board.classList.remove('goalJustEntered'),850);
      goalRingTimeout=setTimeout(()=>board.classList.remove('goalRingsActive'),2200);
    }
    prevGoalActive=true;
    board.classList.add(publicGoalTeam==='blue'?'goalBlueActive':'goalOrangeActive');
    if(publicGoalTeam==='blue') el('publicBlue').classList.add('winnerGlow');
    else el('publicOrange').classList.add('winnerGlow');
    w.classList.remove('show');
    return;
  }
  prevGoalActive=false;

  if(finalActive){
    if(s.winner==='blue'){
      board.classList.add('finalBlueActive');
      el('publicBlue').classList.add('winnerGlow');
    }else if(s.winner==='orange'){
      board.classList.add('finalOrangeActive');
      el('publicOrange').classList.add('winnerGlow');
    }else{
      board.classList.add('finalTieActive');
      el('publicBlue').classList.add('tieGlow');
      el('publicOrange').classList.add('tieGlow');
    }
    w.classList.remove('show');
    return;
  }

  if(lastTenActive){
    const lastColor=s.lastTenColor||DEFAULTS.lastTenColor||'#982ABC';
    const lastDur=Math.max(250,Number(s.lastTenPeriodMs||DEFAULTS.lastTenPeriod||1000));
    board.classList.add('lastTenActive');
    board.style.setProperty('--lastColor',lastColor);
    board.style.setProperty('--lastDur',lastDur+'ms');
  }

  // v4.30 normal scoreboard polish:
  // Add subtle leader/tie emphasis only after special screens are handled.
  applyAudienceScoreMood(b,o);

  // Fallback winner styling for any future state that reports a winner without END_GAME.
  if(s.winner==='blue'){
    board.classList.add('winnerBlue');
    el('publicBlue').classList.add('winnerGlow');
    w.textContent='BLUE WINS';
    w.classList.add('show');
  }else if(s.winner==='orange'){
    board.classList.add('winnerOrange');
    el('publicOrange').classList.add('winnerGlow');
    w.textContent='ORANGE WINS';
    w.classList.add('show');
  }else if(s.winner==='tie'){
    board.classList.add('winnerTie');
    el('publicBlue').classList.add('tieGlow');
    el('publicOrange').classList.add('tieGlow');
    w.textContent='TIE GAME';
    w.classList.add('show');
  }else{
    w.classList.remove('show');
  }
}

function recipe(s=latest){const c=s.countdownStepColors||[getVal('hex1Color'),getVal('hex2Color'),getVal('hex3Color')];return 'FIFA ROBOT SOCCER SETTINGS\nmatchSeconds='+Math.round(Number(s.matchDurationMs||90000)/1000)+'\ncolors: blue='+getVal('blueColor')+' orange='+getVal('orangeColor')+' countdownField='+getVal('fieldColor')+' gameStart='+getVal('startColor')+'\ncountdownHexColors='+getVal('hex1Color')+','+getVal('hex2Color')+','+getVal('hex3Color')+'\ncountdown: fieldMode='+getVal('countdownMode').toUpperCase()+' hexStyle='+getVal('hexStyle').toUpperCase()+' stepMs='+getNum('countStep')+' fieldEffectMs='+getNum('fieldPeriod')+' hexRotationMs='+getNum('hexRotation')+' hexTrailPercent='+getNum('hexTrail')+' brightness='+getNum('fieldMin')+'/'+getNum('fieldMax')+'/'+getNum('hexBright')+'\ngoal: mode='+getVal('goalMode')+' durationMs='+getNum('goalDuration')+' travelMs='+getNum('goalTravel')+' spacingMs='+getNum('goalInterval')+' emitMs='+getNum('goalEmit')+' popMs='+getNum('goalPop')+' autoResume='+(getVal('autoGoalResumeEnabled')==='1'?'true':'false')+' autoResumeDelayMs='+getNum('autoGoalResumeMs')+'\nfinale: durationMs='+getNum('finaleDuration')+' winnerSpeed='+getNum('winnerSpeed')+' hexRainbowSpeed='+getNum('rainbowSpeed')+' ringSpacing='+getNum('ringSpacing')+'\nlast10: color='+getVal('lastTenColor')+' enabled='+(getVal('lastTenEnabled')==='1'?'true':'false')+' periodMs='+getNum('lastTenPeriod')+' brightness='+getNum('lastTenMin')+'/'+getNum('lastTenMax')+'\nidle: speed='+getNum('idleSpeed')+' brightness='+getNum('idleMin')+'/'+getNum('idleMax')+'\noutput: ledBrightnessPercent='+getNum('ledBrightness')+' audioVolumePercent='+getNum('audioVolume')}function copyRecipe(){const r=recipe();el('recipe').value=r;navigator.clipboard&&navigator.clipboard.writeText(r).catch(()=>{})}
function syncControls(s){setColor('blueColor',s.blueGoalColor||DEFAULTS.blueColor);setColor('orangeColor',s.orangeGoalColor||DEFAULTS.orangeColor);const cs=s.countdownStepColors||[];setColor('hex1Color',cs[0]||s.countdownHexColor||DEFAULTS.hex1Color);setColor('hex2Color',cs[1]||DEFAULTS.hex2Color);setColor('hex3Color',cs[2]||DEFAULTS.hex3Color);setColor('fieldColor',s.countdownFieldColor||DEFAULTS.fieldColor);setColor('startColor',s.gameStartColor||DEFAULTS.startColor);setColor('lastTenColor',s.lastTenColor||DEFAULTS.lastTenColor);setSelect('countdownMode',String(s.countdownEffectMode||DEFAULTS.countdownMode).toLowerCase());setSelect('hexStyle',String(s.countdownHexStyle||DEFAULTS.hexStyle).toLowerCase());setPair('countStep',s.countdownStepMs??DEFAULTS.countStep);setPair('fieldPeriod',s.countdownFieldEffectPeriodMs??DEFAULTS.fieldPeriod);setPair('hexRotation',s.countdownHexRotationMs??DEFAULTS.hexRotation);setPair('hexTrail',s.countdownHexTrailPercent??DEFAULTS.hexTrail);setPair('fieldMin',s.countdownFieldMinBrightness??DEFAULTS.fieldMin);setPair('fieldMax',s.countdownFieldMaxBrightness??DEFAULTS.fieldMax);setPair('hexBright',s.countdownHexBrightness??DEFAULTS.hexBright);setSelect('lastTenEnabled',s.lastTenEnabled?1:0);setPair('lastTenPeriod',s.lastTenPeriodMs??DEFAULTS.lastTenPeriod);const lb=String(s.lastTenBrightness||'75/135').split('/');setPair('lastTenMin',lb[0]||DEFAULTS.lastTenMin);setPair('lastTenMax',lb[1]||DEFAULTS.lastTenMax);setPair('idleSpeed',s.inGameIdleSpeed??DEFAULTS.idleSpeed);const ib=String(s.inGameIdleBrightness||'10/255').split('/');setPair('idleMin',ib[0]||DEFAULTS.idleMin);setPair('idleMax',ib[1]||DEFAULTS.idleMax);setSelect('goalMode',String(s.goalTravelMode||DEFAULTS.goalMode).toLowerCase());setPair('goalDuration',s.goalEffectMs??DEFAULTS.goalDuration);setPair('goalTravel',s.goalWaveTravelMs??DEFAULTS.goalTravel);setPair('goalInterval',s.goalWaveEmitIntervalMs??DEFAULTS.goalInterval);setPair('goalEmit',s.goalWaveEmitEndMs??DEFAULTS.goalEmit);setPair('goalPop',s.goalSourcePopMs??DEFAULTS.goalPop);setSelect('autoGoalResumeEnabled',s.autoResumeAfterGoal?1:0);setPair('autoGoalResumeMs',s.autoResumeAfterGoalDelayMs??DEFAULTS.autoGoalResumeMs);setPair('finaleDuration',s.matchEndEffectMs??DEFAULTS.finaleDuration);setPair('winnerSpeed',s.matchEndWinnerSpeed??DEFAULTS.winnerSpeed);setPair('rainbowSpeed',s.matchEndRainbowSpeed??DEFAULTS.rainbowSpeed);setPair('ringSpacing',s.matchEndRingSpacing??DEFAULTS.ringSpacing);setPair('ledBrightness',s.ledBrightness??DEFAULTS.ledBrightness);setPair('audioVolume',s.volume??DEFAULTS.audioVolume);setSelect('testMode',s.effectTestMode?1:0);if(exists('recipe'))el('recipe').value=recipe(s)}
function paintCommon(prefix,s){
  if(!exists('timer'+prefix))return;
  // Do not use audience-only variables such as goalActive here.
  // This function drives Setup/Operator panels and must never throw.
  const gs=normState(s.gameState);
  const timer=(gs==='COUNTDOWN'&&s.countdownValue>0)?String(s.countdownValue):fmt(s.matchRemainingMs);
  el('timer'+prefix).textContent=timer;
  el('blueScore'+prefix).textContent=s.blueScore??0;
  el('orangeScore'+prefix).textContent=s.orangeScore??0;
  const chip=el('stateChip'+prefix);
  chip.textContent=prettyState(s.gameState);
  chip.className=classState(s.gameState);
  el('halfLabel'+prefix).textContent=halfText(s);
}
async function update(){try{const r=await fetch('/api/state',{cache:'no-store'});const s=await r.json();latest=s;paintCommon('Setup',s);paintCommon('Op',s);syncMatchLength(s);if(!['INPUT','SELECT'].includes(document.activeElement.tagName))syncControls(s);if(exists('resumeGoalBtn')){
    const resumeBtn=el('resumeGoalBtn');
    const paused=(normState(s.gameState)==='GOAL_PAUSE');
    resumeBtn.disabled=!paused;
    if(paused && s.autoResumeAfterGoal){
      const remaining=Math.ceil(Number(s.autoResumeGoalRemainingMs||0)/1000);
      resumeBtn.textContent='Resume Now · Auto in '+remaining+'s';
    }else{
      resumeBtn.textContent=paused?'Resume Match After Goal':'Resume After Goal';
    }
    resumeBtn.classList.toggle('attention',paused);
  }['Setup','Op'].forEach(p=>{if(exists('ipBadge'+p))el('ipBadge'+p).textContent='IP '+(s.ip||'---');if(exists('audioBadge'+p))el('audioBadge'+p).textContent=s.audioBusy?'Audio Playing':'Audio Idle';if(exists('trackInfo'+p))el('trackInfo'+p).textContent='Track '+(s.currentTrack||0);if(exists('ledMode'+p))el('ledMode'+p).textContent=prettyMode(s.ledMode);if(exists('scoring'+p))el('scoring'+p).textContent=s.scoringEnabled?'Enabled':'Disabled';const lp=sensorPair(s.leftSensorA,s.leftSensorB,s.leftGoalArmed);const rp=sensorPair(s.rightSensorA,s.rightSensorB,s.rightGoalArmed);if(exists('leftPair'+p)){el('leftPair'+p).textContent=lp[0];el('leftPair'+p).className='v '+lp[1]}if(exists('rightPair'+p)){el('rightPair'+p).textContent=rp[0];el('rightPair'+p).className='v '+rp[1]}});if(exists('lastEventOp'))el('lastEventOp').textContent=s.lastEvent||'Ready';if(exists('rejectsOp'))el('rejectsOp').textContent=Number(s.leftGoalRejectedSingles||0)+Number(s.rightGoalRejectedSingles||0);if(Array.isArray(s.eventLog)&&exists('eventLogOp'))el('eventLogOp').innerHTML=s.eventLog.slice().reverse().map(e=>'<div class="logItem">'+esc(e)+'</div>').join('');updatePublic(s)}catch(e){if(exists('stateChipSetup'))el('stateChipSetup').textContent='Offline';if(exists('stateChipOp'))el('stateChipOp').textContent='Offline'}}
function initPairs(){['matchSecs','ledBrightness','audioVolume','countStep','fieldPeriod','hexRotation','hexTrail','fieldMin','fieldMax','hexBright','goalDuration','goalTravel','goalInterval','goalEmit','goalPop','autoGoalResumeMs','finaleDuration','winnerSpeed','rainbowSpeed','ringSpacing','lastTenPeriod','lastTenMin','lastTenMax','idleSpeed','idleMin','idleMax'].forEach(id=>linkPair(id));['blueColor','orangeColor','hex1Color','hex2Color','hex3Color','fieldColor','startColor','lastTenColor'].forEach(id=>el(id)?.addEventListener('change',applyColors));['countdownMode','hexStyle','lastTenEnabled'].forEach(id=>el(id)?.addEventListener('change',applyCountdown));el('goalMode')?.addEventListener('change',applyGoal);el('autoGoalResumeEnabled')?.addEventListener('change',applyGoalResume)}
initPairs();
['pointerdown','touchstart','mousemove'].forEach(evt=>{
  el('publicMode')?.addEventListener(evt,pulsePublicControls,{passive:true});
});
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


static void handleResumeSecondHalf() {
  startSecondHalf();
  server.send(200, "application/json", stateJson());
}

static void handleResumeAfterGoal() {
  resumeAfterGoal();
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


static void handleStandby() {
  enterStandbyIdleMode();
  server.send(200, "application/json", stateJson());
}

static void handleLedSelfTest() {
  setLedMode(LED_MODE_LED_SELF_TEST);
  addEvent("LED self-test active: red, green, blue, orange cycle");
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
  if (gameState == STATE_IN_GAME) {
    handleBlueGoal(true);
  } else {
    setLedMode(LED_MODE_BLUE_GOAL);
    audioPlayTrackInterrupting(AUDIO_TRACK_GOAL, "fake-blue-goal-test");
    addEvent("Test Blue goal sound and animation");
  }
  server.send(200, "application/json", stateJson());
}

static void handleFakeOrangeGoal() {
  if (gameState == STATE_IN_GAME) {
    handleOrangeGoal(true);
  } else {
    setLedMode(LED_MODE_ORANGE_GOAL);
    audioPlayTrackInterrupting(AUDIO_TRACK_GOAL, "fake-orange-goal-test");
    addEvent("Test Orange goal sound and animation");
  }
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
      audioPlayTrackForced(track);
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
    countdownStepColors[0] = countdownHexColor;
  }

  if (server.hasArg("hex1")) countdownStepColors[0] = parseWebHexColor(server.arg("hex1"), DEFAULT_COUNTDOWN_SECOND_3_COLOR);
  if (server.hasArg("hex2")) countdownStepColors[1] = parseWebHexColor(server.arg("hex2"), DEFAULT_COUNTDOWN_SECOND_2_COLOR);
  if (server.hasArg("hex3")) countdownStepColors[2] = parseWebHexColor(server.arg("hex3"), DEFAULT_COUNTDOWN_SECOND_1_COLOR);

  countdownHexColor = countdownStepColors[0];

  if (server.hasArg("field")) {
    countdownFieldColor = parseWebHexColor(server.arg("field"), DEFAULT_COUNTDOWN_FIELD_COLOR);
  }

  if (server.hasArg("start")) {
    gameStartColor = parseWebHexColor(server.arg("start"), DEFAULT_GAME_START_COLOR);
  }

  if (server.hasArg("lastTen")) {
    lastTenPulseColor = parseWebHexColor(server.arg("lastTen"), LAST_TEN_PULSE_COLOR_DEFAULT);
  }

  addEvent("Animation colors updated");
  server.send(200, "application/json", stateJson());
}

static void handleCountdownTuning() {
  if (server.hasArg("mode")) {
    countdownFieldEffectMode = parseCountdownFieldEffectMode(server.arg("mode"));
  }

  if (server.hasArg("hexStyle")) {
    countdownHexAnimationMode = parseCountdownHexStyle(server.arg("hexStyle"));
  }

  if (server.hasArg("hexRotation")) {
    countdownHexRotationMs = clampUL((unsigned long)server.arg("hexRotation").toInt(), COUNTDOWN_HEX_ROTATION_MS_MIN, COUNTDOWN_HEX_ROTATION_MS_MAX);
  }

  if (server.hasArg("hexTrail")) {
    int requestedTrail = server.arg("hexTrail").toInt();
    if (requestedTrail < COUNTDOWN_HEX_TRAIL_PERCENT_MIN) requestedTrail = COUNTDOWN_HEX_TRAIL_PERCENT_MIN;
    if (requestedTrail > COUNTDOWN_HEX_TRAIL_PERCENT_MAX) requestedTrail = COUNTDOWN_HEX_TRAIL_PERCENT_MAX;
    countdownHexTrailPercent = (uint8_t)requestedTrail;
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

  if (server.hasArg("lastTenEnabled")) {
    lastTenWarningEnabled = server.arg("lastTenEnabled").toInt() != 0;
  }

  if (server.hasArg("lastTenPeriod")) {
    lastTenPulsePeriodMs = clampUL((unsigned long)server.arg("lastTenPeriod").toInt(), LAST_TEN_PULSE_PERIOD_MIN, LAST_TEN_PULSE_PERIOD_MAX);
  }

  if (server.hasArg("lastTenMin")) {
    lastTenMinBrightness = clampBrightnessValue(server.arg("lastTenMin").toInt());
  }

  if (server.hasArg("lastTenMax")) {
    lastTenMaxBrightness = clampBrightnessValue(server.arg("lastTenMax").toInt());
  }

  if (server.hasArg("idleSpeed")) {
    int requestedSpeed = server.arg("idleSpeed").toInt();
    if (requestedSpeed < IN_GAME_IDLE_SPEED_MIN) requestedSpeed = IN_GAME_IDLE_SPEED_MIN;
    if (requestedSpeed > IN_GAME_IDLE_SPEED_MAX) requestedSpeed = IN_GAME_IDLE_SPEED_MAX;
    inGameIdleSpeed = (uint8_t)requestedSpeed;
  }

  if (server.hasArg("idleMin")) {
    inGameIdleMinBrightness = clampBrightnessValue(server.arg("idleMin").toInt());
  }

  if (server.hasArg("idleMax")) {
    inGameIdleMaxBrightness = clampBrightnessValue(server.arg("idleMax").toInt());
  }

  addEvent("Countdown/idle tuning updated");
  server.send(200, "application/json", stateJson());
}


static void handleGoalResumeTuning() {
  if (server.hasArg("enabled")) {
    autoResumeAfterGoalEnabled = server.arg("enabled").toInt() != 0;
  }

  if (server.hasArg("delay")) {
    autoResumeAfterGoalDelayMs = clampUL((unsigned long)server.arg("delay").toInt(), 1000UL, 30000UL);
  }

  String msg = "Goal resume mode: ";
  msg += autoResumeAfterGoalEnabled ? "auto after " : "manual";
  if (autoResumeAfterGoalEnabled) {
    msg += String(autoResumeAfterGoalDelayMs);
    msg += " ms";
  }

  addEvent(msg);
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

static void handleTestMode() {
  if (server.hasArg("enabled")) {
    effectTestMode = server.arg("enabled").toInt() != 0;
  }

  addEvent(effectTestMode ? "Effect test mode enabled" : "Effect test mode disabled");
  server.send(200, "application/json", stateJson());
}

static void handleTestCountdown() {
  setLedMode(LED_MODE_COUNTDOWN);
  addEvent("Test countdown LED effect");
  server.send(200, "application/json", stateJson());
}

static void handleTestLastTen() {
  setLedMode(LED_MODE_LAST_TEN_TEST);
  addEvent("Test last-10-second LED effect");
  server.send(200, "application/json", stateJson());
}

static void handleTestIdle() {
  setLedMode(LED_MODE_IN_GAME);
  addEvent("Test in-game idle LED effect");
  server.send(200, "application/json", stateJson());
}

static void handleNotFound() {
  server.send(404, "text/plain", "Not found");
}

static void beginWeb() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/state", HTTP_GET, handleState);

  server.on("/api/start", HTTP_POST, handleStart);
  server.on("/api/standby", HTTP_POST, handleStandby);
  server.on("/api/match-length", HTTP_POST, handleMatchLength);
  server.on("/api/resume-second-half", HTTP_POST, handleResumeSecondHalf);
  server.on("/api/resume-after-goal", HTTP_POST, handleResumeAfterGoal);
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
  server.on("/api/goal-resume", HTTP_POST, handleGoalResumeTuning);
  server.on("/api/end-tuning", HTTP_POST, handleEndTuning);
  server.on("/api/center-shift", HTTP_POST, handleCenterShift);
  server.on("/api/center-test", HTTP_POST, handleCenterTest);
  server.on("/api/blackout", HTTP_POST, handleBlackout);
  server.on("/api/test-mode", HTTP_POST, handleTestMode);
  server.on("/api/test-countdown", HTTP_POST, handleTestCountdown);
  server.on("/api/test-last-ten", HTTP_POST, handleTestLastTen);
  server.on("/api/test-idle", HTTP_POST, handleTestIdle);
  server.on("/api/test-leds", HTTP_POST, handleLedSelfTest);

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
  WiFi.setHostname(OTA_HOSTNAME);

#if STA_USE_STATIC_IP
  if (!WiFi.config(sta_local_IP, sta_gateway, sta_subnet)) {
    Serial.println("[WARN] Static IP configuration failed.");
  }
#else
  Serial.println("[INIT] Using DHCP IP from router.");
#endif

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

  if (MDNS.begin(MDNS_HOSTNAME)) {
    MDNS.addService("http", "tcp", 80);
    Serial.print("[INIT] mDNS ready: http://");
    Serial.print(MDNS_HOSTNAME);
    Serial.println(".local");
  } else {
    Serial.println("[WARN] mDNS failed. Use the numeric IP address shown above.");
  }
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

    if (c >= '1' && c <= '9') {
      audioPlayTrackForced((uint16_t)(c - '0'));
    } else if (c == 's' || c == 'S') {
      startMatchWithDuration(DEFAULT_MATCH_DURATION_SECONDS);
    } else if (c == 't' || c == 'T') {
      startMatchWithDuration(QUICK_MATCH_DURATION_SECONDS);
    } else if (c == 'r' || c == 'R') {
      resumeAfterGoal();
    } else if (c == 'e' || c == 'E') {
      endMatch(true);
    } else if (c == '0') {
      resetGame();
    } else if (c == 'p' || c == 'P') {
      enterStandbyIdleMode();
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
  Serial.println("FIFA Robot Soccer Demo Controller v4.32");
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
    Serial.print("http://");
    Serial.print(MDNS_HOSTNAME);
    Serial.println(".local");
#endif
  }

  setLedMode(LED_MODE_ATTRACT);

  addEvent("System booted");

  Serial.println("[READY] Serial fallback:");
  Serial.println("  s   = start default 90 second match");
  Serial.println("  t   = quick 20 second test match");
  Serial.println("  r   = resume after goal pause");
  Serial.println("  e   = end match");
  Serial.println("  0   = reset");
  Serial.println("  p   = standby / idle mode");
  Serial.println("  b   = fake Blue goal");
  Serial.println("  o   = fake Orange goal");
  Serial.println("  h/H = hit / big hit");
  Serial.println("  1=start, 2=countdown, 3=1st BGM, 4=resume, 5=2nd BGM, 6=end, 7=goal, 8=reset, 9=idle");
  Serial.println("  q   = stop audio");
  Serial.println("  c   = center test");
  Serial.println("  x/X = center X shift down/up");
  Serial.println("  y/Y = center Y coordinate shift down/up");
  Serial.println("  i/I = physical center LED index shift back/forward");
  Serial.println("  z   = reset center shift to demo defaults");
}

void loop() {
  // v4.31:
  // Service sensor flags first so interrupt-captured events are handled
  // before the web UI, LEDs, or audio service work gets time.
  updateSensorsAndGoals();

  if (otaReady) {
    ArduinoOTA.handle();
  }

  if (wifiReady) {
    server.handleClient();
    // Intentionally no MDNS.update() here.
    // ESP32 Arduino core 3.x generally services mDNS internally; adding update()
    // can break builds on cores where that method is not exposed.
  }

  updateAudioState();
  updateGame();

  // Service once more before LED rendering, so a sensor edge that arrives during
  // web/audio/game processing can still be validated before the LED frame update.
  updateSensorsAndGoals();

  updateLeds();
  serviceGameplayBackgroundAudio();
  serviceStandbyIdleAudio();

  serviceSerial();
  drainAudioRx();

  delay(1);
}
