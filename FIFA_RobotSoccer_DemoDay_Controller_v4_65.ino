/*
  FIFA Robot Soccer Demo Controller v4.65 Final-30 Max Brightness Clarification
  LoR Core V3 / ESP32

  Stage:
  - Demo day integrated controller v3.3
  - Web color picker for goal/countdown/start LED animation colors
  - Countdown hex animation uses a 10-second dramatic color ramp plus white comet styles
  - Web UI supports effect test mode, final-30 warning, and countdown/idle tuning
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
  - Audience scoreboard stability, anti-clipping, and premium motion polish for cleaner demo-day presentation
  - Expanded premium LED choreography across attract, in-game, countdown, goal, final-warning, and finale states
  - v4.37: premium show-control flow, configurable branding/team identity, audience attract/ready/post scenes, safer operator controls, and reinforced anti-clipping scoreboard rendering
  - v4.38: single vs two-half match format, start-next-match flow, persistent show settings, operator/audience route split, demo lock, rehearsal scenes, and operator PIN UI gate
  - v4.40: landscape audience readability hotfix (ticker/branding moved below scorecards, team-name clipping fixes, goal banner anti-overlap) + default auto goal resume 5000ms
  - v4.42: web route recovery for iPad/browser cached paths (/index.html, trailing slash, unknown GET fallback)
  - v4.44: standby explainer slides + start-soon transition + full-screen countdown recovery + live timer/score minimal layout sizing fix
  - v4.45: landscape audience fit pass (Get Ready headline sizing tune + larger live score card usage in lower half)
  - v4.46: standby slide centering and full-height usage polish for iPad landscape
  - v4.47: standby copy refresh + color-highlighted Blue/Orange team mentions on attract slides
  - v4.48: FIFA-style standby title gradient for event headline slides
  - v4.49: attract "How To Play" readability boost with slide-specific sizing and anti-overlap framing for 13-inch iPad landscape
  - v4.51: finalized 10-track audio show-flow (windowed playback + Preferences + web panel/API), route-safe API fallbacks, and upgraded winner sparkle accents
  - v4.52: audience scoreboard minimal readability mode (timer + scores only, no clipping text overlays) and ESP-NOW ball link integration (pair/poll/cmd/status)
  - v4.53: restored downtime slideshow/story states, restored full-screen countdown emphasis, restored bright centered GOAL text overlay, and limited minimal-clean layout to live in-game only
  - v4.54: landscape scoreboard fill pass (remove unused lower-space in live + downtime scenes), smoother passive motion timing, longer event-title hold in standby slideshow, three-line Blue/VS/Orange attract copy, and suppress Presented-By line during goal/countdown overlays
  - v4.55: real Pre-game state, explicit 10-track audio flow cleanup, LED mode structure cleanup (OFF-IDLE / PRE-GAME / ON-IDLE), scoreboard CSS consolidation, and strict audio window stop-at-end reliability
  - v4.61: default 120-second match, 10-second countdown, 30-second warning phase, refreshed countdown color animation, and audience countdown sizing cleanup
  - v4.62: attract title/copy polish, stronger attract and countdown motion, Game Over / Controllers Down screen, red field end warning, and final-30 rainbow-hex warning pulse
  - v4.64: iPad countdown ring anchored to viewport center, countdown LEDs changed to full-field rainbow wash with a second-synced hex shooting star, and final-30 hex brightness forced to max.
  - v4.65: Final-30 warning frames temporarily render at the hard 50% FastLED cap, so the white field and rainbow hex are visibly max-bright even if the normal demo brightness slider is lower.
  - v4.57: operator action cleanup (retired Ready Check / Start Next Match flow), hard-fixed Reset/End actions, larger live timer with tuned score scale, re-centered dramatic countdown, stronger red countdown field pulse, smoother second-synced countdown LED choreography, and polished final screen layout
  - v4.58: scoreboard pin stability fix. Audience scoreboard now stays active through Pre-game -> Start Match -> Countdown without flashing the operator UI.
  - v4.60: fixes hard-pin CSS breaking special audience layouts, recenters Pre-game/Attract/Countdown scenes, removes duplicate standby slide wording, and adds safer text sizing for long slide titles.
  - Hard LED safety:
      1. Brightness cannot exceed 50%
      2. Full-field white is allowed, but still limited by the 50% brightness cap
      3. Demo effects avoid full-power white because hardware safety still matters

  Web UI:
  - Clean control dashboard with scoreboard mode toggle
  - Access Point default:
      SSID: RobotSoccer_BACKUP
      Password: fifabot123
      URL: http://192.168.4.1
  - Optional router mode remains available behind DEMO_USE_ACCESS_POINT

  Important LoR Core V3 pin mapping:
  - Right goal sensor A:
      LoR IO_07 signal = ESP32 GPIO13
  - Right goal sensor B:
      LoR IO_10 signal = ESP32 GPIO4
  - Left goal sensor A:
      LoR IO_11 signal = ESP32 GPIO22
  - Left goal sensor B:/
      LoR IO_12 signal = ESP32 GPIO21
  - Audio:
      LoR IO_01 signal = ESP32 GPIO32 = ESP32 RX from module TXD
      LoR IO_02 signal = ESP32 GPIO25 = ESP32 TX to module RXD
      LoR IO_03 signal = ESP32 GPIO26 = BUSY
  - Field LED strip:
      LoR IO_04 signal = ESP32 GPIO27

  E18-D80NK sensors:
  - ACTIVE LOW
  - Use pinMode INPUT, not INPUT_PULLUP
  - If signal is 5V, use level shifter or voltage divider before ESP32 GPIO
  - Goal only counts when both sensors on the same goal confirm within the validation window

  Audio SD card files:
  - 00001.mp3 = downtime idle track 1
  - 00002.mp3 = downtime idle track 2
  - 00003.mp3 = downtime idle track 3
  - 00004.mp3 = downtime idle track 4
  - 00005.mp3 = downtime idle track 5
  - 00006.mp3 = pre-countdown buildup / hype
  - 00007.mp3 = countdown (10-second countdown + GO)
  - 00008.mp3 = in-game gameplay BGM
  - 00009.mp3 = goal sound
  - 00010.mp3 = winner / end-game celebration
*/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WebServer.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <FastLED.h>
#include <Preferences.h>
#include <math.h>

// v4.37 carry-forward compile fix:
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
#define DEFAULT_MATCH_DURATION_SECONDS  120UL
#define QUICK_MATCH_DURATION_SECONDS    20UL
#define COUNTDOWN_SECONDS                10UL

// Ball link (ESP-NOW) integration:
// Set to 0 to disable the ball protocol layer if you need to trim flash/debug.
#define BALL_LINK_ENABLED                1
#define BALL_LINK_PROTO_VER              5
#define BALL_LINK_MAX_PEERS              8
#define BALL_LINK_RX_QUEUE_CAP           16
#define BALL_LINK_POLL_INTERVAL_MS       120UL
#define BALL_LINK_CMD_INTERVAL_MS        120UL
#define BALL_LINK_LINK_TIMEOUT_MS        3000UL

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

// Full-field white is allowed in v4.63 because output brightness is still capped at 50%.
#define LED_BLOCK_FULL_FIELD_WHITE 0

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
// Countdown now uses a 10-second dramatic field/hex sequence with an optional comet.
#define COUNTDOWN_STEP_MS_DEFAULT          1000UL
#define COUNTDOWN_STEP_MS_MIN             400UL
#define COUNTDOWN_STEP_MS_MAX             2000UL
#define COUNTDOWN_FIELD_EFFECT_PERIOD_DEFAULT 1000UL
#define COUNTDOWN_FIELD_EFFECT_PERIOD_MIN     80UL
#define COUNTDOWN_FIELD_EFFECT_PERIOD_MAX     2500UL
#define COUNTDOWN_FIELD_MIN_BRIGHTNESS_DEFAULT 15
#define COUNTDOWN_FIELD_MAX_BRIGHTNESS_DEFAULT 190
#define COUNTDOWN_HEX_BRIGHTNESS_DEFAULT       255

// Smooth hex countdown tuning. Default: one full rotation per countdown second.
#define COUNTDOWN_HEX_ROTATION_MS_DEFAULT      1000UL
#define COUNTDOWN_HEX_ROTATION_MS_MIN          150UL
#define COUNTDOWN_HEX_ROTATION_MS_MAX          2500UL
#define COUNTDOWN_HEX_TRAIL_PERCENT_DEFAULT    18
#define COUNTDOWN_HEX_TRAIL_PERCENT_MIN        8
#define COUNTDOWN_HEX_TRAIL_PERCENT_MAX        90
#define COUNTDOWN_COLOR_COUNT                  10

// Center line portion used during the hex countdown.
// This lights the two physical center strips only where they pass through the hex.
#define COUNTDOWN_CENTER_LINE_Y_MIN 18.0f
#define COUNTDOWN_CENTER_LINE_Y_MAX 43.0f
#define COUNTDOWN_ZONE_COUNT 5

// User-adjustable demo colors. White is allowed, but global LED brightness remains capped at 50%.
#define DEFAULT_BLUE_GOAL_COLOR     CRGB(0, 80, 255)
#define DEFAULT_ORANGE_GOAL_COLOR   CRGB(255, 80, 0)
#define DEFAULT_COUNTDOWN_HEX_COLOR   CRGB(0, 255, 0)
#define DEFAULT_COUNTDOWN_FIELD_COLOR CRGB(255, 24, 0)
#define DEFAULT_GAME_START_COLOR      CRGB(0, 255, 0)

// Optional 10-step accent colors for countdown 10 through 1.
// These are intentionally high-energy, non-white colors for demo-day visibility.
#define DEFAULT_COUNTDOWN_COLOR_10 CRGB(0, 210, 255)
#define DEFAULT_COUNTDOWN_COLOR_9  CRGB(0, 110, 255)
#define DEFAULT_COUNTDOWN_COLOR_8  CRGB(45, 70, 255)
#define DEFAULT_COUNTDOWN_COLOR_7  CRGB(110, 45, 255)
#define DEFAULT_COUNTDOWN_COLOR_6  CRGB(180, 35, 255)
#define DEFAULT_COUNTDOWN_COLOR_5  CRGB(255, 0, 210)
#define DEFAULT_COUNTDOWN_COLOR_4  CRGB(255, 0, 120)
#define DEFAULT_COUNTDOWN_COLOR_3  CRGB(255, 60, 0)
#define DEFAULT_COUNTDOWN_COLOR_2  CRGB(255, 160, 0)
#define DEFAULT_COUNTDOWN_COLOR_1  CRGB(255, 18, 0)

// Backward-compatible names used by the web color pickers.
#define DEFAULT_COUNTDOWN_SECOND_3_COLOR DEFAULT_COUNTDOWN_COLOR_10
#define DEFAULT_COUNTDOWN_SECOND_2_COLOR DEFAULT_COUNTDOWN_COLOR_5
#define DEFAULT_COUNTDOWN_SECOND_1_COLOR DEFAULT_COUNTDOWN_COLOR_1

// White/near-white user-selected colors are allowed in v4.63. The 50% brightness cap remains the safety limit.
#define USER_COLOR_WHITE_COMPONENT_LIMIT 255

// Final warning phase effect. Variable names remain lastTen for web/API compatibility.
#define WARNING_PHASE_DURATION_MS    30000UL
#define LAST_TEN_WARNING_ENABLED_DEFAULT 1
#define LAST_TEN_PULSE_COLOR_DEFAULT CRGB(255, 255, 255)
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

const uint16_t MAX_TRACK = 10;

// Audio track map (event-day final):
//   00001..00005 = downtime idle rotation
//   00006         = pre-game buildup / hype
//   00007         = countdown (10-second countdown + GO)
//   00008         = in-game BGM
//   00009         = goal sound
//   00010         = winner celebration
static const uint16_t AUDIO_TRACK_IDLE_MIN       = 1;
static const uint16_t AUDIO_TRACK_IDLE_MAX       = 5;
static const uint16_t AUDIO_TRACK_BUILDUP        = 6;
static const uint16_t AUDIO_TRACK_COUNTDOWN      = 7;
static const uint16_t AUDIO_TRACK_GAMEPLAY_BGM   = 8;
static const uint16_t AUDIO_TRACK_GOAL           = 9;
static const uint16_t AUDIO_TRACK_WINNER         = 10;

// Estimated source track lengths. Runtime window timing can shorten these.
const unsigned long TRACK_LENGTH_MS[MAX_TRACK + 1] = {
  0,
  60000,   // 1 downtime idle 1
  60000,   // 2 downtime idle 2
  60000,   // 3 downtime idle 3
  60000,   // 4 downtime idle 4
  60000,   // 5 downtime idle 5
  126000,  // 6 pre-game buildup / hype
  10000,   // 7 countdown (10-second countdown + GO)
  120000,  // 8 gameplay BGM
  5000,    // 9 goal
  12000    // 10 winner celebration
};

// Playback windows are runtime-tunable from the dashboard.
// DY-HV20T seek-to-time is not reliably available in this project path.
// Start offsets are implemented as silent pre-roll (temporary mute until start second).
static const char* AUDIO_PREF_NAMESPACE = "fifa_audio_v461";
// v4.61 intentionally uses a fresh namespace so old 90s/3s countdown settings do not override the new demo flow.
static const char* SETTINGS_PREF_NAMESPACE = "fifa_ctrl_v461";
const uint16_t AUDIO_WINDOW_DEFAULT_START_SEC[MAX_TRACK + 1] = {
  0,
  0,   // 1
  0,   // 2
  0,   // 3
  0,   // 4
  0,   // 5
  0,   // 6
  0,   // 7
  0,   // 8
  0,   // 9
  0    // 10
};
const uint16_t AUDIO_WINDOW_DEFAULT_END_SEC[MAX_TRACK + 1] = {
  0,
  60,  // 1
  60,  // 2
  60,  // 3
  60,  // 4
  60,  // 5
  126, // 6
  10,  // 7
  120, // 8
  5,   // 9
  12   // 10
};

enum AudioShowState {
  AUDIO_SHOW_DOWNTIME = 0,
  AUDIO_SHOW_BUILDUP,
  AUDIO_SHOW_COUNTDOWN,
  AUDIO_SHOW_GAMEPLAY,
  AUDIO_SHOW_GOAL_ONE_SHOT,
  AUDIO_SHOW_WINNER,
  AUDIO_SHOW_MANUAL_TEST,
  AUDIO_SHOW_STOPPED
};

// Audio command timing guards. These small delays make the DY-HV20T less likely
// to ignore rapid STOP -> PLAY changes during countdown and goal events.
static const unsigned long AUDIO_COMMAND_SETTLE_MS = 150UL;
static const unsigned long AUDIO_HARD_STOP_GAP_MS = 90UL;
static const unsigned long GAME_START_TO_BGM_DELAY_MS = 650UL;

HardwareSerial MP3Serial(2);

// ============================================================
// WEB SERVER
// ============================================================

WebServer server(80);
Preferences settingsPrefs;

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

const unsigned long LED_UPDATE_MS = 20;

enum LedMode {
  LED_MODE_OFF_IDLE = 0,
  LED_MODE_PRE_GAME,
  LED_MODE_ON_IDLE,
  LED_MODE_COUNTDOWN,
  LED_MODE_GAME_START_GREEN,
  LED_MODE_BLUE_GOAL,
  LED_MODE_ORANGE_GOAL,
  LED_MODE_HIT,
  LED_MODE_BIG_HIT,
  LED_MODE_MATCH_END,
  LED_MODE_LAST_TEN,
  LED_MODE_LED_SELF_TEST,
  LED_MODE_CENTER_TEST,
  LED_MODE_BLACKOUT,
  // Compatibility aliases for older JS/diagnostics names.
  LED_MODE_ATTRACT = LED_MODE_OFF_IDLE,
  LED_MODE_IN_GAME = LED_MODE_ON_IDLE,
  LED_MODE_LAST_TEN_TEST = LED_MODE_LAST_TEN
};

LedMode ledMode = LED_MODE_OFF_IDLE;
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

CountdownFieldEffectMode countdownFieldEffectMode = COUNTDOWN_FIELD_HEARTBEAT;
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
  DEFAULT_COUNTDOWN_COLOR_10,
  DEFAULT_COUNTDOWN_COLOR_9,
  DEFAULT_COUNTDOWN_COLOR_8,
  DEFAULT_COUNTDOWN_COLOR_7,
  DEFAULT_COUNTDOWN_COLOR_6,
  DEFAULT_COUNTDOWN_COLOR_5,
  DEFAULT_COUNTDOWN_COLOR_4,
  DEFAULT_COUNTDOWN_COLOR_3,
  DEFAULT_COUNTDOWN_COLOR_2,
  DEFAULT_COUNTDOWN_COLOR_1
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
// New v4.62 match-end pre-finale warning.
// The field and iPad show a short red GAME OVER / CONTROLLERS DOWN scene before the winner finale begins.
const unsigned long GAME_OVER_WARNING_MS = 2600UL;
const unsigned long MATCH_END_EFFECT_MS_DEFAULT = 13500UL;
const unsigned long MATCH_END_EFFECT_MS_MIN = 3000UL;
const unsigned long MATCH_END_EFFECT_MS_MAX = 20000UL;

// Winner finale tuning. These are web-adjustable.
// winnerSpeed controls how fast the winner-colored rings move.
// rainbowSpeed controls how fast the center hex rainbow gradient spins.
// ringSpacing controls how close together the winner rings are.
unsigned long matchEndEffectMs = MATCH_END_EFFECT_MS_DEFAULT;
uint8_t matchEndWinnerSpeed = 255;
uint8_t matchEndRainbowSpeed = 18;
uint8_t matchEndRingSpacing = 37;

// Finale profile:
// 0 = Clean, 1 = Showtime (default), 2 = Maximum Safe
uint8_t finaleIntensityMode = 1;

// Audience messaging and show storytelling settings.
const char* DEFAULT_EVENT_TITLE = "2026 FIFA FAN FESTIVAL";
const char* DEFAULT_EVENT_SUBTITLE = "Robot Soccer Arena";
const char* DEFAULT_PRESENTED_BY = "Presented by Humber Polytechnic and Lord of Robots";
const char* DEFAULT_ATTRACT_MESSAGES = "Choose your bot|Blue Bots vs Orange Bots|120 second robot soccer challenge|2 vs 2 robot showdown";
const char* DEFAULT_LAST_TEN_TEXT = "FINAL 30";

// Team display names:
// By default these stay Blue Bots / Orange Bots.
// Custom names are optional and runtime-configurable from the web UI.
const char* DEFAULT_BLUE_TEAM_NAME = "Blue Bots";
const char* DEFAULT_ORANGE_TEAM_NAME = "Orange Bots";

String eventTitle = DEFAULT_EVENT_TITLE;
String eventSubtitle = DEFAULT_EVENT_SUBTITLE;
String presentedByLine = DEFAULT_PRESENTED_BY;
String attractMessagesCsv = DEFAULT_ATTRACT_MESSAGES;
String lastTenWarningText = DEFAULT_LAST_TEN_TEXT;
bool showPresentedByLine = true;
bool showEventBrandingInLive = true;

bool customTeamNamesEnabled = false;
String blueTeamCustomName = DEFAULT_BLUE_TEAM_NAME;
String orangeTeamCustomName = DEFAULT_ORANGE_TEAM_NAME;

// Player flow / post-match loop controls.
bool autoReturnToStandbyEnabled = true;
unsigned long autoReturnToStandbyDelayMs = 15000UL;
unsigned long postMatchStateStartMs = 0;
bool lastTenDramaticMode = true;

// Match format:
// 0 = single match (default demo-day mode)
// 1 = two-half match (legacy flow)
enum MatchFormatMode {
  MATCH_FORMAT_SINGLE = 0,
  MATCH_FORMAT_TWO_HALF
};
MatchFormatMode matchFormatMode = MATCH_FORMAT_SINGLE;

// Keep winner/final score visible for at least this long before any auto-return.
unsigned long postMatchMinimumHoldMs = 8000UL;

// Demo-day safe surface mode:
// hides advanced controls and keeps operator flow compact.
bool demoLockModeEnabled = false;

// Rehearsal scene override for scoreboard and 20-foot readability checks.
enum RehearsalSceneMode {
  REHEARSAL_SCENE_NONE = 0,
  REHEARSAL_SCENE_ATTRACT,
  REHEARSAL_SCENE_COUNTDOWN,
  REHEARSAL_SCENE_GOAL_BLUE,
  REHEARSAL_SCENE_GOAL_ORANGE,
  REHEARSAL_SCENE_FINAL_TEN,
  REHEARSAL_SCENE_WIN_BLUE,
  REHEARSAL_SCENE_WIN_ORANGE,
  REHEARSAL_SCENE_WIN_TIE,
  REHEARSAL_SCENE_READABILITY
};
RehearsalSceneMode rehearsalSceneMode = REHEARSAL_SCENE_NONE;
bool rehearsalSceneActive = false;
unsigned long rehearsalSceneStartMs = 0;
unsigned long rehearsalSceneDurationMs = 4500UL;

// Optional PIN placeholder:
// We intentionally keep this disabled by default because full API-level auth
// on open demo AP networks can add operational risk during live demos.
bool operatorPinEnabled = false;
const char* OPERATOR_PIN_CODE = "2468";

enum AudienceScreenOverrideMode {
  AUDIENCE_OVERRIDE_AUTO = 0,
  AUDIENCE_OVERRIDE_ATTRACT,
  AUDIENCE_OVERRIDE_POST_MATCH
};

AudienceScreenOverrideMode audienceOverrideMode = AUDIENCE_OVERRIDE_AUTO;

// Match-end center split:
// Strip A = LEDs 422-507, Strip C = LEDs 509-594.
// If the physical left/right colors are backwards, change this to 0.
#define MATCH_END_CENTER_STRIP_A_IS_BLUE 1

// ============================================================
// GAME STATE
// ============================================================

enum GameState {
  STATE_STANDBY = 0,
  STATE_PRE_GAME,
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
// Halves are still controlled by match state, but gameplay BGM now uses
// a single in-game track (00008.mp3) for both halves.
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
bool autoResumeAfterGoalEnabled = true;
unsigned long autoResumeAfterGoalDelayMs = 5000UL;

int lastCountdownAnnounced = -1;
unsigned long lastCountdownBeepMs = 0;

// Countdown audio is track 00007 in this version.
// The controller starts it once when countdown begins.
bool countdownSequenceStarted = false;

enum GoalTeam {
  GOAL_TEAM_NONE = 0,
  GOAL_TEAM_BLUE,
  GOAL_TEAM_ORANGE
};
GoalTeam lastGoalTeam = GOAL_TEAM_NONE;

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
uint16_t lastIdleDowntimeTrack = 0;

unsigned long currentTrackStartTime = 0;
unsigned long lastAudioCommandMs = 0;

bool trackActive = false;
bool audioMutedForStartOffset = false;

AudioShowState audioShowState = AUDIO_SHOW_STOPPED;

uint16_t trackWindowStartSec[MAX_TRACK + 1] = {0};
uint16_t trackWindowEndSec[MAX_TRACK + 1] = {0};
uint16_t currentTrackStartSec = 0;
uint16_t currentTrackEndSec = 0;

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

#if BALL_LINK_ENABLED
// ============================================================
// BALL LINK (ESP-NOW FMS SIDE)
// ============================================================

enum BallMsgType : uint8_t {
  BALL_MSG_PAIR_REQ = 1,
  BALL_MSG_PAIR_ACK = 2,
  BALL_MSG_POLL = 3,
  BALL_MSG_STATUS = 4,
  BALL_MSG_CMD = 5
};

enum BallCmdState : uint8_t {
  BALL_SYS_STANDBY = 0,
  BALL_SYS_GAME_START = 1,
  BALL_SYS_IN_GAME = 2,
  BALL_SYS_TIME_GATE = 3,
  BALL_SYS_END_GAME = 4,
  BALL_SYS_RESET = 5
};

enum BallCmdAnim : uint8_t {
  BALL_ANIM_STILL = 0,
  BALL_ANIM_X_POS = 1,
  BALL_ANIM_X_NEG = 2,
  BALL_ANIM_Y_POS = 3,
  BALL_ANIM_Y_NEG = 4,
  BALL_ANIM_GOAL = 5
};

struct __attribute__((packed)) BallPairReq {
  uint8_t proto_ver;
  uint8_t msg_type;
  uint8_t desired_fms;
  uint32_t nonce;
};

struct __attribute__((packed)) BallPairAck {
  uint8_t proto_ver;
  uint8_t msg_type;
  uint8_t fms_id;
  uint8_t channel;
  uint32_t nonce;
};

struct __attribute__((packed)) BallPollPkt {
  uint8_t proto_ver;
  uint8_t msg_type;
  uint8_t seq;
};

struct __attribute__((packed)) BallStatusPkt {
  uint8_t proto_ver;
  uint8_t msg_type;
  uint8_t seq;
  int8_t gyroX;
  int8_t gyroY;
  int8_t gyroZ;
  uint8_t moving;
  uint32_t uptime_s;
  uint16_t batt_mv;
  uint8_t batt_pct;
};

struct __attribute__((packed)) BallCmdPkt {
  uint8_t proto_ver;
  uint8_t msg_type;
  uint8_t sys_state;
  uint8_t color;
  uint8_t anim;
  uint16_t beep_hz;
  uint16_t beep_ms;
};

struct BallRxFrame {
  uint8_t src[6];
  uint8_t len;
  uint8_t data[32];
};

struct BallPeerState {
  bool used;
  bool linked;
  uint8_t mac[6];
  uint8_t desiredFms;
  uint8_t channel;
  uint8_t nextPollSeq;
  uint8_t lastStatusSeq;
  unsigned long lastPairReqMs;
  unsigned long lastRxMs;
  unsigned long lastPollMs;
  unsigned long lastCmdMs;
  int8_t gyroX;
  int8_t gyroY;
  int8_t gyroZ;
  bool moving;
  uint32_t uptimeS;
  uint16_t battMv;
  uint8_t battPct;
};

static bool ballLinkReady = false;
static BallPeerState ballPeers[BALL_LINK_MAX_PEERS];
static BallRxFrame ballRxQueue[BALL_LINK_RX_QUEUE_CAP];
static volatile uint8_t ballRxHead = 0;
static volatile uint8_t ballRxTail = 0;
static portMUX_TYPE ballRxMux = portMUX_INITIALIZER_UNLOCKED;
#endif

// ============================================================
// SMALL HELPERS
// ============================================================

static String gameStateName() {
  if (gameState == STATE_STANDBY) return "STANDBY";
  if (gameState == STATE_PRE_GAME) return "PRE_GAME";
  if (gameState == STATE_COUNTDOWN) return "COUNTDOWN";
  if (gameState == STATE_IN_GAME) return "IN_GAME";
  if (gameState == STATE_GOAL_PAUSE) return "GOAL_PAUSE";
  if (gameState == STATE_HALFTIME) return "HALFTIME";
  if (gameState == STATE_END_GAME) return "END_GAME";
  return "UNKNOWN";
}

static String ledModeName() {
  if (ledMode == LED_MODE_OFF_IDLE) return "OFF_IDLE";
  if (ledMode == LED_MODE_PRE_GAME) return "PRE_GAME";
  if (ledMode == LED_MODE_ON_IDLE) return "ON_IDLE";
  if (ledMode == LED_MODE_COUNTDOWN) return "COUNTDOWN";
  if (ledMode == LED_MODE_GAME_START_GREEN) return "GAME_START_GREEN";
  if (ledMode == LED_MODE_BLUE_GOAL) return "BLUE_GOAL";
  if (ledMode == LED_MODE_ORANGE_GOAL) return "ORANGE_GOAL";
  if (ledMode == LED_MODE_HIT) return "HIT";
  if (ledMode == LED_MODE_BIG_HIT) return "BIG_HIT";
  if (ledMode == LED_MODE_MATCH_END) return "MATCH_END";
  if (ledMode == LED_MODE_LAST_TEN) return "LAST_TEN";
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

static String sanitizeDisplayText(String input, const char* fallback, uint8_t maxLen) {
  input.trim();
  input.replace("\r", " ");
  input.replace("\n", " ");

  while (input.indexOf("  ") >= 0) {
    input.replace("  ", " ");
  }

  if (input.length() == 0) {
    input = String(fallback);
  }

  if (input.length() > maxLen) {
    input = input.substring(0, maxLen);
    input.trim();
  }

  if (input.length() == 0) {
    input = String(fallback);
  }

  return input;
}

static String activeBlueTeamName() {
  if (customTeamNamesEnabled) {
    return sanitizeDisplayText(blueTeamCustomName, DEFAULT_BLUE_TEAM_NAME, 24);
  }
  return String(DEFAULT_BLUE_TEAM_NAME);
}

static String activeOrangeTeamName() {
  if (customTeamNamesEnabled) {
    return sanitizeDisplayText(orangeTeamCustomName, DEFAULT_ORANGE_TEAM_NAME, 24);
  }
  return String(DEFAULT_ORANGE_TEAM_NAME);
}

static String audienceOverrideModeName() {
  if (audienceOverrideMode == AUDIENCE_OVERRIDE_ATTRACT) return "attract";
  if (audienceOverrideMode == AUDIENCE_OVERRIDE_POST_MATCH) return "post_match";
  return "auto";
}

static String matchFormatModeName() {
  return (matchFormatMode == MATCH_FORMAT_TWO_HALF) ? "two_half" : "single";
}

static String rehearsalSceneModeName() {
  if (!rehearsalSceneActive || rehearsalSceneMode == REHEARSAL_SCENE_NONE) return "none";
  if (rehearsalSceneMode == REHEARSAL_SCENE_ATTRACT) return "attract";
  if (rehearsalSceneMode == REHEARSAL_SCENE_COUNTDOWN) return "countdown";
  if (rehearsalSceneMode == REHEARSAL_SCENE_GOAL_BLUE) return "goal_blue";
  if (rehearsalSceneMode == REHEARSAL_SCENE_GOAL_ORANGE) return "goal_orange";
  if (rehearsalSceneMode == REHEARSAL_SCENE_FINAL_TEN) return "final_ten";
  if (rehearsalSceneMode == REHEARSAL_SCENE_WIN_BLUE) return "win_blue";
  if (rehearsalSceneMode == REHEARSAL_SCENE_WIN_ORANGE) return "win_orange";
  if (rehearsalSceneMode == REHEARSAL_SCENE_WIN_TIE) return "win_tie";
  if (rehearsalSceneMode == REHEARSAL_SCENE_READABILITY) return "readability";
  return "none";
}

static const char* audioTrackLabel(uint16_t track) {
  if (track == 1) return "Downtime Idle 1";
  if (track == 2) return "Downtime Idle 2";
  if (track == 3) return "Downtime Idle 3";
  if (track == 4) return "Downtime Idle 4";
  if (track == 5) return "Downtime Idle 5";
  if (track == 6) return "Pre-game Buildup";
  if (track == 7) return "Countdown 10-1-GO";
  if (track == 8) return "Gameplay BGM";
  if (track == 9) return "Goal Sound";
  if (track == 10) return "Winner Celebration";
  return "Idle";
}

static const char* audioShowStateName() {
  switch (audioShowState) {
    case AUDIO_SHOW_DOWNTIME: return "downtime";
    case AUDIO_SHOW_BUILDUP: return "buildup";
    case AUDIO_SHOW_COUNTDOWN: return "countdown";
    case AUDIO_SHOW_GAMEPLAY: return "gameplay";
    case AUDIO_SHOW_GOAL_ONE_SHOT: return "goal";
    case AUDIO_SHOW_WINNER: return "winner";
    case AUDIO_SHOW_MANUAL_TEST: return "manual_test";
    default: return "stopped";
  }
}

static MatchFormatMode parseMatchFormatMode(String value) {
  value.toLowerCase();
  if (value == "1" || value == "two_half" || value == "twohalf" || value == "halves" || value == "two") {
    return MATCH_FORMAT_TWO_HALF;
  }
  return MATCH_FORMAT_SINGLE;
}

static RehearsalSceneMode parseRehearsalSceneMode(String value) {
  value.toLowerCase();
  if (value == "attract") return REHEARSAL_SCENE_ATTRACT;
  if (value == "countdown") return REHEARSAL_SCENE_COUNTDOWN;
  if (value == "goal_blue" || value == "blue_goal") return REHEARSAL_SCENE_GOAL_BLUE;
  if (value == "goal_orange" || value == "orange_goal") return REHEARSAL_SCENE_GOAL_ORANGE;
  if (value == "final_ten" || value == "last_ten") return REHEARSAL_SCENE_FINAL_TEN;
  if (value == "win_blue" || value == "blue_win") return REHEARSAL_SCENE_WIN_BLUE;
  if (value == "win_orange" || value == "orange_win") return REHEARSAL_SCENE_WIN_ORANGE;
  if (value == "win_tie" || value == "tie") return REHEARSAL_SCENE_WIN_TIE;
  if (value == "readability" || value == "readability_test") return REHEARSAL_SCENE_READABILITY;
  return REHEARSAL_SCENE_NONE;
}

static bool validateOperatorPinCandidate(String candidate) {
  candidate.trim();
  if (candidate.length() < 3 || candidate.length() > 12) {
    return false;
  }
  for (uint8_t i = 0; i < candidate.length(); i++) {
    char ch = candidate.charAt(i);
    if (ch < '0' || ch > '9') {
      return false;
    }
  }
  return candidate == String(OPERATOR_PIN_CODE);
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
  (void)fallback;
  // v4.63: white and near-white colors are intentionally allowed.
  // The hardware safety rule is now the global 50% LED brightness cap, not color blocking.
  return requested;
}

static String colorToHex(CRGB c) {
  char buf[8];
  snprintf(buf, sizeof(buf), "#%02X%02X%02X", c.r, c.g, c.b);
  return String(buf);
}

static void enforcePostMatchTimingConstraints() {
  postMatchMinimumHoldMs = clampUL(postMatchMinimumHoldMs, 3000UL, 30000UL);
  autoReturnToStandbyDelayMs = clampUL(autoReturnToStandbyDelayMs, 5000UL, 90000UL);
  if (autoReturnToStandbyDelayMs < postMatchMinimumHoldMs) {
    autoReturnToStandbyDelayMs = postMatchMinimumHoldMs;
  }
}

static unsigned long effectivePostMatchAutoReturnDelayMs() {
  unsigned long autoDelay = autoReturnToStandbyDelayMs;
  if (autoDelay < postMatchMinimumHoldMs) {
    autoDelay = postMatchMinimumHoldMs;
  }

  // v4.62: do not auto-return before the new red Game Over scene plus the winner finale can finish.
  unsigned long minimumVisualShowMs = GAME_OVER_WARNING_MS + matchEndEffectMs;
  if (autoDelay < minimumVisualShowMs) {
    autoDelay = minimumVisualShowMs;
  }

  return autoDelay;
}

static void loadDefaultShowSettings() {
  eventTitle = DEFAULT_EVENT_TITLE;
  eventSubtitle = DEFAULT_EVENT_SUBTITLE;
  presentedByLine = DEFAULT_PRESENTED_BY;
  attractMessagesCsv = DEFAULT_ATTRACT_MESSAGES;
  lastTenWarningText = DEFAULT_LAST_TEN_TEXT;
  showPresentedByLine = true;
  showEventBrandingInLive = true;
  customTeamNamesEnabled = false;
  blueTeamCustomName = DEFAULT_BLUE_TEAM_NAME;
  orangeTeamCustomName = DEFAULT_ORANGE_TEAM_NAME;
  matchFormatMode = MATCH_FORMAT_SINGLE;
  autoReturnToStandbyEnabled = true;
  autoReturnToStandbyDelayMs = 15000UL;
  postMatchMinimumHoldMs = 8000UL;
  lastTenDramaticMode = true;
  finaleIntensityMode = 1;
  demoLockModeEnabled = false;
  operatorPinEnabled = false;
  rehearsalSceneActive = false;
  rehearsalSceneMode = REHEARSAL_SCENE_NONE;
  rehearsalSceneStartMs = 0;
  rehearsalSceneDurationMs = 4500UL;
}

static void loadDefaultAudioWindows() {
  for (uint16_t t = 1; t <= MAX_TRACK; t++) {
    trackWindowStartSec[t] = AUDIO_WINDOW_DEFAULT_START_SEC[t];
    trackWindowEndSec[t] = AUDIO_WINDOW_DEFAULT_END_SEC[t];
  }
}

static void normalizeAudioWindowTrack(uint16_t track) {
  if (track == 0 || track > MAX_TRACK) return;

  uint16_t startSec = trackWindowStartSec[track];
  uint16_t endSec = trackWindowEndSec[track];

  if (startSec > 3600) startSec = 3600;
  if (endSec > 3600) endSec = 3600;
  if (endSec == 0) endSec = AUDIO_WINDOW_DEFAULT_END_SEC[track];
  if (endSec <= startSec) endSec = startSec + 1;

  trackWindowStartSec[track] = startSec;
  trackWindowEndSec[track] = endSec;
}

static void setAudioWindowTrack(uint16_t track, uint16_t startSec, uint16_t endSec) {
  if (track == 0 || track > MAX_TRACK) return;
  trackWindowStartSec[track] = startSec;
  trackWindowEndSec[track] = endSec;
  normalizeAudioWindowTrack(track);
}

static bool saveAudioWindowsToPreferences() {
  if (!settingsPrefs.begin(AUDIO_PREF_NAMESPACE, false)) {
    return false;
  }

  for (uint16_t t = 1; t <= MAX_TRACK; t++) {
    String startKey = "s" + String(t);
    String endKey = "e" + String(t);
    settingsPrefs.putUShort(startKey.c_str(), trackWindowStartSec[t]);
    settingsPrefs.putUShort(endKey.c_str(), trackWindowEndSec[t]);
  }

  settingsPrefs.end();
  return true;
}

static bool loadAudioWindowsFromPreferences() {
  loadDefaultAudioWindows();

  if (!settingsPrefs.begin(AUDIO_PREF_NAMESPACE, true)) {
    return false;
  }

  for (uint16_t t = 1; t <= MAX_TRACK; t++) {
    String startKey = "s" + String(t);
    String endKey = "e" + String(t);
    trackWindowStartSec[t] = settingsPrefs.getUShort(startKey.c_str(), trackWindowStartSec[t]);
    trackWindowEndSec[t] = settingsPrefs.getUShort(endKey.c_str(), trackWindowEndSec[t]);
    normalizeAudioWindowTrack(t);
  }

  settingsPrefs.end();
  return true;
}

static bool resetAudioWindowsToDefaultsAndSave() {
  loadDefaultAudioWindows();
  return saveAudioWindowsToPreferences();
}

static bool saveSettingsToPreferences() {
  if (!settingsPrefs.begin(SETTINGS_PREF_NAMESPACE, false)) {
    return false;
  }

  settingsPrefs.putString("event_title", sanitizeDisplayText(eventTitle, DEFAULT_EVENT_TITLE, 54));
  settingsPrefs.putString("event_sub", sanitizeDisplayText(eventSubtitle, DEFAULT_EVENT_SUBTITLE, 64));
  settingsPrefs.putString("presented", sanitizeDisplayText(presentedByLine, DEFAULT_PRESENTED_BY, 84));
  settingsPrefs.putBool("show_pres", showPresentedByLine);
  settingsPrefs.putBool("show_live", showEventBrandingInLive);
  settingsPrefs.putString("attract", sanitizeDisplayText(attractMessagesCsv, DEFAULT_ATTRACT_MESSAGES, 240));
  settingsPrefs.putBool("cust_team", customTeamNamesEnabled);
  settingsPrefs.putString("blue_name", sanitizeDisplayText(blueTeamCustomName, DEFAULT_BLUE_TEAM_NAME, 24));
  settingsPrefs.putString("org_name", sanitizeDisplayText(orangeTeamCustomName, DEFAULT_ORANGE_TEAM_NAME, 24));
  settingsPrefs.putUChar("m_format", (uint8_t)matchFormatMode);
  settingsPrefs.putULong("match_ms", matchDurationMs);
  settingsPrefs.putBool("auto_stby", autoReturnToStandbyEnabled);
  settingsPrefs.putULong("auto_dly", autoReturnToStandbyDelayMs);
  settingsPrefs.putULong("post_hold", postMatchMinimumHoldMs);
  settingsPrefs.putBool("l10_en", lastTenWarningEnabled);
  settingsPrefs.putBool("l10_dram", lastTenDramaticMode);
  settingsPrefs.putString("l10_txt", sanitizeDisplayText(lastTenWarningText, DEFAULT_LAST_TEN_TEXT, 24));
  settingsPrefs.putString("l10_col", colorToHex(lastTenPulseColor));
  settingsPrefs.putUChar("fin_int", finaleIntensityMode);
  settingsPrefs.putUChar("vol_pct", currentVolumePercent);
  settingsPrefs.putUChar("led_pct", currentLedBrightnessPercent);
  settingsPrefs.putBool("demo_lock", demoLockModeEnabled);
  settingsPrefs.putBool("pin_en", operatorPinEnabled);
  settingsPrefs.end();
  return true;
}

static bool loadSettingsFromPreferences() {
  loadDefaultShowSettings();

  if (!settingsPrefs.begin(SETTINGS_PREF_NAMESPACE, true)) {
    return false;
  }

  String loadedEventTitle = settingsPrefs.getString("event_title", eventTitle);
  // Migrate older saved profiles so the audience title updates without forcing a full settings reset.
  if (loadedEventTitle == "2026 FIFA FanFest" || loadedEventTitle == "2026 FIFA FANFEST" || loadedEventTitle == "2026 FIFA Fan Fest" || loadedEventTitle == "2026 FIFA FAN FEST") {
    loadedEventTitle = DEFAULT_EVENT_TITLE;
  }
  eventTitle = sanitizeDisplayText(loadedEventTitle, DEFAULT_EVENT_TITLE, 54);
  eventSubtitle = sanitizeDisplayText(settingsPrefs.getString("event_sub", eventSubtitle), DEFAULT_EVENT_SUBTITLE, 64);
  presentedByLine = sanitizeDisplayText(settingsPrefs.getString("presented", presentedByLine), DEFAULT_PRESENTED_BY, 84);
  showPresentedByLine = settingsPrefs.getBool("show_pres", showPresentedByLine);
  showEventBrandingInLive = settingsPrefs.getBool("show_live", showEventBrandingInLive);
  attractMessagesCsv = sanitizeDisplayText(settingsPrefs.getString("attract", attractMessagesCsv), DEFAULT_ATTRACT_MESSAGES, 240);
  customTeamNamesEnabled = settingsPrefs.getBool("cust_team", customTeamNamesEnabled);
  blueTeamCustomName = sanitizeDisplayText(settingsPrefs.getString("blue_name", blueTeamCustomName), DEFAULT_BLUE_TEAM_NAME, 24);
  orangeTeamCustomName = sanitizeDisplayText(settingsPrefs.getString("org_name", orangeTeamCustomName), DEFAULT_ORANGE_TEAM_NAME, 24);

  uint8_t loadedFormat = settingsPrefs.getUChar("m_format", (uint8_t)matchFormatMode);
  matchFormatMode = (loadedFormat == (uint8_t)MATCH_FORMAT_TWO_HALF) ? MATCH_FORMAT_TWO_HALF : MATCH_FORMAT_SINGLE;

  unsigned long loadedMatchMs = settingsPrefs.getULong("match_ms", matchDurationMs);
  matchDurationMs = clampUL(loadedMatchMs, 10000UL, 600000UL);
  pausedMatchRemainingMs = matchDurationMs;

  autoReturnToStandbyEnabled = settingsPrefs.getBool("auto_stby", autoReturnToStandbyEnabled);
  autoReturnToStandbyDelayMs = settingsPrefs.getULong("auto_dly", autoReturnToStandbyDelayMs);
  postMatchMinimumHoldMs = settingsPrefs.getULong("post_hold", postMatchMinimumHoldMs);
  lastTenWarningEnabled = settingsPrefs.getBool("l10_en", lastTenWarningEnabled);
  lastTenDramaticMode = settingsPrefs.getBool("l10_dram", lastTenDramaticMode);
  lastTenWarningText = sanitizeDisplayText(settingsPrefs.getString("l10_txt", lastTenWarningText), DEFAULT_LAST_TEN_TEXT, 24);
  lastTenPulseColor = parseWebHexColor(settingsPrefs.getString("l10_col", colorToHex(lastTenPulseColor)), LAST_TEN_PULSE_COLOR_DEFAULT);
  finaleIntensityMode = clampU8(settingsPrefs.getUChar("fin_int", finaleIntensityMode));
  if (finaleIntensityMode > 2) finaleIntensityMode = 1;

  demoLockModeEnabled = settingsPrefs.getBool("demo_lock", demoLockModeEnabled);
  operatorPinEnabled = settingsPrefs.getBool("pin_en", operatorPinEnabled);

  uint8_t loadedVolume = settingsPrefs.getUChar("vol_pct", currentVolumePercent);
  if (loadedVolume > 100) loadedVolume = 100;
  audioSetVolumePercent(loadedVolume);

  uint8_t loadedLed = settingsPrefs.getUChar("led_pct", currentLedBrightnessPercent);
  if (loadedLed > LED_BRIGHTNESS_MAX_PERCENT) loadedLed = LED_BRIGHTNESS_MAX_PERCENT;
  setLedBrightnessPercent(loadedLed);

  settingsPrefs.end();

  enforcePostMatchTimingConstraints();
  return true;
}

static bool resetSavedSettingsToDefaults() {
  loadDefaultShowSettings();
  matchDurationMs = DEFAULT_MATCH_DURATION_SECONDS * 1000UL;
  pausedMatchRemainingMs = matchDurationMs;
  lastTenWarningEnabled = LAST_TEN_WARNING_ENABLED_DEFAULT;
  lastTenPulseColor = LAST_TEN_PULSE_COLOR_DEFAULT;
  audioSetVolumePercent(80);
  setLedBrightnessPercent(25);
  enforcePostMatchTimingConstraints();
  return saveSettingsToPreferences();
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

static void safeFastLedShowAtMaxAllowedBrightness() {
  // v4.65:
  // Used only for the final-30 warning frame.
  // Pixel brightness alone was not enough because the user's normal brightness
  // slider can be below the safety cap. This temporarily renders the warning at
  // the hard 50% FastLED cap, then restores the user's normal brightness value.
  enforceLedBrightnessLimit();

  uint8_t restoreBrightnessValue = currentLedBrightnessValue;
  FastLED.setBrightness(LED_BRIGHTNESS_MAX_VALUE);

  if (frameLooksLikeFullWhite()) {
    replaceUnsafeWhiteFrame();
    Serial.println("[LED_SAFETY] Blocked full-field white frame. Replaced with safe cyan.");
  }

  FastLED.show();
  FastLED.setBrightness(restoreBrightnessValue);
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
static bool ledIsHexCenter(uint16_t i);
static float circularDistance01(float a, float b);
static float hexRingPosition01(uint16_t i);

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

static float smoothstep01(float x) {
  x = clampFloat(x, 0.0f, 1.0f);
  return x * x * (3.0f - 2.0f * x);
}

static float normalizedCenterDistance(float xNorm, float yNorm) {
  float dx = xNorm - 0.5f;
  float dy = yNorm - 0.5f;
  float d = sqrtf((dx * dx) + (dy * dy));
  // Corner distance in normalized space is about 0.707.
  return clampFloat(d / 0.707f, 0.0f, 1.0f);
}

static unsigned long currentInGameRemainingMsForRender() {
  if (gameState != STATE_IN_GAME || matchStartMs == 0) return matchDurationMs;

  unsigned long elapsed = millis() - matchStartMs;
  if (elapsed >= matchDurationMs) return 0;
  return matchDurationMs - elapsed;
}

static void renderLastTenPulseFrame(unsigned long remainingMs, unsigned long now) {
  unsigned long elapsedInWindow = WARNING_PHASE_DURATION_MS - remainingMs;
  if (elapsedInWindow > WARNING_PHASE_DURATION_MS) elapsedInWindow = WARNING_PHASE_DURATION_MS;

  // v4.63 final-30 warning:
  // The field becomes a bright white animated warning surface while the center hex
  // runs a once-per-second rainbow shooting-star lap. This reads as urgent without
  // turning the whole arena into seizure soup.
  uint8_t pulseB = aggressiveOneSecondPulse(elapsedInWindow, lastTenPulsePeriodMs, lastTenMinBrightness, lastTenMaxBrightness);
  float urgencyNorm = 1.0f - clampFloat((float)remainingMs / (float)WARNING_PHASE_DURATION_MS, 0.0f, 1.0f);
  float dramaticScale = lastTenDramaticMode ? 1.0f : 0.62f;
  unsigned long secondPhaseMs = elapsedInWindow % 1000UL;
  float shootingHead = (float)secondPhaseMs / 1000.0f;
  float shootingTail = 0.20f;

  for (uint16_t i = 0; i < LED_COUNT; i++) {
    float xNorm = clampFloat(pixelX[i] / FIELD_WIDTH, 0.0f, 1.0f);
    float yNorm = clampFloat(pixelY[i] / FIELD_HEIGHT, 0.0f, 1.0f);
    float centerDist = normalizedCenterDistance(xNorm, yNorm);

    uint8_t shimmer = sin8((uint8_t)((now / 7UL) + (uint8_t)(xNorm * 95.0f) + (uint8_t)(yNorm * 130.0f)));
    uint8_t scan = sin8((uint8_t)((now / 5UL) + (uint8_t)(xNorm * 220.0f) - (uint8_t)(yNorm * 100.0f)));
    float ringPhase = (float)(secondPhaseMs) / 1000.0f;
    float ringRadius = 0.12f + (ringPhase * 0.78f);
    float ringWidth = 0.13f - (urgencyNorm * 0.04f);
    if (ringWidth < 0.06f) ringWidth = 0.06f;
    float ringBand = 1.0f - clampFloat(fabsf(centerDist - ringRadius) / ringWidth, 0.0f, 1.0f);
    ringBand = smoothstep01(ringBand);

    int whiteB = (int)pulseB;
    whiteB += map(shimmer, 0, 255, -18, 24);
    whiteB += map(scan, 0, 255, -10, 28);
    whiteB += (int)(ringBand * ((38.0f + urgencyNorm * 58.0f) * dramaticScale));
    whiteB += (int)((1.0f - centerDist) * (18.0f + urgencyNorm * 20.0f));

    // Keep a tiny team-color edge glow so the field still has identity under the white warning.
    CRGB fieldPixel = scaledColor(CRGB::White, clampU8(whiteB));
    if (!ledIsHexCenter(i)) {
      uint8_t edgeGlow = clampU8((int)(10.0f + urgencyNorm * 24.0f));
      if (xNorm < 0.11f) {
        fieldPixel += scaledColor(blueGoalColor, edgeGlow);
      } else if (xNorm > 0.89f) {
        fieldPixel += scaledColor(orangeGoalColor, edgeGlow);
      }
      leds[i] = fieldPixel;
      continue;
    }

    // Hex shooting star: one full lap every countdown second.
    float pos = hexRingPosition01(i);
    float d = circularDistance01(pos, shootingHead);
    float star = 0.0f;
    if (d < shootingTail) {
      star = 1.0f - (d / shootingTail);
      star = star * star;
    }

    float gradientPos = pos + (float)(elapsedInWindow % 7000UL) / 7000.0f;
    while (gradientPos >= 1.0f) gradientPos -= 1.0f;
    uint8_t hue = (uint8_t)(gradientPos * 255.0f);
    uint8_t starV = clampU8((int)(255.0f * star));
    // v4.65: final-30 hex rainbow stays at full pixel value and is shown through
    // safeFastLedShowAtMaxAllowedBrightness(), so it reaches the hard 50% demo cap.
    CRGB hexPixel = CHSV(hue, 255, 255);
    if (starV > 0) {
      hexPixel += CHSV((uint8_t)(hue + 52), 120, starV);
    }

    // Tiny white glint at the head of the shooting star. It is only a few LEDs, not a full-field flash.
    if (d < 0.030f) {
      hexPixel += scaledColor(CRGB::White, clampU8((int)(190.0f * (1.0f - (d / 0.030f)))));
    }

    uint8_t sparkleNoise = (uint8_t)(((uint32_t)i * 43UL + (elapsedInWindow / 21UL) * 29UL) & 0xFF);
    if (sparkleNoise > 250U) {
      hexPixel += CHSV((uint8_t)(hue + 32), 210, 140);
    }

    leds[i] = hexPixel;
  }
}

static void renderAttractOrInGame(bool inGame) {
  unsigned long now = millis();

  if (now - lastLedUpdateMs < LED_UPDATE_MS) return;
  lastLedUpdateMs = now;

  rainbowHueOffset += rainbowSpeed;

  if (inGame) {
    unsigned long remaining = currentInGameRemainingMsForRender();

    if (lastTenWarningEnabled && remaining > 0UL && remaining <= WARNING_PHASE_DURATION_MS) {
      renderLastTenPulseFrame(remaining, now);
      safeFastLedShowAtMaxAllowedBrightness();
      return;
    }

    // v4.35 in-game premium idle:
    // Team sides remain clear, but with layered arena movement, midfield energy,
    // and subtle center-hex activity so the field feels "broadcast live".
    float lateGame = 0.0f;
    if (remaining > 0UL && remaining <= 30000UL) {
      lateGame = 1.0f - clampFloat((float)remaining / 30000.0f, 0.0f, 1.0f);
    }

    for (uint16_t i = 0; i < LED_COUNT; i++) {
      float xNorm = clampFloat(pixelX[i] / FIELD_WIDTH, 0.0f, 1.0f);
      float yNorm = clampFloat(pixelY[i] / FIELD_HEIGHT, 0.0f, 1.0f);

      bool blueSide = pixelX[i] < virtualCenterX();
      CRGB baseColor = blueSide ? blueGoalColor : orangeGoalColor;

      uint8_t waveA = sin8((uint8_t)((now / 4UL) + (uint8_t)(xNorm * 155.0f) + (uint8_t)(yNorm * 76.0f)));
      uint8_t waveB = sin8((uint8_t)((now / 6UL) + (uint8_t)((1.0f - xNorm) * 98.0f) + (uint8_t)(yNorm * 124.0f)));
      uint8_t pulse = sin8((uint8_t)((now / 3UL) + (uint8_t)(xNorm * 140.0f)));
      uint8_t laneWave = sin8((uint8_t)((now / 5UL) + (uint8_t)(yNorm * 205.0f) + (uint8_t)(xNorm * 72.0f)));
      float centerDistNorm = normalizedCenterDistance(xNorm, yNorm);
      float sidePressure = blueSide ? (1.0f - xNorm) : xNorm;

      int brightness = map(waveA, 0, 255, inGameIdleMinBrightness, inGameIdleMaxBrightness);
      brightness += map(waveB, 0, 255, -16, 32);
      brightness += map(laneWave, 0, 255, -8, 18);
      brightness += (int)(sidePressure * 16.0f);

      // Slight midfield breathing so both center strips read as active but not distracting.
      float centerDistance = fabsf(pixelX[i] - virtualCenterX()) / 10.0f;
      if (centerDistance < 1.0f) {
        uint8_t centerBoost = (uint8_t)((1.0f - centerDistance) * 24.0f);
        brightness += centerBoost;
      }
      brightness += (int)((1.0f - centerDistNorm) * 10.0f);

      if (lateGame > 0.0f) {
        brightness += (int)((float)map(pulse, 0, 255, 0, 34) * lateGame);
      }

      uint8_t clampedB = clampU8(brightness);
      CRGB c = scaledColor(baseColor, clampedB);

      // Edge strobe energy ramps in during the final 30 seconds for a premium
      // "match pressure" look while staying fully team-colored.
      if (lateGame > 0.01f && (xNorm < 0.13f || xNorm > 0.87f || yNorm < 0.09f || yNorm > 0.91f)) {
        uint8_t edgeNoise = (uint8_t)(((uint32_t)i * 29UL + (now / 7UL) * 19UL) & 0xFF);
        if (edgeNoise > 238U) {
          uint8_t edgeBoost = clampU8((int)(45.0f + lateGame * 70.0f));
          c += scaledColor(baseColor, edgeBoost);
        }
      }

      if (ledIsHexCenter(i)) {
        uint8_t hexHue = (uint8_t)(rainbowHueOffset + (uint8_t)(xNorm * 70.0f) + (uint8_t)(yNorm * 110.0f));
        uint8_t hexB = map(waveB, 0, 255, 28, 88);
        hexB = clampU8((int)hexB + map(laneWave, 0, 255, -8, 22));
        c += CHSV(hexHue, 210, hexB);
      }

      leds[i] = c;
    }

    paintCenterStripsSplit(blueGoalColor, orangeGoalColor, inGameIdleMaxBrightness);
    safeFastLedShow();
    return;
  }

  // v4.35 attract mode:
  // Stadium-style gradient + sweeping energy layers with center emphasis.
  for (uint16_t i = 0; i < LED_COUNT; i++) {
    float xNorm = clampFloat(pixelX[i] / FIELD_WIDTH, 0.0f, 1.0f);
    float yNorm = clampFloat(pixelY[i] / FIELD_HEIGHT, 0.0f, 1.0f);

    uint8_t sweepA = sin8((uint8_t)((now / 5UL) + (uint8_t)(xNorm * 210.0f) - (uint8_t)(yNorm * 60.0f)));
    uint8_t sweepB = sin8((uint8_t)((now / 7UL) + (uint8_t)((1.0f - xNorm) * 124.0f) + (uint8_t)(yNorm * 90.0f)));
    uint8_t crowdWave = sin8((uint8_t)((now / 9UL) + (uint8_t)(xNorm * 74.0f) + (uint8_t)(yNorm * 164.0f)));
    float centerDist = normalizedCenterDistance(xNorm, yNorm);

    CRGB sideGradient = blend(blueGoalColor, orangeGoalColor, (uint8_t)(xNorm * 255.0f));
    int brightness = map(sweepA, 0, 255, 16, 88);
    brightness += map(sweepB, 0, 255, -8, 24);
    brightness += map(crowdWave, 0, 255, -5, 12);
    brightness += (int)((1.0f - centerDist) * 24.0f);

    CRGB pixel = scaledColor(sideGradient, clampU8(brightness));

    if (ledIsHexCenter(i)) {
      uint8_t hexHue = (uint8_t)(rainbowHueOffset + (uint8_t)(xNorm * 94.0f) + (uint8_t)(yNorm * 132.0f));
      uint8_t hexV = map(sweepB, 0, 255, 26, 78);
      hexV = clampU8((int)hexV + map(crowdWave, 0, 255, -3, 14));
      pixel += CHSV(hexHue, 230, hexV);
    } else {
      // Extremely small twinkle accents in attract mode keep the field from
      // feeling static when the demo is waiting for the next kickoff.
      uint8_t twinkle = (uint8_t)(((uint32_t)i * 17UL + (now / 13UL) * 11UL) & 0xFF);
      if (twinkle > 251U) {
        pixel += scaledColor(sideGradient, 24);
      }
    }

    leds[i] = pixel;
  }

  paintCenterStripsSplit(blueGoalColor, orangeGoalColor, 70);
  safeFastLedShow();
}

static void renderPreGameEffect() {
  unsigned long now = millis();

  if (now - lastLedUpdateMs < LED_UPDATE_MS) return;
  lastLedUpdateMs = now;

  rainbowHueOffset += max((uint8_t)1, (uint8_t)(rainbowSpeed / 2));
  uint8_t heartbeat = sin8((uint8_t)(now / 6UL));
  uint8_t pulse = map(heartbeat, 0, 255, 52, 128);
  float waveHead = (float)((now / 22UL) % 1000UL) / 1000.0f;
  float waveWidth = 0.20f;

  for (uint16_t i = 0; i < LED_COUNT; i++) {
    float xNorm = clampFloat(pixelX[i] / FIELD_WIDTH, 0.0f, 1.0f);
    float yNorm = clampFloat(pixelY[i] / FIELD_HEIGHT, 0.0f, 1.0f);

    bool blueSide = pixelX[i] < virtualCenterX();
    CRGB baseColor = blueSide ? blueGoalColor : orangeGoalColor;

    uint8_t sweep = sin8((uint8_t)((now / 8UL) + (uint8_t)(xNorm * 150.0f) - (uint8_t)(yNorm * 62.0f)));
    uint8_t lane = sin8((uint8_t)((now / 10UL) + (uint8_t)(yNorm * 180.0f)));
    int brightness = map(sweep, 0, 255, 24, 82);
    brightness += map(lane, 0, 255, -8, 14);

    float sideRamp = blueSide ? (1.0f - xNorm) : xNorm;
    brightness += (int)(sideRamp * 22.0f);

    float chargeDist = fabsf(xNorm - waveHead);
    if (chargeDist > 0.5f) chargeDist = 1.0f - chargeDist;
    float chargeBand = 1.0f - clampFloat(chargeDist / waveWidth, 0.0f, 1.0f);
    chargeBand = smoothstep01(chargeBand);
    brightness += (int)(chargeBand * 42.0f);

    CRGB pixel = scaledColor(baseColor, clampU8(brightness));

    if (ledIsHexCenter(i)) {
      uint8_t hexHue = (uint8_t)(rainbowHueOffset + (uint8_t)(xNorm * 90.0f) + (uint8_t)(yNorm * 110.0f));
      uint8_t hexValue = clampU8((int)pulse + map(sweep, 0, 255, -10, 20));
      pixel += CHSV(hexHue, 200, hexValue);
    }

    leds[i] = pixel;
  }

  paintCenterStripsSplit(blueGoalColor, orangeGoalColor, clampU8((int)(pulse * 0.60f)));
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

  unsigned long safeStepMs = countdownStepMs;
  if (safeStepMs < 1UL) safeStepMs = 1UL;

  // v4.64 countdown LED direction:
  // Full-field rainbow wash across every LED, including the center hex.
  // The center hex still gets the second-synced shooting star lap so the
  // physical field matches the iPad countdown punch without looking like a
  // flat red warning screen.
  float progress = clampFloat((float)elapsed / (float)totalMs, 0.0f, 1.0f);
  unsigned long stepElapsed = elapsed % safeStepMs;
  float stepPhase = (float)stepElapsed / (float)safeStepMs;

  float secondPunch = 1.0f - stepPhase;
  secondPunch = secondPunch * secondPunch;

  float waveRadius = clampFloat(stepPhase * 0.98f, 0.0f, 1.0f);
  float waveWidth = 0.13f;

  // One shooting-star orbit per countdown second.
  float cometHead = stepPhase;
  float cometTail = clampFloat((float)countdownHexTrailPercent / 100.0f, 0.10f, 0.42f);

  uint8_t fieldLow = countdownFieldMinBrightness;
  uint8_t fieldHigh = countdownFieldMaxBrightness;
  if (fieldHigh < fieldLow) {
    uint8_t tmp = fieldHigh;
    fieldHigh = fieldLow;
    fieldLow = tmp;
  }

  // Keep countdown visible even if someone tuned the slider too low during testing.
  if (fieldLow < 35) fieldLow = 35;
  if (fieldHigh < 170) fieldHigh = 170;

  uint8_t baseHue = (uint8_t)((elapsed / 15UL) & 0xFF);

  for (uint16_t i = 0; i < LED_COUNT; i++) {
    float xNorm = clampFloat(pixelX[i] / FIELD_WIDTH, 0.0f, 1.0f);
    float yNorm = clampFloat(pixelY[i] / FIELD_HEIGHT, 0.0f, 1.0f);
    float centerDist = normalizedCenterDistance(xNorm, yNorm);

    uint8_t washA = sin8((uint8_t)((elapsed / 6UL) + (uint8_t)(xNorm * 190.0f) - (uint8_t)(yNorm * 80.0f)));
    uint8_t washB = sin8((uint8_t)((elapsed / 9UL) + (uint8_t)(xNorm * 82.0f) + (uint8_t)(yNorm * 210.0f)));
    uint8_t ripple = sin8((uint8_t)((elapsed / 5UL) + (uint8_t)(centerDist * 255.0f)));

    float waveBand = 1.0f - clampFloat(fabsf(centerDist - waveRadius) / waveWidth, 0.0f, 1.0f);
    waveBand = smoothstep01(waveBand);

    uint8_t hue = (uint8_t)(baseHue + (uint8_t)(xNorm * 112.0f) + (uint8_t)(yNorm * 64.0f) + (uint8_t)(progress * 80.0f));
    int value = map(washA, 0, 255, fieldLow, fieldHigh);
    value += map(washB, 0, 255, -10, 24);
    value += map(ripple, 0, 255, -6, 16);
    value += (int)(secondPunch * 42.0f);
    value += (int)(waveBand * (48.0f + secondPunch * 42.0f));

    CRGB pixel = CHSV(hue, 245, clampU8(value));

    // Small field sparkles ride the expanding second pulse. These are colored,
    // not full-field white, so the effect stays energetic without being ugly.
    uint8_t sparkleNoise = (uint8_t)(((uint32_t)i * 37UL + (stepElapsed / 9UL) * 29UL) & 0xFF);
    if (!ledIsHexCenter(i) && waveBand > 0.30f && sparkleNoise > 248U) {
      pixel += CHSV((uint8_t)(hue + 38), 180, clampU8((int)(90.0f + secondPunch * 120.0f)));
    }

    if (ledIsHexCenter(i)) {
      float pos = hexRingPosition01(i);
      float d = circularDistance01(pos, cometHead);

      // Hex is also a rainbow wash, but with higher baseline brightness so it
      // reads as the focal point of the countdown.
      uint8_t hexHue = (uint8_t)(baseHue + (uint8_t)(pos * 255.0f) + (uint8_t)(progress * 110.0f));
      uint8_t hexBaseV = clampU8(max((int)countdownHexBrightness - 35, 185));
      CRGB hexPixel = CHSV(hexHue, 255, hexBaseV);

      if (d < cometTail) {
        float comet = 1.0f - (d / cometTail);
        comet = comet * comet;
        uint8_t cometB = clampU8((int)(255.0f * comet));
        hexPixel += CHSV((uint8_t)(hexHue + 48), 120, cometB);
      }

      // Tiny white head glint, only a few LEDs around the current second marker.
      if (d < 0.026f) {
        hexPixel += scaledColor(CRGB::White, clampU8((int)(230.0f * (1.0f - (d / 0.026f)))));
      }

      uint8_t hexSparkle = (uint8_t)(((uint32_t)i * 53UL + (stepElapsed / 13UL) * 31UL) & 0xFF);
      if (hexSparkle > 249U) {
        hexPixel += CHSV((uint8_t)(hexHue + 96), 170, 180);
      }

      pixel = hexPixel;
    }

    leds[i] = pixel;
  }
}
static void renderCountdown() {
  unsigned long now = millis();

  if (now - lastLedUpdateMs < LED_UPDATE_MS) return;
  lastLedUpdateMs = now;

  unsigned long elapsed = now - ledModeStartMs;
  unsigned long totalMs = getCountdownTotalMs();
  if (totalMs < 1UL) totalMs = 1UL;
  float progress = clampFloat((float)elapsed / (float)totalMs, 0.0f, 1.0f);

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
      if (progress > 0.45f) {
        uint8_t rush = sin8((uint8_t)((elapsed / 3UL) + i));
        int boosted = (int)hexB + (int)((float)map(rush, 0, 255, 0, 44) * progress);
        hexB = clampU8(boosted);
      }
      leds[i] = scaledColor(hexColor, hexB);
    } else {
      uint8_t fieldBrightness = countdownFieldBrightnessForLed(i, elapsed);
      if (progress > 0.55f) {
        float xNorm = clampFloat(pixelX[i] / FIELD_WIDTH, 0.0f, 1.0f);
        float yNorm = clampFloat(pixelY[i] / FIELD_HEIGHT, 0.0f, 1.0f);
        uint8_t rush = sin8((uint8_t)((elapsed / 4UL) + (uint8_t)(xNorm * 190.0f) - (uint8_t)(yNorm * 110.0f)));
        int boosted = (int)fieldBrightness + (int)((float)map(rush, 0, 255, -8, 24) * progress);
        fieldBrightness = clampU8(boosted);
      }
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
      setLedMode(LED_MODE_ON_IDLE);
    } else {
      setLedMode(LED_MODE_OFF_IDLE);
    }
    return;
  }

  uint8_t pulse = sin8((uint8_t)(elapsed / 3));
  uint8_t brightness = map(pulse, 0, 255, 130, 230);
  float ramp = clampFloat((float)elapsed / (float)GAME_START_GREEN_MS, 0.0f, 1.0f);
  float ringRadius = ramp * 0.96f;
  float ringWidth = 0.16f - (ramp * 0.06f);
  if (ringWidth < 0.06f) ringWidth = 0.06f;
  float sweepHead = ramp;

  for (uint16_t i = 0; i < LED_COUNT; i++) {
    float xNorm = clampFloat(pixelX[i] / FIELD_WIDTH, 0.0f, 1.0f);
    float yNorm = clampFloat(pixelY[i] / FIELD_HEIGHT, 0.0f, 1.0f);
    float centerDist = normalizedCenterDistance(xNorm, yNorm);
    float ringBand = 1.0f - clampFloat(fabsf(centerDist - ringRadius) / ringWidth, 0.0f, 1.0f);
    ringBand = smoothstep01(ringBand);

    float sweepDist = fabsf(xNorm - sweepHead);
    float sweepBand = 1.0f - clampFloat(sweepDist / 0.20f, 0.0f, 1.0f);

    int boosted = (int)brightness + (int)((1.0f - centerDist) * 34.0f);
    boosted += (int)(ringBand * 72.0f);
    boosted += (int)(sweepBand * 30.0f);
    leds[i] = scaledColor(gameStartColor, clampU8(boosted));
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
      setLedMode(LED_MODE_ON_IDLE);
      lastLedUpdateMs = 0;
      renderAttractOrInGame(true);
    } else {
      setLedMode(LED_MODE_OFF_IDLE);
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

    // Extra cinematic layer: a broader pressure ring that rides behind the main wave train.
    // This fills "empty" moments between wave heads without flattening the directional look.
    if (elapsed < goalWaveTravelMs + 320UL) {
      float pressurePos = ((float)elapsed / (float)(goalWaveTravelMs + 320UL)) * 1.06f;
      float pressureBand = 1.0f - clampFloat(fabsf(travelCoord - pressurePos) / 0.12f, 0.0f, 1.0f);
      pressureBand *= 0.42f;
      if (pressureBand > intensity) intensity = pressureBand;
    }

    // Stadium sparkle accents. Randomized by LED index and time so the effect looks alive.
    uint8_t sparkleNoise = (uint8_t)(((uint32_t)i * 41UL + (elapsed / 11UL) * 23UL) & 0xFF);
    if (sparkleNoise > 246 && elapsed < goalEffectMs - 260UL) {
      float sparkle = 0.42f + ((float)(sparkleNoise - 246U) / 9.0f) * 0.30f;
      if (sparkle > intensity) intensity = sparkle;
    }

    // Destination arc adds a graceful end-cap on the defending side so the
    // goal blast feels finished instead of abruptly stopping at the wall.
    if (pathX > 0.70f) {
      float verticalBand = 1.0f - clampFloat(fabsf(yNorm - 0.50f) / 0.34f, 0.0f, 1.0f);
      float destinationDepth = clampFloat((pathX - 0.70f) / 0.30f, 0.0f, 1.0f);
      float arrivalDenom = (float)goalWaveTravelMs * 0.65f;
      if (arrivalDenom < 1.0f) arrivalDenom = 1.0f;
      float arrival = clampFloat(((float)elapsed - ((float)goalWaveTravelMs * 0.45f)) / arrivalDenom, 0.0f, 1.0f);
      float arc = verticalBand * destinationDepth * arrival * 0.34f;
      if (arc > intensity) intensity = arc;
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
      setLedMode(LED_MODE_ON_IDLE);
    } else {
      setLedMode(LED_MODE_OFF_IDLE);
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

static unsigned long getMatchEndTotalEffectMs() {
  return GAME_OVER_WARNING_MS + matchEndEffectMs;
}

static void renderGameOverFieldWarning(unsigned long elapsed) {
  (void)elapsed;

  // Immediate, unmistakable stop signal: everything solid red.
  // No rainbow, no sweep, no pulse. This is the visual equivalent of blowing the whistle.
  fill_solid(leds, LED_COUNT, CRGB(255, 0, 0));
  safeFastLedShow();
}

static void renderMatchEndEffect() {
  unsigned long now = millis();

  if (now - lastLedUpdateMs < LED_UPDATE_MS) return;
  lastLedUpdateMs = now;

  unsigned long rawElapsed = now - ledModeStartMs;

  if (rawElapsed >= getMatchEndTotalEffectMs()) {
    setLedMode(LED_MODE_OFF_IDLE);
    return;
  }

  if (rawElapsed < GAME_OVER_WARNING_MS) {
    renderGameOverFieldWarning(rawElapsed);
    return;
  }

  unsigned long elapsed = rawElapsed - GAME_OVER_WARNING_MS;

  CRGB blue = blueGoalColor;
  CRGB orange = orangeGoalColor;

  bool blueWinner = blueScore > orangeScore;
  bool tied = blueScore == orangeScore;

  CRGB winnerColor = blueWinner ? blue : orange;

  // Match-end winner animation:
  // - Winner field: slower premium rings with heavier sparkle accents.
  // - Center hex: slow rainbow aurora with shooting-star sparkles.
  // - Tie: half blue / half orange sparkle field instead of a generic fallback.
  float speedScale = (float)matchEndWinnerSpeed / 220.0f;
  if (speedScale < 0.04f) speedScale = 0.04f;

  float ringSpacing = (float)matchEndRingSpacing;
  if (ringSpacing < 6.0f) ringSpacing = 6.0f;

  float finaleSpeedScale = 0.72f;
  float finaleEnergyScale = 1.06f;
  float finaleSparkleScale = 1.55f;
  if (finaleIntensityMode == 0) {
    // Clean: calmer rings, but still polished.
    finaleSpeedScale = 0.56f;
    finaleEnergyScale = 0.88f;
    finaleSparkleScale = 1.05f;
  } else if (finaleIntensityMode == 2) {
    // Maximum safe: more glitter, not more strobe.
    finaleSpeedScale = 0.88f;
    finaleEnergyScale = 1.18f;
    finaleSparkleScale = 2.15f;
  }

  uint8_t hexHueBase = (uint8_t)((elapsed * (unsigned long)max((uint8_t)4, matchEndRainbowSpeed)) / 2200UL);
  uint8_t tiePulse = sin8((uint8_t)(elapsed / 12));

  for (uint16_t i = 0; i < LED_COUNT; i++) {
    float x = pixelX[i];
    float y = pixelY[i];
    float xNorm = clampFloat(x / FIELD_WIDTH, 0.0f, 1.0f);
    float yNorm = clampFloat(y / FIELD_HEIGHT, 0.0f, 1.0f);

    if (ledIsHex(i)) {
      float pos = hexRingPosition01(i);

      if (tied) {
        // Tie finale: the hex itself respects the blue/orange field split.
        bool leftSide = x < virtualCenterX();
        CRGB tieSide = leftSide ? blue : orange;
        uint8_t breathe = sin8((uint8_t)((elapsed / 18UL) + (uint8_t)(pos * 180.0f)));
        uint8_t sideB = map(breathe, 0, 255, 95, 220);
        CRGB tieHex = scaledColor(tieSide, sideB);

        float cometHead = (float)((elapsed / 18UL) % 1000UL) / 1000.0f;
        float d = circularDistance01(pos, cometHead);
        if (d < 0.16f) {
          float comet = 1.0f - (d / 0.16f);
          tieHex += scaledColor(tieSide, clampU8((int)(180.0f * comet * comet)));
        }

        uint8_t spark = (uint8_t)(((uint32_t)i * 71UL + (elapsed / 29UL) * 37UL) & 0xFF);
        if (spark > 245U) {
          tieHex += scaledColor(CRGB::White, map(spark, 246, 255, 75, 190));
        }
        leds[i] = tieHex;
        continue;
      }

      // Slow premium rainbow aurora through the hex, with one elegant shooting-star pass.
      float aurora = pos + ((float)(elapsed % 12000UL) / 12000.0f);
      while (aurora >= 1.0f) aurora -= 1.0f;
      uint8_t hue = (uint8_t)(hexHueBase + (uint8_t)(aurora * 255.0f));
      uint8_t valueWave = sin8((uint8_t)((elapsed / 24UL) + (uint8_t)(pos * 210.0f)));
      uint8_t value = map(valueWave, 0, 255, 125, 230);
      CRGB hexPixel = CHSV(hue, 245, value);

      float starHead = (float)((elapsed / 30UL) % 1000UL) / 1000.0f;
      float d = circularDistance01(pos, starHead);
      if (d < 0.18f) {
        float comet = 1.0f - (d / 0.18f);
        comet = comet * comet;
        hexPixel += CHSV((uint8_t)(hue + 26), 180, clampU8((int)(210.0f * comet)));
      }
      if (d < 0.028f) {
        hexPixel += scaledColor(CRGB::White, clampU8((int)(205.0f * (1.0f - d / 0.028f))));
      }

      uint8_t sparkleNoise = (uint8_t)(((uint32_t)i * 67UL + (elapsed / 31UL) * 41UL) & 0xFF);
      if (sparkleNoise > 246U) {
        hexPixel += scaledColor(CRGB::White, map(sparkleNoise, 247, 255, 60, 190));
      }

      leds[i] = hexPixel;
      continue;
    }

    if (tied) {
      bool leftSide = x < virtualCenterX();
      CRGB sideColor = leftSide ? blue : orange;
      uint8_t b = map(tiePulse, 0, 255, 80, 220);
      uint8_t shimmer = sin8((uint8_t)(elapsed / 3 + i * 13));
      uint8_t sweep = sin8((uint8_t)(elapsed / 5 + (uint8_t)(xNorm * 170.0f) - (uint8_t)(yNorm * 74.0f)));
      int tieB = (int)b + (int)((float)map(shimmer, 0, 255, -10, 28) * finaleEnergyScale) + (int)((float)map(sweep, 0, 255, -8, 18) * finaleEnergyScale);
      CRGB tiePixel = scaledColor(sideColor, clampU8(tieB));

      float centerDist = normalizedCenterDistance(xNorm, yNorm);
      if (centerDist < 0.30f) {
        uint8_t neutral = (uint8_t)((1.0f - (centerDist / 0.30f)) * 62.0f);
        tiePixel += scaledColor(CRGB(120, 120, 150), neutral);
      }

      // Thin midfield crown on tie screen for a more intentional finale look.
      float midBandX = 1.0f - clampFloat(fabsf(xNorm - 0.50f) / 0.055f, 0.0f, 1.0f);
      float midBandY = 1.0f - clampFloat(fabsf(yNorm - 0.50f) / 0.28f, 0.0f, 1.0f);
      float midBand = midBandX * midBandY;
      if (midBand > 0.0f) {
        uint8_t midBoost = clampU8((int)(midBand * 85.0f));
        tiePixel += scaledColor(CRGB(145, 145, 185), midBoost);
      }

      // Tie sparkle accents: alternate blue/orange confetti so tie finales
      // still feel celebratory instead of looking like a fallback state.
      uint8_t tieSparkle = (uint8_t)(((uint32_t)i * 59UL + (elapsed / 16UL) * 27UL) & 0xFF);
      uint8_t tieThreshold = (finaleSparkleScale < 1.2f) ? 248 : (finaleSparkleScale > 1.9f ? 239 : 244);
      if (tieSparkle > tieThreshold) {
        CRGB tieSparkColor = leftSide ? blue : orange;
        int tieSparkB = map(tieSparkle, tieThreshold + 1, 255, (int)(95.0f * finaleSparkleScale), (int)(220.0f * finaleSparkleScale));
        tiePixel += scaledColor(tieSparkColor, clampU8(tieSparkB));
        if (tieSparkle > 252U) {
          tiePixel += scaledColor(CRGB::White, 150);
        }
      }

      leds[i] = tiePixel;
      continue;
    }

    // Winner rings. Origin is on the winner's side, so the field feels like
    // it is being claimed by the winner instead of just blinking randomly.
    float originX = blueWinner ? 0.0f : FIELD_WIDTH;
    float originY = FIELD_HEIGHT / 2.0f;
    float dx = x - originX;
    float dy = y - originY;
    float d = sqrtf((dx * dx) + (dy * dy));

    float moving = ((float)elapsed * speedScale * finaleSpeedScale) / 38.0f;
    float ring = fmodf((d - moving), ringSpacing);
    if (ring < 0.0f) ring += ringSpacing;

    float ringDistance = (ring < (ringSpacing - ring)) ? ring : (ringSpacing - ring);
    float ringStrength = 1.0f - clampFloat(ringDistance / (ringSpacing * 0.34f), 0.0f, 1.0f);
    ringStrength = ringStrength * ringStrength;

    // Winner side gets more base energy. Opposite side still lights, but less.
    float fieldBias = blueWinner ? (1.0f - xNorm) : xNorm;
    float base = 0.13f + (fieldBias * 0.28f);
    float sparkle = (sin8((uint8_t)(elapsed / 3 + i * 7)) / 255.0f) * 0.13f;
    float intensity = clampFloat(base + ((ringStrength * 0.78f) * finaleEnergyScale) + (sparkle * finaleEnergyScale), 0.0f, 1.0f);

    CRGB winnerPixel = scaledColor(winnerColor, clampU8((int)(intensity * 255.0f)));

    // Winner crown emphasizes the claimed attacking corridor.
    float crownLaneY = 1.0f - clampFloat(fabsf(yNorm - 0.50f) / 0.19f, 0.0f, 1.0f);
    float crownLaneX = blueWinner ? (1.0f - xNorm) : xNorm;
    float crownMix = crownLaneY * clampFloat(crownLaneX, 0.0f, 1.0f);
    if (crownMix > 0.0f) {
      uint8_t crownBoost = clampU8((int)(crownMix * 52.0f));
      winnerPixel += scaledColor(winnerColor, crownBoost);
    }

    // Finale confetti sparkle: more dramatic, but sparse enough to stay premium.
    uint8_t confetti = (uint8_t)(((uint32_t)i * 53UL + (elapsed / 15UL) * 31UL) & 0xFF);
    uint8_t confettiThreshold = (finaleSparkleScale < 1.2f) ? 248 : (finaleSparkleScale > 1.9f ? 238 : 243);
    if (confetti > confettiThreshold) {
      int confettiB = map(confetti, confettiThreshold + 1, 255, (int)(95.0f * finaleSparkleScale), (int)(230.0f * finaleSparkleScale));
      winnerPixel += scaledColor(winnerColor, clampU8(confettiB));
      if (confetti > 252U) {
        winnerPixel += scaledColor(CRGB::White, 145);
      }
    }

    leds[i] = winnerPixel;
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

  if (ledMode == LED_MODE_OFF_IDLE) {
    renderAttractOrInGame(false);
  } else if (ledMode == LED_MODE_PRE_GAME) {
    renderPreGameEffect();
  } else if (ledMode == LED_MODE_ON_IDLE) {
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
  } else if (ledMode == LED_MODE_LAST_TEN) {
    unsigned long elapsed = (millis() - ledModeStartMs) % WARNING_PHASE_DURATION_MS;
    renderLastTenPulseFrame(WARNING_PHASE_DURATION_MS - elapsed, millis());
    safeFastLedShowAtMaxAllowedBrightness();
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
  unsigned long startMs = (unsigned long)currentTrackStartSec * 1000UL;
  unsigned long endMs = (unsigned long)currentTrackEndSec * 1000UL;
  if (endMs <= startMs) return 1000UL;
  return endMs - startMs;
}

static unsigned long getAudioElapsedMs() {
  if (!trackActive) return 0;
  return millis() - currentTrackStartTime;
}

static unsigned long getAudioRemainingMs() {
  if (!trackActive) return 0;

  unsigned long elapsedMs = getAudioElapsedMs();
  unsigned long startMs = (unsigned long)currentTrackStartSec * 1000UL;
  unsigned long endMs = (unsigned long)currentTrackEndSec * 1000UL;
  if (endMs <= startMs) return 0;

  if (elapsedMs <= startMs) return endMs - startMs;
  if (elapsedMs >= endMs) return 0;
  return endMs - elapsedMs;
}

static int getAudioProgressPercent() {
  if (!trackActive) return 0;

  unsigned long elapsedMs = getAudioElapsedMs();
  unsigned long startMs = (unsigned long)currentTrackStartSec * 1000UL;
  unsigned long endMs = (unsigned long)currentTrackEndSec * 1000UL;
  if (endMs <= startMs) return 0;

  unsigned long windowMs = endMs - startMs;
  if (elapsedMs <= startMs) return 0;
  if (elapsedMs >= endMs) return 100;

  return (int)(((elapsedMs - startMs) * 100UL) / windowMs);
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

static void audioSetModuleVolumePercent(uint8_t percent, bool persistAsUserVolume) {
  if (percent > 100) percent = 100;

  uint8_t moduleVolume = map(percent, 0, 100, 0, 30);

  uint8_t data[1];
  data[0] = moduleVolume;

  sendAudioCommand(DY_CMD_SET_VOLUME, data, 1);

  if (persistAsUserVolume) {
    currentVolumePercent = percent;
  }
  currentModuleVolume = moduleVolume;
}

static void audioSetVolumePercent(uint8_t percent) {
  audioSetModuleVolumePercent(percent, true);
  if (audioMutedForStartOffset) {
    audioSetModuleVolumePercent(0, false);
  }

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

  normalizeAudioWindowTrack(track);

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
  currentTrackStartSec = trackWindowStartSec[track];
  currentTrackEndSec = trackWindowEndSec[track];
  trackActive = true;
  audioMutedForStartOffset = false;

  // DY-HV20T seek-to-time is not reliable in this project path.
  // Start offsets are implemented as silent pre-roll:
  // play from 0s muted, then restore volume at startSec.
  if (currentTrackStartSec > 0) {
    audioSetModuleVolumePercent(0, false);
    audioMutedForStartOffset = true;
    Serial.print("[AUDIO] Pre-roll mute active until ");
    Serial.print(currentTrackStartSec);
    Serial.println("s");
  } else {
    audioSetModuleVolumePercent(currentVolumePercent, false);
  }

  audioCommandCount++;
}

static void audioStopCommon() {
  Serial.println("[AUDIO] Stop command.");
  sendAudioCommand(DY_CMD_STOP, nullptr, 0);

  trackActive = false;
  currentTrack = 0;
  currentTrackStartSec = 0;
  currentTrackEndSec = 0;
  audioMutedForStartOffset = false;
  audioSetModuleVolumePercent(currentVolumePercent, false);
  audioShowState = AUDIO_SHOW_STOPPED;
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
  currentTrackStartSec = 0;
  currentTrackEndSec = 0;
  audioMutedForStartOffset = false;
  audioSetModuleVolumePercent(currentVolumePercent, false);
  audioShowState = AUDIO_SHOW_STOPPED;
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
  audioShowState = AUDIO_SHOW_MANUAL_TEST;
  audioPlayTrackInterrupting(track, "manual-forced");
}

// Countdown-specific helper.
// Countdown requirement:
//   00007.mp3 should be a complete 10-second countdown sequence with a GO tone.
//   Firmware plays it once at countdown start.
//   Do not replay it once per number or the audio becomes messy.
static void audioPlayCountdownSequenceOnce() {
  if (countdownSequenceStarted) {
    return;
  }

  Serial.println("[AUDIO] Countdown sequence track 7 started once");
  audioShowState = AUDIO_SHOW_COUNTDOWN;
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

static void handleAudioWindowCompleteForFlow() {
  // Keep the high-level show state aligned with the current match flow after
  // a windowed track completes. Track timing windows are the source of truth;
  // BUSY remains telemetry-only for diagnostics.
  if (audioShowState == AUDIO_SHOW_MANUAL_TEST) {
    audioShowState = AUDIO_SHOW_STOPPED;
    return;
  }

  if (gameState == STATE_IN_GAME) {
    audioShowState = AUDIO_SHOW_GAMEPLAY;
    return;
  }

  if (gameState == STATE_COUNTDOWN) {
    audioShowState = AUDIO_SHOW_COUNTDOWN;
    return;
  }

  if (gameState == STATE_PRE_GAME) {
    audioShowState = AUDIO_SHOW_BUILDUP;
    return;
  }

  if (gameState == STATE_GOAL_PAUSE) {
    audioShowState = AUDIO_SHOW_GOAL_ONE_SHOT;
    return;
  }

  if (gameState == STATE_END_GAME) {
    audioShowState = AUDIO_SHOW_WINNER;
    return;
  }

  if (gameState == STATE_STANDBY && ledMode == LED_MODE_OFF_IDLE) {
    audioShowState = AUDIO_SHOW_DOWNTIME;
    return;
  }

  audioShowState = AUDIO_SHOW_STOPPED;
}

static void updateAudioState() {
  if (!trackActive) return;

  unsigned long elapsed = getAudioElapsedMs();
  unsigned long startMs = (unsigned long)currentTrackStartSec * 1000UL;
  unsigned long endMs = (unsigned long)currentTrackEndSec * 1000UL;

  if (audioMutedForStartOffset && elapsed >= startMs) {
    audioSetModuleVolumePercent(currentVolumePercent, false);
    audioMutedForStartOffset = false;
    Serial.println("[AUDIO] Pre-roll complete. Restored normal volume.");
  }

  if (endMs <= startMs) {
    endMs = startMs + 1000UL;
  }

  if (endMs > startMs && elapsed >= endMs) {
    Serial.println("[AUDIO] Track window end reached. Sending STOP.");
    sendAudioCommand(DY_CMD_STOP, nullptr, 0);
    trackActive = false;
    currentTrack = 0;
    currentTrackStartSec = 0;
    currentTrackEndSec = 0;
    audioMutedForStartOffset = false;
    audioSetModuleVolumePercent(currentVolumePercent, false);
    handleAudioWindowCompleteForFlow();
    return;
  }
}

static void serviceGameplayBackgroundAudio() {
  if (gameState != STATE_IN_GAME) {
    return;
  }

  unsigned long now = millis();
  if (now < bgmAllowedAfterMs) {
    return;
  }

  if (trackActive && currentTrack == AUDIO_TRACK_GAMEPLAY_BGM) {
    return;
  }

  if (trackActive && currentTrack != 0) {
    return;
  }

  Serial.println("[AUDIO] Starting gameplay BGM track 8");
  audioShowState = AUDIO_SHOW_GAMEPLAY;
  audioPlayTrack(AUDIO_TRACK_GAMEPLAY_BGM);
  lastRequestedBgmTrack = AUDIO_TRACK_GAMEPLAY_BGM;
}

static void servicePreGameAudio() {
  if (gameState != STATE_PRE_GAME) {
    return;
  }

  if (effectTestMode) {
    return;
  }

  if (trackActive && currentTrack == AUDIO_TRACK_BUILDUP) {
    return;
  }

  if (trackActive && currentTrack != 0) {
    return;
  }

  Serial.println("[AUDIO] Starting pre-game buildup track 6");
  audioShowState = AUDIO_SHOW_BUILDUP;
  audioPlayTrack(AUDIO_TRACK_BUILDUP);
}

static uint16_t pickDowntimeIdleTrack() {
  if (AUDIO_TRACK_IDLE_MAX <= AUDIO_TRACK_IDLE_MIN) return AUDIO_TRACK_IDLE_MIN;
  uint16_t pick = (uint16_t)random(AUDIO_TRACK_IDLE_MIN, AUDIO_TRACK_IDLE_MAX + 1);
  if (pick == lastIdleDowntimeTrack) {
    pick++;
    if (pick > AUDIO_TRACK_IDLE_MAX) pick = AUDIO_TRACK_IDLE_MIN;
  }
  return pick;
}

static void serviceStandbyIdleAudio() {
  if (gameState != STATE_STANDBY) {
    return;
  }

  if (ledMode != LED_MODE_OFF_IDLE) {
    return;
  }

  if (effectTestMode) {
    return;
  }

  bool inIdleRange = trackActive &&
                     currentTrack >= AUDIO_TRACK_IDLE_MIN &&
                     currentTrack <= AUDIO_TRACK_IDLE_MAX;
  if (inIdleRange) {
    return;
  }

  if (trackActive && currentTrack != 0) {
    return;
  }

  uint16_t idleTrack = pickDowntimeIdleTrack();
  lastIdleDowntimeTrack = idleTrack;

  Serial.print("[AUDIO] Starting downtime idle track ");
  Serial.println(idleTrack);
  audioShowState = AUDIO_SHOW_DOWNTIME;
  audioPlayTrack(idleTrack);
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

  if (gameState == STATE_PRE_GAME) {
    return matchDurationMs;
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

static bool isLastTenSecondsActive() {
  if (!lastTenWarningEnabled) return false;
  if (gameState != STATE_IN_GAME) return false;
  unsigned long remaining = getMatchRemainingMs();
  return (remaining > 0UL && remaining <= WARNING_PHASE_DURATION_MS);
}

static LedMode desiredLedModeForGameState() {
  unsigned long now = millis();

  // Preserve explicit hard overrides and test modes.
  if (ledMode == LED_MODE_BLACKOUT || ledMode == LED_MODE_LED_SELF_TEST || ledMode == LED_MODE_CENTER_TEST) {
    return ledMode;
  }

  // Preserve short transitional effects while they are active.
  if (ledMode == LED_MODE_HIT && (now - ledModeStartMs) < HIT_EFFECT_MS) return LED_MODE_HIT;
  if (ledMode == LED_MODE_BIG_HIT && (now - ledModeStartMs) < BIG_HIT_EFFECT_MS) return LED_MODE_BIG_HIT;
  if (ledMode == LED_MODE_GAME_START_GREEN && gameState == STATE_IN_GAME && (now - ledModeStartMs) < GAME_START_GREEN_MS) {
    return LED_MODE_GAME_START_GREEN;
  }

  switch (gameState) {
    case STATE_STANDBY:
      return LED_MODE_OFF_IDLE;

    case STATE_PRE_GAME:
      return LED_MODE_PRE_GAME;

    case STATE_COUNTDOWN:
      return LED_MODE_COUNTDOWN;

    case STATE_IN_GAME:
      return isLastTenSecondsActive() ? LED_MODE_LAST_TEN : LED_MODE_ON_IDLE;

    case STATE_GOAL_PAUSE:
      if (goalPauseStartMs > 0 && (now - goalPauseStartMs) <= goalEffectMs) {
        if (lastGoalTeam == GOAL_TEAM_ORANGE) return LED_MODE_ORANGE_GOAL;
        if (lastGoalTeam == GOAL_TEAM_BLUE) return LED_MODE_BLUE_GOAL;
      }
      return LED_MODE_ON_IDLE;

    case STATE_HALFTIME:
      return LED_MODE_OFF_IDLE;

    case STATE_END_GAME:
      if (matchEndMs > 0 && (now - matchEndMs) <= getMatchEndTotalEffectMs()) {
        return LED_MODE_MATCH_END;
      }
      return LED_MODE_OFF_IDLE;
  }

  return LED_MODE_OFF_IDLE;
}

static void applyAutomaticLedModeForState(bool force = false) {
  if (effectTestMode) return;
  LedMode desired = desiredLedModeForGameState();
  if (force || ledMode != desired) {
    setLedMode(desired);
  }
}

static bool ledModeUsesAutomaticStateMapping() {
  return ledMode == LED_MODE_OFF_IDLE ||
         ledMode == LED_MODE_PRE_GAME ||
         ledMode == LED_MODE_ON_IDLE ||
         ledMode == LED_MODE_COUNTDOWN ||
         ledMode == LED_MODE_LAST_TEN ||
         ledMode == LED_MODE_GAME_START_GREEN;
}

static int getCountdownValue() {
  if (gameState != STATE_COUNTDOWN) return 0;
  if (countdownStartMs == 0) return (int)COUNTDOWN_SECONDS;

  unsigned long elapsed = millis() - countdownStartMs;
  unsigned long elapsedSteps = elapsed / countdownStepMs;

  if (elapsedSteps >= COUNTDOWN_SECONDS) return 0;

  return (int)(COUNTDOWN_SECONDS - elapsedSteps);
}

static void enterPreGameMode() {
  if (gameState == STATE_IN_GAME || gameState == STATE_GOAL_PAUSE || gameState == STATE_END_GAME) {
    addEvent("Pre-game ignored during active or final match state");
    return;
  }

  gameState = STATE_PRE_GAME;
  matchStartMs = 0;
  countdownStartMs = 0;
  pausedMatchRemainingMs = matchDurationMs;
  goalPauseStartMs = 0;
  lastCountdownAnnounced = -1;
  lastCountdownBeepMs = 0;
  countdownSequenceStarted = false;
  audienceOverrideMode = AUDIENCE_OVERRIDE_AUTO;
  postMatchStateStartMs = 0;
  lastGoalTeam = GOAL_TEAM_NONE;
  matchEndMs = 0;

  audioShowState = AUDIO_SHOW_BUILDUP;
  audioPlayTrackInterrupting(AUDIO_TRACK_BUILDUP, "pre-game");
  bgmAllowedAfterMs = 0;
  lastRequestedBgmTrack = 0;

  applyAutomaticLedModeForState(true);
  addEvent("Pre-game active: buildup track + standby intro");
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
  audienceOverrideMode = AUDIENCE_OVERRIDE_AUTO;
  postMatchStateStartMs = 0;
  lastGoalTeam = GOAL_TEAM_NONE;
  matchStartMs = 0;
  matchEndMs = 0;

  gameState = STATE_COUNTDOWN;
  countdownStartMs = millis();
  lastCountdownAnnounced = -1;
  lastCountdownBeepMs = 0;
  countdownSequenceStarted = false;

  // Countdown is now its own stage. Track 00006 belongs to STATE_PRE_GAME only.
  // Starting a match always hard-stops pre-game buildup first.
  audioStopHard("start-match");
  bgmAllowedAfterMs = 0;
  lastRequestedBgmTrack = 0;
  audioShowState = AUDIO_SHOW_COUNTDOWN;

  applyAutomaticLedModeForState(true);
  addEvent(matchFormatMode == MATCH_FORMAT_TWO_HALF ? "First half countdown started" : "Match countdown started");
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

  gameState = STATE_STANDBY;
  audienceOverrideMode = AUDIENCE_OVERRIDE_AUTO;
  applyAutomaticLedModeForState(true);
  audioStopHard("halftime");
  bgmAllowedAfterMs = 0;
  lastRequestedBgmTrack = 0;

  addEvent("First half complete. Ready to resume second half.");
}

static void startSecondHalf() {
  if (matchFormatMode != MATCH_FORMAT_TWO_HALF) {
    addEvent("Resume second half ignored: Single Match format is active");
    return;
  }

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
  audienceOverrideMode = AUDIENCE_OVERRIDE_AUTO;
  lastGoalTeam = GOAL_TEAM_NONE;

  applyAutomaticLedModeForState(true);
  audioStopHard("second-half-countdown");
  bgmAllowedAfterMs = 0;
  lastRequestedBgmTrack = 0;
  audioShowState = AUDIO_SHOW_COUNTDOWN;

  addEvent("Second half countdown started");
}

static void endMatch(bool manualEnd) {
  if (gameState == STATE_END_GAME) return;

  gameState = STATE_END_GAME;
  matchEndMs = millis();
  postMatchStateStartMs = matchEndMs;
  secondHalfComplete = (matchFormatMode == MATCH_FORMAT_TWO_HALF) ? (currentHalf == 2) : true;

  applyAutomaticLedModeForState(true);
  audioShowState = AUDIO_SHOW_WINNER;
  audioPlayTrackInterrupting(AUDIO_TRACK_WINNER, "match-end");
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
  audienceOverrideMode = AUDIENCE_OVERRIDE_AUTO;
  postMatchStateStartMs = 0;
  lastGoalTeam = GOAL_TEAM_NONE;
  matchEndMs = 0;

  applyAutomaticLedModeForState(true);
  audioStopHard("reset");
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
  audienceOverrideMode = AUDIENCE_OVERRIDE_AUTO;
  postMatchStateStartMs = 0;
  lastGoalTeam = GOAL_TEAM_NONE;
  matchEndMs = 0;

  applyAutomaticLedModeForState(true);

  // Stop any gameplay/goal/winner/manual audio.
  // Downtime idle rotation (tracks 00001-00005) will resume from serviceStandbyIdleAudio().
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

  applyAutomaticLedModeForState(true);

  // Resume directly back to gameplay BGM.
  audioStopHard("resume-after-goal");
  bgmAllowedAfterMs = millis() + 200UL;
  lastRequestedBgmTrack = 0;
  audioShowState = AUDIO_SHOW_GAMEPLAY;

  addEvent("Gameplay resumed after goal reset");
}

static void pauseMatchForGoal() {
  pausedMatchRemainingMs = getMatchRemainingMs();

  if (pausedMatchRemainingMs == 0 || pausedMatchRemainingMs > matchDurationMs) {
    pausedMatchRemainingMs = matchDurationMs;
  }

  goalPauseStartMs = millis();
  gameState = STATE_GOAL_PAUSE;
  applyAutomaticLedModeForState(true);
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
  lastGoalTeam = GOAL_TEAM_BLUE;
  pauseMatchForGoal();
  audienceOverrideMode = AUDIENCE_OVERRIDE_AUTO;

  audioShowState = AUDIO_SHOW_GOAL_ONE_SHOT;
  audioPlayTrackInterrupting(AUDIO_TRACK_GOAL, "goal");

  if (autoResumeAfterGoalEnabled) {
    String msg = fake ? "Fake Blue goal scored. Auto resume armed for " : "Blue goal scored. Auto resume armed for ";
    msg += String(autoResumeAfterGoalDelayMs / 1000.0f, 1);
    msg += "s";
    addEvent(msg);
  } else {
    String msg = fake ? "Fake " : "";
    msg += activeBlueTeamName();
    msg += " scored. Timer paused for ball reset.";
    addEvent(msg);
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
  lastGoalTeam = GOAL_TEAM_ORANGE;
  pauseMatchForGoal();
  audienceOverrideMode = AUDIENCE_OVERRIDE_AUTO;

  audioShowState = AUDIO_SHOW_GOAL_ONE_SHOT;
  audioPlayTrackInterrupting(AUDIO_TRACK_GOAL, "goal");

  if (autoResumeAfterGoalEnabled) {
    String msg = fake ? "Fake Orange goal scored. Auto resume armed for " : "Orange goal scored. Auto resume armed for ";
    msg += String(autoResumeAfterGoalDelayMs / 1000.0f, 1);
    msg += "s";
    addEvent(msg);
  } else {
    String msg = fake ? "Fake " : "";
    msg += activeOrangeTeamName();
    msg += " scored. Timer paused for ball reset.";
    addEvent(msg);
  }
}

static void adjustOperatorScore(bool blueTeam, int delta) {
  if (delta == 0) {
    return;
  }

  uint16_t before = blueTeam ? blueScore : orangeScore;
  long next = (long)before + (long)delta;
  if (next < 0L) next = 0L;
  if (next > 999L) next = 999L;

  uint16_t after = (uint16_t)next;
  if (after == before) {
    return;
  }

  if (blueTeam) {
    blueScore = after;
  } else {
    orangeScore = after;
  }

  String msg = "Manual score correction: ";
  msg += blueTeam ? "Blue " : "Orange ";
  msg += (delta > 0) ? "+" : "-";
  msg += String(abs(delta));
  msg += " -> ";
  msg += String(blueScore);
  msg += "-";
  msg += String(orangeScore);
  addEvent(msg);
}

static int parseScoreDeltaArg() {
  int delta = 1;

  if (server.hasArg("delta")) {
    delta = server.arg("delta").toInt();
  }

  if (delta > 9) delta = 9;
  if (delta < -9) delta = -9;
  if (delta == 0) delta = 1;

  return delta;
}

static void handleScoreBlue() {
  adjustOperatorScore(true, parseScoreDeltaArg());
  server.send(200, "application/json", stateJson());
}

static void handleScoreOrange() {
  adjustOperatorScore(false, parseScoreDeltaArg());
  server.send(200, "application/json", stateJson());
}

static void replayFinale() {
  if (gameState == STATE_IN_GAME || gameState == STATE_GOAL_PAUSE || gameState == STATE_COUNTDOWN) {
    addEvent("Replay finale ignored during active match");
    return;
  }

  gameState = STATE_END_GAME;
  matchEndMs = millis();
  postMatchStateStartMs = matchEndMs;
  audienceOverrideMode = AUDIENCE_OVERRIDE_POST_MATCH;
  applyAutomaticLedModeForState(true);
  audioShowState = AUDIO_SHOW_WINNER;
  audioPlayTrackInterrupting(AUDIO_TRACK_WINNER, "replay-finale");

  addEvent("Finale replay triggered");
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

  if (rehearsalSceneActive && rehearsalSceneMode != REHEARSAL_SCENE_READABILITY) {
    if (now - rehearsalSceneStartMs >= rehearsalSceneDurationMs) {
      rehearsalSceneActive = false;
      rehearsalSceneMode = REHEARSAL_SCENE_NONE;
    }
  }

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

    // 00007.mp3 contains all countdown audio now:
    // 10-second countdown plus the GO tone. Start it once and let the file play.
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
      if (matchFormatMode == MATCH_FORMAT_TWO_HALF && currentHalf == 1) {
        enterHalftime();
      } else {
        endMatch(false);
      }
    }
  }

  if (gameState == STATE_END_GAME && autoReturnToStandbyEnabled && postMatchStateStartMs > 0) {
    unsigned long autoDelay = effectivePostMatchAutoReturnDelayMs();
    unsigned long elapsedPost = now - postMatchStateStartMs;
    if (elapsedPost >= autoDelay) {
      addEvent("Auto return to standby after match complete");
      enterStandbyIdleMode();
      return;
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

#if BALL_LINK_ENABLED
static bool ballMacEqual(const uint8_t* a, const uint8_t* b) {
  return memcmp(a, b, 6) == 0;
}

static String ballMacToString(const uint8_t mac[6]) {
  char buf[18];
  snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(buf);
}

static uint8_t ballLinkCurrentChannel() {
  uint8_t ch = WiFi.channel();
  if (ch == 0) ch = 1;
  return ch;
}

static int ballFindPeerIndex(const uint8_t mac[6]) {
  for (int i = 0; i < BALL_LINK_MAX_PEERS; i++) {
    if (ballPeers[i].used && ballMacEqual(ballPeers[i].mac, mac)) {
      return i;
    }
  }
  return -1;
}

static int ballAllocPeerIndex(const uint8_t mac[6]) {
  for (int i = 0; i < BALL_LINK_MAX_PEERS; i++) {
    if (!ballPeers[i].used) {
      memset(&ballPeers[i], 0, sizeof(ballPeers[i]));
      ballPeers[i].used = true;
      memcpy(ballPeers[i].mac, mac, 6);
      ballPeers[i].channel = ballLinkCurrentChannel();
      return i;
    }
  }
  return -1;
}

static void ballEnsureEspNowPeer(const uint8_t mac[6], uint8_t channel) {
  if (esp_now_is_peer_exist(mac)) {
    esp_now_del_peer(mac);
  }
  esp_now_peer_info_t p{};
  memcpy(p.peer_addr, mac, 6);
  p.channel = channel;
  p.encrypt = false;
  esp_now_add_peer(&p);
}

static bool ballRxQueuePush(const uint8_t* src, const uint8_t* data, int len) {
  if (!src || !data || len <= 0 || len > 32) return false;
  bool ok = false;
  portENTER_CRITICAL(&ballRxMux);
  uint8_t next = (uint8_t)(ballRxHead + 1);
  if (next >= BALL_LINK_RX_QUEUE_CAP) next = 0;
  if (next != ballRxTail) {
    BallRxFrame& f = ballRxQueue[ballRxHead];
    memcpy(f.src, src, 6);
    f.len = (uint8_t)len;
    memcpy(f.data, data, (size_t)len);
    ballRxHead = next;
    ok = true;
  }
  portEXIT_CRITICAL(&ballRxMux);
  return ok;
}

static bool ballRxQueuePop(BallRxFrame& out) {
  bool ok = false;
  portENTER_CRITICAL(&ballRxMux);
  if (ballRxTail != ballRxHead) {
    out = ballRxQueue[ballRxTail];
    uint8_t next = (uint8_t)(ballRxTail + 1);
    if (next >= BALL_LINK_RX_QUEUE_CAP) next = 0;
    ballRxTail = next;
    ok = true;
  }
  portEXIT_CRITICAL(&ballRxMux);
  return ok;
}

static uint8_t ballMapCmdState() {
  if (gameState == STATE_STANDBY) return BALL_SYS_STANDBY;
  if (gameState == STATE_PRE_GAME) return BALL_SYS_STANDBY;
  if (gameState == STATE_COUNTDOWN) return BALL_SYS_GAME_START;
  if (gameState == STATE_IN_GAME) return BALL_SYS_IN_GAME;
  if (gameState == STATE_GOAL_PAUSE) return BALL_SYS_TIME_GATE;
  if (gameState == STATE_END_GAME) return BALL_SYS_END_GAME;
  if (gameState == STATE_HALFTIME) return BALL_SYS_STANDBY;
  return BALL_SYS_STANDBY;
}

static uint8_t ballMapCmdAnim() {
  if (gameState == STATE_GOAL_PAUSE || ledMode == LED_MODE_BLUE_GOAL || ledMode == LED_MODE_ORANGE_GOAL) {
    return BALL_ANIM_GOAL;
  }
  return BALL_ANIM_STILL;
}

static void ballSendPairAck(const uint8_t mac[6], const BallPairReq& req) {
  BallPairAck ack{};
  ack.proto_ver = BALL_LINK_PROTO_VER;
  ack.msg_type = BALL_MSG_PAIR_ACK;
  ack.fms_id = req.desired_fms;
  ack.channel = ballLinkCurrentChannel();
  ack.nonce = req.nonce;
  esp_now_send(mac, (const uint8_t*)&ack, sizeof(ack));
}

static void ballSendPoll(BallPeerState& p) {
  BallPollPkt poll{};
  poll.proto_ver = BALL_LINK_PROTO_VER;
  poll.msg_type = BALL_MSG_POLL;
  poll.seq = p.nextPollSeq++;
  esp_now_send(p.mac, (const uint8_t*)&poll, sizeof(poll));
  p.lastPollMs = millis();
}

static void ballSendCmd(BallPeerState& p) {
  BallCmdPkt cmd{};
  cmd.proto_ver = BALL_LINK_PROTO_VER;
  cmd.msg_type = BALL_MSG_CMD;
  cmd.sys_state = ballMapCmdState();
  cmd.color = 0;      // Ball firmware intentionally forces white locally.
  cmd.anim = ballMapCmdAnim();
  cmd.beep_hz = 0;
  cmd.beep_ms = 0;
  esp_now_send(p.mac, (const uint8_t*)&cmd, sizeof(cmd));
  p.lastCmdMs = millis();
}

static void ballHandlePairReq(const uint8_t mac[6], const BallPairReq& req) {
  int idx = ballFindPeerIndex(mac);
  bool isNew = false;
  if (idx < 0) {
    idx = ballAllocPeerIndex(mac);
    isNew = idx >= 0;
  }
  if (idx < 0) return;

  BallPeerState& p = ballPeers[idx];
  p.desiredFms = req.desired_fms;
  p.channel = ballLinkCurrentChannel();
  p.lastPairReqMs = millis();
  p.lastRxMs = p.lastPairReqMs;
  p.linked = true;
  ballEnsureEspNowPeer(mac, p.channel);
  ballSendPairAck(mac, req);

  if (isNew) {
    String msg = "Ball paired ";
    msg += ballMacToString(mac);
    msg += " ch ";
    msg += String(p.channel);
    addEvent(msg);
  }
}

static void ballHandleStatus(const uint8_t mac[6], const BallStatusPkt& st) {
  int idx = ballFindPeerIndex(mac);
  if (idx < 0) {
    idx = ballAllocPeerIndex(mac);
    if (idx < 0) return;
    ballPeers[idx].channel = ballLinkCurrentChannel();
    ballEnsureEspNowPeer(mac, ballPeers[idx].channel);
  }

  BallPeerState& p = ballPeers[idx];
  p.linked = true;
  p.lastRxMs = millis();
  p.lastStatusSeq = st.seq;
  p.gyroX = st.gyroX;
  p.gyroY = st.gyroY;
  p.gyroZ = st.gyroZ;
  p.moving = st.moving != 0;
  p.uptimeS = st.uptime_s;
  p.battMv = st.batt_mv;
  p.battPct = st.batt_pct;
}

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
static void ballOnEspNowRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (!info || !data) return;
  ballRxQueuePush(info->src_addr, data, len);
}
#else
static void ballOnEspNowRecv(const uint8_t* mac, const uint8_t* data, int len) {
  if (!mac || !data) return;
  ballRxQueuePush(mac, data, len);
}
#endif

static void ballDrainRxQueue() {
  BallRxFrame f{};
  while (ballRxQueuePop(f)) {
    if (f.len < 2) continue;
    uint8_t proto = f.data[0];
    uint8_t type = f.data[1];
    if (proto != BALL_LINK_PROTO_VER) continue;

    if (type == BALL_MSG_PAIR_REQ && f.len == sizeof(BallPairReq)) {
      BallPairReq req{};
      memcpy(&req, f.data, sizeof(req));
      ballHandlePairReq(f.src, req);
      continue;
    }

    if (type == BALL_MSG_STATUS && f.len == sizeof(BallStatusPkt)) {
      BallStatusPkt st{};
      memcpy(&st, f.data, sizeof(st));
      ballHandleStatus(f.src, st);
      continue;
    }
  }
}

static void setupBallLink() {
  memset(ballPeers, 0, sizeof(ballPeers));
  ballRxHead = 0;
  ballRxTail = 0;
  ballLinkReady = false;

  if (!wifiReady) return;

  if (esp_now_init() != ESP_OK) {
    addEvent("Ball link init failed");
    return;
  }

  esp_now_register_recv_cb(ballOnEspNowRecv);
  ballLinkReady = true;
  addEvent("Ball link ready");
}

static void serviceBallLink() {
  if (!ballLinkReady) return;

  ballDrainRxQueue();
  unsigned long now = millis();

  for (int i = 0; i < BALL_LINK_MAX_PEERS; i++) {
    BallPeerState& p = ballPeers[i];
    if (!p.used) continue;

    if (p.lastRxMs == 0 || (now - p.lastRxMs) > BALL_LINK_LINK_TIMEOUT_MS) {
      p.linked = false;
      continue;
    }

    if ((now - p.lastPollMs) >= BALL_LINK_POLL_INTERVAL_MS) {
      ballSendPoll(p);
    }
    if ((now - p.lastCmdMs) >= BALL_LINK_CMD_INTERVAL_MS) {
      ballSendCmd(p);
    }
  }
}

static uint8_t ballConnectedCount() {
  uint8_t n = 0;
  for (int i = 0; i < BALL_LINK_MAX_PEERS; i++) {
    if (ballPeers[i].used && ballPeers[i].linked) n++;
  }
  return n;
}
#endif

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

  out += "\"customTeamNamesEnabled\":";
  out += customTeamNamesEnabled ? "true" : "false";
  out += ",";

  out += "\"blueTeamName\":\"";
  out += jsonEscape(activeBlueTeamName());
  out += "\",";

  out += "\"orangeTeamName\":\"";
  out += jsonEscape(activeOrangeTeamName());
  out += "\",";

  out += "\"eventTitle\":\"";
  out += jsonEscape(eventTitle);
  out += "\",";

  out += "\"eventSubtitle\":\"";
  out += jsonEscape(eventSubtitle);
  out += "\",";

  out += "\"presentedByLine\":\"";
  out += jsonEscape(presentedByLine);
  out += "\",";

  out += "\"showPresentedByLine\":";
  out += showPresentedByLine ? "true" : "false";
  out += ",";

  out += "\"showEventBrandingInLive\":";
  out += showEventBrandingInLive ? "true" : "false";
  out += ",";

  out += "\"attractMessages\":\"";
  out += jsonEscape(attractMessagesCsv);
  out += "\",";

  out += "\"audienceOverrideMode\":\"";
  out += audienceOverrideModeName();
  out += "\",";

  out += "\"matchDurationMs\":";
  out += String(matchDurationMs);
  out += ",";

  out += "\"matchFormat\":\"";
  out += matchFormatModeName();
  out += "\",";

  out += "\"postMatchMinimumHoldMs\":";
  out += String(postMatchMinimumHoldMs);
  out += ",";

  out += "\"demoLockModeEnabled\":";
  out += demoLockModeEnabled ? "true" : "false";
  out += ",";

  out += "\"operatorPinEnabled\":";
  out += operatorPinEnabled ? "true" : "false";
  out += ",";

  out += "\"rehearsalScene\":\"";
  out += rehearsalSceneModeName();
  out += "\",";

  out += "\"rehearsalSceneActive\":";
  out += rehearsalSceneActive ? "true" : "false";
  out += ",";

  out += "\"rehearsalSceneElapsedMs\":";
  if (rehearsalSceneActive && rehearsalSceneStartMs > 0) {
    out += String(millis() - rehearsalSceneStartMs);
  } else {
    out += "0";
  }
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

  out += "\"countdownSeconds\":";
  out += String(COUNTDOWN_SECONDS);
  out += ",";

  out += "\"warningPhaseMs\":";
  out += String(WARNING_PHASE_DURATION_MS);
  out += ",";

  out += "\"countdownRemainingMs\":";
  if (gameState == STATE_COUNTDOWN && countdownStartMs > 0) {
    unsigned long totalCountdownMs = getCountdownTotalMs();
    unsigned long elapsedCountdownMs = millis() - countdownStartMs;
    if (elapsedCountdownMs >= totalCountdownMs) out += "0";
    else out += String(totalCountdownMs - elapsedCountdownMs);
  } else {
    out += "0";
  }
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

  out += "\"sensorInterruptCount\":";
#if USE_SENSOR_INTERRUPTS
  out += String(totalSensorInterruptCount());
#else
  out += "0";
#endif
  out += ",";

  // Backward-compatible combined status for the old dashboard labels.
  out += "\"blueSensorDetected\":";
  out += dualGoalAnyDetected(leftGoal) ? "true" : "false";
  out += ",";

  out += "\"orangeSensorDetected\":";
  out += dualGoalAnyDetected(rightGoal) ? "true" : "false";
  out += ",";

#if BALL_LINK_ENABLED
  out += "\"ballLinkEnabled\":true,";
  out += "\"ballLinkReady\":";
  out += ballLinkReady ? "true" : "false";
  out += ",";
  out += "\"ballConnectedCount\":";
  out += String(ballConnectedCount());
  out += ",";
  out += "\"ballPeers\":[";
  bool firstBall = true;
  for (int i = 0; i < BALL_LINK_MAX_PEERS; i++) {
    BallPeerState& bp = ballPeers[i];
    if (!bp.used) continue;
    if (!firstBall) out += ",";
    firstBall = false;
    out += "{";
    out += "\"mac\":\"";
    out += ballMacToString(bp.mac);
    out += "\",";
    out += "\"linked\":";
    out += bp.linked ? "true" : "false";
    out += ",";
    out += "\"fmsId\":";
    out += String(bp.desiredFms);
    out += ",";
    out += "\"gyroX\":";
    out += String(bp.gyroX);
    out += ",";
    out += "\"gyroY\":";
    out += String(bp.gyroY);
    out += ",";
    out += "\"gyroZ\":";
    out += String(bp.gyroZ);
    out += ",";
    out += "\"moving\":";
    out += bp.moving ? "true" : "false";
    out += ",";
    out += "\"battMv\":";
    out += String(bp.battMv);
    out += ",";
    out += "\"battPct\":";
    out += String(bp.battPct);
    out += ",";
    out += "\"uptimeS\":";
    out += String(bp.uptimeS);
    out += "}";
  }
  out += "],";
#else
  out += "\"ballLinkEnabled\":false,";
  out += "\"ballLinkReady\":false,";
  out += "\"ballConnectedCount\":0,";
  out += "\"ballPeers\":[],";
#endif

  out += "\"audioBusy\":";
  out += audioBusyPlaying() ? "true" : "false";
  out += ",";

  out += "\"audioShowState\":\"";
  out += audioShowStateName();
  out += "\",";

  out += "\"trackActive\":";
  out += trackActive ? "true" : "false";
  out += ",";

  out += "\"currentTrack\":";
  out += String(currentTrack);
  out += ",";

  out += "\"lastAudioTrack\":";
  out += String(lastAudioTrack);
  out += ",";

  out += "\"currentTrackLabel\":\"";
  out += jsonEscape(String(audioTrackLabel(currentTrack)));
  out += "\",";

  out += "\"currentTrackStartSec\":";
  out += String(currentTrackStartSec);
  out += ",";

  out += "\"currentTrackEndSec\":";
  out += String(currentTrackEndSec);
  out += ",";

  out += "\"currentTrackElapsedMs\":";
  out += String(getAudioElapsedMs());
  out += ",";

  out += "\"currentTrackRemainingMs\":";
  out += String(getAudioRemainingMs());
  out += ",";

  out += "\"currentTrackProgressPercent\":";
  out += String(getAudioProgressPercent());
  out += ",";

  out += "\"audioMutedForStartOffset\":";
  out += audioMutedForStartOffset ? "true" : "false";
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

  out += "\"trackWindows\":[";
  for (uint16_t t = 1; t <= MAX_TRACK; t++) {
    out += "{";
    out += "\"track\":";
    out += String(t);
    out += ",";
    out += "\"label\":\"";
    out += jsonEscape(String(audioTrackLabel(t)));
    out += "\",";
    out += "\"startSec\":";
    out += String(trackWindowStartSec[t]);
    out += ",";
    out += "\"endSec\":";
    out += String(trackWindowEndSec[t]);
    out += "}";
    if (t < MAX_TRACK) out += ",";
  }
  out += "],";

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

  out += "\"fullWhiteBlocked\":false,";

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

  out += "\"lastTenText\":\"";
  out += jsonEscape(lastTenWarningText);
  out += "\",";

  out += "\"lastTenDramaticMode\":";
  out += lastTenDramaticMode ? "true" : "false";
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

  out += "\"gameOverWarningMs\":";
  out += String(GAME_OVER_WARNING_MS);
  out += ",";

  out += "\"matchEndTotalEffectMs\":";
  out += String(getMatchEndTotalEffectMs());
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

  out += "\"finaleIntensityMode\":";
  out += String(finaleIntensityMode);
  out += ",";

  out += "\"autoReturnToStandbyEnabled\":";
  out += autoReturnToStandbyEnabled ? "true" : "false";
  out += ",";

  out += "\"autoReturnToStandbyDelayMs\":";
  out += String(autoReturnToStandbyDelayMs);
  out += ",";

  out += "\"effectivePostMatchAutoReturnDelayMs\":";
  out += String(effectivePostMatchAutoReturnDelayMs());
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

  out += "\"postMatchElapsedMs\":";
  if (gameState == STATE_END_GAME && postMatchStateStartMs > 0) {
    out += String(millis() - postMatchStateStartMs);
  } else {
    out += "0";
  }
  out += ",";

  out += "\"postMatchAutoReturnRemainingMs\":";
  if (gameState == STATE_END_GAME && autoReturnToStandbyEnabled && postMatchStateStartMs > 0) {
    unsigned long effectiveDelay = effectivePostMatchAutoReturnDelayMs();
    unsigned long elapsedPost = millis() - postMatchStateStartMs;
    if (elapsedPost >= effectiveDelay) out += "0";
    else out += String(effectiveDelay - elapsedPost);
  } else {
    out += "0";
  }
  out += ",";

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
  <title>FIFA Robot Soccer Controller v4.62</title>
  <style>
    :root{--bg:#05070d;--panel:#0c1220;--panel2:#111a2e;--line:rgba(255,255,255,.12);--text:#f8fafc;--muted:#94a3b8;--blue:#0050ff;--orange:#ff5000;--green:#00ff72;--red:#ef4444;--yellow:#facc15;--shadow:0 26px 90px rgba(0,0,0,.55)}
    *{box-sizing:border-box}html,body{margin:0;min-height:100%;background:radial-gradient(circle at 20% 0%,rgba(0,80,255,.25),transparent 34%),radial-gradient(circle at 80% 0%,rgba(255,80,0,.22),transparent 34%),linear-gradient(180deg,#07111f,#02040a);color:var(--text);font-family:Inter,ui-sans-serif,system-ui,-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,Arial,sans-serif}button,input,select{font:inherit}.app{width:min(1420px,100%);margin:0 auto;padding:20px}.top{display:flex;justify-content:space-between;align-items:center;gap:16px;margin-bottom:16px}.brand{display:flex;align-items:center;gap:14px}.logo{width:54px;height:54px;border-radius:18px;background:linear-gradient(135deg,#fff,#3b82f6,#f97316);display:grid;place-items:center;color:#020617;font-weight:1000;letter-spacing:-.08em;box-shadow:0 18px 50px rgba(0,0,0,.45)}h1{margin:0;font-size:28px;letter-spacing:-.04em;line-height:1}.sub{color:var(--muted);font-size:13px;margin-top:6px;font-weight:750}.tabs{display:flex;gap:10px;flex-wrap:wrap;justify-content:flex-end}.tab{border:1px solid var(--line);background:rgba(255,255,255,.07);color:var(--text);border-radius:999px;padding:12px 16px;font-weight:950;cursor:pointer}.tab.active{background:#f8fafc;color:#020617}.tab.score{background:linear-gradient(135deg,var(--blue),var(--orange));border:0;color:white}.mode{display:none}.mode.active{display:block}.grid{display:grid;grid-template-columns:1.1fr .9fr;gap:16px}.studioGrid{display:grid;grid-template-columns:repeat(3,1fr);gap:16px}.card{background:linear-gradient(180deg,rgba(255,255,255,.085),rgba(255,255,255,.035));border:1px solid var(--line);border-radius:28px;box-shadow:var(--shadow);overflow:hidden}.card.pad{padding:20px}.span2{grid-column:span 2}.span3{grid-column:span 3}.controlTitle{display:flex;justify-content:space-between;align-items:center;gap:12px;margin-bottom:14px}.controlTitle h2{margin:0;font-size:22px;letter-spacing:-.03em}.badge{border:1px solid var(--line);background:rgba(255,255,255,.08);padding:8px 10px;border-radius:999px;color:var(--muted);font-size:12px;font-weight:900}.sectionNote{color:var(--muted);font-size:13px;margin:-4px 0 14px;line-height:1.35}.scoreRow{display:grid;grid-template-columns:1fr .9fr 1fr;gap:14px;align-items:stretch}.team{border-radius:26px;padding:20px;min-height:190px;display:flex;flex-direction:column;justify-content:space-between;position:relative;overflow:hidden}.team:before{content:"";position:absolute;inset:-35%;background:radial-gradient(circle,rgba(255,255,255,.2),transparent 50%);opacity:.28;animation:drift 7s linear infinite}.team.blue{background:linear-gradient(135deg,#001a70,var(--blue))}.team.orange{background:linear-gradient(135deg,#7a2100,var(--orange))}.teamName{font-size:17px;text-transform:uppercase;letter-spacing:.18em;font-weight:1000;opacity:.9}.score{font-size:110px;line-height:.8;font-weight:1000;letter-spacing:-.1em;z-index:1}.timerBox{border-radius:26px;background:rgba(2,6,23,.84);border:1px solid var(--line);padding:20px;text-align:center;display:flex;flex-direction:column;justify-content:center}.timer{font-size:78px;font-weight:1000;letter-spacing:-.08em;line-height:.9}.state{display:inline-flex;align-self:center;margin-top:12px;padding:10px 14px;border-radius:999px;background:rgba(255,255,255,.09);border:1px solid var(--line);font-size:13px;text-transform:uppercase;letter-spacing:.14em;font-weight:1000}.state.go{background:rgba(0,255,114,.18);color:#86efac}.state.pause{background:rgba(250,204,21,.18);color:#fde68a}.state.end{background:rgba(255,255,255,.18)}.actions{display:grid;grid-template-columns:repeat(2,1fr);gap:12px}.actions button{min-height:74px;border:0;border-radius:22px;color:white;font-weight:1000;font-size:16px;cursor:pointer;box-shadow:0 16px 34px rgba(0,0,0,.32);transition:.12s transform,.18s filter}.actions button:active{transform:translateY(1px) scale(.99)}.actions button:disabled{opacity:.45;filter:grayscale(.75);cursor:not-allowed;box-shadow:none}.primary{background:linear-gradient(135deg,#16a34a,#00ff72);color:#03130a!important}.resume{background:linear-gradient(135deg,#eab308,#f97316);color:#111827!important}.blueBtn{background:linear-gradient(135deg,#0031a8,var(--blue))}.orangeBtn{background:linear-gradient(135deg,#9a3412,var(--orange))}.dark{background:linear-gradient(135deg,#1f2937,#050505);border:1px solid var(--line)!important}.danger{background:linear-gradient(135deg,#7f1d1d,#ef4444)}.wide{grid-column:span 2}.resumeBig{grid-column:span 2;min-height:94px!important;font-size:22px!important}.resumeBig.attention{animation:softPulse 1s ease-in-out infinite;outline:2px solid rgba(255,255,255,.45)}.hidden{display:none!important}.miniGrid{display:grid;grid-template-columns:repeat(2,1fr);gap:12px}.metric{border:1px solid var(--line);background:rgba(255,255,255,.055);border-radius:20px;padding:14px}.metric .k{font-size:11px;color:var(--muted);letter-spacing:.12em;text-transform:uppercase;font-weight:1000}.metric .v{font-size:20px;font-weight:1000;margin-top:5px}.ok{color:#86efac}.warn{color:#fde68a}.bad{color:#fca5a5}.log{max-height:260px;overflow:auto;display:flex;flex-direction:column;gap:8px}.logItem{font-size:13px;color:#cbd5e1;border-bottom:1px solid rgba(255,255,255,.07);padding-bottom:8px}.field{display:grid;grid-template-columns:1fr;gap:7px;margin:12px 0}.field label{font-size:11px;color:var(--muted);letter-spacing:.12em;text-transform:uppercase;font-weight:1000}.select,.num,.color,.textBox{width:100%;border:1px solid var(--line);border-radius:16px;background:rgba(2,6,23,.82);color:var(--text);padding:12px 13px;font-weight:900;outline:none}.select:focus,.num:focus,.textBox:focus{border-color:rgba(255,255,255,.34);box-shadow:0 0 0 4px rgba(0,80,255,.12)}.color{height:48px;padding:5px}.rangeRow{display:grid;grid-template-columns:1fr 86px 62px;gap:8px;align-items:center}.rangeRow input[type=range]{width:100%;accent-color:#60a5fa}.rangeRow .num{padding:10px;text-align:center}.resetBtn{border:1px solid var(--line);background:rgba(255,255,255,.07);color:var(--text);border-radius:14px;padding:10px 8px;font-weight:1000;cursor:pointer}.applyRow{display:flex;gap:10px;flex-wrap:wrap;margin-top:12px}.applyRow button{border:0;border-radius:16px;padding:12px 14px;font-weight:1000;cursor:pointer}.apply{background:#f8fafc;color:#020617}.test{background:rgba(255,255,255,.08);border:1px solid var(--line)!important;color:var(--text)}.recipe{width:100%;min-height:116px;border:1px solid var(--line);border-radius:18px;background:rgba(2,6,23,.86);color:#dbeafe;padding:12px;font-family:ui-monospace,SFMono-Regular,Menlo,Monaco,Consolas,monospace;font-size:12px;line-height:1.45}.smallActions{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;margin-top:10px}.smallActions button{border:1px solid var(--line);border-radius:16px;background:rgba(255,255,255,.07);color:var(--text);padding:12px;font-weight:900;cursor:pointer}.diag{margin-top:12px;border:1px solid var(--line);border-radius:18px;padding:10px;background:rgba(255,255,255,.035)}.diag summary{cursor:pointer;color:var(--muted);font-weight:1000;text-transform:uppercase;letter-spacing:.12em;font-size:12px}@keyframes drift{from{transform:rotate(0)}to{transform:rotate(360deg)}}@keyframes softPulse{0%,100%{filter:brightness(1)}50%{filter:brightness(1.25)}}


    /* SCOREBOARD BASE */
    body.routeScoreboard .app{
      display:none;
    }
    body.routeScoreboard .public{
      display:grid;
    }
    body.routeScoreboard .exitPublic{
      display:none;
    }

    /* v4.61 hard scoreboard pin guard.
       When the audience board is opened from Operator Mode, keep it above the
       operator UI through state changes such as PRE_GAME -> COUNTDOWN -> IN_GAME.
       This is intentionally body-level so a missed .active class cannot flash
       the operator dashboard underneath. */
    body.scoreboardPinnedOpen .app{
      display:none;
    }
    body.scoreboardPinnedOpen .public{
      display:grid !important;
    }
    body.scoreboardPinnedOpen .exitPublic{
      display:block;
    }
    body.routeScoreboard.scoreboardPinnedOpen .exitPublic{
      display:none;
    }

    .public{
      --lastColor:#982ABC;
      --lastDur:1000ms;
      display:none;
      position:fixed;
      inset:0;
      z-index:20;
      width:100vw;
      height:100dvh;
      overflow:hidden;
      padding:clamp(8px,1.2vw,14px);
      row-gap:clamp(8px,1.0vh,12px);
      background:
        radial-gradient(circle at 18% 8%,rgba(0,80,255,.28),transparent 36%),
        radial-gradient(circle at 82% 8%,rgba(255,80,0,.26),transparent 36%),
        linear-gradient(180deg,#061121 0%,#020617 64%,#00040b 100%);
      -webkit-font-smoothing:antialiased;
      text-rendering:optimizeLegibility;
      font-family:"Avenir Next","Segoe UI Variable","Trebuchet MS","Segoe UI",sans-serif;
    }
    .public.active{
      display:grid;
      grid-template-rows:minmax(132px,20vh) minmax(0,1fr) auto;
      align-items:stretch;
    }
    .public::before,
    .public::after{
      content:"";
      position:absolute;
      inset:-20%;
      pointer-events:none;
      z-index:0;
    }
    .public::before{
      opacity:.28;
      background:conic-gradient(from 0deg,transparent,rgba(255,255,255,.10),transparent,rgba(255,255,255,.07),transparent);
      animation:scoreboardHalo 24s linear infinite;
    }
    .public::after{
      opacity:.16;
      background:linear-gradient(90deg,transparent,rgba(255,255,255,.09),transparent);
      animation:scoreboardSheen 9s ease-in-out infinite;
    }
    .publicTop,
    .publicScores,
    .publicInfoBar,
    .publicWinner{
      position:relative;
      z-index:1;
    }
    .publicTop{
      min-height:0;
      border-radius:24px;
      border:1px solid rgba(255,255,255,.12);
      background:linear-gradient(180deg,rgba(255,255,255,.08),rgba(255,255,255,.03));
      box-shadow:inset 0 0 36px rgba(255,255,255,.04),0 16px 48px rgba(0,0,0,.32);
      display:grid;
      place-items:center;
      overflow:hidden;
      animation:scoreboardTopFloat 10s ease-in-out infinite;
    }
    .goalRingEl{
      position:absolute;
      left:50%;
      top:50%;
      width:min(62vw,900px);
      height:min(62vw,900px);
      border-radius:50%;
      transform:translate(-50%,-50%) scale(.80);
      border:2px solid currentColor;
      opacity:0;
      pointer-events:none;
    }
    .publicState{
      position:absolute;
      top:clamp(10px,1.8vh,20px);
      left:50%;
      transform:translateX(-50%);
      max-width:min(96vw,1600px);
      text-align:center;
      text-transform:uppercase;
      font-weight:900;
      font-size:clamp(12px,1.4vw,20px);
      letter-spacing:.16em;
      color:#dbeafe;
      white-space:normal;
      line-height:1.06;
      text-wrap:balance;
    }
    .publicTimer{
      max-width:min(96vw,1650px);
      margin:0 auto;
      text-align:center;
      font-weight:1000;
      font-size:clamp(74px,14vw,190px);
      line-height:.80;
      letter-spacing:-.08em;
      white-space:normal;
      overflow-wrap:anywhere;
      text-shadow:0 0 24px rgba(255,255,255,.18),0 0 78px rgba(96,165,250,.10);
      will-change:transform,filter;
      animation:scoreboardTimerFloat 6.6s ease-in-out infinite;
    }
    .publicTimer.timerTick{
      animation:scoreboardTimerTick .60s cubic-bezier(.16,1,.3,1) both;
    }
    .publicMeta{
      position:absolute;
      bottom:clamp(10px,1.7vh,18px);
      left:50%;
      transform:translateX(-50%);
      max-width:min(96vw,1600px);
      text-align:center;
      text-transform:uppercase;
      font-weight:900;
      font-size:clamp(11px,1.2vw,16px);
      letter-spacing:.14em;
      color:#9fb5d8;
      white-space:normal;
      line-height:1.06;
      text-wrap:balance;
    }
    .publicScores{
      min-height:0;
      display:grid;
      grid-template-columns:1fr 1fr;
      gap:clamp(8px,1.2vw,16px);
      align-items:stretch;
      position:relative;
    }
    .publicTeam{
      min-height:0;
      border-radius:24px;
      border:1px solid rgba(255,255,255,.16);
      padding:clamp(14px,2.2vw,24px);
      display:grid;
      grid-template-columns:minmax(0,1fr) auto;
      gap:clamp(8px,1.0vw,14px);
      align-items:end;
      overflow:hidden;
      isolation:isolate;
      box-shadow:inset 0 0 34px rgba(255,255,255,.05),0 20px 56px rgba(0,0,0,.42);
      animation:scoreboardCardFloat 9s ease-in-out infinite;
    }
    .publicTeam.orange{
      animation-delay:-4.5s;
    }
    .publicTeam::before{
      content:"";
      position:absolute;
      inset:-30%;
      pointer-events:none;
      opacity:.20;
      background:radial-gradient(circle,rgba(255,255,255,.28),transparent 56%);
      animation:scoreboardOrbital 12s linear infinite;
    }
    .publicTeam.blue{
      background:linear-gradient(135deg,#001a70,var(--blue));
    }
    .publicTeam.orange{
      background:linear-gradient(135deg,#7a2100,var(--orange));
    }
    .publicTeam .name{
      min-width:0;
      max-width:100%;
      white-space:nowrap;
      overflow:hidden;
      text-overflow:ellipsis;
      align-self:end;
      text-transform:uppercase;
      letter-spacing:.08em;
      font-weight:1000;
      font-size:clamp(18px,2.5vw,40px);
      line-height:1.02;
      text-shadow:0 0 18px rgba(255,255,255,.18);
      z-index:1;
    }
    .publicTeam .num{
      justify-self:end;
      align-self:end;
      font-variant-numeric:tabular-nums lining-nums;
      font-feature-settings:"tnum" 1,"lnum" 1;
      text-align:right;
      line-height:.84;
      letter-spacing:-.05em;
      font-weight:1000;
      font-size:clamp(118px,18vw,280px);
      min-width:1.36em;
      padding:0 .03em .06em .02em;
      text-shadow:0 0 26px rgba(255,255,255,.20),0 10px 26px rgba(0,0,0,.28);
      background:transparent;
      border:0;
      border-radius:0;
      box-shadow:none;
      z-index:1;
      animation:scoreboardNumberPulse 6.4s ease-in-out infinite;
    }
    .publicInfoBar{
      min-height:clamp(44px,7vh,80px);
      border-radius:16px;
      border:1px solid rgba(255,255,255,.12);
      background:linear-gradient(180deg,rgba(255,255,255,.07),rgba(255,255,255,.02));
      display:flex;
      flex-direction:column;
      justify-content:center;
      align-items:center;
      gap:clamp(2px,.5vh,8px);
      padding:clamp(6px,.9vh,12px) clamp(10px,1.8vw,22px);
      overflow:hidden;
    }
    .publicTicker{
      width:100%;
      max-width:min(96vw,1600px);
      text-align:center;
      text-transform:uppercase;
      font-size:clamp(14px,1.6vw,24px);
      line-height:1.15;
      letter-spacing:.09em;
      font-weight:900;
      color:#f8fafc;
      white-space:nowrap;
      overflow:hidden;
      text-overflow:ellipsis;
      text-shadow:0 0 14px rgba(255,255,255,.18);
    }
    .publicBrandLine{
      width:100%;
      max-width:min(94vw,1500px);
      text-align:center;
      text-transform:uppercase;
      font-size:clamp(10px,1.0vw,14px);
      line-height:1.15;
      letter-spacing:.11em;
      font-weight:900;
      color:#c7d2fe;
      white-space:nowrap;
      overflow:hidden;
      text-overflow:ellipsis;
      opacity:.90;
    }
    .publicWinner{
      position:fixed;
      left:50%;
      top:50%;
      transform:translate(-50%,-50%);
      font-size:clamp(48px,10vw,170px);
      line-height:.84;
      letter-spacing:.08em;
      font-weight:1000;
      text-transform:uppercase;
      text-align:center;
      white-space:nowrap;
      text-shadow:0 0 34px rgba(255,255,255,.50),0 0 100px rgba(255,255,255,.26);
      opacity:0;
      pointer-events:none;
      z-index:8;
    }
    .public.active .publicWinner.show{
      animation:scoreboardWinnerText 1.25s ease-in-out infinite alternate;
    }
    .exitPublic{
      position:fixed;
      right:clamp(12px,2vw,22px);
      top:clamp(12px,2vw,22px);
      z-index:25;
      border:1px solid rgba(255,255,255,.20);
      background:rgba(0,0,0,.42);
      backdrop-filter:blur(12px);
      color:white;
      border-radius:999px;
      padding:12px 16px;
      font-weight:1000;
      opacity:0;
      pointer-events:none;
      transform:translateY(-10px);
      transition:opacity .35s ease,transform .35s ease;
    }
    .public.controlsVisible .exitPublic{
      opacity:1;
      pointer-events:auto;
      transform:translateY(0);
    }
    .publicTeam.scoreLead{
      filter:brightness(1.14) saturate(1.16);
      box-shadow:0 0 0 2px rgba(255,255,255,.40),inset 0 0 64px rgba(255,255,255,.12),0 26px 70px rgba(0,0,0,.50);
    }
    .publicTeam.scoreBehind{
      filter:brightness(.76) saturate(.82);
    }
    .publicTeam.scoreTied{
      filter:brightness(1.06) saturate(1.08);
    }
    .publicTeam.scorePop{
      animation:scoreboardCardPop .95s cubic-bezier(.16,.9,.18,1) both;
      outline:2px solid rgba(255,255,255,.40);
    }
    .publicTeam.scorePop .num{
      animation:scoreboardNumberPop .95s cubic-bezier(.16,.9,.18,1) both;
    }
    .v423RollIn{
      animation:scoreboardScoreRoll .72s cubic-bezier(.16,1,.3,1) both;
    }
    .public.scoreBlueLead .publicTeam.blue,
    .public.scoreOrangeLead .publicTeam.orange{
      filter:brightness(1.14) saturate(1.14);
      box-shadow:0 0 0 2px rgba(255,255,255,.38),inset 0 0 64px rgba(255,255,255,.10),0 26px 72px rgba(0,0,0,.52);
    }
    .public.scoreBlueLead .publicTeam.orange,
    .public.scoreOrangeLead .publicTeam.blue{
      filter:brightness(.78) saturate(.82);
    }
    .public.scoreTieLive .publicTeam{
      filter:brightness(1.05) saturate(1.06);
    }
    .public.postMatchActive .publicTop{
      animation:none;
    }
    .publicTeam.winnerGlow{
      box-shadow:0 0 0 2px rgba(255,255,255,.45),0 0 74px rgba(255,255,255,.24),0 26px 84px rgba(0,0,0,.56);
    }
    .publicTeam.tieGlow{
      box-shadow:0 0 0 2px rgba(255,255,255,.33),0 0 44px rgba(255,255,255,.16),0 24px 74px rgba(0,0,0,.52);
    }

    /* SCOREBOARD LIVE */
    .public.liveMinimalActive .publicInfoBar,
    .public.liveMinimalActive .publicState,
    .public.liveMinimalActive .publicMeta,
    .public.liveMinimalActive .publicTicker,
    .public.liveMinimalActive .publicBrandLine{
      display:none;
    }
    .public.liveMinimalActive .publicTeam .name{
      display:none;
    }
    .public.liveMinimalActive .publicTop{
      min-height:0;
      display:grid;
      place-items:center;
      padding:0;
    }
    .public.liveMinimalActive .publicScores{
      height:100%;
      min-height:0;
      align-items:stretch;
    }
    .public.liveMinimalActive .publicTeam{
      height:100%;
      min-height:0;
      display:grid;
      grid-template-columns:1fr;
      grid-template-rows:1fr;
      align-items:stretch;
      place-items:center;
      padding:clamp(6px,1.2vw,18px);
    }
    .public.liveMinimalActive .publicTeam .num{
      display:grid;
      place-items:center;
      align-self:stretch;
      justify-self:stretch;
      height:100%;
      min-width:0;
      width:100%;
      max-width:none;
      font-size:clamp(260px,38vw,680px);
      text-align:center;
      line-height:.72;
      letter-spacing:-.06em;
      padding:0;
      margin:0;
      background:transparent;
      border:0;
      border-radius:0;
      box-shadow:none;
      animation:none;
    }
    body.routeScoreboard .public.active.liveMinimalActive{
      height:100dvh;
      width:100vw;
      overflow:hidden;
      display:grid;
      grid-template-rows:minmax(132px,21vh) minmax(0,1fr);
      row-gap:clamp(6px,.8vh,12px);
      padding:clamp(6px,1vw,12px);
    }
    body.routeScoreboard .public.liveMinimalActive .publicInfoBar,
    body.routeScoreboard .public.liveMinimalActive .publicState,
    body.routeScoreboard .public.liveMinimalActive .publicMeta,
    body.routeScoreboard .public.liveMinimalActive .publicTicker,
    body.routeScoreboard .public.liveMinimalActive .publicBrandLine{
      display:none;
    }
    body.routeScoreboard .public.liveMinimalActive .publicTop{
      height:100%;
      min-height:0;
      display:grid;
      place-items:center;
      padding:0;
    }
    body.routeScoreboard .public.liveMinimalActive .publicTimer{
      font-size:clamp(118px,18vw,265px);
      line-height:.76;
      text-align:center;
      letter-spacing:-.075em;
      animation:none;
    }
    body.routeScoreboard .public.liveMinimalActive .publicScores{
      height:100%;
      min-height:0;
      display:grid;
      grid-template-columns:1fr 1fr;
      gap:clamp(8px,1.2vw,18px);
      align-items:stretch;
    }
    body.routeScoreboard .public.liveMinimalActive .publicTeam{
      height:100%;
      min-height:0;
      display:grid;
      grid-template-columns:1fr;
      grid-template-rows:1fr;
      align-items:stretch;
      place-items:center;
      padding:clamp(6px,1.2vw,18px);
      overflow:hidden;
    }
    body.routeScoreboard .public.liveMinimalActive .publicTeam .name{
      display:none;
    }
    body.routeScoreboard .public.liveMinimalActive .publicTeam .num{
      display:grid;
      place-items:center;
      justify-self:stretch;
      align-self:stretch;
      height:100%;
      min-width:0;
      max-width:none;
      margin:0;
      padding:0;
      font-size:clamp(300px,42vw,680px);
      line-height:.72;
      letter-spacing:-.08em;
      text-align:center;
      width:100%;
      background:transparent;
      border:none;
      border-radius:0;
      box-shadow:none;
      color:rgba(255,255,255,.96);
      text-shadow:0 14px 44px rgba(0,0,0,.48),0 0 34px rgba(255,255,255,.18);
      animation:none;
    }

    /* SCOREBOARD PRE-GAME */
    .public.preGameActive{
      grid-template-rows:1fr;
      background:
        radial-gradient(circle at 50% 12%,rgba(0,255,153,.24),transparent 34%),
        radial-gradient(circle at 20% 12%,rgba(0,80,255,.24),transparent 34%),
        radial-gradient(circle at 80% 12%,rgba(255,80,0,.24),transparent 34%),
        linear-gradient(180deg,#041224,#01050b);
    }
    .public.preGameActive .publicScores,
    .public.preGameActive .publicInfoBar{
      display:none;
    }
    .public.preGameActive .publicTop{
      min-height:100%;
      border-radius:0;
      border-color:transparent;
      box-shadow:none;
      display:flex;
      flex-direction:column;
      align-items:center;
      justify-content:center;
      gap:clamp(18px,3vh,40px);
      padding:clamp(32px,5vh,72px) clamp(20px,4vw,64px);
      background:transparent;
    }
    .public.preGameActive .publicState{
      position:static;
      transform:none;
      color:#b8ffe0;
      font-size:clamp(22px,2.7vw,42px);
      letter-spacing:.16em;
      line-height:1.06;
      text-shadow:0 0 22px rgba(0,255,153,.45);
      animation:scoreboardReadyPulse 2.2s ease-in-out infinite;
    }
    .public.preGameActive .publicTimer{
      position:static;
      max-width:94vw;
      font-size:clamp(88px,13vw,220px);
      line-height:.86;
      letter-spacing:-.035em;
      text-align:center;
      text-wrap:balance;
      overflow-wrap:normal;
      word-break:normal;
      text-shadow:0 0 30px rgba(255,255,255,.35),0 0 88px rgba(0,255,153,.20);
    }
    .public.preGameActive .publicMeta{
      position:static;
      transform:none;
      font-size:clamp(20px,2.4vw,38px);
      line-height:1.08;
      letter-spacing:.12em;
      color:#e2e8f0;
      text-align:center;
      text-wrap:balance;
    }

    /* SCOREBOARD COUNTDOWN */
    .public.countdownActive{
      position:relative;
      height:100dvh;
      width:100vw;
      overflow:hidden;
      display:grid;
      place-items:center;
      padding:0;
      grid-template-rows:1fr;
      isolation:isolate;
      background:
        radial-gradient(circle at 50% 45%,rgba(255,0,0,.48),transparent 31%),
        radial-gradient(circle at 25% 50%,rgba(0,80,255,.32),transparent 35%),
        radial-gradient(circle at 75% 50%,rgba(255,80,0,.34),transparent 35%),
        radial-gradient(circle at 50% 50%,rgba(255,255,255,.08),transparent 18%),
        linear-gradient(180deg,#1b0305,#020617 58%,#000);
      animation:scoreboardCountdownScreenPulse var(--countDur,1000ms) cubic-bezier(.18,.9,.18,1) infinite;
    }
    .public.countdownActive::before,
    .public.countdownActive::after{
      pointer-events:none;
    }
    .public.countdownActive::before{
      content:"";
      position:fixed;
      left:50%;
      left:50vw;
      top:50%;
      top:50dvh;
      width:145vmax;
      height:145vmax;
      transform:translate3d(-50%,-50%,0);
      transform-origin:center;
      z-index:0;
      opacity:.72;
      background:conic-gradient(from 0deg,rgba(255,0,0,.00),rgba(255,0,0,.26),rgba(255,80,0,.24),rgba(0,80,255,.24),rgba(255,0,0,.00));
      filter:blur(28px);
      animation:scoreboardCountdownSpinDrama calc(var(--countDur,1000ms) * 4) linear infinite;
    }
    .public.countdownActive::after{
      content:"";
      position:fixed;
      left:50%;
      left:50vw;
      top:50%;
      top:50dvh;
      width:min(78vw,78vh);
      height:min(78vw,78vh);
      width:min(78vw,78dvh);
      height:min(78vw,78dvh);
      transform:translate3d(-50%,-50%,0);
      transform-origin:center;
      z-index:1;
      border-radius:50%;
      border:clamp(4px,.65vw,10px) solid rgba(255,80,80,.48);
      box-shadow:0 0 86px rgba(255,0,0,.46),0 0 160px rgba(255,80,0,.20),inset 0 0 62px rgba(255,255,255,.16);
      opacity:.95;
      animation:scoreboardCountdownRingPop var(--countDur,1000ms) ease-out infinite;
    }
    .public.countdownActive .publicScores,
    .public.countdownActive .publicInfoBar,
    .public.countdownActive .publicTicker,
    .public.countdownActive .publicBrandLine,
    .public.countdownActive .publicMeta,
    .public.countdownActive .publicState{
      display:none;
    }
    .public.countdownActive .publicTop{
      position:fixed;
      inset:0;
      z-index:3;
      width:100vw;
      height:100dvh;
      min-height:100dvh;
      display:grid;
      place-items:center;
      padding:0;
      margin:0;
      border:0;
      background:transparent;
      box-shadow:none;
      animation:none;
    }
    .public.countdownActive .publicTimer{
      position:relative;
      z-index:4;
      margin:0;
      transform-origin:center;
      font-size:clamp(280px,42vw,680px);
      line-height:.72;
      letter-spacing:-.09em;
      color:rgba(255,255,255,.96);
      text-align:center;
      text-shadow:0 0 42px var(--countColor,#FF0000),0 0 110px rgba(255,255,255,.42),0 22px 100px rgba(0,0,0,.78);
      animation:
        scoreboardCountdownNumberPop var(--countDur,1000ms) cubic-bezier(.18,.9,.18,1) infinite,
        scoreboardCountdownGlowShift calc(var(--countDur,1000ms) * 2.4) ease-in-out infinite;
    }

    /* SCOREBOARD GOAL */
    .public.goalBlueActive,
    .public.goalOrangeActive{
      grid-template-rows:1fr minmax(0,28vh);
      --goalAccent:#fff;
    }
    .public.goalBlueActive{
      --goalColor:var(--blue);
      background:radial-gradient(circle at 50% 18%,rgba(0,80,255,.86),transparent 32%),linear-gradient(180deg,#020617,#06163d 66%,#000);
    }
    .public.goalOrangeActive{
      --goalColor:var(--orange);
      background:radial-gradient(circle at 50% 18%,rgba(255,80,0,.86),transparent 32%),linear-gradient(180deg,#170901,#3a1605 66%,#000);
    }
    .public.goalBlueActive .publicInfoBar,
    .public.goalOrangeActive .publicInfoBar{
      display:none;
    }
    .public.goalBlueActive .publicTop,
    .public.goalOrangeActive .publicTop{
      border-color:transparent;
      background:transparent;
      box-shadow:none;
    }
    .public.goalBlueActive .publicState,
    .public.goalOrangeActive .publicState{
      color:#fff;
      text-shadow:0 0 28px var(--goalColor);
    }
    .public.goalBlueActive .publicTimer,
    .public.goalOrangeActive .publicTimer{
      font-size:clamp(56px,8.2vw,132px);
      opacity:.58;
      text-shadow:0 0 22px rgba(255,255,255,.46);
      animation:none;
    }
    .public.goalBlueActive .publicScores,
    .public.goalOrangeActive .publicScores{
      height:100%;
      opacity:.88;
    }
    .public.goalBlueActive .publicTeam.blue,
    .public.goalOrangeActive .publicTeam.orange{
      box-shadow:0 0 58px rgba(255,255,255,.16),0 24px 76px rgba(0,0,0,.52);
      transform:scale(1.02);
    }
    .public.goalBlueActive .publicTeam.orange,
    .public.goalOrangeActive .publicTeam.blue{
      opacity:.48;
      filter:saturate(.72) brightness(.78);
    }
    .public.goalBlueActive .goalRingEl,
    .public.goalOrangeActive .goalRingEl{
      color:var(--goalColor);
    }
    .public.goalRingsActive .goalRingEl:nth-child(1){
      animation:scoreboardGoalRing 1.00s cubic-bezier(.16,.9,.18,1) infinite;
    }
    .public.goalRingsActive .goalRingEl:nth-child(2){
      animation:scoreboardGoalRing 1.00s cubic-bezier(.16,.9,.18,1) .28s infinite;
    }
    .public.goalJustEntered .publicWinner{
      animation:scoreboardGoalWinnerPunch .85s cubic-bezier(.16,1,.3,1) both;
    }

    /* SCOREBOARD GAME OVER */
    .public.gameOverActive{
      grid-template-rows:1fr;
      background:
        radial-gradient(circle at 50% 42%,rgba(255,255,255,.22),transparent 18%),
        radial-gradient(circle at 50% 50%,rgba(255,0,0,.96),transparent 42%),
        linear-gradient(180deg,#3a0000,#110000 62%,#000);
      animation:scoreboardGameOverAlarm .42s cubic-bezier(.2,.9,.2,1) infinite alternate;
    }
    .public.gameOverActive::before{
      opacity:.58;
      background:repeating-linear-gradient(90deg,rgba(255,255,255,.00) 0,rgba(255,255,255,.00) 38px,rgba(255,255,255,.11) 40px,rgba(255,255,255,.00) 76px);
      animation:scoreboardGameOverScan .9s linear infinite;
    }
    .public.gameOverActive::after{
      opacity:.42;
      background:radial-gradient(circle at 50% 50%,transparent 0 24%,rgba(255,255,255,.22) 25%,transparent 31%,rgba(255,255,255,.14) 42%,transparent 48%);
      animation:scoreboardGameOverRing .82s ease-out infinite;
    }
    .public.gameOverActive .publicScores,
    .public.gameOverActive .publicInfoBar,
    .public.gameOverActive .publicTicker,
    .public.gameOverActive .publicBrandLine{
      display:none;
    }
    .public.gameOverActive .publicTop{
      width:100vw;
      height:100dvh;
      min-height:100dvh;
      display:grid;
      place-items:center;
      text-align:center;
      padding:clamp(20px,4vw,54px);
      border-color:transparent;
      background:transparent;
      box-shadow:none;
      animation:none;
    }
    .public.gameOverActive .publicState{
      position:static;
      transform:none;
      order:1;
      font-size:clamp(38px,5.6vw,92px);
      line-height:.92;
      letter-spacing:.10em;
      color:#fff;
      text-shadow:0 0 32px rgba(255,255,255,.38),0 0 96px rgba(255,0,0,.72);
      animation:scoreboardGameOverTextPunch .42s cubic-bezier(.2,.9,.2,1) infinite alternate;
    }
    .public.gameOverActive .publicTimer{
      position:static;
      order:2;
      font-size:clamp(78px,14vw,210px);
      line-height:.82;
      letter-spacing:.04em;
      color:#fff;
      text-shadow:0 0 38px rgba(255,255,255,.50),0 0 130px rgba(255,0,0,.80),0 18px 90px rgba(0,0,0,.70);
      animation:scoreboardGameOverTitlePunch .42s cubic-bezier(.2,.9,.2,1) infinite alternate;
    }
    .public.gameOverActive .publicMeta{
      position:static;
      transform:none;
      order:3;
      font-size:clamp(22px,3.3vw,54px);
      line-height:1.02;
      letter-spacing:.12em;
      color:#fee2e2;
      text-shadow:0 0 26px rgba(255,255,255,.28);
    }

    /* SCOREBOARD FINAL */
    .public.finalBlueActive,
    .public.finalOrangeActive,
    .public.finalTieActive{
      grid-template-rows:1fr;
    }
    .public.finalBlueActive{
      --finalColor:var(--blue);
      background:radial-gradient(circle at 50% 18%,rgba(0,80,255,.85),transparent 32%),linear-gradient(180deg,#020617,#06163d 66%,#000);
    }
    .public.finalOrangeActive{
      --finalColor:var(--orange);
      background:radial-gradient(circle at 50% 18%,rgba(255,80,0,.86),transparent 32%),linear-gradient(180deg,#170901,#3a1605 66%,#000);
    }
    .public.finalTieActive{
      --finalColor:#e5e7eb;
      background:linear-gradient(90deg,rgba(0,80,255,.52),rgba(2,6,23,.96),rgba(255,80,0,.52)),#020617;
    }
    .public.finalBlueActive .publicInfoBar,
    .public.finalOrangeActive .publicInfoBar,
    .public.finalTieActive .publicInfoBar{
      display:none;
    }
    .public.finalBlueActive .publicScores,
    .public.finalOrangeActive .publicScores,
    .public.finalTieActive .publicScores{
      display:none;
    }
    .public.finalBlueActive .publicTop,
    .public.finalOrangeActive .publicTop,
    .public.finalTieActive .publicTop{
      width:100vw;
      height:100dvh;
      min-height:100dvh;
      display:grid;
      place-items:center;
      text-align:center;
      padding:clamp(20px,4vw,54px);
      border-color:transparent;
      background:transparent;
      box-shadow:none;
    }
    .public.finalBlueActive .publicState,
    .public.finalOrangeActive .publicState,
    .public.finalTieActive .publicState{
      position:static;
      transform:none;
      margin-bottom:clamp(8px,1.4vh,18px);
      font-size:clamp(18px,2.1vw,30px);
      letter-spacing:.14em;
      color:#fff;
      text-shadow:0 0 30px var(--finalColor);
    }
    .public.finalBlueActive .publicTimer,
    .public.finalOrangeActive .publicTimer,
    .public.finalTieActive .publicTimer{
      font-size:clamp(82px,13vw,190px);
      line-height:.90;
      text-shadow:0 0 34px var(--finalColor),0 0 120px rgba(255,255,255,.28);
      animation:scoreboardFinalTitle 2.0s ease-in-out infinite;
    }
    .public.finalBlueActive .publicMeta,
    .public.finalOrangeActive .publicMeta,
    .public.finalTieActive .publicMeta{
      display:block;
      position:static;
      transform:none;
      margin-top:clamp(12px,2vh,28px);
      font-size:clamp(30px,5vw,78px);
      line-height:.98;
      color:#f8fafc;
      text-shadow:0 0 22px rgba(255,255,255,.24);
    }

    /* SCOREBOARD ATTRACT */
    .public.attractSlidesActive{
      grid-template-rows:1fr;
    }
    .public.attractSlidesActive .publicScores,
    .public.attractSlidesActive .publicInfoBar{
      display:none;
    }
    .public.attractSlidesActive .publicTop{
      min-height:100%;
      border-radius:0;
      border-color:transparent;
      box-shadow:none;
      display:flex;
      flex-direction:column;
      align-items:center;
      justify-content:center;
      gap:clamp(18px,3vh,44px);
      padding:clamp(34px,5.2vh,78px) clamp(22px,4vw,72px);
      background:radial-gradient(circle at 50% 10%,rgba(255,255,255,.08),transparent 36%);
    }

    .public.attractSlidesActive .publicTimer,
    .public.attractSlidesActive .publicState,
    .public.attractSlidesActive .publicMeta{
      will-change:transform,opacity,filter;
    }
    .public.attractSlideEntered .publicState{
      animation:scoreboardAttractTextEnter .72s cubic-bezier(.16,1,.3,1) both;
    }
    .public.attractSlideEntered .publicTimer{
      animation:scoreboardAttractMainEnter .86s cubic-bezier(.16,1,.3,1) both;
    }
    .public.attractSlideEntered .publicMeta{
      animation:scoreboardAttractMetaEnter .94s cubic-bezier(.16,1,.3,1) both;
    }
    .public.attractSlideTitle .publicTop::before,
    .public.attractSlideHow .publicTop::before,
    .public.attractSlideChoose .publicTop::before,
    .public.attractSlideChallenge .publicTop::before,
    .public.attractSlideInfo .publicTop::before{
      content:"";
      position:absolute;
      inset:8%;
      border-radius:50%;
      pointer-events:none;
      opacity:.16;
      border:2px solid rgba(255,255,255,.28);
      box-shadow:0 0 52px rgba(255,255,255,.12),inset 0 0 42px rgba(255,255,255,.08);
      animation:scoreboardAttractOrbPulse 5.6s ease-in-out infinite;
    }
    .public.attractSlidesActive::before{
      opacity:.34;
      background:conic-gradient(from 0deg,rgba(0,80,255,.00),rgba(0,80,255,.24),rgba(255,80,0,.22),rgba(0,210,255,.16),rgba(0,80,255,.00));
      filter:blur(22px);
      animation:scoreboardAttractAurora 18s linear infinite;
    }
    .public.attractSlidesActive::after{
      opacity:.24;
      background:linear-gradient(105deg,transparent 0 28%,rgba(255,255,255,.12) 42%,transparent 58% 100%);
      animation:scoreboardAttractSheen 5.8s ease-in-out infinite;
    }
    .public.attractSlideTitle .publicTimer{
      animation:scoreboardAttractFestivalPulse 4.6s ease-in-out infinite;
    }
    .public.attractSlideTitle::before{
      opacity:.46;
      background:conic-gradient(from 90deg,rgba(0,169,255,.00),rgba(0,169,255,.32),rgba(138,43,226,.26),rgba(255,61,110,.28),rgba(255,199,74,.18),rgba(0,169,255,.00));
    }
    .public.attractSlideHow .publicTop{
      background:radial-gradient(circle at 50% 20%,rgba(0,255,145,.13),transparent 38%);
    }
    .public.attractSlideHow .publicState{
      animation:scoreboardAttractInstructionPulse 2.8s ease-in-out infinite;
    }
    .public.attractSlideChoose::before{
      opacity:.40;
      background:linear-gradient(90deg,rgba(0,80,255,.40),transparent 44%,transparent 56%,rgba(255,80,0,.42));
      animation:scoreboardAttractVsBreathe 3.3s ease-in-out infinite;
    }
    .public.attractSlideChoose .teamWordBlue,
    .public.attractSlideChoose .teamWordOrange{
      animation:scoreboardAttractTeamGlow 2.2s ease-in-out infinite;
    }
    .public.attractSlideChallenge::after{
      opacity:.36;
      background:repeating-radial-gradient(circle at 50% 50%,rgba(255,255,255,.00) 0 56px,rgba(255,255,255,.10) 58px,rgba(255,255,255,.00) 76px);
      animation:scoreboardAttractRings 4.8s linear infinite;
    }
    .public.attractSlideInfo .publicTimer{
      animation:scoreboardAttractInfoFloat 5.4s ease-in-out infinite;
    }
    .public.attractSlidesActive .publicState{
      position:static;
      transform:none;
      order:1;
      width:100%;
      max-width:min(94vw,1500px);
      font-size:clamp(22px,3vw,48px);
      line-height:1.06;
      letter-spacing:.13em;
      text-align:center;
      text-wrap:balance;
      overflow-wrap:normal;
      word-break:normal;
    }
    .public.attractSlidesActive .publicTimer{
      position:static;
      order:2;
      width:100%;
      max-width:min(94vw,1500px);
      font-size:clamp(58px,9.8vw,170px);
      line-height:.88;
      letter-spacing:-.015em;
      text-align:center;
      text-wrap:balance;
      overflow-wrap:normal;
      word-break:normal;
      animation:scoreboardAttractTimer 9s ease-in-out infinite;
    }
    .public.attractSlidesActive .publicTimer.fifaTitleGradient{
      background:linear-gradient(96deg,#0050FF 0%,#00A9FF 26%,#8A2BE2 52%,#FF3D6E 76%,#FFC74A 100%);
      -webkit-background-clip:text;
      background-clip:text;
      color:transparent;
      -webkit-text-fill-color:transparent;
      text-shadow:none;
      filter:drop-shadow(0 0 12px rgba(0,80,255,.40)) drop-shadow(0 0 22px rgba(255,90,40,.26));
    }
    .public.attractSlidesActive .publicMeta{
      position:static;
      transform:none;
      order:3;
      width:100%;
      max-width:min(92vw,1400px);
      font-size:clamp(18px,2.35vw,36px);
      line-height:1.10;
      letter-spacing:.09em;
      text-align:center;
      text-wrap:balance;
      overflow-wrap:normal;
      word-break:normal;
    }
    .public.attractSlidesActive.howToPlaySlideActive .publicTop{
      gap:clamp(16px,2.4vh,34px);
      padding:clamp(32px,4.8vh,72px) clamp(22px,4vw,72px);
    }
    .public.attractSlidesActive.howToPlaySlideActive .publicTimer{
      font-size:clamp(58px,8.6vw,144px);
      line-height:.92;
    }
    .public.attractSlidesActive.howToPlaySlideActive .publicState{
      font-size:clamp(32px,4.4vw,72px);
      line-height:1.08;
    }
    .public.attractSlidesActive.howToPlaySlideActive .publicMeta{
      font-size:clamp(24px,3.0vw,46px);
      line-height:1.10;
    }
    .public.attractSlidesActive.longMainText .publicTimer{
      font-size:clamp(48px,7.8vw,135px);
      line-height:.92;
      letter-spacing:.01em;
    }
    .public.attractSlidesActive.veryLongMainText .publicTimer{
      font-size:clamp(40px,6.6vw,112px);
      line-height:.96;
      letter-spacing:.025em;
    }
    .public.attractSlidesActive.longStateText .publicState{
      font-size:clamp(20px,2.5vw,40px);
      line-height:1.10;
      letter-spacing:.10em;
    }
    .public.attractSlidesActive.longMetaText .publicMeta{
      font-size:clamp(16px,2.0vw,30px);
      line-height:1.12;
      letter-spacing:.075em;
    }
    .teamWordBlue{
      color:#74b2ff;
      text-shadow:0 0 14px rgba(0,80,255,.62),0 0 34px rgba(0,80,255,.38);
      font-weight:1000;
    }
    .teamWordOrange{
      color:#ffb07b;
      text-shadow:0 0 14px rgba(255,80,0,.62),0 0 34px rgba(255,80,0,.38);
      font-weight:1000;
    }

    /* SCOREBOARD START SOON */
    .public.startSoonActive{
      grid-template-rows:1fr;
    }
    .public.startSoonActive .publicScores,
    .public.startSoonActive .publicInfoBar{
      display:none;
    }
    .public.startSoonActive .publicTop{
      min-height:100%;
      border-radius:0;
      border-color:transparent;
      box-shadow:none;
      display:flex;
      flex-direction:column;
      align-items:center;
      justify-content:center;
      gap:clamp(10px,1.8vh,22px);
      padding:clamp(18px,2.8vh,40px) clamp(16px,3.8vw,56px);
      background:radial-gradient(circle at 50% 14%,rgba(0,255,128,.22),transparent 36%),linear-gradient(180deg,#061325,#01050c);
    }
    .public.startSoonActive .publicState{
      position:static;
      transform:none;
      font-size:clamp(17px,2.0vw,28px);
      letter-spacing:.12em;
    }
    .public.startSoonActive .publicTimer{
      position:static;
      font-size:clamp(50px,7.1vw,108px);
      line-height:.94;
      letter-spacing:-.01em;
    }
    .public.startSoonActive .publicMeta{
      position:static;
      transform:none;
      font-size:clamp(14px,1.55vw,22px);
      letter-spacing:.10em;
    }

    /* SCOREBOARD LAST TEN */
    .public.lastTenActive{
      --lastColor:#982ABC;
      --lastDur:1000ms;
    }
    .public.liveMinimalActive.lastTenActive{
      background:
        radial-gradient(circle at 50% 16%,color-mix(in srgb,var(--lastColor) 70%, transparent),transparent 34%),
        linear-gradient(180deg,#050814,#070012 68%,#000);
      animation:scoreboardLastTenBackground var(--lastDur) cubic-bezier(.2,.9,.2,1) infinite;
    }
    .public.liveMinimalActive.lastTenActive::before{
      opacity:.38;
      background:conic-gradient(from 0deg,#ff0040,#ff9a00,#ffff00,#00ff90,#00d2ff,#7a5cff,#ff00d2,#ff0040);
      filter:blur(34px);
      animation:scoreboardFinal30RainbowSpin .85s linear infinite;
    }
    .public.liveMinimalActive.lastTenActive .publicTimer{
      text-shadow:0 0 30px var(--lastColor),0 0 100px rgba(255,255,255,.26);
      animation:scoreboardLastTenTimer var(--lastDur) cubic-bezier(.16,.9,.18,1) infinite;
    }
    .public.liveMinimalActive.lastTenActive .publicTeam{
      animation:scoreboardLastTenCard var(--lastDur) cubic-bezier(.2,.9,.2,1) infinite;
    }
    .public.liveMinimalActive.lastTenActive .publicTeam .num{
      animation:scoreboardLastTenScore var(--lastDur) cubic-bezier(.2,.9,.2,1) infinite;
    }

    /* SCOREBOARD EXIT CONTROLS */

    /* SCOREBOARD MOBILE */
    @media(max-width:1100px){
      .studioGrid{grid-template-columns:1fr 1fr}
      .span3{grid-column:span 2}
      .grid{grid-template-columns:1fr}
      .scoreRow{grid-template-columns:1fr}
      .score{font-size:96px}
      .timer{font-size:72px}
      .top{align-items:flex-start;flex-direction:column}
      .tabs{justify-content:flex-start}
    }
    @media(max-width:740px){
      .studioGrid{grid-template-columns:1fr}
      .span2,.span3{grid-column:span 1}
      .actions{grid-template-columns:1fr}
      .wide,.resumeBig{grid-column:span 1}
      .rangeRow{grid-template-columns:1fr 78px 56px}
      .public{
        padding:8px;
        row-gap:8px;
      }
      .public.active{
        grid-template-rows:minmax(110px,20vh) minmax(0,1fr) auto;
      }
      .publicScores{
        grid-template-columns:1fr;
      }
      .publicTeam .num{
        font-size:clamp(98px,28vw,220px);
        min-width:0;
        width:100%;
        text-align:center;
      }
      .public.attractSlidesActive .publicState{
        font-size:clamp(18px,4.6vw,28px);
      }
      .public.attractSlidesActive .publicTimer{
        font-size:clamp(56px,14.5vw,110px);
      }
      .public.attractSlidesActive .publicMeta{
        font-size:clamp(14px,3.3vw,22px);
      }
      .public.attractSlidesActive.howToPlaySlideActive .publicState{
        font-size:clamp(25px,5.6vw,40px);
      }
      .public.attractSlidesActive.howToPlaySlideActive .publicMeta{
        font-size:clamp(16px,3.8vw,26px);
      }
      .public.startSoonActive .publicTimer{
        font-size:clamp(40px,9.6vw,72px);
      }
      body.routeScoreboard .public.liveMinimalActive .publicTimer{
        font-size:clamp(72px,18vw,140px);
      }
      body.routeScoreboard .public.liveMinimalActive .publicTeam .num{
        font-size:clamp(130px,34vw,300px);
      }
    }

    /* SCOREBOARD KEYFRAMES */
    @keyframes scoreboardHalo{to{transform:rotate(360deg)}}
    @keyframes scoreboardSheen{
      0%,56%{transform:translateX(-135%);opacity:0}
      70%{opacity:.55}
      100%{transform:translateX(135%);opacity:0}
    }
    @keyframes scoreboardTopFloat{
      0%,100%{transform:translateY(0)}
      50%{transform:translateY(-2px)}
    }
    @keyframes scoreboardTimerFloat{
      0%,100%{transform:scale(1)}
      50%{transform:scale(1.010)}
    }
    @keyframes scoreboardTimerTick{
      0%{transform:scale(.94);filter:brightness(1.6)}
      34%{transform:scale(1.06);filter:brightness(1.24)}
      100%{transform:scale(1);filter:brightness(1)}
    }
    @keyframes scoreboardCountdownNumberPop{
      0%{transform:scale(.72);opacity:.12;filter:blur(5px) brightness(2.2)}
      12%{transform:scale(1.18);opacity:1;filter:blur(0) brightness(1.65)}
      28%{transform:scale(.98);opacity:.98;filter:brightness(1.25)}
      62%{transform:scale(1.035);opacity:.92}
      100%{transform:scale(.92);opacity:.62;filter:brightness(.95)}
    }
    @keyframes scoreboardCountdownGlowShift{
      0%,100%{text-shadow:0 0 42px var(--countColor,#FF0000),0 0 110px rgba(255,255,255,.42),0 22px 100px rgba(0,0,0,.78)}
      50%{text-shadow:0 0 62px rgba(255,90,90,.62),0 0 132px rgba(255,255,255,.46),0 22px 110px rgba(0,0,0,.82)}
    }
    @keyframes scoreboardCountdownSpinDrama{
      from{transform:translate3d(-50%,-50%,0) rotate(0deg) scale(1.05)}
      to{transform:translate3d(-50%,-50%,0) rotate(360deg) scale(1.05)}
    }
    @keyframes scoreboardCountdownRingPop{
      0%{transform:translate3d(-50%,-50%,0) scale(.42);opacity:0}
      16%{opacity:.95}
      100%{transform:translate3d(-50%,-50%,0) scale(1.55);opacity:0}
    }
    @keyframes scoreboardCardFloat{
      0%,100%{transform:translateY(0)}
      50%{transform:translateY(-3px)}
    }
    @keyframes scoreboardOrbital{to{transform:rotate(360deg)}}
    @keyframes scoreboardNumberPulse{
      0%,100%{transform:scale(1)}
      50%{transform:scale(1.015)}
    }
    @keyframes scoreboardWinnerText{
      0%,100%{opacity:.14;transform:translate(-50%,-50%) scale(.985)}
      50%{opacity:1;transform:translate(-50%,-50%) scale(1.03)}
    }
    @keyframes scoreboardGoalRing{
      0%{opacity:.92;transform:translate(-50%,-50%) scale(.70)}
      100%{opacity:0;transform:translate(-50%,-50%) scale(1.30)}
    }
    @keyframes scoreboardGoalWinnerPunch{
      0%{opacity:0;transform:translate(-50%,-50%) scale(1.8)}
      36%{opacity:1;transform:translate(-50%,-50%) scale(.96)}
      100%{opacity:1;transform:translate(-50%,-50%) scale(1)}
    }
    @keyframes scoreboardFinalTitle{
      0%,100%{transform:scale(1)}
      50%{transform:scale(1.02)}
    }
    @keyframes scoreboardAttractTimer{
      0%,100%{filter:brightness(1)}
      50%{filter:brightness(1.08)}
    }
    @keyframes scoreboardReadyPulse{
      0%,100%{filter:brightness(1);transform:scale(1)}
      50%{filter:brightness(1.16);transform:scale(1.015)}
    }
    @keyframes scoreboardLastTenBackground{
      0%,100%{filter:brightness(1) saturate(1.02)}
      20%{filter:brightness(1.22) saturate(1.24)}
      55%{filter:brightness(.90) saturate(.92)}
    }
    @keyframes scoreboardLastTenTimer{
      0%{transform:scale(.92);filter:brightness(1.7)}
      20%{transform:scale(1.10);filter:brightness(1.40)}
      100%{transform:scale(1);filter:brightness(1)}
    }
    @keyframes scoreboardLastTenCard{
      0%,100%{filter:brightness(1)}
      20%{filter:brightness(1.26)}
      55%{filter:brightness(.90)}
    }
    @keyframes scoreboardLastTenScore{
      0%,100%{transform:scale(1)}
      20%{transform:scale(1.05)}
      55%{transform:scale(.99)}
    }
    @keyframes scoreboardCountdownScreenPulse{
      0%{filter:brightness(.92) saturate(1.10)}
      18%{filter:brightness(1.24) saturate(1.48)}
      52%{filter:brightness(1.03) saturate(1.22)}
      100%{filter:brightness(.88) saturate(1.04)}
    }
    @keyframes scoreboardGameOverAlarm{
      0%{filter:brightness(.92) saturate(1.20)}
      100%{filter:brightness(1.38) saturate(1.70)}
    }
    @keyframes scoreboardGameOverScan{
      from{transform:translateX(-8%)}
      to{transform:translateX(8%)}
    }
    @keyframes scoreboardGameOverRing{
      0%{transform:scale(.78);opacity:.18}
      35%{opacity:.52}
      100%{transform:scale(1.20);opacity:.06}
    }
    @keyframes scoreboardGameOverTextPunch{
      0%{transform:scale(.99);filter:brightness(1)}
      100%{transform:scale(1.04);filter:brightness(1.32)}
    }
    @keyframes scoreboardGameOverTitlePunch{
      0%{transform:scale(.96);filter:brightness(1)}
      100%{transform:scale(1.055);filter:brightness(1.45)}
    }
    @keyframes scoreboardAttractAurora{
      from{transform:rotate(0deg) scale(1.08)}
      to{transform:rotate(360deg) scale(1.08)}
    }
    @keyframes scoreboardAttractSheen{
      0%,52%{transform:translateX(-140%) skewX(-12deg);opacity:0}
      68%{opacity:.58}
      100%{transform:translateX(140%) skewX(-12deg);opacity:0}
    }
    @keyframes scoreboardAttractFestivalPulse{
      0%,100%{filter:drop-shadow(0 0 12px rgba(0,80,255,.40)) drop-shadow(0 0 22px rgba(255,90,40,.26)) brightness(1)}
      50%{filter:drop-shadow(0 0 22px rgba(0,210,255,.58)) drop-shadow(0 0 38px rgba(255,80,0,.42)) brightness(1.13)}
    }
    @keyframes scoreboardAttractInstructionPulse{
      0%,100%{transform:scale(1);filter:brightness(1)}
      50%{transform:scale(1.025);filter:brightness(1.18)}
    }
    @keyframes scoreboardAttractVsBreathe{
      0%,100%{filter:brightness(.88) saturate(1.0)}
      50%{filter:brightness(1.26) saturate(1.34)}
    }
    @keyframes scoreboardAttractTeamGlow{
      0%,100%{filter:brightness(1)}
      50%{filter:brightness(1.26)}
    }
    @keyframes scoreboardAttractRings{
      from{transform:scale(.92);opacity:.20}
      to{transform:scale(1.18);opacity:.36}
    }
    @keyframes scoreboardAttractInfoFloat{
      0%,100%{transform:translateY(0) scale(1)}
      50%{transform:translateY(-6px) scale(1.015)}
    }
    @keyframes scoreboardFinal30RainbowSpin{
      from{transform:rotate(0deg) scale(1.18)}
      to{transform:rotate(360deg) scale(1.18)}
    }
    @keyframes scoreboardCardPop{
      0%{transform:scale(1)}
      18%{transform:scale(1.045)}
      100%{transform:scale(1)}
    }
    @keyframes scoreboardNumberPop{
      0%{transform:scale(1)}
      22%{transform:scale(1.12)}
      100%{transform:scale(1)}
    }
    @keyframes scoreboardScoreRoll{
      0%{transform:translateY(48%) scale(.88);opacity:0}
      55%{transform:translateY(-8%) scale(1.05);opacity:1}
      100%{transform:translateY(0) scale(1)}
    }
  </style>
</head>
<body>
<div class="app">
  <div class="top">
    <div class="brand"><div class="logo">FS</div><div><h1>FIFA Robot Soccer Controller</h1><div class="sub">Default view: full setup and animation tuning. Switch to Operator Mode for game-time controls.</div></div></div>
    <div class="tabs">
      <button class="tab active" id="setupTab" onclick="showSetup()">Setup & Tuning</button>
      <button class="tab" id="operatorTab" onclick="showOperator()">Operator Mode</button>
      <button class="tab" id="diagTab" onclick="showDiagnostics()">Diagnostics</button>
      <button class="tab score" onclick="showPublic()">Audience Scoreboard</button>
      <div class="connPill" id="connPill">Connecting</div>
    </div>
  </div>

  <div id="setupMode" class="mode active">
    <div class="studioGrid">
      <div class="card pad span3">
        <div class="scoreRow">
          <div class="team blue"><div class="teamName" id="setupBlueName">Blue Bots</div><div class="score" id="blueScoreSetup">0</div></div>
          <div class="timerBox"><div class="timer" id="timerSetup">1:30</div><div class="state" id="stateChipSetup">Standby</div><div class="sub" id="halfLabelSetup">First Half</div></div>
          <div class="team orange"><div class="teamName" id="setupOrangeName">Orange Bots</div><div class="score" id="orangeScoreSetup">0</div></div>
        </div>
      </div>

      <div class="card pad">
        <div class="controlTitle"><h2>Match & Output</h2><div class="badge" id="ipBadgeSetup">IP ---</div></div>
        <div class="sectionNote">These are safe demo controls. LED output remains hard-capped at 50% in firmware.</div>
        <div class="field"><label>Match Length</label><div class="rangeRow"><input type="range" id="matchSecs" min="10" max="600" step="5"><input class="num" type="number" id="matchSecsN" min="10" max="600" step="5"><button class="resetBtn" onclick="resetOne('matchSecs')">Reset</button></div></div>
        <div class="field"><label>LED Brightness Percent</label><div class="rangeRow"><input type="range" id="ledBrightness" min="1" max="50"><input class="num" type="number" id="ledBrightnessN" min="1" max="50"><button class="resetBtn" onclick="resetOne('ledBrightness')">Reset</button></div></div>
        <div class="field"><label>Audio Volume Percent</label><div class="rangeRow"><input type="range" id="audioVolume" min="0" max="100"><input class="num" type="number" id="audioVolumeN" min="0" max="100"><button class="resetBtn" onclick="resetOne('audioVolume')">Reset</button></div></div>
        <div class="applyRow"><button class="apply" onclick="applyOutput()">Apply Output</button><button class="test" onclick="copyRecipe()">Copy Settings Recipe</button></div>
      </div>

      <div class="card pad span3">
        <div class="controlTitle"><h2>Audio Show Flow</h2><div class="badge">track windows + flow test</div></div>
        <div class="sectionNote">Start second uses silent pre-roll (module volume is muted until start time). This is not true seek on DY-HV20T.</div>
        <div class="miniGrid">
          <div class="metric"><div class="k">Audio State</div><div class="v" id="audioFlowState">---</div></div>
          <div class="metric"><div class="k">Current Track</div><div class="v" id="audioFlowTrack">---</div></div>
          <div class="metric"><div class="k">Progress</div><div class="v" id="audioFlowProgress">---</div></div>
          <div class="metric"><div class="k">Remaining</div><div class="v" id="audioFlowRemaining">---</div></div>
          <div class="metric"><div class="k">Pre-roll Mute</div><div class="v" id="audioFlowPreroll">---</div></div>
        </div>
        <div class="smallActions">
          <button onclick="audioFlowAction('downtime')">Start Downtime Idle</button>
          <button onclick="audioFlowAction('start_game_flow')">Start Game Flow</button>
          <button onclick="audioFlowAction('goal')">Test Goal Sound</button>
          <button onclick="audioFlowAction('winner')">Test Winner Celebration</button>
          <button onclick="audioFlowAction('stop')">Stop Audio</button>
        </div>
        <div class="sectionNote">Audio quality depends on MP3 files loaded on the SD card.</div>
        <div id="audioWindowRows" class="miniGrid"></div>
        <div class="applyRow">
          <button class="apply" onclick="saveAudioWindows()">Save Audio Timing</button>
          <button class="test" onclick="resetAudioWindows()">Reset Audio Timing Defaults</button>
        </div>
      </div>

      <div class="card pad span2">
        <div class="controlTitle"><h2>Event Branding & Teams</h2><div class="badge">audience story text</div></div>
        <div class="sectionNote">These fields drive the public screen for attract, pre-game, in-game branding, and goal/finale team naming.</div>
        <div class="miniGrid">
          <div class="field"><label>Event Title</label><input class="textBox" id="eventTitleCfg" maxlength="54" placeholder="2026 FIFA FAN FESTIVAL"></div>
          <div class="field"><label>Event Subtitle</label><input class="textBox" id="eventSubtitleCfg" maxlength="64" placeholder="Robot Soccer Arena"></div>
          <div class="field span2"><label>Presented By Line</label><input class="textBox" id="presentedByCfg" maxlength="84" placeholder="Presented by Humber Polytechnic and Lord of Robots"></div>
          <div class="field"><label>Show Sponsor Line</label><select class="select" id="showPresentedCfg"><option value="1">Show</option><option value="0">Hide</option></select></div>
          <div class="field"><label>Show Branding During Live Match</label><select class="select" id="showLiveBrandCfg"><option value="1">Show</option><option value="0">Hide</option></select></div>
          <div class="field span2"><label>Attract Messages (use | between messages)</label><input class="textBox" id="attractMessagesCfg" maxlength="240" placeholder="Choose your bot|Blue Bots vs Orange Bots|2 vs 2 robot showdown"></div>
          <div class="field"><label>Final Warning Audience Text</label><input class="textBox" id="lastTenTextCfg" maxlength="24" placeholder="FINAL 10"></div>
          <div class="field"><label>Enable Custom Team Names</label><select class="select" id="customNamesEnabledCfg"><option value="0">Use Defaults</option><option value="1">Enable Custom</option></select></div>
          <div class="field"><label>Blue Team Display Name</label><input class="textBox" id="blueTeamNameCfg" maxlength="24" placeholder="Blue Bots"></div>
          <div class="field"><label>Orange Team Display Name</label><input class="textBox" id="orangeTeamNameCfg" maxlength="24" placeholder="Orange Bots"></div>
        </div>
        <div class="applyRow"><button class="apply" onclick="applyBrandingConfig();applyTeamIdentityConfig()">Apply Branding & Team Identity</button><button class="test" onclick="previewAudienceAttract()">Preview Attract Screen</button><button class="test" onclick="previewAudiencePost()">Preview Post-Match Screen</button></div>
      </div>

      <div class="card pad span2">
        <div class="controlTitle"><h2>Animation Colors</h2><div class="badge">live color pickers</div></div>
        <div class="miniGrid">
          <div class="field"><label>Blue Team / Blue Goal</label><input class="color" type="color" id="blueColor"><button class="resetBtn" onclick="resetOne('blueColor')">Reset</button></div>
          <div class="field"><label>Orange Team / Orange Goal</label><input class="color" type="color" id="orangeColor"><button class="resetBtn" onclick="resetOne('orangeColor')">Reset</button></div>
          <div class="field"><label>Countdown Start Color</label><input class="color" type="color" id="hex1Color"><button class="resetBtn" onclick="resetOne('hex1Color')">Reset</button></div>
          <div class="field"><label>Countdown Mid Color</label><input class="color" type="color" id="hex2Color"><button class="resetBtn" onclick="resetOne('hex2Color')">Reset</button></div>
          <div class="field"><label>Countdown Final Color</label><input class="color" type="color" id="hex3Color"><button class="resetBtn" onclick="resetOne('hex3Color')">Reset</button></div>
          <div class="field"><label>Countdown Field Color</label><input class="color" type="color" id="fieldColor"><button class="resetBtn" onclick="resetOne('fieldColor')">Reset</button></div>
          <div class="field"><label>Game Start Flash</label><input class="color" type="color" id="startColor"><button class="resetBtn" onclick="resetOne('startColor')">Reset</button></div>
          <div class="field"><label>Final 30 Warning Pulse</label><input class="color" type="color" id="lastTenColor"><button class="resetBtn" onclick="resetOne('lastTenColor')">Reset</button></div>
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
        <div class="field"><label>Finale Intensity Profile</label><select class="select" id="finaleIntensityCfg"><option value="1">Showtime (Default)</option><option value="0">Clean</option><option value="2">Maximum Safe</option></select></div>
        <div class="applyRow"><button class="apply" onclick="applyFinale();applyShowFlowConfig()">Apply Finale</button><button class="test" onclick="api('/api/end')">Test End Match</button><button class="test" onclick="resetGroup('finale')">Reset Finale</button></div>
      </div>

      <div class="card pad">
        <div class="controlTitle"><h2>Final 30 Seconds</h2><div class="badge">final warning phase</div></div>
        <div class="field"><label>Enabled</label><select class="select" id="lastTenEnabled"><option value="1">Enabled</option><option value="0">Disabled</option></select></div>
        <div class="field"><label>Danger Mode Style</label><select class="select" id="lastTenDramaticCfg"><option value="1">Dramatic</option><option value="0">Simple</option></select></div>
        <div class="field"><label>Pulse / Blink Period Ms</label><div class="rangeRow"><input type="range" id="lastTenPeriod" min="250" max="2000" step="25"><input class="num" type="number" id="lastTenPeriodN"><button class="resetBtn" onclick="resetOne('lastTenPeriod')">Reset</button></div></div>
        <div class="field"><label>LED Brightness Min / Max</label><div class="rangeRow"><input type="range" id="lastTenMin" min="0" max="255"><input class="num" type="number" id="lastTenMinN"><button class="resetBtn" onclick="resetOne('lastTenMin')">Reset</button></div><div class="rangeRow"><input type="range" id="lastTenMax" min="0" max="255"><input class="num" type="number" id="lastTenMaxN"><button class="resetBtn" onclick="resetOne('lastTenMax')">Reset</button></div></div>
        <div class="applyRow"><button class="apply" onclick="applyCountdown();applyShowFlowConfig()">Apply Final 30</button><button class="test" onclick="api('/api/test-last-ten')">Test Final 30</button></div>
      </div>

      <div class="card pad">
        <div class="controlTitle"><h2>Show Flow</h2><div class="badge">post-match behavior</div></div>
        <div class="field"><label>Match Format</label><select class="select" id="matchFormatCfg"><option value="single">Single Match</option><option value="two_half">Two Half Match</option></select></div>
        <div class="field"><label>Auto Return to Standby</label><select class="select" id="autoReturnCfg"><option value="1">Enabled</option><option value="0">Disabled</option></select></div>
        <div class="field"><label>Auto Return Delay (ms)</label><div class="rangeRow"><input type="range" id="autoReturnDelayCfg" min="5000" max="90000" step="500"><input class="num" type="number" id="autoReturnDelayCfgN" min="5000" max="90000" step="500"><button class="resetBtn" onclick="setPair('autoReturnDelayCfg',15000);applyShowFlowConfig()">Reset</button></div></div>
        <div class="field"><label>Post Match Minimum Hold (ms)</label><div class="rangeRow"><input type="range" id="postMatchHoldCfg" min="3000" max="30000" step="500"><input class="num" type="number" id="postMatchHoldCfgN" min="3000" max="30000" step="500"><button class="resetBtn" onclick="setPair('postMatchHoldCfg',8000);applyShowFlowConfig()">Reset</button></div></div>
        <div class="field"><label>Demo Lock Mode</label><select class="select" id="demoLockCfg"><option value="0">Off (Development)</option><option value="1">On (Event Day)</option></select></div>
        <div class="field"><label>Operator PIN UI Gate</label><select class="select" id="operatorPinEnabledCfg"><option value="0">Disabled</option><option value="1">Enabled (UI Gate)</option></select></div>
        <div class="applyRow"><button class="apply" onclick="applyShowFlowConfig()">Apply Show Flow</button><button class="test" onclick="previewAudiencePost()">Preview Post Match</button><button class="test" onclick="previewAudienceAuto()">Return to Auto Audience</button></div>
        <div class="applyRow"><button class="apply" onclick="saveSettings()">Save Settings</button><button class="test" onclick="loadSettings()">Load Saved Settings</button><button class="test" onclick="confirmDanger('Reset Saved Settings','Reset saved settings to defaults? This keeps the firmware running.','/api/settings/reset')">Reset Saved Settings</button></div>
        <div class="sectionNote" id="settingsStatus">Settings are live immediately. Use Save Settings to persist after reboot.</div>
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
          <button onclick="api('/api/test-countdown')">Test Countdown</button><button onclick="api('/api/fake-blue-goal')">Blue Goal</button><button onclick="api('/api/fake-orange-goal')">Orange Goal</button><button onclick="api('/api/test-last-ten')">Final 30</button><button onclick="api('/api/test-idle')">Idle</button><button onclick="api('/api/test-leds')">LED Self Test</button><button onclick="showPublic()">Scoreboard</button>
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
        <details class="diag"><summary>Audio diagnostics</summary><div class="smallActions"><button onclick="api('/api/play?track=1')">Idle 1</button><button onclick="api('/api/play?track=2')">Idle 2</button><button onclick="api('/api/play?track=3')">Idle 3</button><button onclick="api('/api/play?track=4')">Idle 4</button><button onclick="api('/api/play?track=5')">Idle 5</button><button onclick="api('/api/play?track=6')">Buildup</button><button onclick="api('/api/play?track=7')">Countdown</button><button onclick="api('/api/play?track=8')">Gameplay</button><button onclick="api('/api/play?track=9')">Goal</button><button onclick="api('/api/play?track=10')">Winner</button><button onclick="api('/api/stop-audio')">Stop</button></div></details>
      </div>
    </div>
  </div>

  <div id="operatorMode" class="mode">
    <div class="grid">
      <div class="card pad">
        <div class="scoreRow">
          <div class="team blue"><div class="teamName" id="opBlueName">Blue Bots</div><div class="score" id="blueScoreOp">0</div></div>
          <div class="timerBox"><div class="timer" id="timerOp">1:30</div><div class="state" id="stateChipOp">Standby</div><div class="sub" id="halfLabelOp">First Half</div></div>
          <div class="team orange"><div class="teamName" id="opOrangeName">Orange Bots</div><div class="score" id="orangeScoreOp">0</div></div>
        </div>
      </div>
      <div class="card pad">
        <div class="controlTitle"><h2>Show Control</h2><div class="badge" id="ipBadgeOp">IP ---</div></div>
        <div class="sectionNote">Primary controls for live demos. Destructive actions are confirmation-protected.</div>
        <div class="sectionNote">Audience display link: <span id="scoreboardHint">/scoreboard</span></div>
        <div class="sectionNote" id="demoLockStateOp">Demo Lock Off</div>
        <div class="smallActions">
          <button onclick="previewAudienceAttract()">Standby Attract</button>
          <button onclick="previewAudienceAuto()">Audience Auto</button>
          <button onclick="showPublic()">Open Fullscreen Scoreboard</button>
          <button onclick="runReadabilityTest()">Readability Test</button>
        </div>
        <div class="quickOps">
          <button onclick="apiScoreBlue(1)">Blue +1</button>
          <button onclick="apiScoreBlue(-1)">Blue -1</button>
          <button onclick="apiScoreOrange(1)">Orange +1</button>
          <button onclick="apiScoreOrange(-1)">Orange -1</button>
        </div>
        <div class="actions">
          <button class="resume" onclick="api('/api/pre-game')">Pre-game</button>
          <button class="primary" onclick="startFirstHalf()">Start Match</button>
          <button class="primary" id="resumeSecondHalfBtn" onclick="api('/api/resume-second-half')">Resume 2nd Half</button>
          <button class="resume resumeBig" id="resumeGoalBtn" disabled onclick="api('/api/resume-after-goal')">Resume Match After Goal</button>
          <button class="dark" onclick="api('/api/standby')">Standby / Idle</button>
          <button class="dark" onclick="resetMatchNow()">Reset Match</button>
          <button class="blueBtn" onclick="api('/api/fake-blue-goal')">Blue Goal / Test</button>
          <button class="orangeBtn" onclick="api('/api/fake-orange-goal')">Orange Goal / Test</button>
          <button class="dark" onclick="confirmDanger('Replay Finale','Replay winner finale now? Use only between matches.','/api/replay-finale')">Replay Finale</button>
          <button class="danger wide" onclick="endMatchNow()">End Match</button>
        </div>
      </div>
      <div class="card pad"><div class="controlTitle"><h2>Match Setup</h2><div class="badge" id="audioBadgeOp">Audio ---</div></div><div class="miniGrid"><div class="metric"><div class="k">Blue Team</div><div class="v" id="readyBlueName">Blue Bots</div></div><div class="metric"><div class="k">Orange Team</div><div class="v" id="readyOrangeName">Orange Bots</div></div><div class="metric"><div class="k">Match Length</div><div class="v" id="readyMatchLength">90s</div></div><div class="metric"><div class="k">Match Format</div><div class="v" id="readyMatchFormat">Single</div></div><div class="metric"><div class="k">Audience State</div><div class="v" id="readyAudienceState">Auto</div></div><div class="metric"><div class="k">Sensor Health</div><div class="v" id="readySensorHealth">---</div></div><div class="metric"><div class="k">LED Mode</div><div class="v" id="ledModeOp">---</div></div><div class="metric"><div class="k">Audio Track</div><div class="v" id="trackInfoOp">0</div></div><div class="metric"><div class="k">Scoring</div><div class="v" id="scoringOp">---</div></div><div class="metric"><div class="k">Last Event</div><div class="v" id="lastEventOp">Ready</div></div><div class="metric"><div class="k">Rejected Singles</div><div class="v" id="rejectsOp">0</div></div></div></div>
      <div class="card pad"><div class="controlTitle"><h2>Event Log</h2><div class="badge">recent</div></div><div class="log" id="eventLogOp"></div></div>
    </div>
  </div>

  <div id="diagnosticsMode" class="mode">
    <div class="diagWrap">
      <div class="card pad">
        <div class="diagHead">
          <div class="controlTitle"><h2>Live Diagnostics</h2><div class="badge" id="diagConnBadge">Link ---</div></div>
          <div class="diagLive" id="diagLiveStamp">Waiting for updates</div>
        </div>
        <div class="miniGrid">
          <div class="metric"><div class="k">Match State</div><div class="v" id="diagState">---</div></div>
          <div class="metric"><div class="k">Remaining (s)</div><div class="v" id="diagRemain">---</div></div>
          <div class="metric"><div class="k">Current Half</div><div class="v" id="diagHalf">---</div></div>
          <div class="metric"><div class="k">LED Mode</div><div class="v" id="diagLedMode">---</div></div>
          <div class="metric"><div class="k">Audio State</div><div class="v" id="diagAudioState">---</div></div>
          <div class="metric"><div class="k">Current Track</div><div class="v" id="diagTrack">---</div></div>
          <div class="metric"><div class="k">Ball Link</div><div class="v" id="diagBallLink">---</div></div>
          <div class="metric"><div class="k">Uptime</div><div class="v" id="diagUptime">---</div></div>
          <div class="metric"><div class="k">Brightness</div><div class="v" id="diagBrightness">---</div></div>
          <div class="metric"><div class="k">Left Goal (A/B)</div><div class="v" id="diagLeftGoal">---</div></div>
          <div class="metric"><div class="k">Right Goal (A/B)</div><div class="v" id="diagRightGoal">---</div></div>
          <div class="metric"><div class="k">Rejected Singles</div><div class="v" id="diagRejects">---</div></div>
          <div class="metric"><div class="k">Interrupt Count</div><div class="v" id="diagInterrupts">---</div></div>
        </div>
      </div>

      <div class="card pad">
        <div class="controlTitle"><h2>Health Warnings</h2><div class="badge">auto checks</div></div>
        <div class="warnList" id="diagWarnings"></div>
      </div>

      <div class="card pad">
        <div class="controlTitle"><h2>Audio Checklist</h2><div class="badge">track map + test</div></div>
        <div class="sectionNote">Use this during rehearsal to validate SD card track order, loudness consistency, and cue clarity.</div>
        <div class="miniGrid">
          <div class="metric"><div class="k">Busy Pin</div><div class="v" id="diagBusyPin">---</div></div>
          <div class="metric"><div class="k">Current Track</div><div class="v" id="diagTrackNow">---</div></div>
          <div class="metric"><div class="k">Last Track</div><div class="v" id="diagLastTrack">---</div></div>
          <div class="metric"><div class="k">Volume</div><div class="v" id="diagAudioVolume">---</div></div>
        </div>
        <div class="sectionNote">Track labels + expected windows: 1-5 Idle (0-60s), 6 Buildup (0-126s), 7 Countdown (0-10s), 8 Gameplay BGM (0-120s), 9 Goal (0-5s), 10 Winner (0-12s).</div>
        <div class="smallActions">
          <button onclick="api('/api/play?track=1')">1 Idle 1</button>
          <button onclick="api('/api/play?track=2')">2 Idle 2</button>
          <button onclick="api('/api/play?track=3')">3 Idle 3</button>
          <button onclick="api('/api/play?track=4')">4 Idle 4</button>
          <button onclick="api('/api/play?track=5')">5 Idle 5</button>
          <button onclick="api('/api/play?track=6')">6 Buildup</button>
          <button onclick="api('/api/play?track=7')">7 Countdown</button>
          <button onclick="api('/api/play?track=8')">8 Gameplay</button>
          <button onclick="api('/api/play?track=9')">9 Goal</button>
          <button onclick="api('/api/play?track=10')">10 Winner</button>
          <button onclick="api('/api/stop-audio')">Stop Audio</button>
        </div>
      </div>

      <div class="card pad">
        <div class="controlTitle"><h2>Rehearsal Scenes</h2><div class="badge">20 foot check</div></div>
        <div class="smallActions">
          <button onclick="runRehearsalScene('attract')">Attract</button>
          <button onclick="runRehearsalScene('countdown')">Countdown</button>
          <button onclick="runRehearsalScene('goal_blue')">Blue Goal</button>
          <button onclick="runRehearsalScene('goal_orange')">Orange Goal</button>
          <button onclick="runRehearsalScene('final_ten')">Final 10</button>
          <button onclick="runRehearsalScene('win_blue')">Blue Win</button>
          <button onclick="runRehearsalScene('win_orange')">Orange Win</button>
          <button onclick="runRehearsalScene('win_tie')">Tie Finale</button>
          <button onclick="runReadabilityTest()">Readability Test</button>
          <button onclick="clearRehearsalScene()">Clear Preview</button>
        </div>
      </div>

      <div class="card pad">
        <div class="controlTitle"><h2>Recent Events</h2><div class="badge">timestamped</div></div>
        <div class="log" id="diagEvents"></div>
      </div>
    </div>
  </div>
</div>

<div class="public" id="publicMode">
  <div class="publicTop"><div class="goalRingEl"></div><div class="goalRingEl"></div><div class="publicState" id="publicState">Standby</div><div class="publicTimer" id="publicTimer">1:30</div><div class="publicMeta" id="publicMeta">First Half</div></div>
  <div class="publicScores"><div class="publicTeam blue" id="publicBlue"><div class="name" id="publicBlueName">Blue Bots</div><div class="num" id="publicBlueScore">0</div></div><div class="publicTeam orange" id="publicOrange"><div class="name" id="publicOrangeName">Orange Bots</div><div class="num" id="publicOrangeScore">0</div></div></div>
  <div class="publicInfoBar"><div class="publicTicker" id="publicTicker">Choose your bot</div><div class="publicBrandLine" id="publicBrandLine">Presented by Humber Polytechnic and Lord of Robots</div></div>
  <div class="publicWinner" id="publicWinner">Winner</div>
  <button class="exitPublic" onclick="hidePublic()">Exit Scoreboard</button>
</div>

<div class="confirmBackdrop" id="confirmBackdrop">
  <div class="confirmBox">
    <h3 class="confirmTitle" id="confirmTitle">Confirm Action</h3>
    <p class="confirmBody" id="confirmBody">Are you sure?</p>
    <div class="confirmActions">
      <button class="confirmBtn" onclick="closeConfirm()">Cancel</button>
      <button class="confirmBtn danger" id="confirmDangerBtn" onclick="runConfirmed()">Confirm</button>
    </div>
  </div>
</div>

<div class="confirmBackdrop" id="pinBackdrop">
  <div class="confirmBox">
    <h3 class="confirmTitle">Operator Access</h3>
    <p class="confirmBody">Enter operator PIN to unlock controls on this device.</p>
    <input class="textBox" id="operatorPinInput" inputmode="numeric" maxlength="12" placeholder="PIN">
    <p class="confirmBody" id="pinStatusText"></p>
    <div class="confirmActions">
      <button class="confirmBtn" onclick="verifyOperatorPin()">Unlock</button>
    </div>
  </div>
</div>

<script>
const DEFAULTS={matchSecs:120,blueColor:'#0050FF',orangeColor:'#FF5000',hex1Color:'#00D2FF',hex2Color:'#FF00D2',hex3Color:'#FF1800',fieldColor:'#FF1800',startColor:'#00FF00',lastTenColor:'#FFFFFF',countdownMode:'heartbeat',hexStyle:'team_fill_comet',countStep:1000,fieldPeriod:1000,hexRotation:1000,hexTrail:18,fieldMin:15,fieldMax:190,hexBright:255,goalMode:'straight',goalDuration:4200,goalTravel:950,goalInterval:260,goalEmit:2600,goalPop:700,autoGoalResumeEnabled:1,autoGoalResumeMs:5000,finaleDuration:13500,winnerSpeed:255,rainbowSpeed:18,ringSpacing:37,lastTenEnabled:1,lastTenPeriod:1000,lastTenMin:75,lastTenMax:135,idleSpeed:37,idleMin:10,idleMax:255,ledBrightness:25,audioVolume:80,eventTitleCfg:'2026 FIFA FAN FESTIVAL',eventSubtitleCfg:'Robot Soccer Arena',presentedByCfg:'Presented by Humber Polytechnic and Lord of Robots',attractMessagesCfg:'Choose your bot|Blue Bots vs Orange Bots|120 second robot soccer challenge|2 vs 2 robot showdown',lastTenTextCfg:'FINAL 30',customNamesEnabledCfg:0,blueTeamNameCfg:'Blue Bots',orangeTeamNameCfg:'Orange Bots',showPresentedCfg:1,showLiveBrandCfg:1,lastTenDramaticCfg:1,autoReturnCfg:1,autoReturnDelayCfg:15000,postMatchHoldCfg:8000,finaleIntensityCfg:1,matchFormatCfg:'single',demoLockCfg:0,operatorPinEnabledCfg:0,eventTitle:'2026 FIFA FAN FESTIVAL',eventSubtitle:'Robot Soccer Arena',presentedBy:'Presented by Humber Polytechnic and Lord of Robots',attractMessages:'Choose your bot|Blue Bots vs Orange Bots|120 second robot soccer challenge|2 vs 2 robot showdown',lastTenText:'FINAL 30',finaleIntensity:1,autoReturnDelayMs:15000};
/* Map endpoints here if your firmware routes ever change. */
const API_ENDPOINTS={
  preGame:'/api/pre-game',
  start:'/api/start',
  standby:'/api/standby',
  audienceMode:'/api/audience-mode',
  replayFinale:'/api/replay-finale',
  rehearsal:'/api/rehearsal',
  branding:'/api/branding',
  teamIdentity:'/api/team-identity',
  showFlow:'/api/show-flow',
  settingsSave:'/api/settings/save',
  settingsLoad:'/api/settings/load',
  settingsReset:'/api/settings/reset',
  operatorPinVerify:'/api/operator-pin-verify',
  reset:'/api/reset',
  end:'/api/end',
  scoreBlue:'/api/score-blue',
  scoreOrange:'/api/score-orange',
  fakeBlueGoal:'/api/fake-blue-goal',
  fakeOrangeGoal:'/api/fake-orange-goal',
  resumeAfterGoal:'/api/resume-after-goal',
  audioWindow:'/api/audio-window',
  audioWindowSave:'/api/audio-window-save',
  audioWindowReset:'/api/audio-window-reset',
  audioFlow:'/api/audio-flow',
  play:'/api/play',
  state:'/api/state'
};
const ROUTE_PATH=String(window.location.pathname||'/').toLowerCase();
const ROUTE_MODE=(ROUTE_PATH==='/scoreboard'||ROUTE_PATH==='/scoreboard/')?'scoreboard':'operator';
let latest={};
let scoreboardPinned=(ROUTE_MODE==='scoreboard') || (sessionStorage.getItem('scoreboardPinned')==='1');
let publicLastBlueScore=null;
let publicLastOrangeScore=null;
let publicLastTimerText=null;
let publicLastScene='auto';
let publicLastAttractSlideIndex=null;
let publicAttractTransitionTimer=null;
let publicGoalTeam='none';
let prevGoalActive=false;
let wasCountdownActive=false;
let countdownIntroUntilMs=0;
let goalRingTimeout=null;
let goalEntranceTimeout=null;
let timerTickTimeout=null;
let audioWindowsRendered=false;
let confirmActionPath='';
let lastGoodUpdateMs=0;
let consecutiveUpdateErrors=0;
let operatorUnlocked=false;
const CONNECTION_STALE_MS=1800;
const STANDBY_SLIDE_MS=7000;
const STANDBY_TITLE_HOLD_MS=13000;
// Pre-game now carries the "Get Ready" messaging. Keep countdown immediate.
const COUNTDOWN_INTRO_MS=0;
let sensorIssueSince={
  leftPartialMs:0,
  rightPartialMs:0,
  leftClearMs:0,
  rightClearMs:0
};
const PUBLIC_SCORE_ROLL_MS=860;
const PUBLIC_SCORE_POP_MS=1040;
const PUBLIC_TIMER_TICK_MS=620;
function animateScoreRoll(numId){const node=el(numId);if(!node)return;node.classList.remove('v423RollIn');void node.offsetWidth;node.classList.add('v423RollIn');setTimeout(()=>node.classList.remove('v423RollIn'),PUBLIC_SCORE_ROLL_MS)}
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
    timerTickTimeout=setTimeout(()=>node.classList.remove('timerTick'),PUBLIC_TIMER_TICK_MS);
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
  setTimeout(()=>node.classList.remove('scorePop'),PUBLIC_SCORE_POP_MS);
}
function el(id){return document.getElementById(id)}
function exists(id){return !!el(id)}
function fmt(ms){ms=Math.max(0,Number(ms||0));return String(Math.ceil(ms/1000))}
function normState(x){return String(x||'STANDBY').replace(/^STATE_/,'')}
function prettyState(x){const k=normState(x);const m={STANDBY:'Ready',PRE_GAME:'Pre-game',COUNTDOWN:'Starting',IN_GAME:'Live',GOAL_PAUSE:'Goal',HALFTIME:'Halftime',END_GAME:'Final'};return m[k]||k.replace(/_/g,' ').replace(/\b\w/g,c=>c.toUpperCase())}
function prettyMode(x){return String(x||'---').replace(/^LED_MODE_/,'').replace(/_/g,' ').replace(/\b\w/g,c=>c.toUpperCase())}
function esc(s){return String(s||'').replace(/[&<>]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;'}[c]))}
function safeText(v,fallback,maxLen){
  let t=String(v??'').replace(/\s+/g,' ').trim();
  if(!t)t=String(fallback||'');
  if(maxLen&&t.length>maxLen)t=t.slice(0,maxLen).trim();
  return t||String(fallback||'');
}
function colorizeTeamMentions(text){
  const escaped=esc(text).replace(/\n/g,'<br>');
  return escaped.replace(/\b(BLUE(?:\s+BOTS?)?|ORANGE(?:\s+BOTS?)?)\b/gi,m=>{
    const cls=/^BLUE/i.test(m)?'teamWordBlue':'teamWordOrange';
    return '<span class="'+cls+'">'+m+'</span>';
  });
}
function teamNameBlue(s){return safeText(s.blueTeamName||DEFAULTS.blueTeamName,DEFAULTS.blueTeamName,24)}
function teamNameOrange(s){return safeText(s.orangeTeamName||DEFAULTS.orangeTeamName,DEFAULTS.orangeTeamName,24)}
function attractMessages(s){
  const raw=safeText(s.attractMessages||DEFAULTS.attractMessages,DEFAULTS.attractMessages,240);
  const list=raw.split('|').map(x=>safeText(x,'',56)).filter(Boolean);
  return list.length?list:[DEFAULTS.eventSubtitle];
}
function compactSlideKey(v){
  return String(v||'').toUpperCase().replace(/[^A-Z0-9]+/g,' ').trim();
}
function normalizeSlideText(v,maxLen){
  return safeText(v,'',maxLen||64).toUpperCase();
}
function makeSlide(title,state,meta){
  const t=normalizeSlideText(title,64);
  const st=normalizeSlideText(state,64);
  let m=normalizeSlideText(meta,64);
  const tk=compactSlideKey(t);
  const sk=compactSlideKey(st);
  const mk=compactSlideKey(m);
  if(mk && (mk===tk || mk===sk)){
    m='';
  }
  return {title:t,state:st,meta:m};
}
function addSlideUnique(slides,seen,title,state,meta){
  const slide=makeSlide(title,state,meta);
  const key=compactSlideKey(slide.title+' '+slide.state+' '+slide.meta);
  if(!key || seen.has(key))return;
  seen.add(key);
  slides.push(slide);
}
function isDefaultOrDuplicateAttractMessage(msg,eventTitle,eventSubtitle){
  const k=compactSlideKey(msg);
  const titleK=compactSlideKey(eventTitle);
  const subtitleK=compactSlideKey(eventSubtitle);
  if(!k)return true;
  if(k===titleK || k===subtitleK)return true;
  if(k==='CHOOSE YOUR BOT')return true;
  if(k==='BLUE BOTS VS ORANGE BOTS')return true;
  if(k==='90 SECOND ROBOT SOCCER CHALLENGE')return true;
  if(k==='120 SECOND ROBOT SOCCER CHALLENGE')return true;
  if(k==='2 VS 2 ROBOT SHOWDOWN')return true;
  if(k==='2 VS 2 ROBOT SOCCER CHALLENGE')return true;
  if(k==='ROBOT SOCCER ARENA')return true;
  return false;
}
function standbySlides(s){
  const eventTitle=safeText(s.eventTitle||DEFAULTS.eventTitle,DEFAULTS.eventTitle,54);
  const eventSubtitle=safeText(s.eventSubtitle||DEFAULTS.eventSubtitle,DEFAULTS.eventSubtitle,64);
  const customMsgs=attractMessages(s);
  const slides=[];
  const seen=new Set();

  addSlideUnique(slides,seen,eventTitle,eventSubtitle,'');
  addSlideUnique(slides,seen,'HOW TO PLAY','SCORE GOALS BEFORE TIME RUNS OUT','BLUE AND ORANGE COMPETE FOR GOALS');
  addSlideUnique(slides,seen,'CHOOSE YOUR BOT','BLUE BOTS\nVS.\nORANGE BOTS','PICK A SIDE AND GET READY');
  addSlideUnique(slides,seen,'120 SECOND CHALLENGE','FAST 2 VS 2 MATCH','TEAM WITH THE HIGHEST SCORE WINS');

  customMsgs.forEach(msg=>{
    if(isDefaultOrDuplicateAttractMessage(msg,eventTitle,eventSubtitle)){
      return;
    }
    if(/\bBLUE\s+BOTS?\s+VS\.?\s+ORANGE\s+BOTS?\b/i.test(msg)){
      addSlideUnique(slides,seen,'CHOOSE YOUR BOT','BLUE BOTS\nVS.\nORANGE BOTS','PICK A SIDE AND GET READY');
    }else{
      addSlideUnique(slides,seen,msg,'MATCH INFO','NEXT PLAYERS READY');
    }
  });

  return slides.length?slides:[makeSlide(eventTitle,eventSubtitle,'')];
}
function pickStandbySlide(uptimeMs,slides){
  if(!Array.isArray(slides)||!slides.length){
    return {index:0,slide:{title:'',state:'',meta:''}};
  }
  if(slides.length===1){
    return {index:0,slide:slides[0]};
  }
  const loopMs=STANDBY_TITLE_HOLD_MS + ((slides.length-1) * STANDBY_SLIDE_MS);
  let t=Math.max(0,Number(uptimeMs||0)) % Math.max(1,loopMs);
  if(t<STANDBY_TITLE_HOLD_MS){
    return {index:0,slide:slides[0]};
  }
  t-=STANDBY_TITLE_HOLD_MS;
  const idx=1+Math.floor(t/STANDBY_SLIDE_MS);
  const bounded=Math.min(slides.length-1,Math.max(1,idx));
  return {index:bounded,slide:slides[bounded]};
}
function audienceOverrideName(s){
  const x=String(s.audienceOverrideMode||'auto').toLowerCase();
  if(x==='attract')return 'Attract';
  if(x==='post_match'||x==='post')return 'Post Match';
  return 'Auto';
}
async function api(path){
  const keepScoreboardOpen=
    scoreboardPinned ||
    ROUTE_MODE==='scoreboard' ||
    el('publicMode')?.classList.contains('active');

  if(keepScoreboardOpen){
    setScoreboardPinned(true);
    forceScoreboardVisible();
  }

  try{
    if(path.includes(API_ENDPOINTS.reset)||path.includes(API_ENDPOINTS.start)||path.includes(API_ENDPOINTS.preGame)||path.includes(API_ENDPOINTS.end)){
      publicGoalTeam='none';
      publicLastBlueScore=null;
      publicLastOrangeScore=null;
      publicLastTimerText=null;
      prevGoalActive=false;
    }

    const r=await fetch(path,{method:'POST',cache:'no-store'});
    if(!r.ok)throw new Error('http '+r.status);

    forceScoreboardVisible();
    await update();
    forceScoreboardVisible();

    return true;
  }catch(e){
    console.warn('API failed',path,e);
    consecutiveUpdateErrors++;
    setConnectionVisual(false,'Command failed');
    forceScoreboardVisible();
    return false;
  }
}
function enc(v){return encodeURIComponent(v)}
function halfText(s){
  const gs=normState(s.gameState);
  const fmtMode=String(s.matchFormat||'single').toLowerCase();
  if(gs==='PRE_GAME')return 'Pre-game';
  if(gs==='HALFTIME')return 'Halftime';
  if(gs==='END_GAME')return 'Final';
  if(fmtMode!=='two_half'){
    if(gs==='COUNTDOWN')return 'Match Starting';
    return 'Single Match';
  }
  if(gs==='COUNTDOWN')return s.currentHalf===2?'Second Half Starting':'First Half Starting';
  return s.currentHalf===2?'Second Half':'First Half';
}
let publicControlsTimer=0;
function pulsePublicControls(){
  const board=el('publicMode');
  if(!board||!board.classList.contains('active'))return;
  board.classList.add('controlsVisible');
  clearTimeout(publicControlsTimer);
  publicControlsTimer=setTimeout(()=>board.classList.remove('controlsVisible'),3000);
}

function syncScoreboardPinnedBody(){
  const pinned = scoreboardPinned || ROUTE_MODE === 'scoreboard';
  document.body.classList.toggle('scoreboardPinnedOpen', pinned);

  if(pinned){
    const board = el('publicMode');
    if(board){
      board.classList.add('active');
    }
  }
}

function setScoreboardPinned(value){
  scoreboardPinned = Boolean(value) || ROUTE_MODE === 'scoreboard';

  try{
    if(scoreboardPinned){
      sessionStorage.setItem('scoreboardPinned','1');
    }else{
      sessionStorage.removeItem('scoreboardPinned');
    }
  }catch(e){}

  syncScoreboardPinnedBody();
}

function forceScoreboardVisible(){
  if(scoreboardPinned || ROUTE_MODE === 'scoreboard'){
    syncScoreboardPinnedBody();
  }
}

function setConnectionVisual(ok,text){
  const pill=el('connPill');
  const diag=el('diagConnBadge');
  if(pill){
    pill.classList.remove('ok','bad');
    pill.classList.add(ok?'ok':'bad');
    pill.textContent=text||(ok?'Connected':'Reconnecting');
  }
  if(diag){
    diag.classList.remove('ok','bad');
    diag.classList.add(ok?'ok':'bad');
    diag.textContent=ok?'Link OK':'Link Lost';
  }
}

function activateMode(mode){
  const setup=mode==='setup';
  const op=mode==='operator';
  const diag=mode==='diagnostics';
  el('setupMode')?.classList.toggle('active',setup);
  el('operatorMode')?.classList.toggle('active',op);
  el('diagnosticsMode')?.classList.toggle('active',diag);
  el('setupTab')?.classList.toggle('active',setup);
  el('operatorTab')?.classList.toggle('active',op);
  el('diagTab')?.classList.toggle('active',diag);
}

function applyDemoLockUI(s){
  const locked=Boolean(s.demoLockModeEnabled);
  document.body.classList.toggle('demoLockOn',locked);
  if(exists('demoLockStateOp')){
    el('demoLockStateOp').textContent=locked?'Demo Lock On':'Demo Lock Off';
  }
  if(locked&&ROUTE_MODE!=='scoreboard'){
    activateMode('operator');
  }
}

function applyOperatorPinGate(s){
  if(ROUTE_MODE==='scoreboard'){
    el('pinBackdrop')?.classList.remove('active');
    return;
  }
  if(!Boolean(s.operatorPinEnabled)){
    operatorUnlocked=true;
    el('pinBackdrop')?.classList.remove('active');
    return;
  }
  if(!operatorUnlocked){
    el('pinBackdrop')?.classList.add('active');
  }else{
    el('pinBackdrop')?.classList.remove('active');
  }
}

function showSetup(){
  activateMode('setup');
  if(!scoreboardPinned) hidePublic(false);
  else forceScoreboardVisible();
}
function showOperator(){
  activateMode('operator');
  if(!scoreboardPinned) hidePublic(false);
  else forceScoreboardVisible();
}
function showDiagnostics(){
  activateMode('diagnostics');
  if(!scoreboardPinned) hidePublic(false);
  else forceScoreboardVisible();
}
function showPublic(){
  setScoreboardPinned(true);
  forceScoreboardVisible();

  pulsePublicControls();

  if(ROUTE_MODE!=='scoreboard'&&document.documentElement.requestFullscreen){
    document.documentElement.requestFullscreen().catch(()=>{});
  }
}

function hidePublic(exitFs=true){
  if(ROUTE_MODE==='scoreboard')return;

  setScoreboardPinned(false);

  clearTimeout(publicControlsTimer);
  clearTimeout(goalRingTimeout);
  clearTimeout(goalEntranceTimeout);

  const board=el('publicMode');
  if(board){
    board.classList.remove(
      'active',
      'winnerBlue',
      'winnerOrange',
      'winnerTie',
      'countdownActive',
      'goalBlueActive',
      'goalOrangeActive',
      'finalBlueActive',
      'finalOrangeActive',
      'finalTieActive',
      'lastTenActive',
      'goalRingsActive',
      'goalJustEntered',
      'controlsVisible',
      'attractActive',
      'attractSlidesActive',
      'howToPlaySlideActive',
      'preGameActive',
      'postMatchActive',
      'gameOverActive',
      'startSoonActive',
      'liveMinimalActive',
      'attractSlideTitle',
      'attractSlideHow',
      'attractSlideChoose',
      'attractSlideChallenge',
      'attractSlideInfo',
      'longMainText',
      'veryLongMainText',
      'longStateText',
      'longMetaText'
    );
  }

  prevGoalActive=false;
  wasCountdownActive=false;
  countdownIntroUntilMs=0;

  if(exitFs&&document.fullscreenElement){document.exitFullscreen().catch(()=>{})}
}

function apiScoreBlue(delta){
  const d=Math.trunc(Number(delta)||0);
  if(d===0)return;
  api(API_ENDPOINTS.scoreBlue+'?delta='+d);
}

function apiScoreOrange(delta){
  const d=Math.trunc(Number(delta)||0);
  if(d===0)return;
  api(API_ENDPOINTS.scoreOrange+'?delta='+d);
}

function resetMatchNow(){
  if(!confirm('Reset match? This clears score, timer, and returns to standby.')) return;
  api(API_ENDPOINTS.reset);
}

function endMatchNow(){
  if(!confirm('End match now? This forces the final/winner screen.')) return;
  api(API_ENDPOINTS.end);
}

function previewAudienceAttract(){api(API_ENDPOINTS.audienceMode+'?mode=attract')}
function previewAudiencePost(){api(API_ENDPOINTS.audienceMode+'?mode=post_match')}
function previewAudienceAuto(){api(API_ENDPOINTS.audienceMode+'?mode=auto')}

function applyBrandingConfig(){
  api(API_ENDPOINTS.branding
    +'?title='+enc(getVal('eventTitleCfg'))
    +'&subtitle='+enc(getVal('eventSubtitleCfg'))
    +'&presentedBy='+enc(getVal('presentedByCfg'))
    +'&showPresentedBy='+getVal('showPresentedCfg')
    +'&showLiveBrand='+getVal('showLiveBrandCfg')
    +'&attractMessages='+enc(getVal('attractMessagesCfg'))
    +'&lastTenText='+enc(getVal('lastTenTextCfg')));
}

function applyTeamIdentityConfig(){
  api(API_ENDPOINTS.teamIdentity
    +'?enabled='+getVal('customNamesEnabledCfg')
    +'&blueName='+enc(getVal('blueTeamNameCfg'))
    +'&orangeName='+enc(getVal('orangeTeamNameCfg')));
}

function applyShowFlowConfig(){
  api(API_ENDPOINTS.showFlow
    +'?autoReturn='+getVal('autoReturnCfg')
    +'&autoReturnDelay='+getNum('autoReturnDelayCfg')
    +'&postMatchHold='+getNum('postMatchHoldCfg')
    +'&lastTenDramatic='+getVal('lastTenDramaticCfg')
    +'&finaleIntensity='+getVal('finaleIntensityCfg')
    +'&matchFormat='+enc(getVal('matchFormatCfg'))
    +'&demoLock='+getVal('demoLockCfg')
    +'&operatorPinEnabled='+getVal('operatorPinEnabledCfg'));
}

function audioFlowAction(action){
  api(API_ENDPOINTS.audioFlow+'?action='+enc(action));
}

function saveAudioWindows(){
  api(API_ENDPOINTS.audioWindowSave);
}

function resetAudioWindows(){
  api(API_ENDPOINTS.audioWindowReset);
}

function applyAudioWindow(track){
  const t=Math.max(1,Math.min(10,Number(track||0)));
  const start=Math.max(0,Math.floor(Number(getVal('audioStart'+t)||0)));
  const end=Math.max(1,Math.floor(Number(getVal('audioEnd'+t)||1)));
  api(API_ENDPOINTS.audioWindow+'?track='+t+'&start='+start+'&end='+end);
}

function testAudioWindowTrack(track){
  const t=Math.max(1,Math.min(10,Number(track||0)));
  const start=Math.max(0,Math.floor(Number(getVal('audioStart'+t)||0)));
  const end=Math.max(1,Math.floor(Number(getVal('audioEnd'+t)||1)));
  api(API_ENDPOINTS.play+'?track='+t+'&start='+start+'&end='+end);
}

function buildAudioWindowRows(trackWindows){
  if(!exists('audioWindowRows'))return;
  const arr=Array.isArray(trackWindows)?trackWindows:[];
  if(!arr.length)return;
  const rows=arr.map(w=>{
    const t=Number(w.track||0);
    const label=safeText(w.label||('Track '+t),('Track '+t),28);
    const start=Number(w.startSec||0);
    const end=Number(w.endSec||0);
    return '<div class="metric">'
      +'<div class="k">Track '+t+' - '+esc(label)+'</div>'
      +'<div style="display:grid;grid-template-columns:1fr 1fr auto auto;gap:8px;align-items:center;margin-top:8px;">'
      +'<input class="num" id="audioStart'+t+'" type="number" min="0" max="3600" value="'+start+'" placeholder="Start s">'
      +'<input class="num" id="audioEnd'+t+'" type="number" min="1" max="3600" value="'+end+'" placeholder="End s">'
      +'<button class="resetBtn" onclick="applyAudioWindow('+t+')">Apply</button>'
      +'<button class="resetBtn" onclick="testAudioWindowTrack('+t+')">Test</button>'
      +'</div>'
      +'</div>';
  }).join('');
  el('audioWindowRows').innerHTML=rows;
  audioWindowsRendered=true;
}

function syncAudioWindowRows(trackWindows){
  const arr=Array.isArray(trackWindows)?trackWindows:[];
  if(!arr.length)return;
  if(!audioWindowsRendered)buildAudioWindowRows(arr);
  arr.forEach(w=>{
    const t=Number(w.track||0);
    if(t<1||t>10)return;
    const startId='audioStart'+t;
    const endId='audioEnd'+t;
    if(exists(startId) && document.activeElement!==el(startId)){
      setVal(startId,Number(w.startSec||0));
    }
    if(exists(endId) && document.activeElement!==el(endId)){
      setVal(endId,Number(w.endSec||0));
    }
  });
}

function updateAudioShowFlowPanel(s){
  if(exists('audioFlowState')){
    el('audioFlowState').textContent=safeText(s.audioShowState||'stopped','stopped',20).replace(/_/g,' ');
  }
  if(exists('audioFlowTrack')){
    const tn=Number(s.currentTrack||0);
    const tl=safeText(s.currentTrackLabel||'Idle','Idle',28);
    el('audioFlowTrack').textContent='Track '+tn+' - '+tl;
  }
  if(exists('audioFlowProgress')){
    const pct=Math.max(0,Math.min(100,Number(s.currentTrackProgressPercent??0)));
    const elapsed=Math.max(0,Math.round(Number(s.currentTrackElapsedMs||0)/1000));
    el('audioFlowProgress').textContent=pct+'% ('+elapsed+'s elapsed)';
  }
  if(exists('audioFlowRemaining')){
    const rem=Math.max(0,Math.round(Number(s.currentTrackRemainingMs||0)/1000));
    const st=Number(s.currentTrackStartSec||0);
    const en=Number(s.currentTrackEndSec||0);
    el('audioFlowRemaining').textContent=rem+'s ('+st+'-'+en+'s)';
  }
  if(exists('audioFlowPreroll')){
    const st=Math.max(0,Number(s.currentTrackStartSec||0));
    el('audioFlowPreroll').textContent=Boolean(s.audioMutedForStartOffset)?('Silent pre-roll active until '+st+'s'):'Off';
  }
  syncAudioWindowRows(s.trackWindows||[]);
}

function runRehearsalScene(scene,durationMs=4500){
  api(API_ENDPOINTS.rehearsal+'?scene='+enc(scene)+'&durationMs='+Math.max(1500,Math.min(60000,Number(durationMs||4500))));
}

function clearRehearsalScene(){
  api(API_ENDPOINTS.rehearsal+'?scene=none');
}

function runReadabilityTest(){
  api(API_ENDPOINTS.rehearsal+'?scene=readability&durationMs=36000');
}

function setSettingsStatus(msg){
  if(exists('settingsStatus'))el('settingsStatus').textContent=String(msg||'');
}

async function saveSettings(){
  const ok=await api(API_ENDPOINTS.settingsSave);
  setSettingsStatus(ok?'Saved to ESP32 Preferences.':'Save failed. Check connection.');
}

async function loadSettings(){
  const ok=await api(API_ENDPOINTS.settingsLoad);
  setSettingsStatus(ok?'Loaded saved settings profile.':'Load failed. Check connection.');
}

async function verifyOperatorPin(){
  const pin=safeText(getVal('operatorPinInput'),'',12);
  if(!pin){
    if(exists('pinStatusText'))el('pinStatusText').textContent='PIN required.';
    return;
  }
  try{
    const r=await fetch(API_ENDPOINTS.operatorPinVerify+'?pin='+enc(pin),{method:'POST',cache:'no-store'});
    const data=await r.json();
    if(data&&data.ok){
      operatorUnlocked=true;
      el('pinBackdrop')?.classList.remove('active');
      if(exists('pinStatusText'))el('pinStatusText').textContent='';
      if(exists('operatorPinInput'))el('operatorPinInput').value='';
      return;
    }
  }catch(e){}
  operatorUnlocked=false;
  if(exists('pinStatusText'))el('pinStatusText').textContent='Invalid PIN.';
}

function confirmDanger(title,body,path){
  confirmActionPath=String(path||'');
  if(exists('confirmTitle'))el('confirmTitle').textContent=title||'Confirm Action';
  if(exists('confirmBody'))el('confirmBody').textContent=body||'Are you sure?';
  el('confirmBackdrop')?.classList.add('active');
}

function closeConfirm(){
  el('confirmBackdrop')?.classList.remove('active');
  confirmActionPath='';
}

function runConfirmed(){
  const path=confirmActionPath;
  closeConfirm();
  if(path)api(path);
}

function fmtUptime(ms){
  const total=Math.max(0,Math.floor(Number(ms||0)/1000));
  const h=Math.floor(total/3600);
  const m=Math.floor((total%3600)/60);
  const s=total%60;
  if(h>0)return h+'h '+String(m).padStart(2,'0')+'m '+String(s).padStart(2,'0')+'s';
  return m+'m '+String(s).padStart(2,'0')+'s';
}

function updateSensorIssueTimers(s,nowMs){
  const leftPartial=Boolean(s.leftSensorA)!==Boolean(s.leftSensorB);
  const rightPartial=Boolean(s.rightSensorA)!==Boolean(s.rightSensorB);
  const leftClear=!Boolean(s.leftSensorA)&&!Boolean(s.leftSensorB);
  const rightClear=!Boolean(s.rightSensorA)&&!Boolean(s.rightSensorB);
  sensorIssueSince.leftPartialMs=leftPartial?(sensorIssueSince.leftPartialMs||nowMs):0;
  sensorIssueSince.rightPartialMs=rightPartial?(sensorIssueSince.rightPartialMs||nowMs):0;
  sensorIssueSince.leftClearMs=leftClear?(sensorIssueSince.leftClearMs||nowMs):0;
  sensorIssueSince.rightClearMs=rightClear?(sensorIssueSince.rightClearMs||nowMs):0;
}

function diagWarnings(s,nowMs){
  updateSensorIssueTimers(s,nowMs);
  const warnings=[];
  const gs=normState(s.gameState);
  if(consecutiveUpdateErrors>0){
    warnings.push('Network jitter detected. Live state updates briefly interrupted.');
  }
  if(gs==='IN_GAME'&&s.scoringEnabled===false){
    warnings.push('Scoring is disabled while match is IN_GAME.');
  }
  if(sensorIssueSince.leftPartialMs&&nowMs-sensorIssueSince.leftPartialMs>1200){
    warnings.push('Left goal sensors disagree for >1.2s. Check alignment/obstruction.');
  }
  if(sensorIssueSince.rightPartialMs&&nowMs-sensorIssueSince.rightPartialMs>1200){
    warnings.push('Right goal sensors disagree for >1.2s. Check alignment/obstruction.');
  }
  if(gs==='IN_GAME'&&sensorIssueSince.leftClearMs&&nowMs-sensorIssueSince.leftClearMs>2500){
    warnings.push('Left goal pair has been clear >2.5s during play. Verify mounted target detection.');
  }
  if(gs==='IN_GAME'&&sensorIssueSince.rightClearMs&&nowMs-sensorIssueSince.rightClearMs>2500){
    warnings.push('Right goal pair has been clear >2.5s during play. Verify mounted target detection.');
  }
  if(Boolean(s.audioBusy)&&Number(s.audioRemainingMs||0)===0&&Number(s.currentTrack||0)!==0){
    warnings.push('Audio busy line appears stuck while track time is complete.');
  }
  return warnings;
}

function renderWarningList(items){
  if(!exists('diagWarnings'))return;
  if(!items.length){
    el('diagWarnings').innerHTML='<div class="warnItem ok">All key checks look healthy.</div>';
    return;
  }
  el('diagWarnings').innerHTML=items.map(x=>'<div class="warnItem">'+esc(x)+'</div>').join('');
}

function updateDiagnosticsPanel(s){
  if(exists('diagState'))el('diagState').textContent=normState(s.gameState);
  if(exists('diagRemain'))el('diagRemain').textContent=fmt(s.matchRemainingMs);
  if(exists('diagHalf'))el('diagHalf').textContent=halfText(s);
  if(exists('diagLedMode'))el('diagLedMode').textContent=prettyMode(s.ledMode);
  if(exists('diagAudioState'))el('diagAudioState').textContent=s.audioBusy?'Playing':'Idle';
  if(exists('diagTrack')){
    const trackNum=Number(s.currentTrack||0);
    const label=safeText(s.currentTrackLabel||'Idle','Idle',32);
    el('diagTrack').textContent='Track '+trackNum+' - '+label;
  }
  if(exists('diagBallLink')){
    const ready=Boolean(s.ballLinkReady);
    const cnt=Math.max(0,Number(s.ballConnectedCount||0));
    el('diagBallLink').textContent=ready?('Online '+cnt):'Offline';
  }
  if(exists('diagBusyPin'))el('diagBusyPin').textContent=s.audioBusy?'ACTIVE':'IDLE';
  if(exists('diagTrackNow'))el('diagTrackNow').textContent=String(Number(s.currentTrack||0));
  if(exists('diagLastTrack'))el('diagLastTrack').textContent=String(Number(s.lastAudioTrack||0));
  if(exists('diagAudioVolume'))el('diagAudioVolume').textContent=String(Number(s.volume||0))+'%';
  if(exists('diagUptime'))el('diagUptime').textContent=fmtUptime(s.uptimeMs);
  if(exists('diagBrightness'))el('diagBrightness').textContent=String(s.ledBrightness||0)+'% ('+String(s.ledBrightnessRaw||0)+')';
  if(exists('diagLeftGoal'))el('diagLeftGoal').textContent=(s.leftSensorA?'D':'C')+' / '+(s.leftSensorB?'D':'C')+' - '+(s.leftGoalArmed?'Armed':'Not Armed');
  if(exists('diagRightGoal'))el('diagRightGoal').textContent=(s.rightSensorA?'D':'C')+' / '+(s.rightSensorB?'D':'C')+' - '+(s.rightGoalArmed?'Armed':'Not Armed');
  if(exists('diagRejects'))el('diagRejects').textContent=String(Number(s.leftGoalRejectedSingles||0)+Number(s.rightGoalRejectedSingles||0));
  if(exists('diagInterrupts'))el('diagInterrupts').textContent=String(Number(s.sensorInterruptCount||0));
  if(exists('diagLiveStamp'))el('diagLiveStamp').textContent='Updated '+new Date().toLocaleTimeString();
  if(Array.isArray(s.eventLog)&&exists('diagEvents')){
    el('diagEvents').innerHTML=s.eventLog.slice().reverse().map(e=>'<div class="logItem">'+esc(e)+'</div>').join('');
  }
  renderWarningList(diagWarnings(s,Date.now()));
}

function setVal(id,v){if(!exists(id))return;const e=el(id);if(document.activeElement===e)return;e.value=v}function setPair(id,v){setVal(id,v);setVal(id+'N',v)}function getNum(id){return Number(el(id)?.value||0)}function getVal(id){return el(id)?.value||''}function setSelect(id,v){setVal(id,String(v).toLowerCase())}function setColor(id,v){if(v)setVal(id,String(v).toUpperCase())}
function linkPair(id,applyFn){['input','change'].forEach(evt=>{el(id)?.addEventListener(evt,()=>{setVal(id+'N',el(id).value);if(evt==='change'){(applyFn||(()=>routeApply(id)))()}});el(id+'N')?.addEventListener(evt,()=>{setVal(id,el(id+'N').value);if(evt==='change'){(applyFn||(()=>routeApply(id)))()}})})}
function resetOne(id){
  const v=DEFAULTS[id];
  if(v===undefined)return;
  if(exists(id)&&el(id).type==='color')setColor(id,v);
  else if(exists(id)&&el(id).tagName==='SELECT')setSelect(id,v);
  else setPair(id,v);
  routeApply(id);
}

function resetGroup(g){
  const groups={
    colors:['blueColor','orangeColor','hex1Color','hex2Color','hex3Color','fieldColor','startColor','lastTenColor'],
    countdown:['countdownMode','hexStyle','countStep','fieldPeriod','hexRotation','hexTrail','fieldMin','fieldMax','hexBright','lastTenEnabled','lastTenPeriod','lastTenMin','lastTenMax','idleSpeed','idleMin','idleMax'],
    goal:['goalMode','goalDuration','goalTravel','goalInterval','goalEmit','goalPop','autoGoalResumeEnabled','autoGoalResumeMs'],
    finale:['finaleDuration','winnerSpeed','rainbowSpeed','ringSpacing','finaleIntensityCfg'],
    output:['matchSecs','ledBrightness','audioVolume'],
    showflow:['autoReturnCfg','autoReturnDelayCfg','lastTenDramaticCfg','postMatchHoldCfg','matchFormatCfg','demoLockCfg','operatorPinEnabledCfg']
  };
  (groups[g]||[]).forEach(resetOne);
}

function resetAll(){
  Object.keys(DEFAULTS).forEach(resetOne);
  applyColors();
  applyCountdown();
  applyGoal();
  applyGoalResume();
  applyFinale();
  applyOutput();
  applyBrandingConfig();
  applyTeamIdentityConfig();
  applyShowFlowConfig();
}

function routeApply(id){
  if(id==='matchSecs')applyMatchLength();
  else if(['blueColor','orangeColor','hex1Color','hex2Color','hex3Color','fieldColor','startColor','lastTenColor'].includes(id))applyColors();
  else if(['countdownMode','hexStyle','countStep','fieldPeriod','hexRotation','hexTrail','fieldMin','fieldMax','hexBright','lastTenEnabled','lastTenPeriod','lastTenMin','lastTenMax','idleSpeed','idleMin','idleMax'].includes(id))applyCountdown();
  else if(['goalMode','goalDuration','goalTravel','goalInterval','goalEmit','goalPop'].includes(id))applyGoal();
  else if(['autoGoalResumeEnabled','autoGoalResumeMs'].includes(id))applyGoalResume();
  else if(['finaleDuration','winnerSpeed','rainbowSpeed','ringSpacing','finaleIntensityCfg'].includes(id))applyFinale();
  else if(['ledBrightness','audioVolume'].includes(id))applyOutput();
  else if(['autoReturnCfg','autoReturnDelayCfg','lastTenDramaticCfg','postMatchHoldCfg','matchFormatCfg','demoLockCfg','operatorPinEnabledCfg'].includes(id))applyShowFlowConfig();
  else if(['eventTitleCfg','eventSubtitleCfg','presentedByCfg','attractMessagesCfg','lastTenTextCfg','showPresentedCfg','showLiveBrandCfg'].includes(id))applyBrandingConfig();
  else if(['customNamesEnabledCfg','blueTeamNameCfg','orangeTeamNameCfg'].includes(id))applyTeamIdentityConfig();
}
function applyColors(){api('/api/colors?blue='+enc(getVal('blueColor'))+'&orange='+enc(getVal('orangeColor'))+'&hex1='+enc(getVal('hex1Color'))+'&hex2='+enc(getVal('hex2Color'))+'&hex3='+enc(getVal('hex3Color'))+'&field='+enc(getVal('fieldColor'))+'&start='+enc(getVal('startColor'))+'&lastTen='+enc(getVal('lastTenColor')))}
function applyCountdown(){api('/api/countdown-tuning?mode='+enc(getVal('countdownMode'))+'&hexStyle='+enc(getVal('hexStyle'))+'&step='+getNum('countStep')+'&period='+getNum('fieldPeriod')+'&hexRotation='+getNum('hexRotation')+'&hexTrail='+getNum('hexTrail')+'&min='+getNum('fieldMin')+'&max='+getNum('fieldMax')+'&hexBrightness='+getNum('hexBright')+'&lastTenEnabled='+getVal('lastTenEnabled')+'&lastTenPeriod='+getNum('lastTenPeriod')+'&lastTenMin='+getNum('lastTenMin')+'&lastTenMax='+getNum('lastTenMax')+'&idleSpeed='+getNum('idleSpeed')+'&idleMin='+getNum('idleMin')+'&idleMax='+getNum('idleMax'))}
function applyGoal(){api('/api/goal-timing?mode='+enc(getVal('goalMode'))+'&duration='+getNum('goalDuration')+'&travel='+getNum('goalTravel')+'&interval='+getNum('goalInterval')+'&emit='+getNum('goalEmit')+'&pop='+getNum('goalPop'))}function applyGoalResume(){api('/api/goal-resume?enabled='+getVal('autoGoalResumeEnabled')+'&delay='+getNum('autoGoalResumeMs'))}function applyFinale(){api('/api/end-tuning?duration='+getNum('finaleDuration')+'&winnerSpeed='+getNum('winnerSpeed')+'&rainbowSpeed='+getNum('rainbowSpeed')+'&spacing='+getNum('ringSpacing')+'&finaleIntensity='+getVal('finaleIntensityCfg'))}function applyMatchLength(){api('/api/match-length?seconds='+getNum('matchSecs'))}function applyOutput(){api('/api/led-brightness?level='+getNum('ledBrightness'));api('/api/volume?level='+getNum('audioVolume'))}
function matchSeconds(){const v=exists('matchSecs')?Number(el('matchSecs').value||DEFAULTS.matchSecs):DEFAULTS.matchSecs;return Math.max(10,Math.min(600,v||DEFAULTS.matchSecs))}function startFirstHalf(){api('/api/start?seconds='+matchSeconds())}function syncMatchLength(s){const secs=Math.round(Number(s.matchDurationMs||120000)/1000);if(exists('matchSecs')&&document.activeElement!==el('matchSecs'))setVal('matchSecs',secs);if(exists('matchSecsN')&&document.activeElement!==el('matchSecsN'))setVal('matchSecsN',secs)}
function sensorPair(a,b,armed){if(a&&b)return ['Both Detected','warn'];if(a||b)return ['Partial','warn'];return [armed?'Armed':'Clearing',armed?'ok':'bad']}
function classState(s){const gs=normState(s);let cls='state';if(gs==='IN_GAME')cls+=' go';else if(gs==='GOAL_PAUSE'||gs==='COUNTDOWN'||gs==='HALFTIME')cls+=' pause';else if(gs==='END_GAME')cls+=' end';return cls}
function updatePublic(s){
  const gs=normState(s.gameState);
  const board=el('publicMode');
  if(!board)return;
  forceScoreboardVisible();
  const b=Number(s.blueScore??0);
  const o=Number(s.orangeScore??0);
  const teamBlueName=teamNameBlue(s);
  const teamOrangeName=teamNameOrange(s);
  // Audience live display is intentionally normalized for far-distance readability.
  const blueName='Blue';
  const orangeName='Orange';
  const eventTitle=safeText(s.eventTitle||DEFAULTS.eventTitle,DEFAULTS.eventTitle,54);
  const eventSubtitle=safeText(s.eventSubtitle||DEFAULTS.eventSubtitle,DEFAULTS.eventSubtitle,64);
  const presentedBy=safeText(s.presentedByLine||DEFAULTS.presentedBy,DEFAULTS.presentedBy,84);
  const showPresented=Boolean(s.showPresentedByLine!==false);
  const showLiveBrand=Boolean(s.showEventBrandingInLive!==false);
  const lastTenText=safeText(s.lastTenText||DEFAULTS.lastTenText,DEFAULTS.lastTenText,24);
  const override=String(s.audienceOverrideMode||'auto').toLowerCase();

  if(exists('publicBlueName'))el('publicBlueName').textContent=blueName;
  if(exists('publicOrangeName'))el('publicOrangeName').textContent=orangeName;
  if(exists('setupBlueName'))el('setupBlueName').textContent=teamBlueName;
  if(exists('setupOrangeName'))el('setupOrangeName').textContent=teamOrangeName;
  if(exists('opBlueName'))el('opBlueName').textContent=teamBlueName;
  if(exists('opOrangeName'))el('opOrangeName').textContent=teamOrangeName;
  if(exists('readyBlueName'))el('readyBlueName').textContent=teamBlueName;
  if(exists('readyOrangeName'))el('readyOrangeName').textContent=teamOrangeName;

  if(publicLastBlueScore===null || publicLastOrangeScore===null){
    publicLastBlueScore=b;
    publicLastOrangeScore=o;
  }else{
    if(b>publicLastBlueScore){ publicGoalTeam='blue'; scoreFlash('publicBlue'); animateScoreRoll('publicBlueScore'); }
    if(o>publicLastOrangeScore){ publicGoalTeam='orange'; scoreFlash('publicOrange'); animateScoreRoll('publicOrangeScore'); }
    if(b<publicLastBlueScore || o<publicLastOrangeScore) publicGoalTeam='none';
    publicLastBlueScore=b;
    publicLastOrangeScore=o;
  }

  const led=String(s.ledMode||'');
  if(led==='BLUE_GOAL') publicGoalTeam='blue';
  if(led==='ORANGE_GOAL') publicGoalTeam='orange';
  if(gs!=='GOAL_PAUSE' && led!=='BLUE_GOAL' && led!=='ORANGE_GOAL'){
    if(gs==='IN_GAME' || gs==='COUNTDOWN' || gs==='HALFTIME' || gs==='END_GAME' || gs==='STANDBY' || gs==='PRE_GAME') publicGoalTeam='none';
  }

  let winnerState=String(s.winner||'none').toLowerCase();
  let goalActive=(gs==='GOAL_PAUSE') && (publicGoalTeam==='blue' || publicGoalTeam==='orange');
  let finalActive=(gs==='END_GAME') && (winnerState==='blue' || winnerState==='orange' || winnerState==='tie');
  const remainingMs=Number(s.matchRemainingMs||0);
  const testLastTen=(led==='LAST_TEN' || led==='LAST_TEN_TEST');
  const warningMs=Math.max(1000,Number(s.warningPhaseMs||30000));
  let lastTenActive=(s.lastTenEnabled!==false) && ((gs==='IN_GAME' && remainingMs>0 && remainingMs<=warningMs) || testLastTen);
  const testLastTenDisplay=String(30-Math.floor((Number(s.uptimeMs||0)%30000)/1000));
  const rehearsalScene=String(s.rehearsalScene||'none').toLowerCase();
  const rehearsalActive=Boolean(s.rehearsalSceneActive)&&rehearsalScene!=='none';
  let forcedRehearsalScene=rehearsalScene;
  if(rehearsalActive&&rehearsalScene==='readability'){
    const elapsed=Number(s.rehearsalSceneElapsedMs||0);
    const index=Math.floor(elapsed/4500)%8;
    forcedRehearsalScene=['attract','pre_game','countdown','live','goal_blue','final_ten','win_blue','post'][index];
  }

  let scene='live';
  let attractSlideIndex=-1;
  if(override==='attract') scene='attract';
  else if(override==='post' || override==='post_match' || override==='postmatch') scene='post';
  else if(gs==='STANDBY') scene='attract';
  else if(gs==='PRE_GAME') scene='pre_game';
  else if(gs==='HALFTIME') scene='auto';
  else if(gs==='END_GAME') scene='post';

  if(rehearsalActive){
    if(forcedRehearsalScene==='attract') scene='attract';
    else if(forcedRehearsalScene==='pre_game') scene='pre_game';
    else if(forcedRehearsalScene==='post') scene='post';
    else if(forcedRehearsalScene==='countdown') scene='live';
    else if(forcedRehearsalScene==='live') scene='live';

    if(forcedRehearsalScene==='goal_blue'){goalActive=true;publicGoalTeam='blue';}
    if(forcedRehearsalScene==='goal_orange'){goalActive=true;publicGoalTeam='orange';}
    if(forcedRehearsalScene==='final_ten'){lastTenActive=true;}
    if(forcedRehearsalScene==='win_blue'){finalActive=true;winnerState='blue';}
    if(forcedRehearsalScene==='win_orange'){finalActive=true;winnerState='orange';}
    if(forcedRehearsalScene==='win_tie'){finalActive=true;winnerState='tie';}
  }

  const gameOverWarningMs=Math.max(800,Number(s.gameOverWarningMs||2600));
  const postMatchElapsedMs=Math.max(0,Number(s.postMatchElapsedMs||0));
  const gameOverActive=finalActive && !rehearsalActive && postMatchElapsedMs < gameOverWarningMs;

  const countdownViewActive=(gs==='COUNTDOWN') || (rehearsalActive && forcedRehearsalScene==='countdown');
  const nowUiMs=Date.now();
  if(countdownViewActive && !wasCountdownActive){
    countdownIntroUntilMs=nowUiMs + COUNTDOWN_INTRO_MS;
  }
  if(!countdownViewActive){
    countdownIntroUntilMs=0;
  }
  wasCountdownActive=countdownViewActive;
  const showCountdownIntro=countdownViewActive && nowUiMs < countdownIntroUntilMs;
  const countdownStepMsUi=Math.max(250,Number(s.countdownStepMs||DEFAULTS.countStep||1000));
  const countdownRemainingMsUi=Math.max(0,Number(s.countdownRemainingMs||0));
  const countdownFallback=Math.max(1,Math.ceil(countdownRemainingMsUi/countdownStepMsUi));
  const countdownRaw=Number(s.countdownValue);
  let countdownDisplayValue=0;
  if(countdownViewActive){
    if(Number.isFinite(countdownRaw) && countdownRaw>0){
      countdownDisplayValue=Math.max(1,Math.ceil(countdownRaw));
    }else if(countdownRemainingMsUi>0){
      countdownDisplayValue=countdownFallback;
    }else{
      countdownDisplayValue=1;
    }
  }

  // Restore inline display defaults before state-specific rendering.
  if (exists('publicState')) el('publicState').style.display = '';
  if (exists('publicMeta')) el('publicMeta').style.display = '';
  if (exists('publicInfoBar')) el('publicInfoBar').style.display = '';
  if (exists('publicBlueName')) el('publicBlueName').style.display = '';
  if (exists('publicOrangeName')) el('publicOrangeName').style.display = '';
  if (exists('publicWinner')) el('publicWinner').style.display = '';

  // v4.53:
  // Keep the clean minimal layout only during LIVE gameplay.
  // Downtime/ready/countdown/goal/final scenes use storytelling overlays.
  const audienceMinimalView =
    (gs === 'IN_GAME') &&
    !goalActive &&
    !countdownViewActive &&
    !showCountdownIntro &&
    !finalActive;

  if (audienceMinimalView) {
    let mainText = fmt(s.matchRemainingMs);
    if (rehearsalActive && forcedRehearsalScene === 'live') {
      mainText = "72";
    }

    animatePublicTimerIfChanged(mainText);
    const timerNodeMinimal = el('publicTimer');
    if (timerNodeMinimal) {
      timerNodeMinimal.classList.remove('fifaTitleGradient');
      timerNodeMinimal.textContent = mainText;
    }

    if (exists('publicState')) el('publicState').textContent = '';
    if (exists('publicMeta')) el('publicMeta').textContent = '';
    if (exists('publicTicker')) el('publicTicker').textContent = '';
    if (exists('publicBrandLine')) el('publicBrandLine').textContent = '';
    if (exists('publicBlueName')) el('publicBlueName').textContent = '';
    if (exists('publicOrangeName')) el('publicOrangeName').textContent = '';
    if (exists('publicBlueScore')) el('publicBlueScore').textContent = String(b);
    if (exists('publicOrangeScore')) el('publicOrangeScore').textContent = String(o);

    board.classList.remove('winnerBlue','winnerOrange','winnerTie','countdownActive','goalBlueActive','goalOrangeActive','finalBlueActive','finalOrangeActive','finalTieActive','lastTenActive','scoreBlueLead','scoreOrangeLead','scoreTieLive','attractActive','attractSlidesActive','howToPlaySlideActive','preGameActive','postMatchActive','gameOverActive','startSoonActive','liveMinimalActive','attractSlideTitle','attractSlideHow','attractSlideChoose','attractSlideChallenge','attractSlideInfo','attractSlideEntered','longMainText','veryLongMainText','longStateText','longMetaText');
    el('publicBlue').classList.remove('winnerGlow','tieGlow','scoreLead','scoreBehind','scoreTied');
    el('publicOrange').classList.remove('winnerGlow','tieGlow','scoreLead','scoreBehind','scoreTied');
    board.classList.add('liveMinimalActive');

    const wMinimal = el('publicWinner');
    if (wMinimal) wMinimal.classList.remove('show');

    if (lastTenActive) {
      const lastColor = s.lastTenColor || DEFAULTS.lastTenColor || '#982ABC';
      const lastDur = Math.max(250, Number(s.lastTenPeriodMs || DEFAULTS.lastTenPeriod || 1000));
      board.classList.add('lastTenActive');
      board.style.setProperty('--lastColor', lastColor);
      board.style.setProperty('--lastDur', lastDur + 'ms');
    }

    applyAudienceScoreMood(b, o);
    return;
  }

  let mainText=countdownViewActive?String(countdownDisplayValue):(goalActive?'GOAL':(testLastTen?testLastTenDisplay:fmt(s.matchRemainingMs)));
  let stateText=goalActive?'':prettyState(s.gameState);
  let metaText=goalActive?'':halfText(s);
  let tickerText=showLiveBrand?eventSubtitle:'';
  if(rehearsalActive && forcedRehearsalScene==='countdown'){
    const totalPreview=Math.max(1,Number(s.countdownSeconds||10));
    const previewCount=totalPreview-Math.floor((Number(s.rehearsalSceneElapsedMs||0)%(totalPreview*1000))/1000);
    countdownDisplayValue=Math.max(1,previewCount);
    mainText=String(countdownDisplayValue);
  }
  if(rehearsalActive && forcedRehearsalScene==='final_ten'){
    const previewLastTen=30-Math.floor((Number(s.rehearsalSceneElapsedMs||0)%30000)/1000);
    mainText=String(Math.max(1,previewLastTen));
  }
  if(rehearsalActive && forcedRehearsalScene==='live'){
    mainText='72';
  }

  if(showCountdownIntro){
    mainText='GET READY!';
    stateText='MATCH STARTING SOON';
    metaText='BLUE VS ORANGE';
    tickerText='';
  }else if(gameOverActive){
    mainText='GAME OVER!';
    stateText='CONTROLLERS DOWN';
    metaText='WAIT FOR FINAL SCORE';
    tickerText='';
    scene='game_over';
  }else if(finalActive){
    if(winnerState==='blue'){
      mainText='BLUE WINS';
      stateText='MATCH COMPLETE';
      tickerText='NEXT MATCH READY';
      scene='post';
    }else if(winnerState==='orange'){
      mainText='ORANGE WINS';
      stateText='MATCH COMPLETE';
      tickerText='NEXT MATCH READY';
      scene='post';
    }else{
      mainText='TIE GAME';
      stateText='MATCH COMPLETE';
      tickerText='NEXT MATCH READY';
      scene='post';
    }
    metaText='FINAL SCORE '+b+' - '+o;
  }else if(goalActive){
    mainText=fmt(s.matchRemainingMs);
    stateText='GOAL';
    metaText='LIVE SCORE '+b+' - '+o;
    tickerText='';
  }else if(scene==='attract'){
    const slides=standbySlides(s);
    const picked=pickStandbySlide(Number(s.uptimeMs||0),slides);
    const slide=picked.slide||slides[0];
    attractSlideIndex=Number(picked.index||0);
    mainText=slide.title;
    stateText=slide.state;
    metaText=slide.meta;
    tickerText='';
  }else if(scene==='pre_game'){
    mainText='GET READY';
    stateText='PLAYERS STANDBY';
    metaText='GAME ABOUT TO BEGIN';
    tickerText='';
  }else if(scene==='post'){
    mainText='MATCH COMPLETE';
    stateText='FINAL SCORE '+b+' - '+o;
    if(Boolean(s.autoReturnToStandbyEnabled)){
      const rem=Math.ceil(Number(s.postMatchAutoReturnRemainingMs||0)/1000);
      tickerText=rem>0?('Next players ready? Returning to attract in '+rem+'s'):'Next players ready?';
    }else{
      tickerText='Next players ready?';
    }
    metaText=eventTitle;
  }else{
    stateText=(gs==='IN_GAME')?'':prettyState(s.gameState);
    metaText=(gs==='IN_GAME')?'':((showLiveBrand?eventTitle+' - ':'')+halfText(s));
    if(lastTenActive)tickerText=lastTenText;
    else if(gs==='IN_GAME')tickerText='';
    else if(gs==='HALFTIME'){
      mainText='HALFTIME';
      stateText='SECOND HALF READY';
      tickerText='Operator press Resume 2nd Half';
    }else if(countdownViewActive){
      stateText='STARTING';
      tickerText='10 - 9 - 8 - 7 - 6 - 5 - 4 - 3 - 2 - 1 - GO';
    }else{
      tickerText=eventSubtitle;
    }
  }

  animatePublicTimerIfChanged(mainText);
  const timerNode=el('publicTimer');
  const fifaTitleGradientActive=(scene==='attract') && /\bFIFA\b/i.test(mainText);
  const howToPlaySlideActive=(scene==='attract') && /SCORE GOALS BEFORE TIME RUNS OUT/i.test(stateText);
  timerNode.classList.toggle('fifaTitleGradient',fifaTitleGradientActive);
  if(scene==='attract'){
    timerNode.innerHTML=colorizeTeamMentions(mainText);
    el('publicState').innerHTML=colorizeTeamMentions(stateText);
    el('publicMeta').innerHTML=colorizeTeamMentions(metaText);
  }else{
    timerNode.textContent=mainText;
    el('publicState').textContent=stateText;
    el('publicMeta').textContent=metaText;
  }
  if(exists('publicMeta')){
    el('publicMeta').style.display=compactSlideKey(metaText)?'':'none';
  }
  if(exists('publicTicker'))el('publicTicker').textContent=tickerText;
  if(exists('publicBrandLine')){
    const showBrandLine=showPresented && !goalActive && !countdownViewActive && !showCountdownIntro && (scene!=='live' || showLiveBrand);
    el('publicBrandLine').textContent=showBrandLine?presentedBy:'';
  }
  el('publicBlueScore').textContent=b;
  el('publicOrangeScore').textContent=o;

  board.classList.remove('winnerBlue','winnerOrange','winnerTie','countdownActive','goalBlueActive','goalOrangeActive','finalBlueActive','finalOrangeActive','finalTieActive','lastTenActive','scoreBlueLead','scoreOrangeLead','scoreTieLive','attractActive','attractSlidesActive','howToPlaySlideActive','preGameActive','postMatchActive','gameOverActive','startSoonActive','liveMinimalActive','attractSlideTitle','attractSlideHow','attractSlideChoose','attractSlideChallenge','attractSlideInfo','attractSlideEntered','longMainText','veryLongMainText','longStateText','longMetaText');
  el('publicBlue').classList.remove('winnerGlow','tieGlow','scoreLead','scoreBehind','scoreTied');
  el('publicOrange').classList.remove('winnerGlow','tieGlow','scoreLead','scoreBehind','scoreTied');
  if(scene==='attract'){
    const mainLen=compactSlideKey(mainText).length;
    const stateLen=compactSlideKey(stateText).length;
    const metaLen=compactSlideKey(metaText).length;
    board.classList.add('attractActive','attractSlidesActive');
    if(publicLastAttractSlideIndex!==attractSlideIndex){
      publicLastAttractSlideIndex=attractSlideIndex;
      board.classList.add('attractSlideEntered');
      clearTimeout(publicAttractTransitionTimer);
      publicAttractTransitionTimer=setTimeout(()=>board.classList.remove('attractSlideEntered'),980);
    }
    if(mainLen>18)board.classList.add('longMainText');
    if(mainLen>26)board.classList.add('veryLongMainText');
    if(stateLen>28)board.classList.add('longStateText');
    if(metaLen>30)board.classList.add('longMetaText');
    if(howToPlaySlideActive)board.classList.add('howToPlaySlideActive');
    if(fifaTitleGradientActive)board.classList.add('attractSlideTitle');
    else if(howToPlaySlideActive)board.classList.add('attractSlideHow');
    else if(/BLUE\s+BOTS?|ORANGE\s+BOTS?|VS\.?/i.test(mainText+' '+stateText))board.classList.add('attractSlideChoose');
    else if(/120\s+SECOND|CHALLENGE/i.test(mainText+' '+stateText))board.classList.add('attractSlideChallenge');
    else board.classList.add('attractSlideInfo');
  }
  if(scene!=='attract')publicLastAttractSlideIndex=null;
  if(scene==='pre_game')board.classList.add('preGameActive');
  if(scene==='game_over')board.classList.add('gameOverActive');
  if(scene==='post')board.classList.add('postMatchActive');
  if(showCountdownIntro)board.classList.add('startSoonActive');
  if(gs==='IN_GAME' && !goalActive && !finalActive && !countdownViewActive){
    board.classList.add('liveMinimalActive');
  }
  publicLastScene=scene;

  const w=el('publicWinner');

  if(showCountdownIntro){
    w.classList.remove('show');
    return;
  }

  if(countdownViewActive){
    board.classList.add('countdownActive');
    const cs=s.countdownStepColors||[];
    const totalCount=Math.max(1,Number(s.countdownSeconds||10));
    const cv=Math.max(1,Math.min(totalCount,countdownDisplayValue));
    const colorIndex=Math.max(0,Math.min(cs.length-1,totalCount-cv));
    const color=cs[colorIndex]||DEFAULTS.hex1Color||'#00D2FF';
    const countMs=Math.max(300,Number(s.countdownStepMs||DEFAULTS.countStep||1000));
    board.style.setProperty('--countColor',color);
    board.style.setProperty('--countDur',countMs+'ms');
    w.classList.remove('show');
    return;
  }

  if(goalActive){
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
    w.textContent=(publicGoalTeam==='blue'?'BLUE GOAL!':'ORANGE GOAL!');
    w.classList.add('show');
    return;
  }
  prevGoalActive=false;

  if(gameOverActive){
    board.classList.add('gameOverActive');
    w.classList.remove('show');
    return;
  }

  if(finalActive){
    if(winnerState==='blue'){
      board.classList.add('finalBlueActive');
      el('publicBlue').classList.add('winnerGlow');
    }else if(winnerState==='orange'){
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

  applyAudienceScoreMood(b,o);

  if(winnerState==='blue'){
    board.classList.add('winnerBlue');
    el('publicBlue').classList.add('winnerGlow');
    w.textContent=(blueName+' WIN').toUpperCase();
    w.classList.add('show');
  }else if(winnerState==='orange'){
    board.classList.add('winnerOrange');
    el('publicOrange').classList.add('winnerGlow');
    w.textContent=(orangeName+' WIN').toUpperCase();
    w.classList.add('show');
  }else if(winnerState==='tie'){
    board.classList.add('winnerTie');
    el('publicBlue').classList.add('tieGlow');
    el('publicOrange').classList.add('tieGlow');
    w.textContent='TIE GAME';
    w.classList.add('show');
  }else{
    w.classList.remove('show');
  }
}

function recipe(s=latest){
  return 'FIFA ROBOT SOCCER SETTINGS\n'
    +'matchSeconds='+Math.round(Number(s.matchDurationMs||120000)/1000)
    +'\nteamNames='+teamNameBlue(s)+' vs '+teamNameOrange(s)
    +'\nbrandingTitle='+safeText(s.eventTitle||DEFAULTS.eventTitle,DEFAULTS.eventTitle,54)
    +'\ncolors: blue='+getVal('blueColor')+' orange='+getVal('orangeColor')+' countdownField='+getVal('fieldColor')+' gameStart='+getVal('startColor')
    +'\ncountdownHexColors='+getVal('hex1Color')+','+getVal('hex2Color')+','+getVal('hex3Color')
    +'\ncountdown: fieldMode='+getVal('countdownMode').toUpperCase()+' hexStyle='+getVal('hexStyle').toUpperCase()+' stepMs='+getNum('countStep')+' fieldEffectMs='+getNum('fieldPeriod')+' hexRotationMs='+getNum('hexRotation')+' hexTrailPercent='+getNum('hexTrail')+' brightness='+getNum('fieldMin')+'/'+getNum('fieldMax')+'/'+getNum('hexBright')
    +'\ngoal: mode='+getVal('goalMode')+' durationMs='+getNum('goalDuration')+' travelMs='+getNum('goalTravel')+' spacingMs='+getNum('goalInterval')+' emitMs='+getNum('goalEmit')+' popMs='+getNum('goalPop')+' autoResume='+(getVal('autoGoalResumeEnabled')==='1'?'true':'false')+' autoResumeDelayMs='+getNum('autoGoalResumeMs')
    +'\nfinale: durationMs='+getNum('finaleDuration')+' winnerSpeed='+getNum('winnerSpeed')+' hexRainbowSpeed='+getNum('rainbowSpeed')+' ringSpacing='+getNum('ringSpacing')+' intensity='+getVal('finaleIntensityCfg')
    +'\nlast10: color='+getVal('lastTenColor')+' enabled='+(getVal('lastTenEnabled')==='1'?'true':'false')+' periodMs='+getNum('lastTenPeriod')+' dramatic='+(getVal('lastTenDramaticCfg')==='1'?'true':'false')+' brightness='+getNum('lastTenMin')+'/'+getNum('lastTenMax')
    +'\nshowFlow: format='+getVal('matchFormatCfg')+' autoReturn='+(getVal('autoReturnCfg')==='1'?'true':'false')+' delayMs='+getNum('autoReturnDelayCfg')+' postHoldMs='+getNum('postMatchHoldCfg')+' demoLock='+(getVal('demoLockCfg')==='1'?'true':'false')
    +'\noutput: ledBrightnessPercent='+getNum('ledBrightness')+' audioVolumePercent='+getNum('audioVolume');
}
function copyRecipe(){const r=recipe();el('recipe').value=r;navigator.clipboard&&navigator.clipboard.writeText(r).catch(()=>{})}
function syncControls(s){
  setColor('blueColor',s.blueGoalColor||DEFAULTS.blueColor);
  setColor('orangeColor',s.orangeGoalColor||DEFAULTS.orangeColor);
  const cs=s.countdownStepColors||[];
  setColor('hex1Color',cs[0]||s.countdownHexColor||DEFAULTS.hex1Color);
  setColor('hex2Color',cs[1]||DEFAULTS.hex2Color);
  setColor('hex3Color',cs[2]||DEFAULTS.hex3Color);
  setColor('fieldColor',s.countdownFieldColor||DEFAULTS.fieldColor);
  setColor('startColor',s.gameStartColor||DEFAULTS.startColor);
  setColor('lastTenColor',s.lastTenColor||DEFAULTS.lastTenColor);
  setSelect('countdownMode',String(s.countdownEffectMode||DEFAULTS.countdownMode).toLowerCase());
  setSelect('hexStyle',String(s.countdownHexStyle||DEFAULTS.hexStyle).toLowerCase());
  setPair('countStep',s.countdownStepMs??DEFAULTS.countStep);
  setPair('fieldPeriod',s.countdownFieldEffectPeriodMs??DEFAULTS.fieldPeriod);
  setPair('hexRotation',s.countdownHexRotationMs??DEFAULTS.hexRotation);
  setPair('hexTrail',s.countdownHexTrailPercent??DEFAULTS.hexTrail);
  setPair('fieldMin',s.countdownFieldMinBrightness??DEFAULTS.fieldMin);
  setPair('fieldMax',s.countdownFieldMaxBrightness??DEFAULTS.fieldMax);
  setPair('hexBright',s.countdownHexBrightness??DEFAULTS.hexBright);
  setSelect('lastTenEnabled',s.lastTenEnabled?1:0);
  setPair('lastTenPeriod',s.lastTenPeriodMs??DEFAULTS.lastTenPeriod);
  const lb=String(s.lastTenBrightness||'75/135').split('/');
  setPair('lastTenMin',lb[0]||DEFAULTS.lastTenMin);
  setPair('lastTenMax',lb[1]||DEFAULTS.lastTenMax);
  setPair('idleSpeed',s.inGameIdleSpeed??DEFAULTS.idleSpeed);
  const ib=String(s.inGameIdleBrightness||'10/255').split('/');
  setPair('idleMin',ib[0]||DEFAULTS.idleMin);
  setPair('idleMax',ib[1]||DEFAULTS.idleMax);
  setSelect('goalMode',String(s.goalTravelMode||DEFAULTS.goalMode).toLowerCase());
  setPair('goalDuration',s.goalEffectMs??DEFAULTS.goalDuration);
  setPair('goalTravel',s.goalWaveTravelMs??DEFAULTS.goalTravel);
  setPair('goalInterval',s.goalWaveEmitIntervalMs??DEFAULTS.goalInterval);
  setPair('goalEmit',s.goalWaveEmitEndMs??DEFAULTS.goalEmit);
  setPair('goalPop',s.goalSourcePopMs??DEFAULTS.goalPop);
  setSelect('autoGoalResumeEnabled',s.autoResumeAfterGoal?1:0);
  setPair('autoGoalResumeMs',s.autoResumeAfterGoalDelayMs??DEFAULTS.autoGoalResumeMs);
  setPair('finaleDuration',s.matchEndEffectMs??DEFAULTS.finaleDuration);
  setPair('winnerSpeed',s.matchEndWinnerSpeed??DEFAULTS.winnerSpeed);
  setPair('rainbowSpeed',s.matchEndRainbowSpeed??DEFAULTS.rainbowSpeed);
  setPair('ringSpacing',s.matchEndRingSpacing??DEFAULTS.ringSpacing);
  setSelect('finaleIntensityCfg',s.finaleIntensityMode??DEFAULTS.finaleIntensity);
  setPair('ledBrightness',s.ledBrightness??DEFAULTS.ledBrightness);
  setPair('audioVolume',s.volume??DEFAULTS.audioVolume);
  setSelect('testMode',s.effectTestMode?1:0);
  setVal('eventTitleCfg',safeText(s.eventTitle||DEFAULTS.eventTitle,DEFAULTS.eventTitle,54));
  setVal('eventSubtitleCfg',safeText(s.eventSubtitle||DEFAULTS.eventSubtitle,DEFAULTS.eventSubtitle,64));
  setVal('presentedByCfg',safeText(s.presentedByLine||DEFAULTS.presentedBy,DEFAULTS.presentedBy,84));
  setVal('attractMessagesCfg',safeText(s.attractMessages||DEFAULTS.attractMessages,DEFAULTS.attractMessages,240));
  setVal('lastTenTextCfg',safeText(s.lastTenText||DEFAULTS.lastTenText,DEFAULTS.lastTenText,24));
  setSelect('showPresentedCfg',s.showPresentedByLine?1:0);
  setSelect('showLiveBrandCfg',s.showEventBrandingInLive?1:0);
  setSelect('customNamesEnabledCfg',s.customTeamNamesEnabled?1:0);
  setVal('blueTeamNameCfg',teamNameBlue(s));
  setVal('orangeTeamNameCfg',teamNameOrange(s));
  setSelect('lastTenDramaticCfg',s.lastTenDramaticMode?1:0);
  setSelect('autoReturnCfg',s.autoReturnToStandbyEnabled?1:0);
  setPair('autoReturnDelayCfg',s.autoReturnToStandbyDelayMs??DEFAULTS.autoReturnDelayMs);
  setPair('postMatchHoldCfg',s.postMatchMinimumHoldMs??DEFAULTS.postMatchHoldCfg);
  setSelect('matchFormatCfg',String(s.matchFormat||DEFAULTS.matchFormatCfg).toLowerCase());
  setSelect('demoLockCfg',s.demoLockModeEnabled?1:0);
  setSelect('operatorPinEnabledCfg',s.operatorPinEnabled?1:0);
  if(exists('recipe'))el('recipe').value=recipe(s);
}
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
async function update(){
  try{
    const r=await fetch(API_ENDPOINTS.state,{cache:'no-store'});
    if(!r.ok)throw new Error('state http '+r.status);
    const s=await r.json();

    latest=s;
    lastGoodUpdateMs=Date.now();
    consecutiveUpdateErrors=0;
    setConnectionVisual(true,s.ip?('Online '+s.ip):'Online');

    paintCommon('Setup',s);
    paintCommon('Op',s);
    syncMatchLength(s);
    if(!['INPUT','SELECT'].includes(document.activeElement.tagName))syncControls(s);
    applyDemoLockUI(s);
    applyOperatorPinGate(s);

    if(exists('scoreboardHint')){
      const ip=s.ip||'192.168.4.1';
      el('scoreboardHint').textContent='http://'+ip+'/scoreboard';
    }
    if(exists('readyMatchFormat')){
      const twoHalf=String(s.matchFormat||'single').toLowerCase()==='two_half';
      el('readyMatchFormat').textContent=twoHalf?'Two Half Match':'Single Match';
    }
    if(exists('resumeSecondHalfBtn')){
      const twoHalf=String(s.matchFormat||'single').toLowerCase()==='two_half';
      el('resumeSecondHalfBtn').classList.toggle('hidden',!twoHalf);
      el('resumeSecondHalfBtn').disabled=!twoHalf;
    }
    if(ROUTE_MODE==='scoreboard' || scoreboardPinned){
      forceScoreboardVisible();
    }

    if(exists('resumeGoalBtn')){
      const resumeBtn=el('resumeGoalBtn');
      const paused=(normState(s.gameState)==='GOAL_PAUSE');
      resumeBtn.disabled=!paused;
      if(paused&&s.autoResumeAfterGoal){
        const remaining=Math.ceil(Number(s.autoResumeGoalRemainingMs||0)/1000);
        resumeBtn.textContent='Resume Now - Auto in '+remaining+'s';
      }else{
        resumeBtn.textContent=paused?'Resume Match After Goal':'Resume After Goal';
      }
      resumeBtn.classList.toggle('attention',paused);
    }

    ['Setup','Op'].forEach(p=>{
      if(exists('ipBadge'+p))el('ipBadge'+p).textContent='IP '+(s.ip||'---');
      if(exists('audioBadge'+p))el('audioBadge'+p).textContent=s.audioBusy?'Audio Playing':'Audio Idle';
      if(exists('trackInfo'+p)){
        const trackNum=Number(s.currentTrack||0);
        const label=safeText(s.currentTrackLabel||'Idle','Idle',26);
        el('trackInfo'+p).textContent='Track '+trackNum+' - '+label;
      }
      if(exists('ledMode'+p))el('ledMode'+p).textContent=prettyMode(s.ledMode);
      if(exists('scoring'+p))el('scoring'+p).textContent=s.scoringEnabled?'Enabled':'Disabled';

      const lp=sensorPair(s.leftSensorA,s.leftSensorB,s.leftGoalArmed);
      const rp=sensorPair(s.rightSensorA,s.rightSensorB,s.rightGoalArmed);
      if(exists('leftPair'+p)){el('leftPair'+p).textContent=lp[0];el('leftPair'+p).className='v '+lp[1]}
      if(exists('rightPair'+p)){el('rightPair'+p).textContent=rp[0];el('rightPair'+p).className='v '+rp[1]}
    });

    if(exists('lastEventOp'))el('lastEventOp').textContent=s.lastEvent||'Ready';
    if(exists('rejectsOp'))el('rejectsOp').textContent=Number(s.leftGoalRejectedSingles||0)+Number(s.rightGoalRejectedSingles||0);
    if(exists('readyMatchLength')){
      const twoHalf=String(s.matchFormat||'single').toLowerCase()==='two_half';
      const secs=Math.round(Number(s.matchDurationMs||120000)/1000);
      el('readyMatchLength').textContent=twoHalf?(secs+'s / half'):(secs+'s');
    }
    if(exists('readyAudienceState')){
      const modeName=audienceOverrideName(s);
      el('readyAudienceState').textContent=modeName;
    }
    if(exists('readySensorHealth')){
      const healthy=Boolean(s.leftGoalArmed)&&Boolean(s.rightGoalArmed);
      el('readySensorHealth').textContent=healthy?'Ready':'Check Alignment';
      el('readySensorHealth').className='v '+(healthy?'ok':'warn');
    }
    if(Array.isArray(s.eventLog)&&exists('eventLogOp')){
      el('eventLogOp').innerHTML=s.eventLog.slice().reverse().map(e=>'<div class="logItem">'+esc(e)+'</div>').join('');
    }

    updatePublic(s);
    forceScoreboardVisible();
    updateAudioShowFlowPanel(s);
    updateDiagnosticsPanel(s);
  }catch(e){
    consecutiveUpdateErrors++;
    if(exists('stateChipSetup'))el('stateChipSetup').textContent='Offline';
    if(exists('stateChipOp'))el('stateChipOp').textContent='Offline';
    if(exists('diagLiveStamp'))el('diagLiveStamp').textContent='Waiting for reconnect...';

    if(lastGoodUpdateMs===0||Date.now()-lastGoodUpdateMs>CONNECTION_STALE_MS){
      setConnectionVisual(false,'Reconnecting');
    }
  }
}

function initPairs(){
  ['matchSecs','ledBrightness','audioVolume','countStep','fieldPeriod','hexRotation','hexTrail','fieldMin','fieldMax','hexBright','goalDuration','goalTravel','goalInterval','goalEmit','goalPop','autoGoalResumeMs','finaleDuration','winnerSpeed','rainbowSpeed','ringSpacing','lastTenPeriod','lastTenMin','lastTenMax','idleSpeed','idleMin','idleMax','autoReturnDelayCfg','postMatchHoldCfg'].forEach(id=>linkPair(id));
  ['blueColor','orangeColor','hex1Color','hex2Color','hex3Color','fieldColor','startColor','lastTenColor'].forEach(id=>el(id)?.addEventListener('change',applyColors));
  ['countdownMode','hexStyle','lastTenEnabled'].forEach(id=>el(id)?.addEventListener('change',applyCountdown));
  ['goalMode'].forEach(id=>el(id)?.addEventListener('change',applyGoal));
  ['autoGoalResumeEnabled'].forEach(id=>el(id)?.addEventListener('change',applyGoalResume));
  ['finaleIntensityCfg'].forEach(id=>el(id)?.addEventListener('change',()=>{applyFinale();applyShowFlowConfig()}));
  ['lastTenDramaticCfg','autoReturnCfg','matchFormatCfg','demoLockCfg','operatorPinEnabledCfg'].forEach(id=>el(id)?.addEventListener('change',applyShowFlowConfig));
}
initPairs();
['pointerdown','touchstart','mousemove'].forEach(evt=>{
  el('publicMode')?.addEventListener(evt,pulsePublicControls,{passive:true});
});
syncScoreboardPinnedBody();
if(ROUTE_MODE==='scoreboard'){
  document.body.classList.add('routeScoreboard');
  showPublic();
}else if(scoreboardPinned){
  forceScoreboardVisible();
}else if(ROUTE_PATH==='/operator'){
  showOperator();
}
el('confirmBackdrop')?.addEventListener('click',e=>{if(e.target===el('confirmBackdrop'))closeConfirm()});
el('pinBackdrop')?.addEventListener('click',e=>{if(e.target===el('pinBackdrop')){if(exists('operatorPinInput'))el('operatorPinInput').focus();}});
document.addEventListener('keydown',e=>{if(e.key==='Escape')closeConfirm()});
document.addEventListener('keydown',e=>{if(e.key==='Enter'&&el('pinBackdrop')?.classList.contains('active'))verifyOperatorPin()});
setInterval(update,220);update();
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
  server.sendHeader("Location", "/operator", true);
  server.send(302, "text/plain", "Redirecting to /operator");
}

static void handleOperatorPage() {
  sendNoCache();
  server.send_P(200, "text/html", INDEX_HTML);
}

static void handleScoreboardPage() {
  sendNoCache();
  server.send_P(200, "text/html", INDEX_HTML);
}

static unsigned long clampMatchSeconds(unsigned long seconds);

static void handleState() {
  updateAudioState();
  sendNoCache();
  server.send(200, "application/json", stateJson());
}

static void handleAudioWindow() {
  uint16_t track = 0;
  if (server.hasArg("track")) {
    track = (uint16_t)server.arg("track").toInt();
  }

  if (track == 0 || track > MAX_TRACK) {
    addEvent("Audio window ignored: invalid track");
    server.send(200, "application/json", stateJson());
    return;
  }

  uint16_t startSec = trackWindowStartSec[track];
  uint16_t endSec = trackWindowEndSec[track];
  if (server.hasArg("start")) startSec = (uint16_t)server.arg("start").toInt();
  if (server.hasArg("end")) endSec = (uint16_t)server.arg("end").toInt();

  setAudioWindowTrack(track, startSec, endSec);

  if (trackActive && currentTrack == track) {
    currentTrackStartSec = trackWindowStartSec[track];
    currentTrackEndSec = trackWindowEndSec[track];
  }

  String msg = "Audio window track ";
  msg += String(track);
  msg += " -> ";
  msg += String(trackWindowStartSec[track]);
  msg += "s to ";
  msg += String(trackWindowEndSec[track]);
  msg += "s";
  addEvent(msg);

  server.send(200, "application/json", stateJson());
}

static void handleAudioWindowSave() {
  bool ok = saveAudioWindowsToPreferences();
  addEvent(ok ? "Audio window settings saved" : "Audio window save failed");
  server.send(200, "application/json", stateJson());
}

static void handleAudioWindowReset() {
  bool ok = resetAudioWindowsToDefaultsAndSave();
  if (trackActive && currentTrack > 0 && currentTrack <= MAX_TRACK) {
    currentTrackStartSec = trackWindowStartSec[currentTrack];
    currentTrackEndSec = trackWindowEndSec[currentTrack];
  }
  addEvent(ok ? "Audio windows reset to defaults" : "Audio window reset failed");
  server.send(200, "application/json", stateJson());
}

static void handleAudioFlow() {
  String action = "";
  if (server.hasArg("action")) {
    action = server.arg("action");
  }
  action.toLowerCase();

  if (action == "downtime") {
    enterStandbyIdleMode();
    audioShowState = AUDIO_SHOW_DOWNTIME;
    addEvent("Audio flow: downtime idle");
  } else if (action == "start_game_flow") {
    unsigned long seconds = clampMatchSeconds(matchDurationMs / 1000UL);
    startMatchWithDuration(seconds);
    addEvent("Audio flow: start game (countdown -> gameplay)");
  } else if (action == "goal") {
    audioShowState = AUDIO_SHOW_GOAL_ONE_SHOT;
    audioPlayTrackInterrupting(AUDIO_TRACK_GOAL, "audio-flow-goal");
    addEvent("Audio flow: goal sound");
  } else if (action == "winner") {
    audioShowState = AUDIO_SHOW_WINNER;
    audioPlayTrackInterrupting(AUDIO_TRACK_WINNER, "audio-flow-winner");
    addEvent("Audio flow: winner celebration");
  } else if (action == "stop") {
    audioStopCommon();
    addEvent("Audio flow: stopped");
  } else {
    addEvent("Audio flow ignored: unknown action");
  }

  server.send(200, "application/json", stateJson());
}

static void handleStart() {
  unsigned long seconds = DEFAULT_MATCH_DURATION_SECONDS;

  if (server.hasArg("seconds")) {
    seconds = (unsigned long)server.arg("seconds").toInt();
  }

  seconds = clampMatchSeconds(seconds);
  startMatchWithDuration(seconds);
  server.send(200, "application/json", stateJson());
}

static void handlePreGame() {
  enterPreGameMode();
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
    audioShowState = AUDIO_SHOW_GOAL_ONE_SHOT;
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
    audioShowState = AUDIO_SHOW_GOAL_ONE_SHOT;
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

    if (track > 0 && track <= MAX_TRACK) {
      uint16_t startSec = trackWindowStartSec[track];
      uint16_t endSec = trackWindowEndSec[track];
      if (server.hasArg("start")) startSec = (uint16_t)server.arg("start").toInt();
      if (server.hasArg("end")) endSec = (uint16_t)server.arg("end").toInt();
      setAudioWindowTrack(track, startSec, endSec);

      audioPlayTrackForced(track);
      String msg = "Manual audio track ";
      msg += String(track);
      msg += " window ";
      msg += String(trackWindowStartSec[track]);
      msg += "s-";
      msg += String(trackWindowEndSec[track]);
      msg += "s";
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

static void applyCountdownAnchorColors(CRGB startColor, CRGB midColor, CRGB finalColor) {
  // Three web color pickers now drive the full 10-second countdown palette:
  //   startColor = seconds 10 to 6
  //   midColor   = seconds 5 to 3
  //   finalColor = seconds 2 to 1
  // This keeps the UI simple while the firmware renders a full 10-step show.
  for (uint8_t i = 0; i < COUNTDOWN_COLOR_COUNT; i++) {
    if (i <= 4) {
      uint8_t amount = (uint8_t)((uint16_t)i * 255U / 4U);
      countdownStepColors[i] = blend(startColor, midColor, amount);
    } else {
      uint8_t amount = (uint8_t)((uint16_t)(i - 5U) * 255U / 4U);
      countdownStepColors[i] = blend(midColor, finalColor, amount);
    }
  }

  countdownHexColor = countdownStepColors[0];
}

static void handleColors() {
  if (server.hasArg("blue")) {
    blueGoalColor = parseWebHexColor(server.arg("blue"), DEFAULT_BLUE_GOAL_COLOR);
  }

  if (server.hasArg("orange")) {
    orangeGoalColor = parseWebHexColor(server.arg("orange"), DEFAULT_ORANGE_GOAL_COLOR);
  }

  bool countdownColorChanged = server.hasArg("hex") || server.hasArg("hex1") || server.hasArg("hex2") || server.hasArg("hex3");
  if (countdownColorChanged) {
    CRGB startColor = countdownStepColors[0];
    CRGB midColor = countdownStepColors[5];
    CRGB finalColor = countdownStepColors[COUNTDOWN_COLOR_COUNT - 1];

    if (server.hasArg("hex")) {
      startColor = parseWebHexColor(server.arg("hex"), DEFAULT_COUNTDOWN_COLOR_10);
    }
    if (server.hasArg("hex1")) {
      startColor = parseWebHexColor(server.arg("hex1"), DEFAULT_COUNTDOWN_COLOR_10);
    }
    if (server.hasArg("hex2")) {
      midColor = parseWebHexColor(server.arg("hex2"), DEFAULT_COUNTDOWN_COLOR_5);
    }
    if (server.hasArg("hex3")) {
      finalColor = parseWebHexColor(server.arg("hex3"), DEFAULT_COUNTDOWN_COLOR_1);
    }

    applyCountdownAnchorColors(startColor, midColor, finalColor);
  }

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

  if (server.hasArg("finaleIntensity")) {
    int requested = server.arg("finaleIntensity").toInt();
    if (requested < 0) requested = 0;
    if (requested > 2) requested = 2;
    finaleIntensityMode = (uint8_t)requested;
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
  setLedMode(LED_MODE_LAST_TEN);
  addEvent("Test final-30 warning LED effect");
  server.send(200, "application/json", stateJson());
}

static void handleTestIdle() {
  setLedMode(LED_MODE_ON_IDLE);
  addEvent("Test in-game idle LED effect");
  server.send(200, "application/json", stateJson());
}

static void handleAudienceMode() {
  if (server.hasArg("mode")) {
    String mode = server.arg("mode");
    mode.toLowerCase();

    if (mode == "attract") {
      audienceOverrideMode = AUDIENCE_OVERRIDE_ATTRACT;
      addEvent("Audience mode override: attract");
    } else if (mode == "post" || mode == "post_match" || mode == "postmatch") {
      audienceOverrideMode = AUDIENCE_OVERRIDE_POST_MATCH;
      addEvent("Audience mode override: post match");
    } else {
      audienceOverrideMode = AUDIENCE_OVERRIDE_AUTO;
      addEvent("Audience mode override: auto");
    }
  }

  server.send(200, "application/json", stateJson());
}

static void handleReplayFinale() {
  replayFinale();
  server.send(200, "application/json", stateJson());
}

static void handleBrandingConfig() {
  if (server.hasArg("title")) {
    eventTitle = sanitizeDisplayText(server.arg("title"), DEFAULT_EVENT_TITLE, 54);
  }

  if (server.hasArg("subtitle")) {
    eventSubtitle = sanitizeDisplayText(server.arg("subtitle"), DEFAULT_EVENT_SUBTITLE, 64);
  }

  if (server.hasArg("presentedBy")) {
    presentedByLine = sanitizeDisplayText(server.arg("presentedBy"), DEFAULT_PRESENTED_BY, 84);
  }

  if (server.hasArg("showPresentedBy")) {
    showPresentedByLine = server.arg("showPresentedBy").toInt() != 0;
  }

  if (server.hasArg("showLiveBrand")) {
    showEventBrandingInLive = server.arg("showLiveBrand").toInt() != 0;
  }

  if (server.hasArg("attractMessages")) {
    attractMessagesCsv = sanitizeDisplayText(server.arg("attractMessages"), DEFAULT_ATTRACT_MESSAGES, 240);
  }

  if (server.hasArg("lastTenText")) {
    lastTenWarningText = sanitizeDisplayText(server.arg("lastTenText"), DEFAULT_LAST_TEN_TEXT, 24);
  }

  addEvent("Branding and audience text updated");
  server.send(200, "application/json", stateJson());
}

static void handleTeamIdentityConfig() {
  if (server.hasArg("enabled")) {
    customTeamNamesEnabled = server.arg("enabled").toInt() != 0;
  }

  if (server.hasArg("blueName")) {
    blueTeamCustomName = sanitizeDisplayText(server.arg("blueName"), DEFAULT_BLUE_TEAM_NAME, 24);
  }

  if (server.hasArg("orangeName")) {
    orangeTeamCustomName = sanitizeDisplayText(server.arg("orangeName"), DEFAULT_ORANGE_TEAM_NAME, 24);
  }

  addEvent("Team identity updated");
  server.send(200, "application/json", stateJson());
}

static void handleShowFlowConfig() {
  if (server.hasArg("autoReturn")) {
    autoReturnToStandbyEnabled = server.arg("autoReturn").toInt() != 0;
  }

  if (server.hasArg("autoReturnDelay")) {
    autoReturnToStandbyDelayMs = clampUL((unsigned long)server.arg("autoReturnDelay").toInt(), 5000UL, 90000UL);
  }

  if (server.hasArg("postMatchHold")) {
    postMatchMinimumHoldMs = clampUL((unsigned long)server.arg("postMatchHold").toInt(), 3000UL, 30000UL);
  }

  if (server.hasArg("lastTenDramatic")) {
    lastTenDramaticMode = server.arg("lastTenDramatic").toInt() != 0;
  }

  if (server.hasArg("finaleIntensity")) {
    int requested = server.arg("finaleIntensity").toInt();
    if (requested < 0) requested = 0;
    if (requested > 2) requested = 2;
    finaleIntensityMode = (uint8_t)requested;
  }

  if (server.hasArg("matchFormat")) {
    matchFormatMode = parseMatchFormatMode(server.arg("matchFormat"));
  }

  if (server.hasArg("demoLock")) {
    demoLockModeEnabled = server.arg("demoLock").toInt() != 0;
  }

  if (server.hasArg("operatorPinEnabled")) {
    operatorPinEnabled = server.arg("operatorPinEnabled").toInt() != 0;
  }

  enforcePostMatchTimingConstraints();

  addEvent("Show flow settings updated");
  server.send(200, "application/json", stateJson());
}

static void handleSettingsSave() {
  enforcePostMatchTimingConstraints();
  bool ok = saveSettingsToPreferences();
  addEvent(ok ? "Settings saved to ESP32 preferences" : "Settings save failed");
  server.send(200, "application/json", stateJson());
}

static void handleSettingsLoad() {
  bool ok = loadSettingsFromPreferences();
  if (!ok) {
    audioSetVolumePercent(currentVolumePercent);
    setLedBrightnessPercent(currentLedBrightnessPercent);
  }
  addEvent(ok ? "Settings loaded from ESP32 preferences" : "No saved settings found. Defaults loaded.");
  server.send(200, "application/json", stateJson());
}

static void handleSettingsReset() {
  bool ok = resetSavedSettingsToDefaults();
  addEvent(ok ? "Saved settings reset to defaults" : "Saved settings reset failed");
  server.send(200, "application/json", stateJson());
}

static void handleOperatorPinVerify() {
  String pin;
  if (server.hasArg("pin")) {
    pin = server.arg("pin");
  }
  bool ok = validateOperatorPinCandidate(pin);
  String out = "{\"ok\":";
  out += ok ? "true" : "false";
  out += "}";
  server.send(200, "application/json", out);
}

static void handleRehearsal() {
  String sceneArg = "none";
  if (server.hasArg("scene")) {
    sceneArg = server.arg("scene");
  }

  RehearsalSceneMode scene = parseRehearsalSceneMode(sceneArg);
  unsigned long requestedDuration = rehearsalSceneDurationMs;
  if (server.hasArg("durationMs")) {
    requestedDuration = (unsigned long)server.arg("durationMs").toInt();
  }
  requestedDuration = clampUL(requestedDuration, 1500UL, 60000UL);

  if (scene == REHEARSAL_SCENE_NONE) {
    rehearsalSceneActive = false;
    rehearsalSceneMode = REHEARSAL_SCENE_NONE;
    rehearsalSceneStartMs = 0;
    addEvent("Rehearsal preview cleared");
    server.send(200, "application/json", stateJson());
    return;
  }

  rehearsalSceneActive = true;
  rehearsalSceneMode = scene;
  rehearsalSceneStartMs = millis();
  rehearsalSceneDurationMs = requestedDuration;

  if (scene == REHEARSAL_SCENE_ATTRACT) {
    audienceOverrideMode = AUDIENCE_OVERRIDE_ATTRACT;
    setLedMode(LED_MODE_OFF_IDLE);
  } else if (scene == REHEARSAL_SCENE_COUNTDOWN) {
    audienceOverrideMode = AUDIENCE_OVERRIDE_AUTO;
    setLedMode(LED_MODE_COUNTDOWN);
  } else if (scene == REHEARSAL_SCENE_GOAL_BLUE) {
    audienceOverrideMode = AUDIENCE_OVERRIDE_AUTO;
    setLedMode(LED_MODE_BLUE_GOAL);
  } else if (scene == REHEARSAL_SCENE_GOAL_ORANGE) {
    audienceOverrideMode = AUDIENCE_OVERRIDE_AUTO;
    setLedMode(LED_MODE_ORANGE_GOAL);
  } else if (scene == REHEARSAL_SCENE_FINAL_TEN) {
    audienceOverrideMode = AUDIENCE_OVERRIDE_AUTO;
    setLedMode(LED_MODE_LAST_TEN);
  } else if (scene == REHEARSAL_SCENE_WIN_BLUE || scene == REHEARSAL_SCENE_WIN_ORANGE || scene == REHEARSAL_SCENE_WIN_TIE) {
    audienceOverrideMode = AUDIENCE_OVERRIDE_POST_MATCH;
    setLedMode(LED_MODE_MATCH_END);
  } else if (scene == REHEARSAL_SCENE_READABILITY) {
    audienceOverrideMode = AUDIENCE_OVERRIDE_AUTO;
    rehearsalSceneDurationMs = 36000UL;
    setLedMode(LED_MODE_OFF_IDLE);
  }

  String msg = "Rehearsal preview: ";
  msg += rehearsalSceneModeName();
  addEvent(msg);
  server.send(200, "application/json", stateJson());
}

static void handleNotFound() {
  String uri = server.uri();

  // Route recovery for tablet browsers that request cached or slash variants.
  if (server.method() == HTTP_GET) {
    if (uri == "/operator/" || uri == "/operator/index.html" || uri == "/operator/index.htm") {
      server.sendHeader("Location", "/operator", true);
      server.send(302, "text/plain", "Redirecting to /operator");
      return;
    }
    if (uri == "/scoreboard/" || uri == "/scoreboard/index.html" || uri == "/scoreboard/index.htm") {
      server.sendHeader("Location", "/scoreboard", true);
      server.send(302, "text/plain", "Redirecting to /scoreboard");
      return;
    }
    if (uri == "/index.html" || uri == "/index.htm") {
      handleRoot();
      return;
    }
    if (uri == "/favicon.ico") {
      server.send(204, "text/plain", "");
      return;
    }
    if (!uri.startsWith("/api/")) {
      server.sendHeader("Location", "/operator", true);
      server.send(302, "text/plain", "Redirecting to /operator");
      return;
    }
  }

  if (uri.startsWith("/api/")) {
    String out = "{\"error\":\"not_found\",\"path\":\"";
    out += uri;
    out += "\"}";
    server.send(404, "application/json", out);
  } else {
    server.send(404, "text/plain", "Not found");
  }
}

static void beginWeb() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/index.html", HTTP_GET, handleRoot);
  server.on("/index.htm", HTTP_GET, handleRoot);
  server.on("/operator", HTTP_GET, handleOperatorPage);
  server.on("/operator/", HTTP_GET, handleOperatorPage);
  server.on("/scoreboard", HTTP_GET, handleScoreboardPage);
  server.on("/scoreboard/", HTTP_GET, handleScoreboardPage);
  server.on("/api/state", HTTP_GET, handleState);

  server.on("/api/pre-game", HTTP_POST, handlePreGame);
  server.on("/api/start", HTTP_POST, handleStart);
  server.on("/api/standby", HTTP_POST, handleStandby);
  server.on("/api/audience-mode", HTTP_POST, handleAudienceMode);
  server.on("/api/replay-finale", HTTP_POST, handleReplayFinale);
  server.on("/api/rehearsal", HTTP_POST, handleRehearsal);
  server.on("/api/branding", HTTP_POST, handleBrandingConfig);
  server.on("/api/team-identity", HTTP_POST, handleTeamIdentityConfig);
  server.on("/api/show-flow", HTTP_POST, handleShowFlowConfig);
  server.on("/api/settings/save", HTTP_POST, handleSettingsSave);
  server.on("/api/settings/load", HTTP_POST, handleSettingsLoad);
  server.on("/api/settings/reset", HTTP_POST, handleSettingsReset);
  server.on("/api/operator-pin-verify", HTTP_POST, handleOperatorPinVerify);
  server.on("/api/match-length", HTTP_POST, handleMatchLength);
  server.on("/api/resume-second-half", HTTP_POST, handleResumeSecondHalf);
  server.on("/api/resume-after-goal", HTTP_POST, handleResumeAfterGoal);
  server.on("/api/reset", HTTP_POST, handleReset);
  server.on("/api/end", HTTP_POST, handleEnd);
  server.on("/api/score-blue", HTTP_POST, handleScoreBlue);
  server.on("/api/score-orange", HTTP_POST, handleScoreOrange);

  server.on("/api/fake-blue-goal", HTTP_POST, handleFakeBlueGoal);
  server.on("/api/fake-orange-goal", HTTP_POST, handleFakeOrangeGoal);
  server.on("/api/fake-hit", HTTP_POST, handleFakeHit);
  server.on("/api/fake-big-hit", HTTP_POST, handleFakeBigHit);

  server.on("/api/play", HTTP_POST, handlePlay);
  server.on("/api/stop-audio", HTTP_POST, handleStopAudio);
  server.on("/api/volume", HTTP_POST, handleVolume);
  server.on("/api/audio-window", HTTP_POST, handleAudioWindow);
  server.on("/api/audio-window-save", HTTP_POST, handleAudioWindowSave);
  server.on("/api/audio-window-reset", HTTP_POST, handleAudioWindowReset);
  server.on("/api/audio-flow", HTTP_POST, handleAudioFlow);

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

#if BALL_LINK_ENABLED
  // AP+STA keeps the audience AP online while allowing ESP-NOW ball link traffic.
  WiFi.mode(WIFI_AP_STA);
#else
  WiFi.mode(WIFI_AP);
#endif
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
    } else if (c == 'w' || c == 'W') {
      audioPlayTrackForced(10);
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
    } else if (c == 'g' || c == 'G') {
      enterPreGameMode();
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
  Serial.println("FIFA Robot Soccer Demo Controller v4.65 Final-30 Max Brightness Clarification");
  Serial.println("Scoreboard + 4 Sensors + DY-HV20T + Spatial LEDs");
  Serial.println("LED SAFETY: 50% max brightness, full-white allowed at capped output");
  Serial.println("==================================================");

  FastLED.addLeds<CHIPSET, LED_PIN, COLOR_ORDER>(leds, LED_COUNT);

  enforceLedBrightnessLimit();
  FastLED.clear();
  safeFastLedShow();

  setLedBrightnessPercent(currentLedBrightnessPercent);

  buildXYMap();

  setupSensors();

  beginAudio();
  bool settingsLoaded = loadSettingsFromPreferences();
  bool audioWindowsLoaded = loadAudioWindowsFromPreferences();
  if (!settingsLoaded) {
    audioSetVolumePercent(currentVolumePercent);
    setLedBrightnessPercent(currentLedBrightnessPercent);
    Serial.println("[SETTINGS] No saved profile found. Using defaults.");
  } else {
    Serial.println("[SETTINGS] Loaded saved profile from Preferences.");
  }
  if (!audioWindowsLoaded) {
    loadDefaultAudioWindows();
    Serial.println("[AUDIO] No saved audio windows found. Using defaults.");
  } else {
    Serial.println("[AUDIO] Loaded audio windows from Preferences.");
  }
  delay(100);

  setupWiFi();

#if BALL_LINK_ENABLED
  setupBallLink();
#endif

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

  applyAutomaticLedModeForState(true);

  addEvent("System booted");

  Serial.println("[READY] Serial fallback:");
  Serial.println("  s   = start default 120 second match");
  Serial.println("  t   = quick 20 second test match");
  Serial.println("  r   = resume after goal pause");
  Serial.println("  e   = end match");
  Serial.println("  0   = reset");
  Serial.println("  g   = pre-game buildup mode");
  Serial.println("  p   = standby / idle mode");
  Serial.println("  b   = fake Blue goal");
  Serial.println("  o   = fake Orange goal");
  Serial.println("  h/H = hit / big hit");
  Serial.println("  1..9 = manual audio tracks 1 to 9");
  Serial.println("  w   = manual audio track 10 (winner)");
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

#if BALL_LINK_ENABLED
  serviceBallLink();
#endif

  updateAudioState();
  updateGame();
  servicePreGameAudio();

  if (ledModeUsesAutomaticStateMapping() && !rehearsalSceneActive) {
    applyAutomaticLedModeForState();
  }

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
