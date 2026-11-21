# FIFA Robot Soccer — Arena Game Controller (DemoDay line)

## Overview

The main game controller for a robot soccer demo built for a FIFA FanFest project at Humber Polytechnic. One ESP32-based LoR Core v3 board runs the whole show: the match state machine, goal scoring, an 827-LED spatial field display, UART audio cues, and a web scoreboard for the operator and the audience. The controller is the field/show controller only; it does not drive the robots. Version v4.65 is the event-ready baseline, documented by the formal handoff manual in `docs/`.

## Features

- Match state machine: Standby → Pre-game → Countdown (10 s) → In Game (120 s default) → Goal pause with auto-resume → Final 30-second warning → Game over → Winner finale, with an optional two-half format
- Dual-sensor goal validation: two E18-D80NK proximity sensors per goal; both must trigger within a 450 ms window to count a goal, with 180 ms rearm, 1600 ms goal lockout, and 15 ms debounce
- Trigger-on-clear scoring: the sensors are active-low and normally see a surface as mounted, so a goal counts when the pair *stops* detecting
- Interrupt-latched sensor capture (v4.32 onward): an ISR records the sensor change and the main loop still validates, scores, and drives audio and LEDs
- 827 WS2812B field LEDs with spatial effects, a 50% brightness safety cap, and full-field white frames blocked
- 10-track audio show flow through a DY-HV20T UART MP3 module
- Operator dashboard plus a separate full-screen iPad audience scoreboard (timer, Blue/Orange scores, match state, countdown, goal overlays, winner screen)
- Operator PIN gate, demo lock, rehearsal scenes, and a settings "recipe" system so tested values can be locked in as firmware defaults
- An ESP-NOW link to the smart ball was coded (v4.52) but not tested; the ball itself is standalone by design, and the link was left untouched for the event

## Hardware

- LoR Core v3 board (ESP32)
- 827 WS2812B LEDs; field output on LoR IO_04 / GPIO27 in v4.65 (earlier versions used IO_05 / GPIO14)
- Four E18-D80NK proximity sensors: Left goal GPIO22 + GPIO21 (Blue score), Right goal GPIO13 + GPIO4 (Orange score)
- DY-HV20T / DY1900A UART MP3 audio module on LoR IO_01/02/03 (GPIO32/25/26)

## Software and architecture notes

- Developed in PlatformIO with VS Code (Arduino framework for ESP32)
- Libraries include FastLED, ESPAsyncWebServer / AsyncTCP, ArduinoOTA, and mDNS
- The controller hosts the operator web UI and the audience scoreboard; network defaults moved from a standalone access point to router/station mode (mDNS `soccerbots.local`) in v4.32
- Scoring parameters, pin maps, and the full event-day operating procedure are documented in the handoff manual in `docs/`

## File and version guide

| File | Description |
|---|---|
| `RobotSoccerGameController_v2_SensorsAudioWeb.ino` | First integrated controller (May 2026): match logic, two goal sensors, audio, Wi-Fi dashboard |
| `FIFA_RobotSoccer_DemoDay_Controller_v2_1.ino` | Full demo system: 827-LED field, four goal sensors, iPad scoreboard, LED safety rules |
| `FIFA_RobotSoccer_DemoDay_Controller_v4_32.ino` | Adds interrupt-latched goal sensing and router/station networking |
| `FIFA_RobotSoccer_DemoDay_Controller_v4_65.ino` | Latest version and event-ready baseline: show-control flow, operator/audience route split, PIN gate, demo lock |
| `docs/FIFA_SoccerBot_FMS_Handoff_v4_65.pdf` | Formal handoff manual for v4.65: pin maps, state machine, scoring logic, LED map, event-day checklist, troubleshooting guide |

## Engineering notes

- **Pin mapping confusion:** early integration work mixed up LoR IO slot numbers and raw GPIO numbers. The fix was to document every signal by both slot and GPIO and verify against the board before testing.
- **Dead sensor rail:** one LoR board's V+ rail measured about 2.183 V, too low to power the goal sensors, versus about 6 V on a good board. The suspect board was flagged and removed rather than debugged around.
- **Interrupt compile failure:** the first interrupt build failed to compile because of Arduino's automatic prototype generation; a forward declaration fixed it.
- **Scoring logic from field testing:** bench testing assumed goals would trigger on detection. As actually mounted, the sensors normally see a surface, so the shipped logic triggers on clear, which matched the physical setup.
- **Locking in what works:** tested settings (match length, brightness, volume, timings) were copied back into the firmware as defaults through the recipe system, so the event-day configuration did not depend on anyone re-entering values.
