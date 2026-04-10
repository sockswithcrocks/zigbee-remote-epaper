# zigbee-remote-epaper

Custom Zigbee remote based on ESP32-C6 with e-paper display, physical buttons and gesture input.  
Designed to be fully configurable via Home Assistant using blueprints, scripts and helpers.

## Status

Work in progress.  
Core concept is defined, implementation is incomplete.

## Concept

The remote separates input and logic:

- Hardware only sends events (buttons, gestures, states)
- Logic is handled in Home Assistant
- Display content is generated from Home Assistant states

Each page defines its own:

- Button mapping
- Display layout
- Actions

## Input System

- 9 physical buttons
  - 2 fixed: page navigation
  - 7 context-dependent (per page)
- Touchpad (gesture-based)
  - Tap
  - Swipe up/down/left/right
- Optional wake trigger via gyro
- Microphone input (planned, e.g. voice control / Apple TV)

## Pages

Pages are cycled via dedicated buttons.

Each page can define completely different behavior.

Example (media player):

- Back
- Home
- Mute
- Volume up/down
- Play/Pause
- Microphone activation

## Display

- Waveshare e-paper
- Layout defined per page
- Shows:
  - Button labels
  - Icons (e.g. play/pause)
  - States from Home Assistant
  - System information (optional)

Display content is driven entirely by Home Assistant.

## Hardware

Core components:

- ESP32-C6-WROOM-1-N8
- Waveshare e-paper display
- Waveshare IO expander (buttons)
- Touchpad 5 Click (SQI5550)

Planned / not finalized:

- I2C gyro (wake / sleep handling)
- I2C microphone

## Power

- Battery powered
- Deep sleep enabled
- Wake-up via:
  - Button press
  - Motion (planned via gyro)

## Software Architecture

- Zigbee-based communication (ESP32-C6)
- Custom quirks for integration
- Designed to work with systems supporting Zigbee quirks
- Currently focused on Home Assistant

Home Assistant handles:

- Logic (scripts, blueprints)
- State evaluation
- Display data generation

## Home Assistant Integration

Configuration is intended to be flexible via:

- Blueprints
- Scripts
- Helpers

Typical flow:

1. Input event (button / gesture)
2. Event sent via Zigbee
3. Home Assistant processes logic
4. Actions executed
5. Display state updated and pushed back

## Layout Example

+------------------------------+
|         {{HEADER}}           |
+--------------+---------------+
|    {{01}}    |    {{02}}     |
+--------------+---------------+
|    {{03}}    |    {{04}}     |
+--------------+---------------+
|    {{05}}    |    {{06}}     |
+--------------+---------------+
|    {{07}}    |    {{08}}     |
+--------------+---------------+
|    {{09}}    |    {{10}}     |
+--------------+---------------+

### Description

- `{{HEADER}}`  
  Page title and global status (e.g. connection, battery)

- `{{01}} – {{10}}`  
  Context-dependent fields:
  - Button labels
  - Icons
  - State indicators (from Home Assistant)

Each field can represent:
- a button action
- a status value (e.g. temperature)
- a symbolic state (e.g. play/pause, on/off)

Mapping is defined per page in Home Assistant.

## Roadmap

- Basic input handling (buttons, gestures)
- Zigbee communication layer
- Home Assistant blueprint
- Display rendering pipeline
- Power optimization
- PCB design
- 3D printed enclosure

## License

TBD
