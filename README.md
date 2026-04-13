# Programmable HV ECF System

![Python](https://img.shields.io/badge/Python-3.x-blue)
![Firmware](https://img.shields.io/badge/Firmware-Arduino-teal)
![Protocol](https://img.shields.io/badge/Protocol-HV%2010--token-orange)

This project controls an 8-channel high-voltage (HV) routing system with:
- 8 relay outputs
- 2 independent HV setpoints (HV1/HV2)
- 1x MCP4728 DAC at I2C address `0x60` (channels A/B used)

The GUI workflow remains tab-based:
- Communication
- Program Action
- Live Control

## Architecture

- Firmware target: `src/main.ino`
- GUI: `python_scripts/gui.py`
- Serial transport/helpers: `python_scripts/communication.py`
- Entry point: `main.py`

The GUI supports channels 1-20 for programming and plotting.
- Hardware-driven channels: 1-8
- Virtual channels: 9-20 (valid in GUI, ignored by current firmware output)

## Serial Protocol

All commands are wrapped in angle brackets.

### 1) Control frame (runtime)

`<r1,r2,r3,r4,r5,r6,r7,r8,v1,v2>`

- `r1..r8`: strict relay bits, each must be `0` or `1`
- `v1`, `v2`: HV setpoints in volts (float), clamped to `0..6000`

Example:

`<1,0,1,0,1,0,0,1,2400.0,1800.0>`

### 2) Mapping frame (sent on connect)

`<MAP,m1,m2,m3,m4,m5,m6,m7,m8>`

- `m1..m8`: each is `1` or `2`
- Default map: `[1,1,1,1,1,1,2,2]`

Mapping is intended to be configured before runtime control.

### 3) Telemetry and errors

- Control echo: `ACT:r1,r2,r3,r4,r5,r6,r7,r8,v1,v2`
- Mapping echo: `MAP:m1,m2,m3,m4,m5,m6,m7,m8`
- Error format: `ERR:<CODE>`

Common codes:
- `ERR:MALFORMED`
- `ERR:BAD_MAP`
- `ERR:WATCHDOG`
- `ERR:MAP_LOCKED`

## Safety Behavior

- Boot defaults: relays OFF, HV1/HV2 at 0 V
- Watchdog: if no valid control frame within timeout, relays OFF + HV outputs to 0 V
- GUI Pause/E-Stop/Reset: force relays OFF + HV1/HV2 to 0 V
- Command clamp: HV setpoints are clamped to `0..6000 V`

## HV to DAC Conversion

Firmware computes DAC values using explicit electrical constants:
- `DAC_REFERENCE_VOLTAGE = 5.0 V`
- `HV_AMPLIFIER_GAIN = 1333.33`
- `MAX_PHYSICAL_HV_VOLTAGE = DAC_REFERENCE_VOLTAGE * HV_AMPLIFIER_GAIN`

The conversion path is:
1. Clamp command voltage to safe command range (`0..6000 V`)
2. Clamp against physical max (`MAX_PHYSICAL_HV_VOLTAGE`)
3. Convert HV to DAC voltage by dividing by amplifier gain
4. Map DAC voltage to 12-bit code (`0..4095`)

This keeps the formula correct if you change gain or DAC reference voltage in firmware constants.

## MAP Policy

- GUI exposes an 8-channel mapping editor in Communication tab
- Mapping is sent once at connect via `mapping_on_connect`
- Mapping controls are editable only while disconnected
- Runtime MAP edits from GUI debug box are blocked
- Firmware can lock MAP after first valid control frame (`ERR:MAP_LOCKED`)

## Setup

### Python

Install dependencies:

```bash
pip install -r requirements.txt
```

Run GUI:

```bash
python main.py
```

### Firmware

This repository is configured for PlatformIO (`platformio.ini`) and MCP4728.

Typical build/upload:

```bash
pio run
pio run -t upload
```

