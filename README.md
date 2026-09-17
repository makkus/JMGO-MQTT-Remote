# JMGO MQTT Remote

ESP32 firmware that controls a JMGO projector from MQTT. It wakes the projector with BLE advertising packets, sends navigation and power-menu commands over LAN TCP, and publishes simple state updates for Home Assistant or other MQTT clients.

## Features

- BLE wake advertising burst for powering on the projector.
- LAN control over TCP port `9005` for navigation and power actions.
- MQTT command topic for automation.
- MQTT state topic with retained status updates.
- Arduino OTA support for wireless firmware updates.

## Hardware And Stack

- Supported ESP boards:
  - ESP32 Dev Module / ESP32 DevKit.
  - ESP32-WROOM-32 based boards.
  - ESP32-WROVER based boards.
- PlatformIO with Arduino framework.
- Libraries:
  - `h2zero/NimBLE-Arduino`
  - `knolleary/PubSubClient`
- A JMGO projector reachable from the same LAN.
- MQTT broker, such as Mosquitto or Home Assistant MQTT.

## Tested Projectors

- JMGO N1
- JMGO N1S
- JMGO N1 Ultra

## Configuration

Secrets are not stored in tracked source files. Create a local `secrets.h` file in your PlatformIO include path and fill in your local values.

Configure:

- Wi-Fi SSID and password.
- MQTT broker host, port, username, and password.
- OTA hostname and password.
- Projector IP address and LAN control port.

The same variable names are listed in `.env.example` for documentation or external automation. The firmware itself uses `secrets.h`.

## MQTT

Default topics:

```text
jmgo/remote/cmd
jmgo/remote/sequence
jmgo/remote/state
```

Supported commands:

| Command | Action |
| --- | --- |
| `wake` | Sends only the BLE wake burst |
| `on`, `wake_hdmi1`, `power_on` | Wakes the projector, waits for startup, then selects HDMI1 |
| `wake_hdmi2` | Wakes the projector, waits for startup, then selects HDMI2 |
| `hdmi1` | Runs the LAN navigation macro for HDMI1 |
| `hdmi2` | Runs the LAN navigation macro for HDMI2 |
| `power_menu` | Opens the projector power menu |
| `power_off`, `off` | Opens the power menu, moves down, and confirms shutdown |
| `up`, `down`, `left`, `right`, `ok`, `enter` | Sends individual LAN navigation keys |
| `back`, `home`, `menu`, `settings` | Sends the corresponding remote button |
| `volume_up`, `volume_down` | Raises or lowers the projector volume |
| `search` | Opens Android text search |

## Button Sequences

Publish a comma-separated sequence to `jmgo/remote/sequence` to send several
LAN buttons in order. Each item is a supported button name, optionally followed
by `@<milliseconds>` to override the delay after key-up. Buttons use the normal
120 ms press duration. The special `wake` action sends the BLE wake burst. The
final item's delay is ignored.

```text
menu@250,menu
up,up,right@300,ok
wake@60000,home
```

For example:

```bash
mosquitto_pub -h <mqtt-host> -p 1883 -u <user> -P '<password>' -t jmgo/remote/sequence -m 'menu@250,menu'
```

Sequences accept the individual LAN buttons listed above (including `enter` as
an alias for `ok`) and `wake`, up to 16 items. For example,
`wake@60000,home` wakes the projector, waits 60 seconds, then opens Home.
Invalid input, unknown buttons, or delays over 60 seconds are rejected without
sending any buttons.
The state topic reports `sequence_start`, `sequence_done`, `sequence_invalid`,
or `sequence_key_failed`.

Example:

```bash
mosquitto_pub -h <mqtt-host> -p 1883 -u <user> -P '<password>' -t jmgo/remote/cmd -m hdmi1
```

## Build And Upload

Build the serial environment:

```bash
pio run -e esp32dev
```

Upload over USB:

```bash
pio run -e esp32dev -t upload --upload-port <serial-port>
```

Upload over OTA:

Set your ESP32 address and OTA password in the `esp32ota` environment locally, then run:

```bash
pio run -e esp32ota -t upload
```

Monitor serial output:

```bash
pio device monitor -b 115200
```
