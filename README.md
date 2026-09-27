# arcx ring Kasa bulb controller

`arcx_kasa_ctrl` controls a TP-Link Kasa smart bulb with an arcx ring. The ring connects
through a Nordic nRF52 dongle, and the bulb is controlled over the local network.

| Ring button | Action |
|---|---|
| Center | Turn the bulb on or off |
| Forward | Brightness up by one step (turns the bulb on if it is off) |
| Backward | Brightness down by one step (ignored while the bulb is off) |

Brightness stays between 1% and 100%. Left, right and SOS are ignored.

The project also builds `arcx_ring_client`, which prints the ring's key events, battery level
and device information and is useful for checking the ring on its own.

The ring is an nRF52-based BLE peripheral. When powered on it advertises as
`arcx-<4-digit-hex-serial>`, for example `arcx-e24e`.

## GATT services and characteristics

| Service | UUID | Characteristics |
|---|---|---|
| Generic Access | `0x1800` | Device name, connection parameters |
| Generic Attribute | `0x1801` | Service Changed |
| Device Information | `0x180A` | Manufacturer Name `0x2A29`, Hardware Revision `0x2A27` (e.g. `0.11.0`), Firmware Revision `0x2A26` (e.g. `0.12.4`), Software Revision `0x2A28`, PnP ID `0x2A50` |
| Battery | `0x180F` | Battery Level `0x2A19`: percentage, Read + Notify |
| Secure DFU | `0xFE59` | Over-the-air firmware update |
| arcx custom | `00001666-1212-efde-1523-785feabcd124` | See below |

### arcx custom service

| Characteristic | UUID | Properties | Notes |
|---|---|---|---|
| Key Event | `00001667-1212-efde-1523-785feabcd124` | Read, Write, Notify | Not yet used |
| Extended Key Event | `00001669-1212-efde-1523-785feabcd124` | Read, Write, Notify | Key press and release events, e.g. `05-00-00-00` |

Both characteristics have a Client Characteristic Configuration descriptor (`0x2902`); write
`01 00` to it to enable notifications.

### Key events

The ring has five buttons: center, forward, backward, left and right. Each button sends one
notification when pressed and another when released. Holding the center button for more
than 10 seconds sends an SOS event.

The first byte of the notification is an `ArcxRingState` value:

| Value | Event | Value | Event |
|---|---|---|---|
| 0 | `CENTER_PRESS` | 6 | `LEFT_PRESS` |
| 1 | `CENTER_RELEASE` | 7 | `LEFT_RELEASE` |
| 2 | `FORWARD_PRESS` | 8 | `RIGHT_PRESS` |
| 3 | `FORWARD_RELEASE` | 9 | `RIGHT_RELEASE` |
| 4 | `BACKWARD_PRESS` | 10 | `SOS` |
| 5 | `BACKWARD_RELEASE` | | |

## Build

Requirements: Linux, `gcc`/`g++`, `cmake`, `git`, `libudev-dev`.

```sh
git clone --recurse-submodules <repo-url>
cd arcx_kasa_ctrl
make
```

The first `make` builds Nordic's pc-ble-driver (a git submodule) into `build/`, applying the
patch in `patches/`. The default target is SoftDevice API v5; use `make SD_API=6` if your
dongle runs the v6 connectivity firmware.

## Run

Plug in an nRF52 dongle flashed with Nordic's connectivity firmware
(it shows up as "nRF52 Connectivity"), then run:

```sh
./arcx_kasa_ctrl <serial_port> <ring_name> <bulb_ip>
./arcx_kasa_ctrl /dev/ttyACM0 arcx-e24e 192.168.1.50
```

| Argument / option | Description |
|---|---|
| `serial_port` | Serial port of the dongle, e.g. `/dev/ttyACM0` |
| `ring_name` | Advertised name of the ring, or a prefix of it, e.g. `arcx-e24e` |
| `bulb_ip` | IP address (or host name) of the Kasa bulb |
| `-s <percent>` | Brightness step per press (default `10`) |
| `-t <ms>` | Fade time for each change (default `0`) |
| `-b <baud>` | Baud rate (default `1000000`) |
| `-v`, `-vv` | Verbose output: key events, advertisers and the GATT table; `-vv` adds the driver packet trace |

The program first reads the bulb's state and exits if the bulb can't be reached. It then scans
for the ring and connects to it. Example output:

```
[20:31:00.402] BULB   Desk lamp (KL130(UK)) at 192.168.1.50 is off, brightness 40%
[20:31:01.911] READY  Center: on/off, forward: brighter, backward: dimmer (Ctrl-C to quit)
[20:31:02.290] BULB   Desk lamp on, brightness 40%
[20:31:03.018] BULB   Desk lamp on, brightness 50%
```

The bulb's state is read before every change, so changes made in the Kasa app are picked up.
If several presses arrive while a change is still in progress, they are combined into one.
Press Ctrl-C to disconnect and exit. If the ring disconnects, the program scans and reconnects
automatically.

### Kasa protocol

The bulb is controlled with the Kasa local protocol on TCP port 9999: a 4-byte length followed
by XOR-encrypted JSON. The commands are `system.get_sysinfo` and
`smartlife.iot.smartbulb.lightingservice.transition_light_state`. This works with the
KL/LB-series bulbs (e.g. KL110, KL125, KL130, LB130). Some newer hardware and firmware revisions
accept only TP-Link's authenticated KLAP protocol and refuse connections on port 9999; those bulbs
are not supported. Kasa smart plugs and light strips use different commands and are rejected.

### Ring monitor

```sh
./arcx_ring_client               # connect to the first device named arcx-*
./arcx_ring_client arcx-e24e     # connect to a specific ring
./arcx_ring_client -v            # also list nearby advertisers and the GATT table
```

`arcx_ring_client` takes an optional `-p <port>`; it auto-detects the dongle by default.
