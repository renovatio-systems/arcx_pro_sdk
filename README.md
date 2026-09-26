# arcx ring BLE client

C client that connects to an arcx ring through a Nordic nRF52 dongle and prints its key
events, battery level and device information.

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
cd arcx_pro_sdk
make
```

The first `make` builds Nordic's pc-ble-driver (a git submodule) into `build/`, applying the
patch in `patches/`. The default target is SoftDevice API v5; use `make SD_API=6` if your
dongle runs the v6 connectivity firmware.

## Run

Plug in an nRF52 dongle flashed with Nordic's connectivity firmware
(it shows up as "nRF52 Connectivity"), then run:

```sh
./arcx_ring_client               # connect to the first device named arcx-*
./arcx_ring_client arcx-e24e     # connect to a specific ring
./arcx_ring_client -v            # also list nearby advertisers and the GATT table
```

| Option | Description |
|---|---|
| `device_name` | Advertised name, or its prefix, to connect to (default `arcx-`) |
| `-p <port>` | Serial port of the dongle (auto-detected by default) |
| `-b <baud>` | Baud rate (default `1000000`) |
| `-v`, `-vv` | Verbose output; `-vv` adds the driver packet trace |

Example output:

```
[20:31:02.114] KEY    FORWARD_PRESS     ( 2)    [ext: 02-00-00-00]
[20:31:02.290] KEY    FORWARD_RELEASE   ( 3)    [ext: 03-00-00-00]
[20:31:05.410] BATT   Battery level: 100%
```

Press Ctrl-C to disconnect and exit. If the ring disconnects, the client scans and
reconnects automatically.
