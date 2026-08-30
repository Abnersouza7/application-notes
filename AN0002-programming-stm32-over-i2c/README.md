# AN0002 — Programming STM32 microcontrollers over I2C with the Binho Pulsar

[**Read the PDF**](https://cdn.binho.io/application-notes/AN0002/AN0002.pdf) ·
[**Download the assets**](https://cdn.binho.io/application-notes/AN0002/AN0002-assets.zip)

The system bootloader programmed into the ROM of STM32 microcontrollers offers an I2C
interface on most families. Internal flash can be erased, programmed and verified over two
wires with no debug probe attached. This note documents that interface and supplies a
reference utility that implements it against a Binho Pulsar or a Binho Supernova.

## Contents

| File | Description |
|---|---|
| `AN0002.md` | The application note, in Markdown |
| `AN0002.pdf` | The application note, formatted |
| `AN0002-assets.zip` | The assets archive, as served from the CDN |
| `assets/stm32_flash.py` | Reference utility: identify, erase, program, verify, run |
| `assets/test_hex_parser.py` | Tests for the Intel HEX parser |
| `assets/firmware/` | Two verification images: source, makefile and prebuilt binaries |
| `assets/PROTOCOL.md` | Wire-level protocol reference |
| `assets/LICENSE` | MIT license covering the example code |
| `figures/` | Figures used by the note |

## Quick start

```
pip install binhopulsar
python assets/stm32_flash.py --bus i2c info
python assets/stm32_flash.py --bus i2c flash assets/firmware/an0002_image_a.bin --address 0x08000000
```

With a serial terminal open on the ST-LINK virtual COM port at 115200 8N1, the board then
reports `IMAGE A` once a second. Flashing `an0002_image_b.bin` changes it to `IMAGE B`,
which confirms the update took effect rather than only that the bytes matched.

To drive a Supernova from its I3C port instead, install `binhosupernova` and add
`--i2c-port i3c` to each command.

Wiring, bus configuration and the full procedure are in the note. Note that the bootloader's
I2C pins are `PB3` and `PB4` on this part, and the address is `0x67`; both differ per device
and are listed in AN2606.

## Verified configuration

| Item | Value |
|---|---|
| Target | NUCLEO-H503RB (STM32H503RB, device ID `0x474`) |
| Adapter | Binho Pulsar rev C, firmware 4.4.0 |
| Host | `binhopulsar` 1.2.0, Python 3.12 |
| Also verified | Binho Supernova rev B driving I2C from its I3C port |

Programming from the Supernova's dedicated I2C port is not covered by this revision. Other
STM32 families listed in the note are recognized by the utility but were not tested.
