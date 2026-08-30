# AN0001 — Programming STM32 microcontrollers over I3C with the Binho Supernova

[**Read the PDF**](https://cdn.binho.io/application-notes/AN0001/AN0001.pdf) ·
[**Download the assets**](https://cdn.binho.io/application-notes/AN0001/AN0001-assets.zip)

STM32H5 series microcontrollers include an I3C interface in the system bootloader programmed
into ROM. Internal flash can be erased, programmed and verified over a two-wire I3C bus with
no debug probe attached. This note documents that interface and supplies a reference utility
that implements it against a Binho Supernova.

## Contents

| File | Description |
|---|---|
| `AN0001.md` | The application note, in Markdown |
| `AN0001.pdf` | The application note, formatted |
| `AN0001-assets.zip` | The assets archive, as served from the CDN |
| `assets/stm32_flash.py` | Reference utility, shared with AN0002: identify, erase, program, verify, run |
| `assets/test_hex_parser.py` | Tests for the Intel HEX parser |
| `assets/firmware/` | Two verification images: source, makefile and prebuilt binaries |
| `assets/PROTOCOL.md` | Wire-level protocol reference |
| `assets/LICENSE` | MIT license covering the example code |
| `figures/` | Figures used by the note |

## Quick start

```
pip install binhosupernova
python assets/stm32_flash.py --bus i3c info
python assets/stm32_flash.py --bus i3c flash assets/firmware/an0001_image_a.bin --address 0x08000000
```

With a serial terminal open on the ST-LINK virtual COM port at 115200 8N1, the board then
reports `IMAGE A` once a second. Flashing `an0001_image_b.bin` changes it to `IMAGE B`,
which confirms the update took effect rather than only that the bytes matched.

Wiring, bus configuration and the full procedure are in the note.

## Verified configuration

| Item | Value |
|---|---|
| Target | NUCLEO-H503RB (STM32H503RB, device ID `0x474`) |
| Adapter | Binho Supernova rev B, firmware 4.4.0 |
| Host | `binhosupernova` 4.2.0, Python 3.12 |

Other STM32 families listed in the note are recognized by the utility but were not tested.
