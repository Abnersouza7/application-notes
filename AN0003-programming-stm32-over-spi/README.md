# AN0003 — Programming STM32 microcontrollers over SPI with the Binho Pulsar

[**Read the PDF**](https://cdn.binho.io/application-notes/AN0003/AN0003.pdf) ·
[**Download the assets**](https://cdn.binho.io/application-notes/AN0003/AN0003-assets.zip)

The system bootloader programmed into the ROM of STM32 microcontrollers offers an SPI
interface on many families. Internal flash can be erased, programmed and verified with no
debug probe attached. This note documents that interface and supplies a reference utility
that implements it against a Binho Pulsar.

## Contents

| File | Description |
|---|---|
| `AN0003.md` | The application note, in Markdown |
| `AN0003.pdf` | The application note, formatted |
| `AN0003-assets.zip` | The assets archive, as served from the CDN |
| `assets/stm32_flash.py` | Reference utility, shared with AN0001 and AN0002 |
| `assets/test_hex_parser.py` | Tests for the Intel HEX parser |
| `assets/firmware/` | Two verification images: source, makefile and prebuilt binaries |
| `assets/PROTOCOL.md` | Wire-level protocol reference |
| `assets/LICENSE` | MIT license covering the example code |
| `figures/` | Figures used by the note |

## Quick start

```
pip install binhopulsar
python assets/stm32_flash.py --bus spi --reset-via cs info
python assets/stm32_flash.py --bus spi --reset-via cs flash assets/firmware/an0003_image_a.bin --address 0x08000000
```

With a serial terminal open on the ST-LINK virtual COM port at 115200 8N1, the board then
reports `IMAGE A` once a second. Flashing `an0003_image_b.bin` changes it to `IMAGE B`,
which confirms the update took effect rather than only that the bytes matched.

## Two things to know before wiring

**The bootloader's SPI pins are not the board's SPI header.** On the STM32H503 the ROM offers
SPI1 on `PA7`/`PA0`/`PA8`/`PB8`, SPI2 on `PB1`/`PB14`/`PB10`/`PB12` and SPI3 on
`PC12`/`PC11`/`PC10`/`PD2`, with no alternates. The NUCLEO-H503RB's Arduino SPI header carries
`PA5`/`PA6`/`PA7`/`PC9`, which is the application mapping. Only `PA7` is common to both. This
note uses SPI2.

**`0xA5` is both the busy value and an ordinary data byte.** A host that skips it while
looking for the start of a reply passes every identification command and then corrupts flash
reads. Section 5.6 of the note gives the framing that works.

## Verified configuration

| Item | Value |
|---|---|
| Target | NUCLEO-H503RB (STM32H503RB, device ID `0x474`) |
| Adapter | Binho Pulsar rev C, firmware 4.4.0 |
| Host | `binhopulsar` 1.2.0, Python 3.12 |
| Interface | SPI2, mode 0, NRST and BOOT0 driven from spare chip selects |

A Binho Supernova rev B was verified on the same wiring, completing the same cycles and the
same hostile-image test. The timing figures in the note are from the Pulsar. Other STM32
families listed in the note are recognized by the utility but were not tested.
