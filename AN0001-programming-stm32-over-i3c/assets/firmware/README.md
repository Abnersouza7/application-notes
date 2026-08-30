# AN0001 verification images

Two small firmware images for the NUCLEO-H503RB. They exist so that a firmware
update performed over I3C can be confirmed from outside the programming tool:
flash one, look at the serial output, flash the other, and watch it change.

| File | Description |
|---|---|
| `an0001_image_a.bin` | Prebuilt Image A, slow LED blink |
| `an0001_image_b.bin` | Prebuilt Image B, fast LED blink |
| `main.c` | Source for both images |
| `link.ld` | Linker script, 128 KB flash at 0x08000000, 32 KB SRAM |
| `Makefile` | Builds both images |

They are 868 and 864 bytes, and both are linked to run from `0x08000000`.

## What they do

Each image prints a banner at reset and then one heartbeat line per second, at
**115200 8N1** on the ST-LINK virtual COM port. The heartbeat matters: a banner
printed only at reset is missed if the terminal is opened afterwards, and the
point of these images is to be able to tell at a glance which one is running.

```
========================================
  Binho  |  AN0001
  Programming STM32 microcontrollers
  over I3C with the Binho Supernova
========================================
  Running: IMAGE A
  Kernel clock: 32 MHz
  LED blink: 1 Hz
========================================
  IMAGE A  alive, 1 s
  IMAGE A  alive, 2 s
```

Image B is identical except that it reports `IMAGE B` and blinks the LED at
5 Hz rather than 1 Hz, so the two can also be told apart with no terminal
attached.

## Board connections used

| Signal | Pin | Note |
|---|---|---|
| USART3_TX | PA4 | Alternate function 13, routed to the ST-LINK VCP |
| User LED (LD2) | PA5 | Active high |

## Building

```
make          # both images into build/
make hex      # also emit Intel HEX
make clean
```

Requires the Arm GNU toolchain; built and tested with 14.2.Rel1. There is no
HAL, no CubeMX and no CMSIS dependency: every register the program touches is
defined in `main.c`, so the whole example is one readable file.

## A note on the clock

The images do not reconfigure the clock tree. At reset the STM32H503 runs from
the HSI with the bus prescalers at 1, so the USART kernel clock is the HSI
divided by `HSIDIV`. That divider is **read at runtime** rather than assumed,
and the baud divisor is computed from it.

This is worth copying. On this part the reset default is HSIDIV = 2, giving
32 MHz, not the 64 MHz the HSI itself runs at. Hardcoding 64 MHz produces a baud
rate that is out by a factor of two, and the only symptom is garbled output.

## License

MIT, the same as the rest of the AN0001 example code. See `../LICENSE`.
