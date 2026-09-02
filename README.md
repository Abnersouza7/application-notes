# Binho application notes

Technical how-to documents for Binho hardware, with the scripts and assets needed to
reproduce each one.

Every note is published in three places:

| Form | Where |
|---|---|
| PDF | `https://cdn.binho.io/application-notes/<ID>/<ID>.pdf` |
| Assets archive | `https://cdn.binho.io/application-notes/<ID>/<ID>-assets.zip` |
| Source and assets | This repository, one folder per note |

## Notes

| ID | Title | Rev | Assets |
|---|---|---|---|
| [AN0001](AN0001-programming-stm32-over-i3c/) | Programming STM32 microcontrollers over I3C with the Binho Supernova | 1.0 | [PDF](https://cdn.binho.io/application-notes/AN0001/AN0001.pdf) · [ZIP](https://cdn.binho.io/application-notes/AN0001/AN0001-assets.zip) |
| [AN0002](AN0002-programming-stm32-over-i2c/) | Programming STM32 microcontrollers over I2C with the Binho Pulsar | 1.0 | [PDF](https://cdn.binho.io/application-notes/AN0002/AN0002.pdf) · [ZIP](https://cdn.binho.io/application-notes/AN0002/AN0002-assets.zip) |
| [AN0003](AN0003-programming-stm32-over-spi/) | Programming STM32 microcontrollers over SPI with the Binho Pulsar | 1.0 | [PDF](https://cdn.binho.io/application-notes/AN0003/AN0003.pdf) · [ZIP](https://cdn.binho.io/application-notes/AN0003/AN0003-assets.zip) |
| [AN0004](AN0004-evaluating-bosch-mems-over-i3c/) | Interfacing Bosch Sensortec MEMS sensors over I3C with the Binho Supernova | 2.0 | [PDF](https://cdn.binho.io/application-notes/AN0004/AN0004.pdf) · [ZIP](https://cdn.binho.io/application-notes/AN0004/AN0004-assets.zip) |
| [AN0005](AN0005-stmicro-mems-over-i3c/) | Interfacing STMicroelectronics MEMS sensors over I3C with the Binho Supernova | 1.0 | [PDF](https://cdn.binho.io/application-notes/AN0005/AN0005.pdf) · [ZIP](https://cdn.binho.io/application-notes/AN0005/AN0005-assets.zip) |
| [AN0006](AN0006-tdk-mems-over-i3c/) | Interfacing TDK InvenSense MEMS sensors over I3C with the Binho Supernova | 1.0 | [PDF](https://cdn.binho.io/application-notes/AN0006/AN0006.pdf) · [ZIP](https://cdn.binho.io/application-notes/AN0006/AN0006-assets.zip) |
| [AN0007](AN0007-amsosram-tof-over-i3c/) | Depth imaging with an ams OSRAM time-of-flight sensor over I3C with the Binho Supernova | 1.0 | [PDF](https://cdn.binho.io/application-notes/AN0007/AN0007.pdf) · [ZIP](https://cdn.binho.io/application-notes/AN0007/AN0007-assets.zip) |

## Repository layout

```
AN0001-<slug>/
  AN0001.md          the note, in Markdown, with front matter
  AN0001.pdf         built PDF (committed so the repo is self-contained)
  AN0001-assets.zip  built archive, the same one served from the CDN
  assets/            scripts and files the note tells the reader to download
  figures/           images referenced from the Markdown
  README.md          short landing page for the folder
_shared/
  legal.md           legal boilerplate, shared by every note
_template/
  scaffold for a new note
```

## Reproducing a note

Each note is self-contained. Download the assets archive, or clone this repository and use
the `assets/` folder directly. Requirements are stated in the note itself.

## Building

The build and publishing tooling lives in Binho's internal content repository, not here.
The Markdown source, figures and assets in this repository are the inputs to it.

## Contributing

These notes describe procedures that were run on real hardware. Corrections are welcome,
particularly reports of behavior that differs on a part or board revision we did not test.
Open an issue with the device, board and firmware versions involved.

## License

Two different terms apply in this repository, so the distinction is worth stating plainly.

- **Example code** — everything under each note's `assets/` directory — is released under the
  MIT license. Each note carries the licence text at `assets/LICENSE`, and a copy travels
  inside its assets archive.
- **The application notes themselves**, meaning the Markdown and PDF documents and their
  figures, are © Binho Inc. They may be read, printed and shared as published; they are not
  MIT licensed.

There is deliberately no repository-level `LICENSE` file. GitHub derives the single licence
label it shows from that file, and a top-level MIT would announce that the documents are MIT
too, which they are not. The licence sits with the code it covers.
