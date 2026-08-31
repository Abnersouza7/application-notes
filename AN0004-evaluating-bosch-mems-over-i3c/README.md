# AN0004 — Evaluating Bosch Sensortec MEMS sensors over I3C with the Binho Supernova

A MIPI I3C target implements a subset of the specification, and neither the datasheet nor the
result code returned by a transfer reliably says which subset. This note establishes that
subset by measurement, on the BMI323, BMP581 and BMP585.

| | |
|---|---|
| PDF | [AN0004.pdf](AN0004.pdf) |
| Assets | [AN0004-assets.zip](AN0004-assets.zip) |
| Hardware | Binho Supernova, Binho I3C Target Board (BIN103), Bosch Shuttle 3.0 boards |

The reference utility `assets/i3c_mems.py` is shared with AN0005. Its source of truth is
`_shared/tools/i3c_mems.py`; the copy here is generated and must not be edited in place.
