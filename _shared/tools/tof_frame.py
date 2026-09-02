#!/usr/bin/env python3
"""Frame decoding for the AMS OSRAM TMF8829 direct time-of-flight sensor.

The TMF8829 returns a grid of zones rather than a single distance. Each zone
can carry a noise floor, a crosstalk estimate, and up to four detected peaks,
and each peak carries a distance, a signal-to-noise ratio and optionally a
signal strength. An 8x8 frame is 64 zones; a 48x32 frame is 1536.

The datasheet does not define this layout. It defers to a separate document,
"TMF8829 Host Driver Communication", and the structures below were taken from
the vendor's own Python driver, which is MIT licensed:

    https://github.com/ams-OSRAM/tmf8829_driver_python

    struct _tmf8829FrameHeader   16 bytes  id, layout, payload, fNumber,
                                           temperature[3], bdv, refPos[2]
    struct _tmf8829FrameFooter   12 bytes  t0, t1, frameStatus, reserved, eof

Three things about this format are worth knowing before reading the code,
because each one silently produces garbage rather than an error if you get it
wrong.

**The layout byte sizes every zone.** It is not a constant. Peak count
lives in bits 0-2 and three optional fields hang off bits 3-5, so a zone is
anywhere from 3 to 24 bytes wide: peaks * (3 + 2*signal) + 2*noise + 2*xtalk.
Decode it first, or mis-frame the entire payload.

**Distances are in quarter millimetres.** That is the 0.25 mm resolution the
datasheet advertises, and reading the field as millimetres puts a wall four
times too far away without ever looking wrong.

**The two largest grids arrive interleaved.** A 32x32 or 48x32 measurement is
more data than one frame carries, so the part sends sixteen rows at a time and
flags which half in the layout byte. The halves are not the top and the bottom
of the image: the first carries the even rows and the second the odd ones.
Stacking them instead of interleaving them produces a picture that looks
plausible and is wrong, which is the worst kind of wrong.

This module has no hardware dependency on purpose. It decodes bytes, and it
can synthesise bytes in the same format, so the renderer can be built and
tested before the sensor is on the bench.
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass, field

FRAME_HEADER_SIZE = 16
FRAME_FOOTER_SIZE = 12
FID_MASK = 0xF0
FID_RESULTS = 0x10          # header id for a result frame
FPM_MASK = 0x0F             # low nibble of the id: which focal-plane mode
FRAME_EOF = 0xE0F7          # last two bytes of every frame
FRAME_VALID = 0x01          # frameStatus bit that says the frame is usable
DISTANCE_IN_MM = 0x01       # footer reserved bit 0: already scaled, do not divide
MAX_PEAKS = 4

# The header's payload field counts every byte after it, so a frame is four
# bytes (id, layout, payload) plus whatever it says. Reading the length out of
# the frame beats computing it from the configured mode, because it stays
# right when the result format is not what was asked for.
HEADER_PREFIX = 4

# A frame is read starting at FIFOSTATUS (0xFA) rather than at the FIFO data
# register itself, because the four systick registers sit between them and the
# read auto-increments through all of it. Those five bytes arrive ahead of the
# frame and are not part of it. Measured on hardware: the frame id 0x10 lands
# at offset 5, not offset 0.
PRE_HEADER_SIZE = 5

# Distances are reported in quarter millimetres. Measured: a wall that read
# 10981 raw is at 2745 mm, which is where the wall actually is.
DISTANCE_SCALE = 4

# The layout byte, which is the same encoding as the CFG_RESULT_FORMAT
# register the frame was produced under.
LAYOUT_PEAKS = 0x07
LAYOUT_SIGNAL = 0x08
LAYOUT_NOISE = 0x10
LAYOUT_XTALK = 0x20
LAYOUT_SUBFRAME = 0x40


@dataclass(frozen=True)
class Mode:
    """One of the sensor's zone grids, and how it gets to the host.

    `subframes` is the part that catches people out: the two largest grids do
    not fit in a single frame, so the sensor sends sixteen rows at a time and
    the host has to stitch them.
    """
    name: str
    columns: int
    rows: int
    subframes: int
    command: int            # CMD_LOAD_CFG_* that selects this mode

    @property
    def zones(self) -> int:
        return self.columns * self.rows

    @property
    def rows_per_subframe(self) -> int:
        return self.rows // self.subframes


MODES = {
    "8x8": Mode("8x8", 8, 8, 1, 0x40),
    "8x8-long": Mode("8x8-long", 8, 8, 1, 0x41),
    "8x8-accurate": Mode("8x8-accurate", 8, 8, 1, 0x42),
    "16x16": Mode("16x16", 16, 16, 1, 0x43),
    "32x32": Mode("32x32", 32, 32, 2, 0x45),
    "48x32": Mode("48x32", 48, 32, 2, 0x47),
}


@dataclass
class Peak:
    """One detected object within one zone."""
    distance_mm: int
    snr: int
    signal: int = 0

    @property
    def valid(self) -> bool:
        # A peak with no distance is an empty slot rather than an object at
        # zero millimetres. The part reports up to four and pads the rest.
        return self.distance_mm > 0


@dataclass
class Zone:
    """One zone: a noise floor, a crosstalk estimate, and its peaks."""
    noise: int = 0
    xtalk: int = 0
    peaks: list[Peak] = field(default_factory=list)

    @property
    def objects(self) -> list[Peak]:
        return [p for p in self.peaks if p.valid]

    @property
    def nearest(self) -> Peak | None:
        objects = self.objects
        return min(objects, key=lambda p: p.distance_mm) if objects else None

    @property
    def distance_mm(self) -> int | None:
        peak = self.nearest
        return peak.distance_mm if peak else None

    @property
    def snr(self) -> int:
        peak = self.nearest
        return peak.snr if peak else 0


@dataclass
class Frame:
    """A decoded result frame, or one half of one."""
    number: int
    columns: int
    rows: int
    zones: list[Zone]
    temperature_c: int = 0
    peaks_per_zone: int = 1
    frame_status: int = FRAME_VALID
    subframe: int = 0
    layout: int = 0x01
    fp_mode: int = 0
    valid: bool = True

    def zone(self, column: int, row: int) -> Zone:
        return self.zones[row * self.columns + column]

    @property
    def distances(self) -> list[int]:
        return [z.distance_mm for z in self.zones if z.distance_mm is not None]

    @property
    def coverage(self) -> float:
        """Fraction of zones that saw anything at all."""
        return len(self.distances) / len(self.zones) if self.zones else 0.0

    def summary(self) -> str:
        seen = self.distances
        if not seen:
            return (f"frame {self.number}  {self.columns}x{self.rows}  "
                    f"no returns  {self.temperature_c} C")
        ordered = sorted(seen)
        median = ordered[len(ordered) // 2]
        multi = sum(1 for z in self.zones if len(z.objects) > 1)
        return (f"frame {self.number}  {self.columns}x{self.rows}  "
                f"{len(seen)}/{len(self.zones)} zones "
                f"({self.coverage * 100:.0f}%)  "
                f"near {min(seen)} mm  median {median} mm  far {max(seen)} mm  "
                f"{multi} multi-object  {self.temperature_c} C")


def zone_layout(layout: int) -> tuple[int, bool, bool, bool]:
    """Unpack the header's layout byte into what each zone contains.

    This is what makes a frame self-describing, and it has to be read before
    anything else, because every field after the header is positioned by it.
    Measured on hardware in the default 8x8 mode it reads 0x01: one peak, no
    signal, noise or crosstalk, which is three bytes per zone and 192
    bytes for 64 of them.
    """
    peaks = layout & LAYOUT_PEAKS
    peaks = peaks if 1 <= peaks <= MAX_PEAKS else 1
    return (peaks, bool(layout & LAYOUT_SIGNAL),
            bool(layout & LAYOUT_NOISE), bool(layout & LAYOUT_XTALK))


def zone_stride(peaks: int, has_signal: bool = False, has_noise: bool = False,
                has_xtalk: bool = False) -> int:
    """Bytes per zone, given what the layout byte says is present."""
    peak_size = 5 if has_signal else 3      # distance u16, snr u8, signal u16
    return (2 if has_noise else 0) + (2 if has_xtalk else 0) + peak_size * peaks


def frame_size(mode: Mode, layout: int) -> int:
    """Total bytes to read for one frame of this mode, pre-header included.

    Worth computing rather than reading a fixed size: the sensor holds frames
    in a FIFO, and a short read leaves the remainder there to corrupt the next
    one.
    """
    stride = zone_stride(*zone_layout(layout))
    points = mode.columns * mode.rows_per_subframe
    return (PRE_HEADER_SIZE + FRAME_HEADER_SIZE + points * stride
            + FRAME_FOOTER_SIZE)


def size_from_header(header: bytes, pre_header: int = PRE_HEADER_SIZE) -> int:
    """Total frame size, read out of the frame's own header.

    The header's payload field counts everything after itself, so this needs
    only the first few bytes of a frame and does not care what mode or result
    format produced it. Worth cross-checking against `frame_size`: when the
    two disagree, the configuration that was written is not the one the sensor
    is measuring under.
    """
    if len(header) < pre_header + HEADER_PREFIX:
        raise ValueError(f"need {pre_header + HEADER_PREFIX} bytes of header, "
                         f"got {len(header)}")
    payload = struct.unpack_from("<H", header, pre_header + 2)[0]
    return pre_header + HEADER_PREFIX + payload


def decode(data: bytes, columns: int, rows: int,
           pre_header: int = PRE_HEADER_SIZE) -> Frame:
    """Decode one result frame as read from FIFOSTATUS onward.

    The grid shape is supplied by the caller because it is a property of the
    mode the sensor was configured into rather than something the frame
    announces. Everything else comes from the bytes, including how wide each
    zone is.

    For a mode that arrives in halves, `rows` is the rows in this half.
    """
    if len(data) < pre_header + FRAME_HEADER_SIZE + FRAME_FOOTER_SIZE:
        raise ValueError(f"frame too short: {len(data)} bytes")

    base = pre_header
    fid, layout, payload, number = struct.unpack_from("<BBHI", data, base)
    temperature = struct.unpack_from("<3b", data, base + 8)[0]
    peaks, has_signal, has_noise, has_xtalk = zone_layout(layout)
    stride = zone_stride(peaks, has_signal, has_noise, has_xtalk)
    expected = columns * rows

    body = data[base + FRAME_HEADER_SIZE:]
    available = len(body) // stride
    if available < expected:
        raise ValueError(
            f"frame holds {available} zones at {stride} bytes each, "
            f"{columns}x{rows} needs {expected}; the grid does not match the "
            f"configured mode")

    # The footer is read before the zones, not after, because it says
    # what units they are in. Bit 0 of its reserved byte means the firmware
    # already converted to millimetres, and dividing again would put every
    # surface at a quarter of its true distance.
    tail = base + FRAME_HEADER_SIZE + expected * stride
    footer = data[tail:tail + FRAME_FOOTER_SIZE]
    if len(footer) == FRAME_FOOTER_SIZE:
        status, reserved = footer[8], footer[9]
        eof = struct.unpack_from("<H", footer, 10)[0]
    else:
        status, reserved, eof = 0, 0, 0
    scale = 1 if reserved & DISTANCE_IN_MM else DISTANCE_SCALE

    peak_size = 5 if has_signal else 3
    zones: list[Zone] = []
    for index in range(expected):
        at = index * stride
        noise = struct.unpack_from("<H", body, at)[0] if has_noise else 0
        at += 2 if has_noise else 0
        xtalk = struct.unpack_from("<H", body, at)[0] if has_xtalk else 0
        at += 2 if has_xtalk else 0
        found = []
        for p in range(peaks):
            offset = at + p * peak_size
            if has_signal:
                distance, snr, signal = struct.unpack_from("<HBH", body, offset)
            else:
                distance, snr = struct.unpack_from("<HB", body, offset)
                signal = 0
            found.append(Peak(distance // scale, snr, signal))
        zones.append(Zone(noise, xtalk, found))

    # Both checks matter and they check different things. The EOF marker says
    # the frame came out of the FIFO in one piece; frameStatus says the sensor
    # thinks the measurement behind it is worth having.
    return Frame(number=number, columns=columns, rows=rows, zones=zones,
                 temperature_c=temperature, peaks_per_zone=peaks,
                 frame_status=status, layout=layout, fp_mode=fid & FPM_MASK,
                 subframe=1 if layout & LAYOUT_SUBFRAME else 0,
                 valid=((fid & FID_MASK) == FID_RESULTS
                        and eof == FRAME_EOF
                        and bool(status & FRAME_VALID)))


def stitch(halves: list[Frame], mode: Mode) -> Frame:
    """Interleave the two halves of a 32x32 or 48x32 measurement.

    The first frame carries the even rows of the image and the second carries
    the odd ones, so the two are woven together rather than stacked. Stacking
    them looks almost right: the scene appears twice, at half the vertical
    resolution, which is easy to mistake for a working decoder.

    Ordered by the subframe flag rather than by arrival, so a dropped or
    duplicated frame is an error here instead of a picture that is subtly
    wrong.
    """
    if mode.subframes == 1:
        return halves[0]
    ordered = sorted(halves, key=lambda f: f.subframe)
    if len(ordered) != mode.subframes:
        raise ValueError(f"{mode.name} needs {mode.subframes} subframes, "
                         f"got {len(ordered)}")
    if [f.subframe for f in ordered] != list(range(mode.subframes)):
        raise ValueError("subframes are not consecutive; a frame was dropped")

    zones: list[Zone] = []
    for row in range(mode.rows):
        part = ordered[row % mode.subframes]
        source = row // mode.subframes
        zones.extend(part.zones[source * mode.columns:
                                (source + 1) * mode.columns])
    first = ordered[0]
    return Frame(number=first.number, columns=mode.columns, rows=mode.rows,
                 zones=zones, temperature_c=first.temperature_c,
                 peaks_per_zone=first.peaks_per_zone,
                 frame_status=first.frame_status, layout=first.layout,
                 valid=all(f.valid for f in ordered))


def encode(frame_number: int, columns: int, rows: int, zones: list[Zone],
           peaks: int = MAX_PEAKS, temperature_c: int = 27,
           has_signal: bool = True, has_noise: bool = True,
           has_xtalk: bool = True, subframe: int = 0) -> bytes:
    """Build frame bytes in the sensor's own format.

    Used to exercise the decoder and the renderer without hardware. It is the
    inverse of decode, so a round trip through both tests the layout rather
    than a mock. It defaults to the richest layout so a round trip exercises
    every optional field, not only the ones the default mode emits.
    """
    layout = (peaks
              | (LAYOUT_SIGNAL if has_signal else 0)
              | (LAYOUT_NOISE if has_noise else 0)
              | (LAYOUT_XTALK if has_xtalk else 0)
              | (LAYOUT_SUBFRAME if subframe else 0))
    stride = zone_stride(peaks, has_signal, has_noise, has_xtalk)
    payload = len(zones) * stride + FRAME_FOOTER_SIZE + FRAME_HEADER_SIZE - 4

    out = bytearray(b"\x00" * PRE_HEADER_SIZE)
    out += struct.pack("<BBHI", FID_RESULTS, layout, payload, frame_number)
    out += struct.pack("<3bB", temperature_c, temperature_c, temperature_c, 0)
    out += struct.pack("<HH", 0, 0)                     # refPos
    for zone in zones:
        if has_noise:
            out += struct.pack("<H", zone.noise)
        if has_xtalk:
            out += struct.pack("<H", zone.xtalk)
        for p in range(peaks):
            peak = zone.peaks[p] if p < len(zone.peaks) else Peak(0, 0, 0)
            distance = peak.distance_mm * DISTANCE_SCALE
            if has_signal:
                out += struct.pack("<HBH", distance, peak.snr, peak.signal)
            else:
                out += struct.pack("<HB", distance, peak.snr)
    out += struct.pack("<II", 0, 0)                     # t0, t1
    out += struct.pack("<BBH", FRAME_VALID, 0, FRAME_EOF)
    return bytes(out)


def synthetic(columns: int, rows: int, frame_number: int = 0,
              scene: str = "hand") -> Frame:
    """A physically plausible scene, for developing against without hardware.

    Not random noise. Real depth data has structure the renderer has to cope
    with: a background wall, an object nearer the sensor, edges where a zone
    straddles both and reports two peaks, and returns that get weaker
    with distance. A renderer that looks good on random numbers can look
    unreadable on a real scene.
    """
    zones: list[Zone] = []
    for row in range(rows):
        for column in range(columns):
            x = (column - columns / 2 + 0.5) / (columns / 2)
            y = (row - rows / 2 + 0.5) / (rows / 2)
            wall = 2400 + int(180 * x)           # a wall, slightly angled
            peaks: list[Peak] = []

            if scene == "hand":
                # A rounded object hovering left of centre.
                reach = math.hypot(x + 0.25, y + 0.1)
                if reach < 0.55:
                    near = (600 + int(220 * reach * reach)
                            + (frame_number * 7) % 40)
                    snr = max(8, 60 - int(50 * reach))
                    peaks.append(Peak(near, snr, 900 - int(600 * reach)))
                    if 0.42 < reach < 0.55:      # edge: sees past the object
                        peaks.append(Peak(wall, 14, 120))
            elif scene == "corner":
                # Two surfaces meeting, so distance ramps in both axes.
                wall = 900 + int(1400 * max(0.0, x)) + int(700 * max(0.0, y))

            if len(peaks) < 2:
                falloff = max(4, 42 - int(wall / 90))
                peaks.append(Peak(wall, falloff, max(20, 400 - wall // 8)))

            # Far corners of a wide field of view fall off and drop out.
            if math.hypot(x, y) > 1.25:
                peaks = [Peak(0, 0, 0)]

            zones.append(Zone(noise=30 + (column * row) % 12,
                              xtalk=18, peaks=peaks[:MAX_PEAKS]))
    return Frame(number=frame_number, columns=columns, rows=rows, zones=zones,
                 temperature_c=27, peaks_per_zone=MAX_PEAKS,
                 frame_status=FRAME_VALID, valid=True)
