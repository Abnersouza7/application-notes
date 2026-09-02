#!/usr/bin/env python3
"""Frame decoding for the AMS OSRAM TMF8829 direct time-of-flight sensor.

The TMF8829 returns a grid of depth points rather than a single distance. Each
point carries a noise floor, a crosstalk estimate, and up to four detected
peaks, and each peak carries a distance, a signal-to-noise ratio and a signal
strength. An 8x8 frame is 64 of those; a 48x32 frame is 1536.

The datasheet does not define this layout. It defers to a separate document,
"TMF8829 Host Driver Communication", and the structures below were taken from
the vendor's own Python driver, which is MIT licensed:

    https://github.com/ams-OSRAM/tmf8829_driver_python
    tmf8829/tmf8829_application_defines.py

    struct _tmf8829FrameHeader   16 bytes
    struct _tmf8829PeakSignal     5 bytes   distance, snr, signal
    struct _tmf8829MPResult                 noise, xtalk, peaks[4]
    struct _tmf8829FrameFooter   12 bytes

The header's `layout` byte says how many peaks are present and how wide each
one is, so a frame is only self-describing once that byte is read. Everything
here decodes from the bytes rather than assuming a fixed size, because the
same sensor produces frames of very different shapes depending on the mode it
was configured into.

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
FID_RESULTS = 0x10          # header id for a result frame
MAX_PEAKS = 4

# Zone grids the part supports, as (columns, rows).
GRIDS = {
    "8x8": (8, 8),
    "16x16": (16, 16),
    "32x32": (32, 32),
    "48x32": (48, 32),
}


@dataclass
class Peak:
    """One detected object within one depth point."""
    distance_mm: int
    snr: int
    signal: int

    @property
    def valid(self) -> bool:
        # A peak with no distance is an empty slot rather than an object at
        # zero millimetres. The part reports up to four and pads the rest.
        return self.distance_mm > 0


@dataclass
class Zone:
    """One depth point: a noise floor, a crosstalk estimate, and its peaks."""
    noise: int
    xtalk: int
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
    """A decoded result frame."""
    number: int
    columns: int
    rows: int
    zones: list[Zone]
    temperature_c: int
    peaks_per_zone: int
    frame_status: int
    valid: bool

    def zone(self, column: int, row: int) -> Zone:
        return self.zones[row * self.columns + column]

    @property
    def distances(self) -> list[int]:
        return [z.distance_mm for z in self.zones if z.distance_mm is not None]

    @property
    def coverage(self) -> float:
        """Fraction of depth points that saw anything at all."""
        return len(self.distances) / len(self.zones) if self.zones else 0.0

    def summary(self) -> str:
        seen = self.distances
        if not seen:
            return (f"frame {self.number}  {self.columns}x{self.rows}  "
                    f"no returns  {self.temperature_c} C")
        seen_sorted = sorted(seen)
        median = seen_sorted[len(seen_sorted) // 2]
        multi = sum(1 for z in self.zones if len(z.objects) > 1)
        return (f"frame {self.number}  {self.columns}x{self.rows}  "
                f"{len(seen)}/{len(self.zones)} zones "
                f"({self.coverage * 100:.0f}%)  "
                f"near {min(seen)} mm  median {median} mm  far {max(seen)} mm  "
                f"{multi} multi-object  {self.temperature_c} C")


def peaks_per_zone(layout: int) -> int:
    """Peaks encoded per depth point, from the header's layout byte.

    The layout byte packs several fields, of which the peak count is the one
    that changes the size of every zone in the frame. Values outside 1..4 are
    treated as the full four rather than trusted, because a wrong count here
    silently mis-frames the entire payload.
    """
    count = layout & 0x07
    return count if 1 <= count <= MAX_PEAKS else MAX_PEAKS


def zone_stride(peaks: int) -> int:
    """Bytes per depth point: noise, crosstalk, then five bytes per peak."""
    return 2 + 2 + 5 * peaks


def decode(data: bytes, columns: int, rows: int) -> Frame:
    """Decode one result frame.

    The grid shape is supplied by the caller because it is a property of the
    mode the sensor was configured into rather than something the frame
    announces. Everything else comes from the bytes.
    """
    if len(data) < FRAME_HEADER_SIZE + FRAME_FOOTER_SIZE:
        raise ValueError(f"frame too short: {len(data)} bytes")

    fid, layout, payload_len, number = struct.unpack_from("<BBHI", data, 0)
    temperature = struct.unpack_from("<3b", data, 8)[0]
    peaks = peaks_per_zone(layout)
    stride = zone_stride(peaks)
    expected_zones = columns * rows

    body = data[FRAME_HEADER_SIZE:len(data) - FRAME_FOOTER_SIZE]
    available = len(body) // stride
    if available < expected_zones:
        raise ValueError(
            f"frame holds {available} depth points, {columns}x{rows} needs "
            f"{expected_zones}; the grid does not match the configured mode")

    zones: list[Zone] = []
    for index in range(expected_zones):
        base = index * stride
        noise, xtalk = struct.unpack_from("<HH", body, base)
        found = []
        for p in range(peaks):
            offset = base + 4 + p * 5
            distance, snr, signal = struct.unpack_from("<HBH", body, offset)
            found.append(Peak(distance, snr, signal))
        zones.append(Zone(noise, xtalk, found))

    status = data[len(data) - FRAME_FOOTER_SIZE + 8]
    return Frame(number=number, columns=columns, rows=rows, zones=zones,
                 temperature_c=temperature, peaks_per_zone=peaks,
                 frame_status=status, valid=(fid == FID_RESULTS))


def encode(frame_number: int, columns: int, rows: int, zones: list[Zone],
           peaks: int = MAX_PEAKS, temperature_c: int = 27) -> bytes:
    """Build frame bytes in the sensor's own format.

    Used to exercise the decoder and the renderer without hardware. It is the
    inverse of decode, so a round trip through both is a real test of the
    layout rather than of a mock.
    """
    stride = zone_stride(peaks)
    payload_len = len(zones) * stride + FRAME_FOOTER_SIZE
    out = bytearray()
    out += struct.pack("<BBHI", FID_RESULTS, peaks, payload_len, frame_number)
    out += struct.pack("<3b", temperature_c, temperature_c, temperature_c)
    out += struct.pack("<B", 0)
    out += struct.pack("<HH", 0, 0)
    for zone in zones:
        out += struct.pack("<HH", zone.noise, zone.xtalk)
        for p in range(peaks):
            peak = zone.peaks[p] if p < len(zone.peaks) else Peak(0, 0, 0)
            out += struct.pack("<HBH", peak.distance_mm, peak.snr, peak.signal)
    out += struct.pack("<II", 0, 0)
    out += struct.pack("<BBH", 0, 0, 0xFFFF)
    return bytes(out)


def synthetic(columns: int, rows: int, frame_number: int = 0,
              scene: str = "hand") -> Frame:
    """A physically plausible scene, for developing against without hardware.

    Not random noise. Real depth data has structure the renderer has to cope
    with: a background wall, an object nearer the sensor, edges where a depth
    point straddles both and reports two peaks, and returns that get weaker
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
                    near = 600 + int(220 * reach * reach) + (frame_number * 7) % 40
                    snr = max(8, 60 - int(50 * reach))
                    peaks.append(Peak(near, snr, 900 - int(600 * reach)))
                    if 0.42 < reach < 0.55:      # edge: sees past the object
                        peaks.append(Peak(wall, 14, 120))
            elif scene == "wall":
                pass
            elif scene == "corner":
                # Two surfaces meeting, so distance ramps in both axes.
                wall = 900 + int(1400 * max(0.0, x)) + int(700 * max(0.0, y))

            if not peaks or len(peaks) == 1:
                falloff = max(4, 42 - int(wall / 90))
                peaks.append(Peak(wall, falloff, max(20, 400 - wall // 8)))

            # Far corners of a wide field of view fall off and drop out.
            if math.hypot(x, y) > 1.25:
                peaks = [Peak(0, 0, 0)]

            zones.append(Zone(noise=30 + (column * row) % 12,
                              xtalk=18, peaks=peaks[:MAX_PEAKS]))
    return Frame(number=frame_number, columns=columns, rows=rows, zones=zones,
                 temperature_c=27, peaks_per_zone=MAX_PEAKS,
                 frame_status=0, valid=True)
