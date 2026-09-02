#!/usr/bin/env python3
"""Render a TMF8829 depth frame in a terminal.

A depth grid is an image, and printing it as a table of numbers throws away
the thing that makes it useful: you cannot see a shape in a table of numbers.
These renderers aim at the opposite, so that a person watching a live sensor
can tell at a glance what is in front of it.

Three ideas do most of the work.

**Two zones per character cell.** A terminal cell is about twice as tall as it
is wide, so one cell per depth point renders a square grid as a stretched
rectangle. Drawing the upper half block with a foreground colour and letting
the background show through the lower half puts two vertically adjacent depth
points in one cell, which makes the aspect ratio right and doubles the
effective resolution. A 32x32 frame then fits in 16 terminal rows.

**Confidence is drawn, not printed.** Every depth point carries a
signal-to-noise ratio, and a low-confidence reading that looks identical to a
high-confidence one is actively misleading. Colours are mixed toward the
background in proportion to confidence, so uncertain depth points recede and
the eye is drawn to the readings worth trusting.

**Multi-object depth points are marked.** The part reports up to four objects
per depth point, which is genuinely unusual, and a renderer that shows only
the nearest hides it. Cells where the sensor saw through an edge are marked so
that capability is visible rather than described.

Colour is 24-bit ANSI. A monochrome fallback using shaded characters is
provided for terminals or captures that cannot carry colour, and it is what
the application note prints, since a PDF cannot show ANSI colour anyway.
"""

from __future__ import annotations

import shutil

from tof_frame import Frame, Zone

# Shading ramp, from nothing to a solid block. Used by the monochrome
# renderer and by anything that has to survive being pasted into a document.
RAMP = " .:-=+*#%@"

# A perceptual colour ramp for depth. Near is warm, far is cool, which is the
# convention depth cameras use and which reads correctly for most people. The
# stops are sampled from turbo, which unlike a rainbow does not invent
# banding that the data does not contain.
TURBO = [
    (48, 18, 59), (70, 66, 170), (60, 125, 221), (46, 180, 205),
    (74, 218, 141), (150, 240, 78), (216, 234, 48), (254, 188, 45),
    (250, 126, 27), (215, 62, 12), (150, 24, 3), (95, 8, 2),
]


def _lerp(a: tuple[int, int, int], b: tuple[int, int, int],
          t: float) -> tuple[int, int, int]:
    return tuple(int(round(a[i] + (b[i] - a[i]) * t)) for i in range(3))


def depth_colour(distance_mm: int | None, near_mm: int, far_mm: int,
                 snr: int = 255, background=(16, 16, 20),
                 confidence_floor: int = 12) -> tuple[int, int, int]:
    """Map a distance to a colour, faded toward the background by confidence."""
    if distance_mm is None:
        return background
    span = max(1, far_mm - near_mm)
    t = min(1.0, max(0.0, (distance_mm - near_mm) / span))
    position = t * (len(TURBO) - 1)
    low = int(position)
    high = min(low + 1, len(TURBO) - 1)
    colour = _lerp(TURBO[low], TURBO[high], position - low)

    # Confidence controls how much of the colour survives. A reading at the
    # noise floor is drawn as barely there rather than as a confident value.
    weight = min(1.0, max(0.0, (snr - confidence_floor) / 48.0))
    weight = 0.25 + 0.75 * weight
    return _lerp(background, colour, weight)


def _fg(rgb) -> str:
    return f"\x1b[38;2;{rgb[0]};{rgb[1]};{rgb[2]}m"


def _bg(rgb) -> str:
    return f"\x1b[48;2;{rgb[0]};{rgb[1]};{rgb[2]}m"


RESET = "\x1b[0m"


def auto_range(frame: Frame) -> tuple[int, int]:
    """Pick the near and far ends of the colour ramp from the frame itself.

    A fixed range wastes most of the ramp when everything in view is at a
    similar distance, which is the normal case. Percentiles rather than the
    extremes, so one stray far reading does not flatten the whole image.
    """
    seen = sorted(frame.distances)
    if not seen:
        return 0, 1
    near = seen[int(len(seen) * 0.02)]
    far = seen[int(len(seen) * 0.98) - 1] if len(seen) > 1 else near + 1
    if far - near < 40:                       # a nearly flat scene
        far = near + 40
    return near, far


def render_colour(frame: Frame, near_mm: int | None = None,
                  far_mm: int | None = None, mark_multi: bool = True,
                  indent: str = "  ") -> str:
    """Half-block colour render: two depth points per character cell."""
    if near_mm is None or far_mm is None:
        near_mm, far_mm = auto_range(frame)
    lines = []
    for row in range(0, frame.rows, 2):
        cells = []
        for column in range(frame.columns):
            top = frame.zone(column, row)
            bottom = (frame.zone(column, row + 1)
                      if row + 1 < frame.rows else None)
            top_rgb = depth_colour(top.distance_mm, near_mm, far_mm, top.snr)
            bottom_rgb = (depth_colour(bottom.distance_mm, near_mm, far_mm,
                                       bottom.snr)
                          if bottom else (16, 16, 20))
            glyph = "▀"                   # upper half block
            if mark_multi and (len(top.objects) > 1 or
                               (bottom and len(bottom.objects) > 1)):
                glyph = "▒"               # medium shade marks see-through
            cells.append(f"{_fg(top_rgb)}{_bg(bottom_rgb)}{glyph}")
        lines.append(indent + "".join(cells) + RESET)
    return "\n".join(lines)


def render_mono(frame: Frame, near_mm: int | None = None,
                far_mm: int | None = None, indent: str = "  ") -> str:
    """Monochrome render, one character per depth point.

    This is what goes in a document. Near is dense, far is sparse, and a depth
    point with no return is blank, so the shape survives being printed.
    """
    if near_mm is None or far_mm is None:
        near_mm, far_mm = auto_range(frame)
    span = max(1, far_mm - near_mm)
    lines = []
    for row in range(frame.rows):
        cells = []
        for column in range(frame.columns):
            zone = frame.zone(column, row)
            if zone.distance_mm is None:
                cells.append(" ")
                continue
            t = min(1.0, max(0.0, (zone.distance_mm - near_mm) / span))
            # Near is dense, so invert: index 0 of the ramp is the far end.
            index = int((1.0 - t) * (len(RAMP) - 1))
            cells.append(RAMP[index])
        lines.append(indent + "".join(cells))
    return "\n".join(lines)


def render_numeric(frame: Frame, indent: str = "  ",
                   max_columns: int = 16) -> str:
    """Distances in centimetres, for reading exact values off a small grid."""
    lines = []
    columns = min(frame.columns, max_columns)
    for row in range(frame.rows):
        cells = []
        for column in range(columns):
            distance = frame.zone(column, row).distance_mm
            cells.append("   . " if distance is None else f"{distance / 10:5.0f}")
        lines.append(indent + "".join(cells))
    if frame.columns > columns:
        lines.append(indent + f"({frame.columns - columns} further columns "
                              f"not shown)")
    return "\n".join(lines)


def render_histogram(frame: Frame, width: int = 48, bins: int = 12,
                     indent: str = "  ") -> str:
    """Where the returns actually are, as a distance histogram.

    The depth image says where things are in the field of view. This says how
    many surfaces are in front of the sensor and how far away they are, which
    is the question the image is worst at answering.
    """
    seen = frame.distances
    if not seen:
        return indent + "no returns"
    near, far = min(seen), max(seen)
    span = max(1, far - near)
    counts = [0] * bins
    for distance in seen:
        index = min(bins - 1, (distance - near) * bins // span)
        counts[index] += 1
    peak = max(counts) or 1
    lines = []
    for index, count in enumerate(counts):
        low = near + span * index // bins
        bar = "█" * max(0, count * width // peak)
        lines.append(f"{indent}{low / 10:6.0f} cm {bar} {count if count else ''}")
    return "\n".join(lines)


def render_scale(near_mm: int, far_mm: int, width: int = 40,
                 indent: str = "  ") -> str:
    """A colour key, so the image can be read as distances."""
    cells = []
    for i in range(width):
        t = i / max(1, width - 1)
        distance = int(near_mm + t * (far_mm - near_mm))
        cells.append(f"{_bg(depth_colour(distance, near_mm, far_mm))} ")
    bar = "".join(cells) + RESET
    return (f"{indent}{bar}\n"
            f"{indent}{near_mm / 10:.0f} cm{' ' * max(1, width - 14)}"
            f"{far_mm / 10:.0f} cm")


def supports_colour() -> bool:
    """Whether writing ANSI colour to this terminal makes sense."""
    import os
    import sys
    if not sys.stdout.isatty():
        return False
    if os.environ.get("NO_COLOR"):
        return False
    return os.environ.get("TERM", "") not in ("dumb", "")


def render(frame: Frame, style: str = "auto", **kwargs) -> str:
    """Render by name, defaulting to whatever the terminal can carry."""
    if style == "auto":
        style = "colour" if supports_colour() else "mono"
    if style == "colour":
        return render_colour(frame, **kwargs)
    if style == "mono":
        return render_mono(frame, **kwargs)
    if style == "numeric":
        return render_numeric(frame, **kwargs)
    if style == "histogram":
        return render_histogram(frame, **kwargs)
    raise ValueError(f"unknown style {style!r}")


def fits_terminal(frame: Frame) -> bool:
    width = shutil.get_terminal_size((80, 24)).columns
    return frame.columns + 4 <= width
