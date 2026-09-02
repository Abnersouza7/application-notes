#!/usr/bin/env python3
"""Watch a TMF8829 time-of-flight sensor from the command line.

    i3c_tof.py demo --mode 32x32          # no hardware needed
    i3c_tof.py bringup --firmware app.hex
    i3c_tof.py live --firmware app.hex --mode 16x16
    i3c_tof.py frame --firmware app.hex --style numeric

`demo` renders a synthetic scene and needs no adapter, which makes it the
quickest way to see what the other subcommands will look like and to check
that a terminal can carry the colour output.

The four parts this is assembled from are separate on purpose:

    tof_frame.py    bytes to zones, and back
    tof_render.py   zones to something a person can read
    tof_device.py   the sensor: bootloader, firmware, configuration, frames
    i3c_tof.py      this file, which only decides what to show

Nothing above this file knows about terminals, and nothing below it knows
about the bus.
"""

from __future__ import annotations

import argparse
import sys
import time

TOOL_VERSION = "1.0"


def _use_utf8():
    """Ask the terminal for UTF-8, and report whether it agreed.

    The renderers draw with half-block and shade characters, and a console
    left on a legacy codepage raises rather than drawing something worse. It
    is worth asking once at startup and falling back deliberately, rather
    than letting a render fail halfway through a live view.
    """
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8")
        except (AttributeError, OSError):
            pass
    encoding = (getattr(sys.stdout, "encoding", "") or "").lower()
    return encoding.replace("-", "") in ("utf8", "utf16", "utf32")


def _import_shared():
    """Import the shared modules whether or not this file was run in place."""
    import os
    here = os.path.dirname(os.path.abspath(__file__))
    if here not in sys.path:
        sys.path.insert(0, here)


_import_shared()
UNICODE_OK = _use_utf8()

import tof_render as render                                    # noqa: E402
from tof_frame import MODES, synthetic                          # noqa: E402

render.use_unicode(UNICODE_OK)

CLEAR_SCREEN = "\x1b[2J\x1b[H"
HIDE_CURSOR = "\x1b[?25l"
SHOW_CURSOR = "\x1b[?25h"


# --------------------------------------------------------------------------
# Display
# --------------------------------------------------------------------------


def show(frame, style, scale=True, histogram=False, indent="  "):
    """One frame, rendered, with the context needed to read it.

    A depth image on its own is ambiguous: the same picture means different
    things depending on what the colour ramp is mapped to. The scale is part
    of the render, not decoration.
    """
    near, far = render.auto_range(frame)
    lines = [indent + frame.summary(), ""]
    lines.append(render.render(frame, style, indent=indent)
                 if style != "histogram"
                 else render.render_histogram(frame, indent=indent))
    if scale and style == "colour":
        lines.append("")
        lines.append(render.render_scale(near, far, indent=indent))
    if histogram and style != "histogram":
        lines.append("")
        lines.append(render.render_histogram(frame, indent=indent))
    return "\n".join(lines)


def resolve_style(requested, frame):
    """Pick a render style that will actually fit and actually display."""
    if requested != "auto":
        return requested
    if not render.supports_colour():
        return "mono"
    return "colour" if render.fits_terminal(frame) else "numeric"


# --------------------------------------------------------------------------
# Commands
# --------------------------------------------------------------------------


def cmd_demo(args):
    """Render a synthetic scene, so the display can be checked without a part."""
    mode = MODES[args.mode]
    if args.frames == 1:
        frame = synthetic(mode.columns, mode.rows, 1, args.scene)
        print(show(frame, resolve_style(args.style, frame),
                   histogram=args.histogram))
        return 0

    print(HIDE_CURSOR, end="")
    try:
        for number in range(1, args.frames + 1):
            frame = synthetic(mode.columns, mode.rows, number, args.scene)
            style = resolve_style(args.style, frame)
            print(CLEAR_SCREEN + show(frame, style, histogram=args.histogram))
            time.sleep(args.interval)
    except KeyboardInterrupt:
        pass
    finally:
        print(SHOW_CURSOR, end="")
    return 0


def _open(args):
    """Open the bus and bring a TMF8829 up to a running measurement."""
    from i3c_mems import Bus
    from tof_device import bring_up

    bus = Bus(serial=args.serial, verbose=args.verbose)
    bus.open()
    bus.configure(voltage_mv=args.voltage, push_pull=args.push_pull,
                  open_drain=args.open_drain, drive=args.drive)
    mode = MODES[args.mode]
    device = bring_up(bus, args.firmware, mode,
                      period_ms=args.period, peaks=args.peaks,
                      signal=args.signal, noise=args.noise, xtalk=args.xtalk)
    return bus, device, mode


def cmd_bringup(args):
    """Take the part from power-on to measuring, and report each step."""
    bus, device, mode = _open(args)
    try:
        layout = device.peek_layout(mode)
        peaks = layout & 0x07
        print(f"  frames report layout 0x{layout:02X}: {peaks} peak(s) per "
              f"zone")
        frame = device.next_frame(mode, layout, hdr=args.hdr)
        print(f"  {frame.summary()}")
        print("  the part is measuring; run 'live' to watch it")
    finally:
        device.stop()
        bus.close()
    return 0


def cmd_frame(args):
    """Capture one frame and print it."""
    bus, device, mode = _open(args)
    try:
        layout = device.peek_layout(mode)
        frame = device.next_frame(mode, layout, hdr=args.hdr)
        style = resolve_style(args.style, frame)
        print()
        print(show(frame, style, histogram=args.histogram))
    finally:
        device.stop()
        bus.close()
    return 0


def cmd_live(args):
    """Watch the sensor until interrupted."""
    bus, device, mode = _open(args)
    print(HIDE_CURSOR, end="")
    shown, started, dropped = 0, time.monotonic(), 0
    try:
        layout = device.peek_layout(mode)
        style = None
        while args.frames == 0 or shown < args.frames:
            try:
                frame = device.next_frame(mode, layout, hdr=args.hdr)
            except Exception:
                # A dropped frame is worth counting and not worth stopping
                # for: the part is free-running and the next one is already
                # on its way.
                dropped += 1
                if dropped > 10:
                    raise
                continue
            shown += 1
            if style is None:
                style = resolve_style(args.style, frame)
            rate = shown / max(1e-6, time.monotonic() - started)
            body = show(frame, style, histogram=args.histogram)
            footer = (f"  {rate:.1f} frames/s"
                      + (f", {dropped} dropped" if dropped else "")
                      + "   ctrl-c to stop")
            print(CLEAR_SCREEN + body + "\n" + footer)
    except KeyboardInterrupt:
        pass
    finally:
        print(SHOW_CURSOR, end="")
        try:
            device.stop()
        finally:
            bus.close()
    elapsed = time.monotonic() - started
    print(f"\n  {shown} frames in {elapsed:.1f} s "
          f"({shown / max(1e-6, elapsed):.1f} frames/s)"
          + (f", {dropped} dropped" if dropped else ""))
    return 0


def cmd_modes(args):
    """List the zone grids and what each one costs to read."""
    from tof_frame import frame_size
    print(f"  {'mode':<14}{'grid':>8}{'zones':>8}{'frames':>8}"
          f"{'bytes/frame':>14}")
    for name, mode in MODES.items():
        grid = f"{mode.columns}x{mode.rows}"
        print(f"  {name:<14}{grid:>8}{mode.zones:>8}"
              f"{mode.subframes:>8}{frame_size(mode, 0x01):>14}")
    print("\n  bytes/frame is for one peak per zone and no optional "
          "fields;\n  each of --signal, --noise and --xtalk adds 2 bytes to "
          "every zone.")
    return 0


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------


def build_parser():
    parser = argparse.ArgumentParser(
        prog="i3c_tof.py",
        description="Watch an AMS OSRAM TMF8829 time-of-flight sensor.")
    parser.add_argument("--version", action="version",
                        version=f"i3c_tof.py {TOOL_VERSION}")
    subparsers = parser.add_subparsers(dest="command", metavar="command")

    def add(name, function, help_text):
        sub = subparsers.add_parser(name, help=help_text)
        sub.set_defaults(function=function)
        return sub

    def add_display(sub):
        sub.add_argument("--mode", default="8x8", choices=sorted(MODES),
                         help="zone grid (default 8x8)")
        sub.add_argument("--style", default="auto",
                         choices=["auto", "colour", "mono", "numeric",
                                  "histogram"],
                         help="how to draw the grid (default auto)")
        sub.add_argument("--histogram", action="store_true",
                         help="also show the distance histogram")

    def add_hardware(sub):
        add_display(sub)
        sub.add_argument("--firmware", required=True,
                         help="path to the TMF8829 RAM application, as Intel HEX")
        sub.add_argument("--period", type=int, default=100,
                         help="measurement period in ms (default 100)")
        sub.add_argument("--peaks", type=int, default=1, choices=[1, 2, 3, 4],
                         help="objects reported per zone (default 1)")
        sub.add_argument("--signal", action="store_true",
                         help="include signal strength per peak")
        sub.add_argument("--noise", action="store_true",
                         help="include the noise floor per zone")
        sub.add_argument("--xtalk", action="store_true",
                         help="include the crosstalk estimate per zone")
        sub.add_argument("--hdr", action="store_true",
                         help="read frames over HDR-DDR instead of SDR "
                              "(8x8 and 16x16 only)")
        sub.add_argument("--serial", default=None,
                         help="serial number of the Supernova to open")
        sub.add_argument("--voltage", type=int, default=3300,
                         help="bus voltage in mV (default 3300)")
        sub.add_argument("--push-pull", default="PUSH_PULL_5_MHZ_50_DC")
        sub.add_argument("--open-drain", default="OPEN_DRAIN_1_MHZ")
        sub.add_argument("--drive", default="FAST_MODE")
        sub.add_argument("-v", "--verbose", action="store_true")

    sub = add("demo", cmd_demo, "render a synthetic scene, without hardware")
    add_display(sub)
    sub.add_argument("--scene", default="hand",
                     choices=["hand", "wall", "corner"])
    sub.add_argument("--frames", type=int, default=1,
                     help="how many frames to animate (default 1)")
    sub.add_argument("--interval", type=float, default=0.1)

    sub = add("bringup", cmd_bringup,
              "download the firmware and start measuring, reporting each step")
    add_hardware(sub)

    sub = add("frame", cmd_frame, "capture and print a single frame")
    add_hardware(sub)

    sub = add("live", cmd_live, "watch the sensor until interrupted")
    add_hardware(sub)
    sub.add_argument("--frames", type=int, default=0,
                     help="stop after this many frames (default: run until "
                          "interrupted)")

    add("modes", cmd_modes, "list the zone grids and what each one costs")
    return parser


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    if not getattr(args, "command", None):
        parser.print_help()
        return 2
    try:
        return args.function(args)
    except KeyboardInterrupt:
        print("\ninterrupted", file=sys.stderr)
        return 130
    except Exception as exc:                        # noqa: BLE001
        print(f"error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
