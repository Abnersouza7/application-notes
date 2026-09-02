#!/usr/bin/env python3
"""Bring-up and control for the AMS OSRAM TMF8829, over a Supernova bus.

Getting depth data out of this part takes four steps, and only the last one
looks like a normal sensor driver. In order:

1. **Get it into the bootloader.** A TMF8829 powers up into whichever of the
   bootmonitor or the ROM application its fuses select; the ROM application
   answers a version query and rejects everything else. Neither application
   implements the bootloader command set, so a part running one has to be sent
   back to the bootmonitor first.

2. **Download the RAM application.** The ranging firmware ships as an Intel
   HEX file and is written to the part over the bus every time it powers up.
   This is not unusual for dToF, and it is the step that surprises people
   coming from a MEMS sensor where the part simply works when you talk to it.

3. **Load a configuration page and start measuring.** The part has a
   preconfigured page per zone grid, so choosing a resolution is a single
   command rather than a table of register writes.

4. **Read frames out of a FIFO.** Not registers. A frame is one read that
   starts at the FIFO status register and runs to the end of the frame, and
   the length depends on what the layout byte in the frame says.

Everything below is deliberately explicit about which of those four steps it
belongs to, because the failure modes look identical from the outside: the
part answers, and the answers are wrong.

Protocol details come from the vendor's MIT-licensed driver:
https://github.com/ams-OSRAM/tmf8829_driver_python
"""

from __future__ import annotations

import time

from binhosupernova.commands.i3c.definitions import TransferMode

from i3c_mems import MemsError, payload_of
from tof_frame import (MODES, FRAME_HEADER_SIZE, Frame, Mode, PRE_HEADER_SIZE,
                       decode, frame_size, stitch)

# Registers common to the bootloader and the application.
APP_ID = 0x00               # 0x80 bootloader, 0x02 ROM app, 0x01 RAM app
MAJOR, MINOR = 0x01, 0x02
CMD_STAT = 0x08             # write a command here, read the status back
PREV_CMD = 0x09

# Host interface registers, at the top of the map.
INT_STATUS = 0xE1           # write-1-to-clear
INT_ENAB = 0xE2
DEVICE_ID = 0xE3
RESET = 0xF7
ENABLE = 0xF8
FIFOSTATUS = 0xFA           # a frame read starts here
FIFO = 0xFF

INT_RESULTS = 0x01          # a result frame is waiting in the FIFO

# Application ids returned by register 0x00.
APP_BOOTLOADER = 0x80
APP_ROM = 0x02
APP_RAM = 0x01

# ENABLE bits. powerup_select makes the part ignore its fuses and wait for
# host commands; pon keeps it powered while it does.
POWERUP_SELECT = 1 << 4
PON = 1 << 2
FORCE_BOOTMONITOR = POWERUP_SELECT | PON
CPU_READY = 1 << 6

SOFT_RESET = 1 << 6         # RESET register

# Bootloader commands.
#
# The part has two CPUs, and both of them need the image. W_FIFO writes to one;
# W_FIFO_BOTH writes to both, and it is the one to use. Downloading with
# W_FIFO produces a part that reports a successful download, accepts
# START_RAM_APP, answers it with OK, and then halts: register 0x00 reads 0xFF
# and cpu_ready in ENABLE stays low, because the second CPU has no firmware.
#
# On a part that has been running before, this can appear to work, because the
# second CPU's RAM still holds a valid image from earlier. It fails on the
# first cold start after a power cycle, which is the one case that matters.
BL_W_FIFO = 0x44            # address u32, length in 32-bit words u16, one CPU
BL_W_FIFO_BOTH = 0x45       # the same, to both CPUs
BL_START_RAM_APP = 0x16
BL_READY = 0x00

# Application commands.
CMD_MEASURE = 0x10
CMD_CLEAR_STATUS = 0x11
CMD_WRITE_PAGE_AND_MEASURE = 0x14
CMD_LOAD_CONFIG_PAGE = 0x16
CMD_STOP = 0xFF

STAT_OK = 0x00
STAT_ACCEPTED = 0x01        # a cyclic measurement is running

CMD_STATUS_NAMES = {
    0x00: "OK", 0x01: "accepted, running", 0x02: "config rejected",
    0x03: "application error", 0x04: "result frames too large",
    0x05: "VCSEL config error", 0x06: "wakeup timed out",
    0x07: "unexpected reset", 0x08: "unknown command",
    0x09: "unknown config id", 0x0E: "oscillator tune failed",
}

# Configuration page registers, valid after a LOAD_CFG command.
CFG_PERIOD_MS = 0x22        # u16
CFG_KILO_ITERATIONS = 0x24  # u16
CFG_FP_MODE = 0x26
CFG_RESULT_FORMAT = 0x2A

# Download chunk. The bootloader takes a 4-byte-aligned block per command, so
# the chunk size trades command overhead against the largest write the
# adapter will carry in one transfer.
DOWNLOAD_CHUNK = 256

# The largest single read the adapter will perform. Measured by binary search
# on the hardware: 1024 bytes succeeds, 1025 returns nothing at all. The cliff
# is worth knowing about because it is not a truncation. A frame larger than
# this does not come back short, it does not come back, and the sensor is
# still holding it in the FIFO.
#
# Both grids above 16x16 are larger than this, so a full-resolution frame
# always takes more than one read. That is what the chunked read below is for.
MAX_READ = 1024

# HDR-DDR. The part's BCR declares advanced capabilities, and it does deliver
# them, but two things about its DDR reads differ from the SDR path and both
# were established by measurement rather than found in a document.
#
# The command byte is a *word* address, not a register address and not an
# opaque command: register = (command - 0x80) * 2. Sweeping 0x80 to 0xFF walks
# the register map in steps of two, which is what identified it.
#
# DDR moves 16-bit words, and each word arrives with its bytes in the opposite
# order to the SDR read of the same registers, so the buffer has to be
# unswapped before anything can be decoded from it.
DDR_COMMAND_BASE = 0x80

# A DDR read of the FIFO region starts at register 0xFA, and only four bytes
# of FIFO bookkeeping precede the frame rather than the five an SDR read sees.
DDR_FIFO_COMMAND = DDR_COMMAND_BASE + FIFOSTATUS // 2
DDR_PRE_HEADER = 4


def parse_intel_hex(path: str) -> list[tuple[int, bytes]]:
    """Read an Intel HEX file into contiguous (address, bytes) segments.

    Only the two record types the vendor's firmware image uses are handled:
    extended linear address (0x04) sets the upper 16 bits, data (0x00)
    carries the bytes. Records that are contiguous with the previous one are
    merged, so the download runs as a few long streams rather than as one
    command per sixteen-byte record.
    """
    segments: list[tuple[int, bytes]] = []
    upper, address, buffer = 0, None, bytearray()
    with open(path, "r", newline="") as handle:
        for line in handle:
            line = line.strip()
            if not line.startswith(":"):
                continue
            count = int(line[1:3], 16)
            offset = int(line[3:7], 16)
            record = int(line[7:9], 16)
            data = bytes.fromhex(line[9:9 + count * 2])
            if record == 0x04:
                if buffer:
                    segments.append((address, bytes(buffer)))
                    buffer, address = bytearray(), None
                upper = int.from_bytes(data, "big") << 16
            elif record == 0x00:
                here = upper + offset
                if address is not None and here == address + len(buffer):
                    buffer += data
                else:
                    if buffer:
                        segments.append((address, bytes(buffer)))
                    address, buffer = here, bytearray(data)
    if buffer:
        segments.append((address, bytes(buffer)))
    return segments


class Tmf8829:
    """A TMF8829 on a Supernova bus.

    Wraps an open `i3c_mems.Bus`. The dynamic address is re-read after every
    reset, because a reset drops the part off the bus and it has to be
    enumerated again before anything else will work.
    """

    def __init__(self, bus, address=None):
        self.bus = bus
        self.address = address if address is not None else self._enumerate()

    # ---- transport -------------------------------------------------------

    def _enumerate(self) -> int:
        table = self.bus.init_bus()
        if not table:
            raise MemsError("no target answered dynamic address assignment")
        return table[0]["dynamic_address"]

    def reenumerate(self) -> int:
        self.address = self._enumerate()
        return self.address

    def read(self, register: int, count: int = 1) -> bytes:
        """Read `count` bytes from `register` onward.

        The register pointer auto-increments, which is what makes a frame
        readable in one transfer and is why a frame read starts at the FIFO
        status register rather than at the FIFO itself.
        """
        ok, response = self.bus.try_call(
            self.bus.device.i3cControllerRead, self.address,
            TransferMode.I3C_SDR, [register], count)
        if not ok:
            return b""
        return bytes(payload_of(response) or b"")

    def write(self, register: int, values) -> bool:
        ok, response = self.bus.try_call(
            self.bus.device.i3cControllerWrite, self.address,
            TransferMode.I3C_SDR, [register], list(values))
        code = response.get("result") if isinstance(response, dict) else response
        return ok and code == "SUCCESS"

    def read_u8(self, register: int) -> int | None:
        value = self.read(register)
        return value[0] if value else None

    def write_u16(self, register: int, value: int) -> bool:
        return self.write(register, [value & 0xFF, (value >> 8) & 0xFF])

    # ---- step 1: get into the bootloader ---------------------------------

    def app_id(self) -> int | None:
        return self.read_u8(APP_ID)

    def version(self) -> tuple[int, int, int] | None:
        info = self.read(APP_ID, 3)
        return tuple(info) if len(info) == 3 else None

    def force_bootmonitor(self, settle: float = 0.5) -> int | None:
        """Send a running application back to the bootmonitor.

        There is no bootloader command for this, because the application does
        not implement bootloader commands. The route in is through the host
        interface registers, which stay live whatever is running: set
        powerup_select so the part ignores its fuses on the way up, then soft
        reset it.
        """
        self.write(ENABLE, [FORCE_BOOTMONITOR])
        time.sleep(0.05)
        self.write(RESET, [SOFT_RESET])
        time.sleep(settle)
        self.reenumerate()
        return self.app_id()

    def command(self, code: int, params=(), timeout: float = 1.5):
        """Issue a command and wait for the status to stop echoing it.

        Both the bootloader and the application work this way, and the
        handshake is the same in both: CMD_STAT reads back as the command
        while it is in progress and as a status code when it is done. Polling
        for "not the command I sent" rather than for a particular value is
        what makes it work for commands that finish before the first read.
        """
        frame = [code] if not params else [code, len(params), *params]
        if not self.write(CMD_STAT, frame):
            return None, "write failed"
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            status = self.read_u8(CMD_STAT)
            if status is not None and status != code:
                return status, CMD_STATUS_NAMES.get(status, f"0x{status:02X}")
            time.sleep(0.005)
        return None, "timeout"

    # ---- step 2: download the RAM application ----------------------------

    def download(self, segments, chunk: int = DOWNLOAD_CHUNK, progress=None):
        """Write the RAM application and report how long it took.

        Each block is announced with a write-FIFO command carrying the
        destination address and a length in 32-bit words, then streamed to the
        FIFO register. Blocks are padded to a word boundary because the length
        is expressed in words and a partial word cannot be described.

        W_FIFO_BOTH rather than W_FIFO, so both of the part's CPUs are loaded.
        See the note on the command constants: getting this wrong downloads
        cleanly and then halts on start.
        """
        written, started = 0, time.monotonic()
        for address, data in segments:
            data = data + b"\x00" * ((-len(data)) % 4)
            for offset in range(0, len(data), chunk):
                block = data[offset:offset + chunk]
                target = address + offset
                params = (list(target.to_bytes(4, "little"))
                          + list((len(block) // 4).to_bytes(2, "little")))
                status, note = self.command(BL_W_FIFO_BOTH, params)
                if status != BL_READY:
                    raise MemsError(
                        f"W_FIFO_BOTH rejected at 0x{target:08X}: {note}")
                if not self.write(FIFO, block):
                    raise MemsError(f"FIFO write failed at 0x{target:08X}")
                written += len(block)
                if progress:
                    progress(written)
        return written, time.monotonic() - started

    def start_ram_app(self, settle: float = 0.5) -> int | None:
        """Hand control to the downloaded application.

        Starting the application restarts the part, which drops it off the
        bus, so it has to be enumerated again before its registers can be read.
        The command itself reports OK either way: skip the re-enumeration and
        every subsequent read returns 0xFF, which looks like an application
        that failed to start rather than a target that is no longer addressed.
        """
        self.command(BL_START_RAM_APP, timeout=3.0)
        time.sleep(settle)
        try:
            self.reenumerate()
        except MemsError:
            return None
        return self.app_id()

    # ---- step 3: configure and measure -----------------------------------

    def configure(self, mode: Mode, period_ms: int = 100,
                  kilo_iterations: int | None = None,
                  peaks: int = 1, signal: bool = False, noise: bool = False,
                  xtalk: bool = False):
        """Load the preconfigured page for a zone grid and adjust it.

        The per-grid LOAD_CFG commands are the reason this is short. They set
        the SPAD map, the iteration count and the timing that the grid needs,
        and leave the page in registers 0x22 upward where anything worth
        overriding can be overridden before it is written back.

        `peaks`, `signal`, `noise` and `xtalk` control the result format, and
        they cost bandwidth: every field enabled is 2 more bytes per zone,
        times up to 1536 zones, on every frame.
        """
        status, note = self.command(mode.command, timeout=2.0)
        if status != STAT_OK:
            raise MemsError(f"loading the {mode.name} page failed: {note}")

        result_format = (peaks & 0x07)
        result_format |= 0x08 if signal else 0
        result_format |= 0x10 if noise else 0
        result_format |= 0x20 if xtalk else 0
        self.write(CFG_RESULT_FORMAT, [result_format])
        self.write_u16(CFG_PERIOD_MS, period_ms)
        if kilo_iterations is not None:
            self.write_u16(CFG_KILO_ITERATIONS, kilo_iterations)
        return result_format

    def start(self, timeout: float = 3.0):
        """Write the configuration page back and start a cyclic measurement.

        A cyclic measurement reports STAT_ACCEPTED rather than STAT_OK, and
        keeps running until it is stopped. Treating "accepted" as a failure
        is an easy mistake here, since every other command reports OK.
        """
        self.write(INT_ENAB, [INT_RESULTS])
        self.write(INT_STATUS, [0xFF])          # clear anything stale
        status, note = self.command(CMD_WRITE_PAGE_AND_MEASURE, timeout=timeout)
        if status not in (STAT_OK, STAT_ACCEPTED):
            raise MemsError(f"measurement not started: {note}")
        return status

    def stop(self):
        status, note = self.command(CMD_STOP, timeout=2.0)
        self.write(INT_ENAB, [0x00])
        return status, note

    # ---- step 4: read frames ---------------------------------------------

    def interrupt(self) -> int:
        """Read the interrupt status and clear whatever was set.

        Write-1-to-clear, and cleared before the frame is read rather than
        after: the sensor is free-running, so clearing afterwards can discard
        the flag for the next frame that arrived while this one was being
        read.
        """
        status = self.read_u8(INT_STATUS) or 0
        if status:
            self.write(INT_STATUS, [status])
        return status

    def read_bytes(self, size: int) -> bytes:
        """Pull `size` bytes of one frame out of the FIFO.

        The first read starts at the FIFO status register so the systick and
        status bytes come along with it, which is the layout the decoder
        expects. Any read after that starts at the FIFO data register itself:
        that register does not auto-increment, so reading it repeatedly
        continues the same frame rather than restarting it.

        Chunking is not optional above 16x16. The adapter will not carry more
        than MAX_READ bytes in one transfer, and every grid larger than that
        produces frames which exceed it.
        """
        chunks, taken = [], 0
        while taken < size:
            want = min(MAX_READ, size - taken)
            register = FIFOSTATUS if taken == 0 else FIFO
            chunk = self.read(register, want)
            if len(chunk) != want:
                raise MemsError(
                    f"short frame read: {taken + len(chunk)} of {size} bytes")
            chunks.append(chunk)
            taken += want
        return b"".join(chunks)

    def hdr_read(self, command: int, count: int) -> bytes:
        """One HDR-DDR read, unswapped into the order SDR would have given.

        Each 16-bit word arrives with its bytes reversed relative to an SDR
        read of the same registers, so the swap here is not an endianness
        preference, it is what makes the two paths return the same bytes.
        """
        from binhosupernova.commands.i3c.definitions import TransferMode  # noqa: F401

        length = count + count % 2          # DDR moves whole words
        ok, response = self.bus.try_call(
            self.bus.device.i3cControllerHdrDdrRead, self.address, command,
            length)
        data = bytes(payload_of(response) or b"") if ok else b""
        self.bus.try_call(self.bus.device.i3cControllerTriggerHdrExitPattern)
        if len(data) < count:
            raise MemsError(f"HDR-DDR read returned {len(data)} of {count} bytes")
        return bytes(data[index ^ 1] for index in range(len(data)))[:count]

    def read_frame_hdr(self, mode: Mode, layout: int = 0x01) -> Frame:
        """Read one frame over HDR-DDR instead of SDR.

        Verified against the SDR path byte for byte on the bench, so the two
        return the same zones for the same frame.

        Limited to grids whose whole frame fits in one transfer. A DDR command
        addresses a 16-bit word, so no command lands on the FIFO data register
        at 0xFF on its own, and a DDR frame read therefore cannot be continued
        across transfers the way the SDR one is.
        """
        size = frame_size(mode, layout) - (PRE_HEADER_SIZE - DDR_PRE_HEADER)
        if size > MAX_READ:
            raise MemsError(
                f"a {mode.name} frame is {size} bytes, over the {MAX_READ}-byte "
                f"transfer limit, and a DDR read cannot be chunked")
        raw = self.hdr_read(DDR_FIFO_COMMAND, size)
        return decode(raw, mode.columns, mode.rows_per_subframe,
                      pre_header=DDR_PRE_HEADER)

    def read_frame(self, mode: Mode, layout: int = 0x01,
                   hdr: bool = False) -> Frame:
        """Read and decode one frame, or one half of one.

        The read length has to be right. The frame sits in a FIFO, so reading
        too little leaves a tail behind that turns up as the head of the next
        frame, and the failure shows up as a plausible-looking image rather
        than as an error.
        """
        if hdr:
            return self.read_frame_hdr(mode, layout)
        raw = self.read_bytes(frame_size(mode, layout))
        return decode(raw, mode.columns, mode.rows_per_subframe)

    def next_frame(self, mode: Mode, layout: int = 0x01,
                   timeout: float = 2.0, hdr: bool = False) -> Frame:
        """Wait for a complete measurement, stitching halves if the mode uses them."""
        halves: list[Frame] = []
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.interrupt() & INT_RESULTS:
                halves.append(self.read_frame(mode, layout, hdr=hdr))
                if len(halves) == mode.subframes:
                    return stitch(halves, mode)
            else:
                time.sleep(0.002)
        raise MemsError(f"no frame within {timeout:.1f} s "
                        f"({len(halves)} of {mode.subframes} subframes)")

    def peek_layout(self, mode: Mode, timeout: float = 2.0) -> int:
        """Read the layout byte of the next frame without decoding it.

        The frame is self-describing but the read length depends on the
        description, which is circular. Breaking it takes one short read of
        the header, and it is worth doing once at startup rather than
        assuming the result format took effect.
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.interrupt() & INT_RESULTS:
                header = self.read(FIFOSTATUS,
                                   PRE_HEADER_SIZE + FRAME_HEADER_SIZE)
                if len(header) > PRE_HEADER_SIZE:
                    layout = header[PRE_HEADER_SIZE + 1]
                    # That read left the body of the frame in the FIFO. Drain
                    # it so the first real frame starts at a frame boundary,
                    # in chunks because the remainder can exceed one transfer.
                    remaining = (frame_size(mode, layout)
                                 - PRE_HEADER_SIZE - FRAME_HEADER_SIZE)
                    while remaining > 0:
                        want = min(MAX_READ, remaining)
                        if len(self.read(FIFO, want)) != want:
                            break
                        remaining -= want
                    self.write(INT_STATUS, [0xFF])
                    return layout
            time.sleep(0.002)
        raise MemsError("no frame to inspect")


def bring_up(bus, hex_path: str, mode: Mode = MODES["8x8"], report=print,
             **config):
    """Take a TMF8829 from power-on to a running measurement.

    Written as one function because the four steps are not independently
    useful: a part that is in the bootloader but has no firmware is no more
    able to range than one that was never touched.
    """
    device = Tmf8829(bus)
    version = device.version()
    report(f"  target at 0x{device.address:02X}, "
           f"APP_ID 0x{version[0]:02X} v{version[1]}.{version[2]}")

    if device.app_id() != APP_BOOTLOADER:
        report("  an application is running; forcing the bootmonitor")
        if device.force_bootmonitor() != APP_BOOTLOADER:
            raise MemsError("could not reach the bootloader")
        report(f"    bootloader at 0x{device.address:02X}")

    segments = parse_intel_hex(hex_path)
    total = sum(len(data) for _, data in segments)
    report(f"  downloading {total:,} bytes in {len(segments)} segment(s)")
    written, elapsed = device.download(segments)
    report(f"    {written:,} bytes in {elapsed:.2f} s "
           f"({written / elapsed / 1024:.1f} kB/s)")

    if device.start_ram_app() != APP_RAM:
        raise MemsError("the RAM application did not start")
    version = device.version()
    report(f"  RAM application v{version[1]}.{version[2]} running")

    result_format = device.configure(mode, **config)
    report(f"  {mode.name} configured, result format 0x{result_format:02X}")
    device.start()
    report(f"  measuring")
    return device
