#!/usr/bin/env python3
"""Exercise the management interface of a CMIS optical module from a Binho adapter.

CMIS separates the register model from the transport that carries it: chapter 5
defines READ, WRITE and TEST over a 256 byte window, and Appendix B defines the
Management Communication Interface that carries them. This tool implements the
register half once and swaps the transport underneath it, so the same commands
run over I2C, SPI or I3C.

    python cmis_mci.py selfcheck
    python cmis_mci.py scan
    python cmis_mci.py bringup
    python cmis_mci.py identify
    python cmis_mci.py read  --page 01h --byte 251
    python cmis_mci.py write --byte 127 --data 01
    python cmis_mci.py rollover
    python cmis_mci.py timing
    python cmis_mci.py budget --image-kib 1024
    python cmis_mci.py spi-probe   --transport spi
    python cmis_mci.py ibi         --transport i3c --seconds 10

A CMIS module answers at address 0x50 on every transport, because the I2CMCI
control byte A0h shifted right by one is 0x50 and no CMIS revision provides for
address strapping.

Requires the binhosupernova package for a Supernova, or binhopulsar for a
Pulsar. The Pulsar has no I3C interface, so --transport i3c needs a Supernova.
The selfcheck and budget commands need neither an adapter nor a module.
"""

import argparse
import math
import queue
import statistics
import struct
import sys
import time

TOOL_VERSION = "1.0"

CMIS_ADDRESS = 0x50


class CmisError(RuntimeError):
    """Anything that should stop the command with a readable message."""


# --------------------------------------------------------------------------
# CMIS register access layer
#
# Transport independent. Everything here is CMIS 5.3 chapter 5 and Appendix
# B.1, and none of it changes when the MCI underneath changes.
# --------------------------------------------------------------------------

UPPER_BASE, SEGMENT = 0x80, 0x80
BANK_SELECT, PAGE_SELECT = 126, 127          # 00h:126 (RW), 00h:127 (RWW)
CHECKSUM_BYTE, CHECKSUM_SPAN = 222, (128, 222)

# Section 5.2.2.1 cites the full page read advertisement at 01h:251.4. The field
# is FullPageReadSupported at 01h:251.1-0, a two bit encoding, which is also what
# the CMIS 5.4 revision history records.
FULL_PAGE_READ = (0x00, 0x01, 251)
NMAX_DEFAULT, NMAX_FULL_PAGE = 8, 128

# Section 5.2.2.2: "A successful WRITE writes a sequence of up to eight given
# byte values". FullPageReadSupported raises Nmax for reads only.
WRITE_NMAX = 8

MODULE_STATES = {
    1: "ModuleLowPwr", 2: "ModulePwrUp", 3: "ModuleReady",
    4: "ModulePwrDn", 5: "ModuleFault",
}

SFF8024 = {
    0x18: "QSFP-DD", 0x19: "OSFP", 0x1E: "QSFP112",
    0x1F: "DSFP", 0x24: "OSFP-XD", 0x25: "OSFP-XD",
}

# Table 10-4 maxima, in milliseconds. A module may advertise anything up to these.
LIMITS_MS = {
    "tREAD": 0.5, "tWRITE": 10.0, "tWRITENV": 80.0,
    "tBPC": 10.0, "tCDBF": 4960.0, "tMgmtInit": 2000.0,
}

# CDB lives on page 9Fh. Its Local Payload is 120 bytes, of which Write Firmware
# Block LPL (0103h) spends 4 on the block offset. The Extended Payload is pages
# A0h to AFh, 16 pages of 128 bytes, written by Write Firmware Block EPL (0104h).
CDB_PAGE, LPL_BLOCK, EPL_BLOCK, EPL_ACCESS = 0x9F, 116, 16 * 128, 128


def split_access(byte_addr, length, nmax):
    """Split one logical access into transactions the module is allowed to answer.

    Two independent limits apply, and both are easy to miss.

    Appendix B.1.2: the current byte address rolls over within its own 128 byte
    segment, 127 back to 0 and 255 back to 128, and never crosses between Lower
    and Upper Memory. An access that would cross does not read on into the next
    segment, it wraps back on itself.

    Section 5.2.2: a single access carries at most Nmax bytes.
    """
    if length <= 0:
        raise ValueError("length must be positive")
    if not 0 <= byte_addr <= 0xFF or byte_addr + length > 0x100:
        raise ValueError(f"access {byte_addr}..{byte_addr + length - 1} leaves the 256 byte window")

    out, addr, left = [], byte_addr, length
    while left:
        n = min(left, nmax, SEGMENT - (addr % SEGMENT))
        out.append((addr, n))
        addr, left = addr + n, left - n
    return out


class Ral:
    """The three CMIS primitives, READ, WRITE and TEST, over any MCI."""

    def __init__(self, mci, nmax=NMAX_DEFAULT):
        self.mci = mci
        self.nmax = nmax
        self.page = None

    # -- primitives --------------------------------------------------------

    def read(self, byte_addr, length=1):
        return b"".join(self.mci.read(a, n)
                        for a, n in split_access(byte_addr, length, self.nmax))

    def write(self, byte_addr, data):
        offset = 0
        for a, n in split_access(byte_addr, len(data), min(self.nmax, WRITE_NMAX)):
            self.mci.write(a, data[offset:offset + n])
            offset += n

    def test(self):
        """Readiness poll, B.2.5.5 on I2CMCI and B.3.8.5 on SPIMCI."""
        try:
            self.mci.read(0x00, 1)
            return True
        except CmisError:
            return False

    # -- paging ------------------------------------------------------------

    def select(self, bank, page, verify=True):
        """Select a bank and page, and confirm the module accepted it.

        Section 8.2.15 lets the module override a page write: selecting an
        unsupported page leaves PageSelect holding a supported one instead, with
        no error reported anywhere. A page that was written and never read back
        is therefore not evidence of the page the host is on.
        """
        if hasattr(self.mci, "select"):
            # SPIMCI carries bank, page and byte in every transaction header, so
            # there is no page select register and no cross transaction state.
            self.mci.select(bank, page)
            self.page = (bank, page)
            return
        if self.page != (bank, page):
            self.mci.write(BANK_SELECT, bytes([bank]))
            self.mci.write(PAGE_SELECT, bytes([page]))
            self.page = (bank, page)
        if verify:
            got = (self.mci.read(BANK_SELECT, 1)[0], self.mci.read(PAGE_SELECT, 1)[0])
            if got != (bank, page):
                self.page = got
                raise CmisError(
                    f"module overrode the page selection: asked for bank {bank:#04x} "
                    f"page {page:#04x}, it is on bank {got[0]:#04x} page {got[1]:#04x} "
                    "(CMIS 8.2.15, page not supported)")

    def read_page(self, bank, page, byte_addr, length=1):
        if byte_addr < UPPER_BASE:
            raise ValueError(f"{byte_addr:#04x} is in Lower Memory, paging does not apply")
        self.select(bank, page)
        return self.read(byte_addr, length)

    # -- the bring-up reads -------------------------------------------------

    def identity(self):
        ident, rev, mci = self.read(0, 3)
        return {
            "identifier": ident,                          # 00h:0, SFF-8024
            "cmis_revision": f"{rev >> 4}.{rev & 0x0F}",   # 00h:1 is decimal coded
            "mci_max_speed": (mci >> 2) & 0x0F,            # 00h:2.5-2
        }

    def module_state(self):
        """00h:3.3-1, the Module State Machine state, Table 6-14."""
        return (self.read(3, 1)[0] >> 1) & 0x07

    def nmax_from_advertisement(self):
        """01h:251.1-0: 00 unknown, 01 not supported, 10 supported."""
        bank, page, byte = FULL_PAGE_READ
        advertised = self.read_page(bank, page, byte, 1)[0] & 0x03
        return NMAX_FULL_PAGE if advertised == 0x02 else NMAX_DEFAULT

    def page00_checksum(self):
        """Return (computed, stored) for the Page 00h checksum at 00h:222.

        94 bytes of module specific content have to agree with a byte the module
        itself wrote, so this is the one bring-up check that cannot pass by
        accident on an absent, unpowered or unarmed target.
        """
        low, high = CHECKSUM_SPAN
        body = self.read_page(0x00, 0x00, low, high - low)
        stored = self.read_page(0x00, 0x00, CHECKSUM_BYTE, 1)[0]
        return sum(body) & 0xFF, stored


# --------------------------------------------------------------------------
# Adapters
#
# One class per Binho adapter, hiding the differences between the two SDKs.
# They are not drop-in compatible: the Pulsar I2C calls take a bus selector
# that the Supernova calls do not, and each SDK ships its own enums.
# --------------------------------------------------------------------------

class Adapter:
    """Common interface to a Binho USB host adapter."""

    kind = "adapter"
    buses = ()

    def __init__(self, serial=None, verbose=False):
        self.serial = serial
        self.verbose = verbose
        self.device = None
        self._responses = queue.Queue()
        self._ibis = queue.Queue()
        self._next_id = 0
        self._opened = False

    def open(self):
        result = self.device.open(serial=self.serial)
        if result.get("opcode") != 0:
            raise CmisError(f"could not open {self.kind}: {result.get('message')}")
        self._opened = True
        result = self.device.onEvent(self._on_event)
        if result.get("opcode") != 0:
            raise CmisError(f"could not register callback: {result.get('message')}")

    def close(self):
        if self._opened:
            self.device.close()
            self._opened = False

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *exc):
        self.close()
        return False

    def _on_event(self, response, system_message):
        # Must return promptly: the SDK calls this from its receive path.
        if response is None:
            return
        if isinstance(response, dict) and response.get("id") == 0:
            self._ibis.put(response)
        else:
            self._responses.put(response)

    def call(self, method, *args, timeout=5.0, allowed_results=(), **kwargs):
        """Invoke an SDK method and block for the response with a matching id."""
        self._next_id = (self._next_id % 65534) + 1
        request_id = self._next_id

        submission = method(request_id, *args, **kwargs)
        if submission.get("opcode") != 0:
            raise CmisError(f"{method.__name__} rejected: {submission.get('message')}")

        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise CmisError(f"{method.__name__}: timed out waiting for a response")
            try:
                response = self._responses.get(timeout=min(remaining, 0.25))
            except queue.Empty:
                continue
            if response.get("id") != request_id:
                continue                      # stale response from a timed-out request
            if self.verbose:
                print(f"    <- {response}")
            result = response.get("result")
            if result not in (None, "SUCCESS") and result not in allowed_results:
                raise CmisError(f"{method.__name__} failed: {result}")
            return response

    def wait_ibi(self, timeout=3.0):
        try:
            return self._ibis.get(timeout=timeout)
        except queue.Empty:
            return None

    def drain_ibis(self):
        while True:
            try:
                self._ibis.get_nowait()
            except queue.Empty:
                return

    # -- SPI, identical across both SDKs -----------------------------------

    def _spi_definitions(self):
        raise NotImplementedError

    def spi_init(self, frequency_hz, mode=0, chip_select=0):
        d = self._spi_definitions()
        self.call(self.device.setI2cSpiUartGpioVoltage, 3300)
        args = (d.SpiControllerBitOrder.MSB,
                getattr(d.SpiControllerMode, f"MODE_{mode}"),
                d.SpiControllerDataWidth._8_BITS_DATA,
                getattr(d.SpiControllerChipSelect, f"CHIP_SELECT_{chip_select}"),
                d.SpiControllerChipSelectPolarity.ACTIVE_LOW,
                frequency_hz)
        response = self.call(self.device.spiControllerInit, *args,
                             allowed_results=("INTERFACE_ALREADY_INITIALIZED",))
        # An interface that is already up ignores the settings passed to init, so
        # they have to be applied again through SetParameters.
        if response.get("result") != "SUCCESS":
            self.call(self.device.spiControllerSetParameters, *args)

    def spi_transfer(self, payload):
        payload = list(payload)
        response = self.call(self.device.spiControllerTransfer,
                             len(payload), payload, timeout=15.0)
        for key in ("payload", "data", "rx", "received"):
            value = response.get(key)
            if isinstance(value, (list, tuple)):
                return list(value)
        raise CmisError(f"no payload in the SPI response: {response}")


class SupernovaAdapter(Adapter):
    """Binho Supernova. Drives I3C, I2C and SPI."""

    kind = "supernova"
    buses = ("i3c", "i2c", "spi")

    def __init__(self, serial=None, verbose=False):
        super().__init__(serial, verbose)
        from binhosupernova.supernova import Supernova
        from binhosupernova.commands.i2c.definitions import I2cPullUpResistorsValue
        self.device = Supernova()
        self._PullUps = I2cPullUpResistorsValue

    def _spi_definitions(self):
        import binhosupernova.commands.spi.definitions as d
        return d

    def i2c_init(self, frequency_hz, pullup_ohms):
        self.call(self.device.setI2cSpiUartGpioVoltage, 3300)
        pull = _nearest_pullup(self._PullUps, pullup_ohms)
        response = self.call(self.device.i2cControllerInit, frequency_hz, pull,
                             allowed_results=("INTERFACE_ALREADY_INITIALIZED",))
        if response.get("result") != "SUCCESS":
            self.call(self.device.i2cControllerSetParameters, frequency_hz, pull)

    def i2c_write(self, address, subaddress, data):
        self.call(self.device.i2cControllerWrite, address, list(subaddress), list(data))

    def i2c_read(self, address, subaddress, length):
        r = self.call(self.device.i2cControllerRead, address, length, list(subaddress))
        return bytes(r.get("payload") or b"")

    def i2c_scan(self):
        r = self.call(self.device.i2cControllerScanBus, timeout=10.0)
        return r.get("addresses") or r.get("payload") or []

    # -- I3C, Supernova only ------------------------------------------------

    def i3c_init(self, push_pull, open_drain, drive, voltage_mv=3300):
        from binhosupernova.commands.i3c.definitions import (
            I3cPushPullTransferRate, I3cOpenDrainTransferRate, I2cTransferRate,
            I3cDriveStrength)
        try:
            rates = (getattr(I3cPushPullTransferRate, push_pull),
                     getattr(I3cOpenDrainTransferRate, open_drain),
                     I2cTransferRate._400KHz,
                     getattr(I3cDriveStrength, drive))
        except AttributeError as exc:
            raise CmisError(f"unknown rate or drive strength: {exc}") from None
        self.call(self.device.setI3cVoltage, voltage_mv)
        response = self.call(self.device.i3cControllerInit, *rates,
                             allowed_results=("INTERFACE_ALREADY_INITIALIZED",))
        # Init discards its arguments when the interface is already up, so the
        # settings have to be sent again or the previous run's rates stay in force.
        if response.get("result") != "SUCCESS":
            self.call(self.device.i3cControllerSetParameters, *rates)

    def i3c_init_bus(self):
        self.call(self.device.i3cControllerInitBus, timeout=10.0)
        r = self.call(self.device.i3cControllerGetTargetDevicesTable)
        return r.get("table") or r.get("payload") or []

    def _i3c_mode(self):
        from binhosupernova.commands.i3c.definitions import TransferMode
        return TransferMode.I3C_SDR

    def i3c_write(self, address, subaddress, data):
        self.call(self.device.i3cControllerWrite, address, self._i3c_mode(),
                  list(subaddress), list(data))

    def i3c_read(self, address, subaddress, length):
        r = self.call(self.device.i3cControllerRead, address, self._i3c_mode(),
                      list(subaddress), length)
        return bytes(r.get("payload") or b"")

    def i3c_enec_ibi(self, address, enable=True):
        method = self.device.i3cDirectENEC if enable else self.device.i3cDirectDISEC
        from binhosupernova.commands.i3c.definitions import I3cTargetEvents
        self.call(method, address, I3cTargetEvents.ENINT)


class PulsarAdapter(Adapter):
    """Binho Pulsar. Drives I2C and SPI. It has no I3C interface."""

    kind = "pulsar"
    buses = ("i2c", "spi")

    def __init__(self, serial=None, verbose=False, i2c_bus="A"):
        super().__init__(serial, verbose)
        from binhopulsar.pulsar import Pulsar
        from binhopulsar.commands.i2c.definitions import I2cBus, I2cPullUpResistorsValue
        self.device = Pulsar()
        self._PullUps = I2cPullUpResistorsValue
        try:
            self._bus = getattr(I2cBus, f"I2C_BUS_{i2c_bus.upper()}")
        except AttributeError:
            raise CmisError(f"unknown Pulsar I2C bus '{i2c_bus}', use A or B") from None

    def _spi_definitions(self):
        import binhopulsar.commands.spi.definitions as d
        return d

    def i2c_init(self, frequency_hz, pullup_ohms):
        self.call(self.device.setI2cSpiUartGpioVoltage, 3300)
        pull = _nearest_pullup(self._PullUps, pullup_ohms)
        response = self.call(self.device.i2cControllerInit, self._bus, frequency_hz, pull,
                             allowed_results=("INTERFACE_ALREADY_INITIALIZED",))
        if response.get("result") != "SUCCESS":
            self.call(self.device.i2cControllerSetParameters, self._bus, frequency_hz, pull)

    def i2c_write(self, address, subaddress, data):
        self.call(self.device.i2cControllerWrite, self._bus, address,
                  list(subaddress), list(data))

    def i2c_read(self, address, subaddress, length):
        r = self.call(self.device.i2cControllerRead, self._bus, address, length,
                      list(subaddress))
        return bytes(r.get("payload") or b"")

    def i2c_scan(self):
        r = self.call(self.device.i2cControllerScanBus, self._bus, timeout=10.0)
        return r.get("addresses") or r.get("payload") or []


def _nearest_pullup(enum_cls, ohms):
    """Pick the enum member closest to the requested pull-up value."""
    best, best_delta = None, None
    for member in enum_cls:
        if "DISABLE" in member.name:
            continue
        text = member.name.replace("I2C_PULLUP_", "").replace("Ohm", "").replace("_", ".")
        try:
            value = float(text[:-1]) * 1000 if text.lower().endswith("k") else float(text)
        except ValueError:
            continue
        delta = abs(value - ohms)
        if best_delta is None or delta < best_delta:
            best, best_delta = member, delta
    if best is None:
        raise CmisError("no usable pull-up value in this SDK's enum")
    return best


# --------------------------------------------------------------------------
# Management communication interfaces
#
# Each one presents read(byte_addr, n) and write(byte_addr, data) to the RAL.
# --------------------------------------------------------------------------

class I2cMci:
    """CMIS Appendix B.2, the normative I2C management interface."""

    name = "I2CMCI"

    def __init__(self, adapter, address=CMIS_ADDRESS, frequency_hz=400_000, pullup_ohms=1000):
        self.adapter, self.address = adapter, address
        adapter.i2c_init(frequency_hz, pullup_ohms)

    def present(self):
        return self.address in list(self.adapter.i2c_scan())

    def read(self, byte_addr, n):
        try:
            return self.adapter.i2c_read(self.address, [byte_addr], n)
        except CmisError as exc:
            raise CmisError(f"read {byte_addr:#04x} x{n}: {exc}") from None

    def write(self, byte_addr, data):
        self.adapter.i2c_write(self.address, [byte_addr], data)


class I3cMci:
    """CMIS carried over I3C SDR private transfers, a Binho derivation.

    No published revision of CMIS defines an I3C management interface. CMIS 5.4
    section 1.3.2 lists an "I2C-compatible MCI based on I3C" among features
    expected in a forthcoming revision, and Table B-1 in both 5.3 and 5.4 lists
    exactly two MCI variants, I2CMCI and SPIMCI.
    """

    name = "I3CMCI"
    BINDING_NOTE = "Binho derivation, not an OIF CMIS binding"

    def __init__(self, adapter, address=CMIS_ADDRESS,
                 push_pull="PUSH_PULL_12_5_MHZ_50_DC", open_drain="OPEN_DRAIN_1_MHZ",
                 drive="FAST_MODE"):
        if "i3c" not in adapter.buses:
            raise CmisError(f"the {adapter.kind} has no I3C interface, use a Supernova")
        self.adapter, self.address = adapter, address
        adapter.i3c_init(push_pull, open_drain, drive)

    def assign_address(self):
        table = self.adapter.i3c_init_bus()
        if table:
            first = table[0]
            self.address = (first.get("dynamic_address") if isinstance(first, dict)
                            else getattr(first, "dynamic_address", self.address))
        return table

    def read(self, byte_addr, n):
        return self.adapter.i3c_read(self.address, [byte_addr], n)

    def write(self, byte_addr, data):
        self.adapter.i3c_write(self.address, [byte_addr], data)


class SpiMci:
    """CMIS Appendix B.3, the normative SPI management interface, new in 5.3.

    Every transaction is 4 + N + M bytes: a 4 byte control word carrying the full
    bank, page and byte address, N flow control bytes whose last byte is the
    module's ACK (00h) or NACK (FFh), then M = EncodedSize + 1 data bytes.
    """

    name = "SPIMCI"
    ACK, NACK, MAX_TRANSFER = 0x00, 0xFF, 2048

    def __init__(self, adapter, n_flow=2, read_bit=0, frequency_hz=1_000_000,
                 mode=0, chip_select=0):
        self.adapter, self.n_flow = adapter, n_flow
        # The R/Wn polarity is contradictory in the specification: B.3.7.1.1 says
        # 1 requests a read, while B.3.8.1 to B.3.8.5 and Figures B-14 to B-17
        # show 0. The default follows the figures; spi-probe establishes which
        # one a given module implements.
        self.read_bit = read_bit
        self.page = (0, 0)
        adapter.spi_init(frequency_hz, mode=mode, chip_select=chip_select)

    def select(self, bank, page):
        """SPIMCI carries the page in every header, so this writes no register."""
        self.page = (bank, page)

    def control_word(self, is_read, length, byte_addr):
        """The 4 byte Transaction Control phase, B.3.7.1, Figure B-13."""
        if not 1 <= length <= self.MAX_TRANSFER:
            raise ValueError(f"SPIMCI carries 1 to {self.MAX_TRANSFER} bytes, not {length}")
        bank, page = self.page
        rwn = self.read_bit if is_read else 1 - self.read_bit
        word = ((rwn << 31) | ((bank & 0x0F) << 27) | ((length - 1) << 16)
                | (page << 8) | byte_addr)
        return list(word.to_bytes(4, "big"))

    def _transact(self, is_read, byte_addr, length, data=b""):
        payload = self.control_word(is_read, length, byte_addr) + [0] * self.n_flow
        payload += [0] * length if is_read else list(data)
        back = self.adapter.spi_transfer(payload)
        if len(back) != len(payload):
            raise CmisError(f"adapter returned {len(back)} bytes for a {len(payload)} byte transfer")
        ack = back[4 + self.n_flow - 1]
        if ack == self.NACK:
            raise CmisError(f"module NACKed the access at {byte_addr:#04x}, it is busy")
        if ack != self.ACK:
            raise CmisError(f"flow control byte was {ack:#04x}, neither ACK (00h) nor "
                            "NACK (FFh), so N is wrong")
        return bytes(back[4 + self.n_flow:])

    def probe_n(self, maximum=16):
        """Find N by sweeping it until a read of Page 00h validates its checksum.

        B.3.7.1.2 requires both sides to derive N identically, but N is computed
        from 00h:213 and 00h:27, registers that cannot be read until N is already
        known. The specification names no bootstrap value, so sweep.
        """
        for n in range(2, maximum + 1):
            self.n_flow = n
            try:
                computed, stored = Ral(self).page00_checksum()
            except (CmisError, IndexError, ValueError):
                continue
            if computed == stored:
                return n
        raise CmisError(f"no N in 2 to {maximum} produced a valid Page 00h checksum")

    def read(self, byte_addr, n):
        return self._transact(True, byte_addr, n)

    def write(self, byte_addr, data):
        self._transact(False, byte_addr, len(data), bytes(data))


# --------------------------------------------------------------------------
# Simulated module, for the offline self-check
# --------------------------------------------------------------------------

class SimulatedModule:
    """A CMIS target that obeys the B.1.2 wrap and the 8.2.15 page override."""

    SUPPORTED_PAGES = (0x00, 0x01)

    def __init__(self):
        self.lower = bytearray(0x80)
        self.pages = {p: bytearray(0x80) for p in self.SUPPORTED_PAGES}
        self.lower[0:3] = bytes([0x18, 0x53, 0x08])       # QSFP-DD, CMIS 5.3
        self.lower[3] = 3 << 1                            # ModuleReady
        self.pages[0x00][0] = 0x18
        self.pages[0x00][1:17] = b"BINHO           "[:16]
        self.pages[0x01][251 - 0x80] = 0x02               # FullPageReadSupported
        self.pages[0x00][CHECKSUM_BYTE - 0x80] = sum(self.pages[0x00][0:222 - 0x80]) & 0xFF
        self.reads = []

    def _arena(self, addr):
        if addr < UPPER_BASE:
            return self.lower
        page = self.lower[PAGE_SELECT]
        return self.pages[page if page in self.SUPPORTED_PAGES else 0x00]

    def read(self, addr, n):
        self.reads.append((addr, n))
        arena, base = self._arena(addr), 0 if addr < UPPER_BASE else UPPER_BASE
        return bytes(arena[(addr - base + i) % SEGMENT] for i in range(n))

    def write(self, addr, data):
        if addr == PAGE_SELECT and data[0] not in self.SUPPORTED_PAGES:
            self.lower[PAGE_SELECT] = 0x00                # 8.2.15, silently corrected
            return
        arena, base = self._arena(addr), 0 if addr < UPPER_BASE else UPPER_BASE
        for i, b in enumerate(data):
            arena[(addr - base + i) % SEGMENT] = b


# --------------------------------------------------------------------------
# Commands
# --------------------------------------------------------------------------

def ascii_field(raw):
    return "".join(chr(b) if 0x20 <= b < 0x7F else "." for b in raw).rstrip()


def parse_page(text):
    """Accept 01h, 0x01 or 1 for a page or bank number."""
    text = text.strip().lower()
    if text.endswith("h"):
        return int(text[:-1], 16)
    return int(text, 0)


def cmd_selfcheck(args):
    """Check the address arithmetic against a simulated module. No hardware."""
    checks = 0

    # An access never crosses the Lower and Upper Memory boundary, and never
    # exceeds Nmax.
    assert split_access(124, 8, 8) == [(124, 4), (128, 4)], split_access(124, 8, 8)
    assert split_access(0, 20, 8) == [(0, 8), (8, 8), (16, 4)]
    assert split_access(128, 94, 128) == [(128, 94)]
    assert split_access(0x7F, 1, 8) == [(0x7F, 1)]
    checks += 1
    print("  [OK] an access splits at 7Fh and at Nmax")

    module = SimulatedModule()
    ral = Ral(module)

    assert ral.identity() == {"identifier": 0x18, "cmis_revision": "5.3", "mci_max_speed": 2}
    assert MODULE_STATES[ral.module_state()] == "ModuleReady"
    checks += 1
    print("  [OK] identity and module state decode")

    module.reads.clear()
    computed, stored = ral.page00_checksum()
    assert computed == stored, (computed, stored)
    assert all(n <= NMAX_DEFAULT for _, n in module.reads), module.reads
    checks += 1
    print("  [OK] Page 00h checksum agrees, in accesses of Nmax or fewer bytes")

    ral.nmax = ral.nmax_from_advertisement()
    assert ral.nmax == NMAX_FULL_PAGE
    module.reads.clear()
    assert ral.page00_checksum()[0] == stored
    assert (128, 94) in module.reads, module.reads
    checks += 1
    print("  [OK] FullPageReadSupported raises Nmax, and the 94 byte read fits in one access")

    try:
        ral.select(0x00, 0x99)
    except CmisError as exc:
        assert "overrode" in str(exc)
    else:
        raise AssertionError("selecting an unsupported page must not look like success")
    checks += 1
    print("  [OK] a module that overrides an unsupported page is caught, not believed")

    assert ral.test()
    checks += 1
    print("  [OK] TEST answers on a live module")

    # A write is capped at 8 bytes even when reads are not, so a 16 byte write
    # must become two accesses.
    ral.nmax = NMAX_FULL_PAGE
    module.reads.clear()
    ral.write(0x20, bytes(range(16)))
    assert ral.read(0x20, 16) == bytes(range(16))
    checks += 1
    print("  [OK] a write stays within the 8 byte WRITE limit while reads use Nmax")

    print(f"\ncmis_mci.py {TOOL_VERSION} self-check: {checks}/{checks} OK")
    return 0


def wire_ms(n_data, frequency_hz):
    """Rough time on the wire: 4 framing and address bytes plus payload, 9 bits each."""
    return (4 + n_data) * 9 / frequency_hz * 1000


def _budget(image_bytes, block, access, write_ms, frequency_hz):
    blocks, per_block = math.ceil(image_bytes / block), math.ceil(block / access)
    bus = blocks * per_block * wire_ms(access, frequency_hz) / 1000
    return blocks, per_block, bus, blocks * write_ms / 1000


def cmd_budget(args):
    """Print the firmware update time budget. No hardware."""
    image_bytes, write_ms = args.image_kib * 1024, args.write_ms
    rows = [
        ("I2CMCI 400 kHz, LPL 0103h", LPL_BLOCK, WRITE_NMAX, 400_000),
        ("I3C SDR 12.5 Mb/s, LPL 0103h", LPL_BLOCK, WRITE_NMAX, 12_500_000),
        ("I2CMCI 400 kHz, EPL 0104h", EPL_BLOCK, EPL_ACCESS, 400_000),
        ("I3C SDR 12.5 Mb/s, EPL 0104h", EPL_BLOCK, EPL_ACCESS, 12_500_000),
    ]
    print(f"Firmware update budget, {image_bytes / 1024:.0f} KiB image, "
          f"tWRITE {write_ms:.0f} ms per block")
    print(f"  {'path':<30}{'blocks':>8}{'per block':>12}{'bus':>10}{'hold-off':>12}{'total':>10}")
    totals = []
    for label, block, access, frequency in rows:
        blocks, per_block, bus, holdoff = _budget(image_bytes, block, access, write_ms, frequency)
        totals.append(bus + holdoff)
        print(f"  {label:<30}{blocks:>8}{per_block:>9} txn{bus:>8.2f} s{holdoff:>10.2f} s"
              f"{bus + holdoff:>9.2f} s")

    slow, clock_only, payload_only, fast = totals[0], totals[1], totals[2], totals[3]
    print(f"\n  Bus clock alone, 400 kHz to 12.5 Mb/s, about 31 times the wire speed: "
          f"{slow / clock_only:.2f} times faster.")
    print(f"  Payload size alone, LPL to EPL at 400 kHz: {slow / payload_only:.1f} times faster.")
    print(f"  Both together: {slow / fast:.1f} times faster.")
    print("\n  The hold-off column is what moves the total, and it is not a property of")
    print("  the wire. It shrinks because a larger block means fewer blocks, each")
    print("  paying tWRITE once.")
    print("\n  These are arithmetic from the CMIS Table 10-4 maxima and the access")
    print("  limits in section 5.2.2, not measurements. Run 'timing' against a module")
    print("  to find out which hold-offs it actually takes.")
    return 0


def open_adapter(args):
    """Open the adapter the user asked for, or the only one attached."""
    wanted = args.adapter
    if wanted is None:
        candidates = []
        for cls in (SupernovaAdapter, PulsarAdapter):
            try:
                cls()
            except ImportError:
                continue
            except CmisError:
                continue
            candidates.append(cls)
        if not candidates:
            raise CmisError(
                "no adapter SDK is installed.\n"
                "    pip install binhosupernova     (Supernova)\n"
                "    pip install binhopulsar        (Pulsar)")
        cls = candidates[0]
    else:
        cls = SupernovaAdapter if wanted == "supernova" else PulsarAdapter

    if args.transport == "i3c" and "i3c" not in cls.buses:
        raise CmisError("the Pulsar has no I3C interface, use a Supernova for --transport i3c")

    adapter = cls(serial=args.serial, verbose=args.verbose)
    adapter.open()
    return adapter


def open_mci(adapter, args):
    if args.transport == "i2c":
        return I2cMci(adapter, address=args.address, frequency_hz=args.frequency)
    if args.transport == "i3c":
        mci = I3cMci(adapter, address=args.address)
        mci.assign_address()
        return mci
    return SpiMci(adapter, n_flow=args.n or 2, read_bit=args.read_bit or 0,
                  frequency_hz=args.frequency if args.frequency != 400_000 else 1_000_000)


def cmd_scan(args):
    with open_adapter(args) as adapter:
        if args.transport == "i3c":
            mci = I3cMci(adapter, address=args.address)
            table = mci.assign_address()
            if not table:
                print("no I3C targets enumerated")
                return 1
            for entry in table:
                get = entry.get if isinstance(entry, dict) else lambda k, d=None: getattr(entry, k, d)
                print(f"static {get('static_address', 0) or 0:#04x} -> "
                      f"dynamic {get('dynamic_address', 0) or 0:#04x}  "
                      f"BCR {get('bcr', 0) or 0:#04x}  DCR {get('dcr', 0) or 0:#04x}")
            return 0
        if args.transport == "spi":
            print("SPIMCI has no bus scan: one dedicated CSn per target, no addressing.")
            print("Use 'spi-probe' instead, which establishes N and the R/Wn polarity.")
            return 0
        mci = I2cMci(adapter, address=args.address, frequency_hz=args.frequency)
        found = list(adapter.i2c_scan())
        for address in found:
            marker = "  <- CMIS module" if address == args.address else ""
            print(f"{address:#04x}{marker}")
        if args.address not in found:
            print(f"\nnothing answers at {args.address:#04x}: check power and the "
                  "management pins before looking at software")
            return 1
        return 0


def cmd_bringup(args):
    """The sequence a host runs at module discovery, in the order it runs it."""
    with open_adapter(args) as adapter:
        mci = open_mci(adapter, args)
        ral = Ral(mci)
        print(f"MCI     : {mci.name}")
        if isinstance(mci, I3cMci):
            print(f"binding : {mci.BINDING_NOTE}")

        if isinstance(mci, I2cMci) and not mci.present():
            print(f"FAIL    : nothing answers at {args.address:#04x}")
            return 1
        print(f"TEST()  : {'ready' if ral.test() else 'rejecting accesses'}")

        ident = ral.identity()
        print(f"00h:0   : {ident['identifier']:#04x} "
              f"{SFF8024.get(ident['identifier'], 'form factor not in this table')}")
        print(f"00h:1   : CMIS {ident['cmis_revision']}")
        print(f"00h:2   : MciMaxSpeed code {ident['mci_max_speed']}")
        state = ral.module_state()
        print(f"00h:3   : {MODULE_STATES.get(state, f'reserved({state})')}")

        temperature, vcc = struct.unpack(">hH", ral.read(14, 4))
        print(f"00h:14  : {temperature / 256:.2f} C   (S16, 1/256 C)")
        print(f"00h:16  : {vcc * 1e-4:.3f} V   (U16, 100 uV)")

        ral.nmax = ral.nmax_from_advertisement()
        print(f"01h:251 : FullPageReadSupported gives Nmax {ral.nmax} bytes for reads")

        page00 = ral.read_page(0x00, 0x00, 129, 53)
        print(f"00h:129 : VendorName '{ascii_field(page00[0:16])}'")
        print(f"00h:148 : VendorPN   '{ascii_field(page00[19:35])}'")
        print(f"00h:166 : VendorSN   '{ascii_field(page00[37:53])}'")

        computed, stored = ral.page00_checksum()
        ok = computed == stored
        print(f"00h:222 : checksum computed {computed:#04x}, module stores {stored:#04x}, "
              f"{'OK' if ok else 'MISMATCH'}")
        if not ok:
            print("FAIL    : 94 bytes of Page 00h disagree with the module's own checksum")
            return 1
        print("\nPASS    : present, readable, paged correctly and self consistent.")
        return 0


def cmd_identify(args):
    with open_adapter(args) as adapter:
        ral = Ral(open_mci(adapter, args))
        ident = ral.identity()
        print(f"identifier    {ident['identifier']:#04x} "
              f"{SFF8024.get(ident['identifier'], 'not in this table')}")
        print(f"CMIS revision {ident['cmis_revision']}")
        page00 = ral.read_page(0x00, 0x00, 129, 61)
        for label, lo, hi in (("VendorName", 0, 16), ("VendorOUI", 16, 19),
                              ("VendorPN", 19, 35), ("VendorRev", 35, 37),
                              ("VendorSN", 37, 53), ("DateCode", 53, 61)):
            raw = page00[lo:hi]
            shown = raw.hex(" ") if label == "VendorOUI" else f"'{ascii_field(raw)}'"
            print(f"{label:<13} {shown}")
        return 0


def cmd_read(args):
    with open_adapter(args) as adapter:
        ral = Ral(open_mci(adapter, args))
        ral.nmax = args.nmax or NMAX_DEFAULT
        if args.byte < UPPER_BASE:
            data = ral.read(args.byte, args.count)
            where = f"00h:{args.byte}"
        else:
            data = ral.read_page(args.bank, args.page, args.byte, args.count)
            where = f"{args.page:02X}h:{args.byte}"
        print(f"{where} x{args.count} = {data.hex(' ')}")
        return 0


def cmd_write(args):
    data = bytes.fromhex(args.data.replace(" ", ""))
    with open_adapter(args) as adapter:
        ral = Ral(open_mci(adapter, args))
        if args.byte >= UPPER_BASE:
            ral.select(args.bank, args.page)
        ral.write(args.byte, data)
        # A write with no read-back is not evidence that anything landed.
        back = ral.read(args.byte, len(data))
        print(f"wrote {data.hex(' ')}, read back {back.hex(' ')}, "
              f"{'match' if back == data else 'DIFFERENT'}")
        return 0 if back == data else 1


def cmd_rollover(args):
    """Establish which address wrap rule the module implements.

    CMIS Appendix B.1.2 wraps the current byte address within its own 128 byte
    segment. NXP AN15071 section 2.1.1 wraps at FFh across the whole 256 byte
    space. Reading 8 bytes from 00h:124 tells the two apart: bytes 4 to 7 are
    either 00h:0-3 or the start of Page 00h.
    """
    with open_adapter(args) as adapter:
        mci = open_mci(adapter, args)
        tail = mci.read(124, 8)[4:]
        head = mci.read(0, 4)
        page00 = mci.read(128, 4)
        print(f"8 bytes from 00h:124, last four : {tail.hex(' ')}")
        print(f"00h:0-3                         : {head.hex(' ')}")
        print(f"Page 00h:128-131                : {page00.hex(' ')}")
        if tail == head:
            print("\nsegment wrap, as CMIS B.1.2 specifies")
        elif tail == page00:
            print("\nran on into Upper Memory, as NXP AN15071 2.1.1 specifies")
        else:
            print("\ninconclusive: neither pattern matched")
        print("Either way, do not issue an access that crosses 7Fh. Split it.")
        return 0


def cmd_spi_probe(args):
    """Sweep N and the R/Wn polarity until the Page 00h checksum validates.

    A completed SPIMCI transaction proves nothing on its own. ACK is 00h, so an
    idle bus, an unpowered module or an unarmed target all return a byte perfect
    ACK followed by zero data. Only the checksum separates them.
    """
    with open_adapter(args) as adapter:
        polarities = [args.read_bit] if args.read_bit is not None else [0, 1]
        for read_bit in polarities:
            mci = SpiMci(adapter, n_flow=args.n or 2, read_bit=read_bit,
                         frequency_hz=args.frequency if args.frequency != 400_000 else 1_000_000)
            try:
                n = args.n if args.n else mci.probe_n()
                mci.n_flow = n
                computed, stored = Ral(mci).page00_checksum()
            except (CmisError, IndexError, ValueError):
                continue
            if computed == stored:
                print(f"N        : {n} flow control bytes, transaction is 4 + {n} + M")
                print(f"R/Wn     : {read_bit} means READ on this module")
                print(f"00h:222  : checksum {computed:#04x} matches the stored byte")
                print("\nPASS     : the module is present and self consistent.")
                return 0
        print("FAIL     : no combination of N and R/Wn produced a valid Page 00h checksum.\n"
              "           Either the module is absent, or CSn, CLK, IOTI or IITO are\n"
              "           miswired. On SPIMCI a silent bus still returns a perfect ACK.")
        return 1


def cmd_ibi(args):
    """Route the module interrupt onto the bus and count what arrives.

    CMIS requires a discrete Interrupt pin in the Management Signalling Layer,
    and it tells the host only that something happened. An I3C in-band interrupt
    carries a mandatory data byte, so the cause can travel with the interrupt.
    """
    with open_adapter(args) as adapter:
        mci = I3cMci(adapter, address=args.address)
        mci.assign_address()
        adapter.drain_ibis()
        adapter.i3c_enec_ibi(mci.address, True)
        print(f"ENEC sent to {mci.address:#04x}, listening for {args.seconds:g} s")
        deadline, seen = time.time() + args.seconds, []
        while time.time() < deadline:
            event = adapter.wait_ibi(timeout=min(0.5, args.seconds))
            if event is not None:
                seen.append(event)
        adapter.i3c_enec_ibi(mci.address, False)
        print(f"{len(seen)} in-band interrupt(s) in {args.seconds:g} s")
        for event in seen[:args.show]:
            print(f"  {event}")
        return 0


def cmd_timing(args):
    """Time the access phases a firmware update is built from.

    This writes no firmware. It exercises reads, a page change and reaching the
    CDB page, and prints each next to its CMIS Table 10-4 ceiling.
    """
    def timed(fn, repeats):
        samples = []
        for _ in range(repeats):
            start = time.perf_counter()
            fn()
            samples.append((time.perf_counter() - start) * 1000)
        return statistics.median(samples), max(samples)

    with open_adapter(args) as adapter:
        ral = Ral(open_mci(adapter, args))
        if not ral.test():
            print("FAIL   : the module is rejecting accesses")
            return 1

        rows = [("single byte read", lambda: ral.read(0, 1), LIMITS_MS["tREAD"]),
                ("8 byte read, the WRITE limit", lambda: ral.read(0, 8), None)]
        ral.nmax = ral.nmax_from_advertisement()
        if ral.nmax > WRITE_NMAX:
            rows.append((f"94 byte read, Nmax {ral.nmax}",
                         lambda: ral.read_page(0, 0, 128, 94), None))
        rows.append(("page change 00h to 01h to 00h",
                     lambda: [ral.select(0x00, p, verify=False) for p in (0x01, 0x00)],
                     LIMITS_MS["tBPC"] * 2))
        rows.append(("select CDB page 9Fh",
                     lambda: ral.select(0x00, CDB_PAGE, verify=False), LIMITS_MS["tBPC"]))

        print(f"{'phase':<34}{'median':>11}{'max':>11}{'CMIS max':>12}")
        measured = {}
        for name, fn, limit in rows:
            median, worst = timed(fn, args.repeats)
            measured[name] = median
            print(f"{name:<34}{median:>9.3f} ms{worst:>9.3f} ms"
                  f"{(f'{limit:.1f} ms' if limit else '-'):>12}")

        per_block = math.ceil(LPL_BLOCK / WRITE_NMAX)
        one = measured["8 byte read, the WRITE limit"]
        print(f"\nOne {LPL_BLOCK} byte LPL firmware block is {per_block} write accesses, "
              f"about {per_block * one:.1f} ms of bus time,")
        print(f"against a tWRITE hold-off of up to {LIMITS_MS['tWRITE']:.0f} ms "
              "that the module pays once per block.")
        return 0


# --------------------------------------------------------------------------
# Command line
#
# Session options are accepted both before and after the subcommand. The parent
# parser uses SUPPRESS defaults so that whichever parser saw an option last
# wins, and the real defaults are applied afterwards.
# --------------------------------------------------------------------------

SESSION_DEFAULTS = {
    "transport": "i2c", "adapter": None, "serial": None, "address": CMIS_ADDRESS,
    "frequency": 400_000, "verbose": False, "n": None, "read_bit": None, "nmax": None,
}


def add_session_options(parser):
    group = parser.add_argument_group("session")
    group.add_argument("--transport", choices=("i2c", "i3c", "spi"), default=argparse.SUPPRESS,
                       help="which MCI carries CMIS (default: i2c)")
    group.add_argument("--adapter", choices=("supernova", "pulsar"), default=argparse.SUPPRESS,
                       help="which adapter to open (default: whichever SDK is installed)")
    group.add_argument("--serial", default=argparse.SUPPRESS,
                       help="adapter serial number, for a bench with more than one")
    group.add_argument("--address", type=lambda s: int(s, 0), default=argparse.SUPPRESS,
                       help="module address (default: 0x50)")
    group.add_argument("--frequency", type=int, default=argparse.SUPPRESS,
                       help="bus clock in Hz (default: 400000 on I2C, 1000000 on SPI)")
    group.add_argument("--n", type=int, default=argparse.SUPPRESS,
                       help="SPIMCI flow control byte count; omit to probe")
    group.add_argument("--read-bit", type=int, choices=(0, 1), default=argparse.SUPPRESS,
                       help="SPIMCI R/Wn value meaning READ; omit to probe")
    group.add_argument("--nmax", type=int, default=argparse.SUPPRESS,
                       help="override the read access limit")
    group.add_argument("--verbose", action="store_true", default=argparse.SUPPRESS,
                       help="print every SDK response")


def main(argv=None):
    parent = argparse.ArgumentParser(add_help=False)
    add_session_options(parent)

    parser = argparse.ArgumentParser(
        prog="cmis_mci.py", parents=[parent],
        description=__doc__.splitlines()[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--version", action="version", version=f"cmis_mci.py {TOOL_VERSION}")
    subparsers = parser.add_subparsers(dest="command", required=True)

    for name, handler, help_text in (
            ("selfcheck", cmd_selfcheck, "check the address arithmetic offline"),
            ("scan", cmd_scan, "report what answers on the bus"),
            ("bringup", cmd_bringup, "the module discovery sequence, end to end"),
            ("identify", cmd_identify, "decode the identity and vendor block"),
            ("rollover", cmd_rollover, "establish which address wrap rule the module uses"),
            ("spi-probe", cmd_spi_probe, "establish SPIMCI N and R/Wn polarity"),
            ("timing", cmd_timing, "time the access phases against Table 10-4"),
    ):
        sub = subparsers.add_parser(name, parents=[parent], help=help_text)
        sub.set_defaults(handler=handler)

    sub = subparsers.add_parser("read", parents=[parent], help="read registers")
    sub.set_defaults(handler=cmd_read)
    sub.add_argument("--bank", type=parse_page, default=0)
    sub.add_argument("--page", type=parse_page, default=0)
    sub.add_argument("--byte", type=lambda s: int(s, 0), required=True)
    sub.add_argument("--count", type=int, default=1)

    sub = subparsers.add_parser("write", parents=[parent], help="write registers, with a read-back")
    sub.set_defaults(handler=cmd_write)
    sub.add_argument("--bank", type=parse_page, default=0)
    sub.add_argument("--page", type=parse_page, default=0)
    sub.add_argument("--byte", type=lambda s: int(s, 0), required=True)
    sub.add_argument("--data", required=True, help="hex bytes, for example 'A5' or '01 02'")

    sub = subparsers.add_parser("ibi", parents=[parent], help="count in-band interrupts (I3C)")
    sub.set_defaults(handler=cmd_ibi, transport="i3c")
    sub.add_argument("--seconds", type=float, default=10.0)
    sub.add_argument("--show", type=int, default=5)

    sub = subparsers.add_parser("budget", parents=[parent],
                                help="firmware update time budget, offline")
    sub.set_defaults(handler=cmd_budget)
    sub.add_argument("--image-kib", type=int, default=1024)
    sub.add_argument("--write-ms", type=float, default=LIMITS_MS["tWRITE"])

    args = parser.parse_args(argv)
    for key, value in SESSION_DEFAULTS.items():
        if not hasattr(args, key):
            setattr(args, key, value)
    if not hasattr(args, "repeats"):
        args.repeats = 20

    try:
        return args.handler(args)
    except CmisError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    sys.exit(main())
