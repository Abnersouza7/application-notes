#!/usr/bin/env python3
"""
stm32_flash.py -- Flash an STM32 over I2C or I3C using a Binho USB host adapter.

Talks to the system bootloader programmed into the ROM of STM32 microcontrollers,
over either of the two serial interfaces Binho adapters can drive:

    I3C   AN5927   Binho Supernova only
    I2C   AN4221   Binho Supernova or Binho Pulsar

    pip install binhosupernova        # for a Supernova
    pip install binhopulsar           # for a Pulsar

    python stm32_flash.py info
    python stm32_flash.py flash firmware.hex

The adapter and the bus are detected automatically when only one choice is
possible. Both can be forced with --adapter and --bus.

Wiring (adapter -> STM32 Nucleo):
    I2C SCL / SDA / GND  ->  target I2C bootloader pins
    I3C SCL / SDA / GND  ->  target I3C port          (Supernova only)
    GPIO 1               ->  NRST    (optional, for automatic bootloader entry)
    GPIO 2               ->  BOOT0   (optional, for automatic bootloader entry)

Without the GPIO wires, use --manual and the tool prompts for bootloader entry.

Why the two buses differ
------------------------
The command set is the same on both, but status is reported differently, and
that shapes the host code:

  I3C   The target raises an in-band interrupt carrying one payload byte. There
        is no readable status register, so every step is "send bytes, then wait
        for an interrupt".

  I2C   The target answers a read transaction with one status byte, which is the
        familiar request/response shape. Long flash operations either stretch
        the clock or, using the no-stretch command variants, answer polls with a
        busy byte until they finish.

Copyright (c) 2026 Binho Inc. Released under the MIT license.
"""

import argparse
import queue
import sys
import time

# --------------------------------------------------------------------------
# Bootloader protocol constants (common to both buses)
# --------------------------------------------------------------------------

ACK_BYTE = 0x79
NACK_BYTE = 0x1F
BUSY_BYTE = 0x76

I3C_SYNC_BYTE = 0x5A  # opens an I3C bootloader session; not used on I2C

# Over SPI the same value prefixes every command, not just the first, and the
# target emits SPI_BUSY_BYTE whenever it has nothing to say: it is the SPI
# peripheral's underrun pattern rather than a status the bootloader chooses.
SPI_SYNC_BYTE = 0x5A
SPI_BUSY_BYTE = 0xA5

CMD_GET = 0x00
CMD_GET_VERSION = 0x01
CMD_GET_ID = 0x02
CMD_READ_MEMORY = 0x11
CMD_GO = 0x21
CMD_WRITE_MEMORY = 0x31
CMD_EXT_ERASE = 0x44
CMD_WRITE_PROTECT = 0x63
CMD_WRITE_UNPROTECT = 0x73
CMD_READOUT_PROTECT = 0x82
CMD_READOUT_UNPROTECT = 0x92

# No-stretch variants. A flash erase or program takes far longer than a bus
# transaction, and the plain commands hold the target's clock low until they
# finish. A controller that cannot tolerate clock stretching uses these instead:
# the target answers reads with BUSY (0x76) while it works, and the host polls.
CMD_NS_WRITE_MEMORY = 0x32
CMD_NS_ERASE = 0x45

COMMAND_NAMES = {
    0x00: "Get", 0x01: "Get Version", 0x02: "Get ID", 0x03: "Speed",
    0x11: "Read Memory", 0x21: "Go", 0x31: "Write Memory",
    0x32: "No-Stretch Write Memory", 0x44: "Extended Erase",
    0x45: "No-Stretch Erase", 0x50: "Special Command",
    0x51: "Extended Special Command", 0x63: "Write Protect",
    0x64: "No-Stretch Write Protect", 0x73: "Write Unprotect",
    0x74: "No-Stretch Write Unprotect", 0x82: "Readout Protect",
    0x83: "No-Stretch Readout Protect", 0x92: "Readout Unprotect",
    0x93: "No-Stretch Readout Unprotect", 0xA1: "Checksum",
}

ERASE_MASS = 0xFFFF
FLASH_BASE = 0x08000000

# Device IDs of parts whose bootloader offers I3C, plus the STM32H503 used for
# the I2C measurements. Not exhaustive: check AN2606 for a given part.
DEVICE_IDS = {
    0x474: "STM32H503",
    0x478: "STM32H523/H533",
    0x484: "STM32H562/H563/H573",
    0x485: "STM32H7R/H7S",
    0x454: "STM32U3 (U385)",
}

STATUS_NAMES = {ACK_BYTE: "ACK", NACK_BYTE: "NACK", BUSY_BYTE: "BUSY"}

# The I2C bootloader answers on a fixed 7-bit address, given per part in AN2606.
# It is not the same on every device: the STM32H503 uses 0x67, and AN2606 states
# it as 0b1100111x, where x is the read/write bit. Override with --i2c-address.
DEFAULT_I2C_ADDRESS = 0x67


class BootloaderError(Exception):
    """A bootloader command failed, or the link misbehaved."""


# --------------------------------------------------------------------------
# Firmware image loading
# --------------------------------------------------------------------------

def _merge_segments(segments):
    """Sort by address and merge segments that are contiguous."""
    if not segments:
        return []
    segments.sort(key=lambda s: s[0])
    merged = [[segments[0][0], bytearray(segments[0][1])]]
    for addr, data in segments[1:]:
        last_addr, last_data = merged[-1]
        if addr == last_addr + len(last_data):
            last_data.extend(data)
        else:
            merged.append([addr, bytearray(data)])
    return [(a, bytes(d)) for a, d in merged]


def load_intel_hex(path):
    """Parse an Intel HEX file into a list of (address, bytes) segments.

    Implemented inline so the tool needs no package beyond an adapter SDK.
    Handles record types 00 (data), 01 (EOF), 02 (extended segment address),
    04 (extended linear address), and 03/05 (start address, ignored).
    """
    segments = []
    upper = 0

    with open(path, "r", encoding="ascii", errors="strict") as fh:
        for lineno, raw in enumerate(fh, 1):
            line = raw.strip()
            if not line:
                continue
            if not line.startswith(":"):
                raise ValueError(f"{path}:{lineno}: record does not start with ':'")
            try:
                raw_bytes = bytes.fromhex(line[1:])
            except ValueError as exc:
                raise ValueError(f"{path}:{lineno}: invalid hex data ({exc})") from None
            if len(raw_bytes) < 5:
                raise ValueError(f"{path}:{lineno}: record too short")

            count = raw_bytes[0]
            if len(raw_bytes) != count + 5:
                raise ValueError(
                    f"{path}:{lineno}: byte count {count} does not match record "
                    f"({len(raw_bytes) - 5} data bytes present)")

            offset = (raw_bytes[1] << 8) | raw_bytes[2]
            rectype = raw_bytes[3]
            data = raw_bytes[4:4 + count]
            checksum = raw_bytes[4 + count]

            if (sum(raw_bytes[:-1]) + checksum) & 0xFF:
                raise ValueError(f"{path}:{lineno}: checksum mismatch")

            if rectype == 0x00:
                segments.append((upper + offset, data))
            elif rectype == 0x01:
                break
            elif rectype == 0x02:
                if count != 2:
                    raise ValueError(f"{path}:{lineno}: bad extended segment address record")
                upper = ((data[0] << 8) | data[1]) << 4
            elif rectype == 0x04:
                if count != 2:
                    raise ValueError(f"{path}:{lineno}: bad extended linear address record")
                upper = ((data[0] << 8) | data[1]) << 16
            elif rectype in (0x03, 0x05):
                pass
            else:
                raise ValueError(f"{path}:{lineno}: unsupported record type 0x{rectype:02X}")

    if not segments:
        raise ValueError(f"{path}: no data records found")
    return _merge_segments(segments)


def load_binary(path, address):
    with open(path, "rb") as fh:
        data = fh.read()
    if not data:
        raise ValueError(f"{path}: file is empty")
    return [(address, data)]


def load_firmware(path, address):
    lowered = path.lower()
    if lowered.endswith((".hex", ".ihex", ".ihx")):
        return load_intel_hex(path)
    return load_binary(path, address)


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

    # -- lifecycle --------------------------------------------------------

    def open(self):
        result = self.device.open(serial=self.serial)
        if result.get("opcode") != 0:
            raise BootloaderError(f"could not open {self.kind}: {result.get('message')}")
        self._opened = True
        result = self.device.onEvent(self._on_event)
        if result.get("opcode") != 0:
            raise BootloaderError(f"could not register callback: {result.get('message')}")

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

    # -- plumbing ---------------------------------------------------------

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
            raise BootloaderError(f"{method.__name__} rejected: {submission.get('message')}")

        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise BootloaderError(f"{method.__name__}: timed out waiting for a response")
            try:
                response = self._responses.get(timeout=min(remaining, 0.25))
            except queue.Empty:
                continue
            if response.get("id") != request_id:
                continue  # stale response from an earlier, timed-out request
            if self.verbose:
                print(f"    <- {response}")
            result = response.get("result")
            if result not in (None, "SUCCESS") and result not in allowed_results:
                raise BootloaderError(f"{method.__name__} failed: {result}")
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

    # -- to be provided by each adapter -----------------------------------

    def gpio_output(self, pin, level):
        raise NotImplementedError

    def gpio_input(self, pin):
        raise NotImplementedError

    def gpio_write(self, pin, level):
        raise NotImplementedError

    # The bus rate actually programmed, which differs from the one asked for
    # where the hardware offers only a fixed set.
    i2c_actual_hz = None

    # Longest page list one Extended Erase may carry, where the controller
    # imposes a limit the bootloader does not. None means no limit.
    i2c_max_pages_per_erase = None

    # Seconds to let the target settle before reading a status byte. An I2C
    # controller rides out a target that holds SCL, so it needs nothing here;
    # see SupernovaAdapter for the case that does.
    status_settle_s = 0.0

    def i2c_init(self, frequency_hz, pullup_ohms):
        raise NotImplementedError

    def i2c_write(self, address, data):
        raise NotImplementedError

    def i2c_read(self, address, length):
        raise NotImplementedError

    def i2c_scan(self):
        raise NotImplementedError

    # -- SPI ---------------------------------------------------------------
    # Both adapter packages expose an identical SPI API, so the calls live
    # here and each adapter only supplies its own definitions module.

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
        # An interface that is already up ignores the settings passed to init,
        # so they have to be applied again through SetParameters. Skipping this
        # leaves the bus on whatever it was configured with last, which looks
        # like a wiring fault because every transfer is silently misdirected.
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
        raise BootloaderError(f"no payload in the SPI response: {response}")


class SupernovaAdapter(Adapter):
    """Binho Supernova. Drives both I3C and I2C."""

    kind = "supernova"
    buses = ("i3c", "i2c", "spi")

    def _spi_definitions(self):
        import binhosupernova.commands.spi.definitions as d
        return d

    def __init__(self, serial=None, verbose=False, i2c_bus=None, i2c_port="dedicated"):
        super().__init__(serial, verbose)
        self.i2c_port = i2c_port
        # An I3C controller is not obliged to tolerate a target holding SCL,
        # because I3C targets never stretch the clock, and this one aborts the
        # transfer with BUS_TIMEOUT rather than waiting. The bootloader
        # stretches for a moment after a command before it settles into
        # NACKing its address, so when the I3C port is driving the legacy bus,
        # read the status byte just late enough to catch the NACK instead,
        # which the busy-poll already retries safely.
        if i2c_port == "i3c":
            self.status_settle_s = 0.002
            # Driving legacy I2C from the I3C controller, an STM32H503 rejects
            # an erase carrying more than two pages and rejects a mass erase
            # outright. The same target accepts both from a Pulsar on its
            # dedicated I2C port, so the limits are the controller's.
            self.i2c_max_pages_per_erase = 2
        from binhosupernova.supernova import Supernova
        from binhosupernova.commands.gpio.definitions import (
            GpioPinNumber, GpioLogicLevel, GpioFunctionality)
        from binhosupernova.commands.i2c.definitions import I2cPullUpResistorsValue

        self.device = Supernova()
        self._GpioPinNumber = GpioPinNumber
        self._GpioLogicLevel = GpioLogicLevel
        self._GpioFunctionality = GpioFunctionality
        self._PullUps = I2cPullUpResistorsValue

    @staticmethod
    def detect():
        try:
            from binhosupernova import getConnectedSupernovaDevicesList
        except ImportError:
            return []
        return getConnectedSupernovaDevicesList()

    # -- GPIO -------------------------------------------------------------

    def _pin(self, n):
        try:
            return getattr(self._GpioPinNumber, f"GPIO_{n}")
        except AttributeError:
            raise BootloaderError(f"this adapter has no GPIO {n}") from None

    def gpio_output(self, pin, level):
        self.call(self.device.gpioConfigurePin, self._pin(pin),
                  self._GpioFunctionality.DIGITAL_OUTPUT,
                  self._GpioLogicLevel.HIGH if level else self._GpioLogicLevel.LOW)

    def gpio_input(self, pin):
        self.call(self.device.gpioConfigurePin, self._pin(pin),
                  self._GpioFunctionality.DIGITAL_INPUT)

    def gpio_write(self, pin, level):
        self.call(self.device.gpioDigitalWrite, self._pin(pin),
                  self._GpioLogicLevel.HIGH if level else self._GpioLogicLevel.LOW)

    # -- I2C --------------------------------------------------------------

    def i2c_init(self, frequency_hz, pullup_ohms):
        # The Supernova can carry I2C on either of two connectors. On the I3C
        # port it signals I2C through the I3C controller's I2C transfer mode,
        # so the same two wires serve both protocols.
        self.i2c_actual_hz = frequency_hz
        if self.i2c_port == "i3c":
            from binhosupernova.commands.i3c.definitions import I2cTransferRate
            # The I3C controller offers a fixed set of legacy rates, so the
            # requested clock snaps to the nearest of them. Record what was
            # actually programmed; reporting the request instead would state a
            # bus speed the hardware never ran at.
            rate = min(I2cTransferRate,
                       key=lambda m: abs(_i2c_rate_hz(m) - frequency_hz))
            self.i2c_actual_hz = int(_i2c_rate_hz(rate))
            self.i3c_init("PUSH_PULL_2_5_MHZ_25_DC", "OPEN_DRAIN_1_MHZ", 3300,
                          i2c_rate=rate)
            return
        self.call(self.device.setI2cSpiUartGpioVoltage, 3300)
        pull = _nearest_pullup(self._PullUps, pullup_ohms)
        # The adapter keeps its I2C peripheral initialised between runs, so a
        # second invocation is rejected. Applying the parameters to the running
        # interface is required, or the requested clock is silently ignored and
        # whatever the previous run selected stays in force.
        response = self.call(self.device.i2cControllerInit, frequency_hz, pull,
                             allowed_results=("INTERFACE_ALREADY_INITIALIZED",))
        if response.get("result") != "SUCCESS":
            self.call(self.device.i2cControllerSetParameters, frequency_hz, pull)

    # A controller reports START_STOP_ERROR when it cannot drive a START,
    # which means SDA or SCL never reached the idle high state. On the
    # dedicated I2C port that is almost always a bus with no pull-ups: not
    # every Supernova can switch its own on, and i2cSetPullUpResistors then
    # answers FEATURE_NOT_SUPPORTED_BY_HARDWARE. The raw result code sends
    # people looking for a protocol bug, so say what it actually means.
    def _explain_start_stop(self, exc):
        if "START_STOP_ERROR" not in str(exc) or self.i2c_port != "dedicated":
            return exc
        return BootloaderError(
            f"{exc}\n"
            "  The controller could not drive a START, so SDA or SCL never went\n"
            "  high. The dedicated I2C port needs bus pull-ups, and this unit\n"
            "  may not be able to supply its own. Either fit external pull-ups\n"
            "  (2.2k-4.7k to the bus voltage) on SDA and SCL, or drive the\n"
            "  target from the I3C port instead with --i2c-port i3c.")

    def i2c_write(self, address, data):
        if self.i2c_port == "i3c":
            from binhosupernova.commands.i3c.definitions import TransferMode
            self.call(self.device.i3cControllerWrite, address, TransferMode.I2C_MODE,
                      [], list(data), False)
            return
        try:
            self.call(self.device.i2cControllerWrite, address, [], list(data))
        except BootloaderError as exc:
            raise self._explain_start_stop(exc) from None

    def i2c_read(self, address, length):
        if self.i2c_port == "i3c":
            from binhosupernova.commands.i3c.definitions import TransferMode
            r = self.call(self.device.i3cControllerRead, address, TransferMode.I2C_MODE,
                          [], length, False)
            return bytes(r.get("payload") or b"")
        r = self.call(self.device.i2cControllerRead, address, length, [])
        return bytes(r.get("payload") or b"")

    def i2c_scan(self):
        r = self.call(self.device.i2cControllerScanBus, timeout=10.0)
        return r.get("addresses") or r.get("payload") or []


class PulsarAdapter(Adapter):
    """Binho Pulsar. Drives I2C, SPI and UART. It has no I3C interface."""

    kind = "pulsar"
    buses = ("i2c", "spi")

    def __init__(self, serial=None, verbose=False, i2c_bus="A"):
        super().__init__(serial, verbose)
        from binhopulsar.pulsar import Pulsar
        from binhopulsar.commands.gpio.definitions import (
            GpioPinNumber, GpioLogicLevel, GpioFunctionality)
        from binhopulsar.commands.i2c.definitions import I2cBus, I2cPullUpResistorsValue

        self.device = Pulsar()
        self._GpioPinNumber = GpioPinNumber
        self._GpioLogicLevel = GpioLogicLevel
        self._GpioFunctionality = GpioFunctionality
        self._PullUps = I2cPullUpResistorsValue
        try:
            self._bus = getattr(I2cBus, f"I2C_BUS_{i2c_bus.upper()}")
        except AttributeError:
            raise BootloaderError(f"unknown Pulsar I2C bus '{i2c_bus}'; use A or B") from None

    def _spi_definitions(self):
        import binhopulsar.commands.spi.definitions as d
        return d

    @staticmethod
    def detect():
        try:
            from binhopulsar import getConnectedPulsarDevicesList
        except ImportError:
            return []
        return getConnectedPulsarDevicesList()

    def _pin(self, n):
        try:
            return getattr(self._GpioPinNumber, f"GPIO_{n}")
        except AttributeError:
            raise BootloaderError(f"this adapter has no GPIO {n}") from None

    def gpio_output(self, pin, level):
        self.call(self.device.gpioConfigurePin, self._pin(pin),
                  self._GpioFunctionality.DIGITAL_OUTPUT,
                  self._GpioLogicLevel.HIGH if level else self._GpioLogicLevel.LOW)

    def gpio_input(self, pin):
        self.call(self.device.gpioConfigurePin, self._pin(pin),
                  self._GpioFunctionality.DIGITAL_INPUT)

    def gpio_write(self, pin, level):
        self.call(self.device.gpioDigitalWrite, self._pin(pin),
                  self._GpioLogicLevel.HIGH if level else self._GpioLogicLevel.LOW)

    # The Pulsar I2C calls take a bus selector; the Supernova ones do not.
    def i2c_init(self, frequency_hz, pullup_ohms):
        self.call(self.device.setI2cSpiUartGpioVoltage, 3300)
        pull = _nearest_pullup(self._PullUps, pullup_ohms)
        response = self.call(self.device.i2cControllerInit, self._bus, frequency_hz, pull,
                             allowed_results=("INTERFACE_ALREADY_INITIALIZED",))
        if response.get("result") != "SUCCESS":
            self.call(self.device.i2cControllerSetParameters, self._bus, frequency_hz, pull)

    def i2c_write(self, address, data):
        self.call(self.device.i2cControllerWrite, self._bus, address, [], list(data))

    def i2c_read(self, address, length):
        r = self.call(self.device.i2cControllerRead, self._bus, address, length, [])
        return bytes(r.get("payload") or b"")

    def i2c_scan(self):
        r = self.call(self.device.i2cControllerScanBus, self._bus, timeout=10.0)
        return r.get("addresses") or r.get("payload") or []


def _i2c_rate_hz(member):
    """Turn an I2cTransferRate member name such as _400KHz into a number."""
    text = member.name.lstrip("_").replace("_", ".").lower()
    if text.endswith("khz"):
        return float(text[:-3]) * 1_000
    if text.endswith("mhz"):
        return float(text[:-3]) * 1_000_000
    return 0.0


def _nearest_pullup(enum_cls, ohms):
    """Pick the enum member closest to the requested pull-up value."""
    best, best_delta = None, None
    for member in enum_cls:
        if "DISABLE" in member.name:
            continue
        text = member.name.replace("I2C_PULLUP_", "").replace("Ohm", "")
        text = text.replace("_", ".")
        try:
            value = float(text[:-1]) * 1000 if text.lower().endswith("k") else float(text)
        except ValueError:
            continue
        delta = abs(value - ohms)
        if best_delta is None or delta < best_delta:
            best, best_delta = member, delta
    if best is None:
        raise BootloaderError("no usable pull-up setting found")
    return best


ADAPTERS = {"supernova": SupernovaAdapter, "pulsar": PulsarAdapter}


def detect_adapters():
    """Return [(kind, device_dict), ...] for every attached adapter."""
    found = []
    for kind, cls in ADAPTERS.items():
        for dev in cls.detect():
            found.append((kind, dev))
    return found


# --------------------------------------------------------------------------
# I3C support on the Supernova
#
# Kept apart from the I2C methods above because only the Supernova has an I3C
# interface. A Pulsar rejects --bus i3c before it gets this far.
# --------------------------------------------------------------------------

def _supernova_i3c_methods(cls):
    """Attach the I3C methods to SupernovaAdapter without cluttering its body."""

    def i3c_init(self, push_pull, open_drain, voltage_mv, i2c_rate=None):
        from binhosupernova.commands.i3c.definitions import (
            I3cPushPullTransferRate, I3cOpenDrainTransferRate, I2cTransferRate)
        rates = (
            getattr(I3cPushPullTransferRate, push_pull),
            getattr(I3cOpenDrainTransferRate, open_drain),
            i2c_rate or I2cTransferRate._100KHz,
        )
        self.call(self.device.setI3cVoltage, voltage_mv)
        response = self.call(self.device.i3cControllerInit, *rates,
                             allowed_results=("INTERFACE_ALREADY_INITIALIZED",))
        if response.get("result") != "SUCCESS":
            self.call(self.device.i3cControllerSetParameters, *rates)

    def i3c_discover(self):
        """Run ENTDAA and return the target device table."""
        self.call(self.device.i3cControllerInitBus, timeout=10.0)
        table = self.call(self.device.i3cControllerGetTargetDevicesTable, timeout=10.0)
        return table.get("table") or []

    def i3c_accept_ibis(self, address):
        from binhosupernova.commands.i3c.definitions import (
            TargetType, TargetInterruptRequest, ControllerRoleRequest,
            SetdasaConfiguration, SetaasaConfiguration, EntdaaConfiguration,
            IBiTimestamp, PendingReadCapability)
        self.call(self.device.i3cControllerSetTargetDeviceConfiguration, address, {
            "targetType": TargetType.I3C_DEVICE,
            "IBIRequest": TargetInterruptRequest.ACCEPT_IBI,
            "CRRequest": ControllerRoleRequest.REJECT_CRR,
            "daaUseSETDASA": SetdasaConfiguration.DO_NOT_USE_SETDASA,
            "daaUseSETAASA": SetaasaConfiguration.DO_NOT_USE_SETAASA,
            "daaUseENTDAA": EntdaaConfiguration.USE_ENTDAA,
            "ibiTimestampEnable": IBiTimestamp.DISABLE_IBIT,
            "pendingReadCapability": PendingReadCapability.DISABLE_AUTOMATIC_READ,
        })

    def i3c_write(self, address, data):
        from binhosupernova.commands.i3c.definitions import TransferMode
        self.call(self.device.i3cControllerWrite, address, TransferMode.I3C_SDR,
                  [], list(data), False)

    def i3c_read(self, address, length):
        from binhosupernova.commands.i3c.definitions import TransferMode
        r = self.call(self.device.i3cControllerRead, address, TransferMode.I3C_SDR,
                      [], length, False)
        return bytes(r.get("payload") or b"")

    for fn in (i3c_init, i3c_discover, i3c_accept_ibis, i3c_write, i3c_read):
        setattr(cls, fn.__name__, fn)
    return cls


SupernovaAdapter = _supernova_i3c_methods(SupernovaAdapter)


# --------------------------------------------------------------------------
# Target reset control
# --------------------------------------------------------------------------

class ResetController:
    """Drives NRST and BOOT0 to select what the target boots into.

    NRST is only ever driven low or released to high impedance, never driven
    high, so the adapter does not fight the reset circuitry on the target board.
    """

    # Whether this controller can also select the application. The GPIO
    # version can, because it drives BOOT0 both ways.
    can_run_application = True

    def __init__(self, adapter, nrst_pin=1, boot0_pin=2,
                 reset_hold=0.05, boot_delay=0.25):
        self.adapter = adapter
        self.nrst = nrst_pin
        self.boot0 = boot0_pin
        self.reset_hold = reset_hold
        self.boot_delay = boot_delay

    def _pulse_reset(self):
        self.adapter.gpio_output(self.nrst, 0)
        self.adapter.gpio_write(self.nrst, 0)
        time.sleep(self.reset_hold)
        self.adapter.gpio_input(self.nrst)

    def reset_into_bootloader(self):
        self.adapter.gpio_output(self.boot0, 1)
        self.adapter.gpio_write(self.boot0, 1)
        self._pulse_reset()
        time.sleep(self.boot_delay)

    def reset_into_application(self):
        self.adapter.gpio_output(self.boot0, 0)
        self.adapter.gpio_write(self.boot0, 0)
        self._pulse_reset()

    def release(self):
        try:
            self.adapter.gpio_input(self.nrst)
            self.adapter.gpio_input(self.boot0)
        except BootloaderError:
            pass


class SpiResetController:
    """Reset control with NRST and BOOT0 on chip selects rather than GPIO.

    No call in either adapter package sets a chip select to a level, so this
    borrows the one thing chip selects do offer: an active-low select idles
    high and is driven low for exactly as long as a transfer lasts. Selecting
    the NRST line and clocking a slow transfer is therefore a reset pulse of a
    known length, and the BOOT0 line sits high while it is not the selected
    one, which is what puts the part into the bootloader.

    The same trick cannot select the application, because that needs BOOT0
    held low while NRST is pulsed and only one line can be selected at a time.
    The bootloader's Go command covers that case instead.
    """

    can_run_application = False

    def __init__(self, adapter, nrst_cs=1, boot0_cs=2, bus_cs=0,
                 reset_hold=0.16, boot_delay=0.3, speed_hz=10000):
        self.adapter = adapter
        self.nrst_cs = nrst_cs
        self.boot0_cs = boot0_cs
        self.bus_cs = bus_cs
        self.boot_delay = boot_delay
        self.speed_hz = speed_hz
        # 8 clocks per byte, so the pulse width follows from the byte count.
        self.pulse_bytes = max(1, int(reset_hold * speed_hz / 8))

    def reset_into_bootloader(self):
        self.adapter.spi_init(self.speed_hz, chip_select=self.nrst_cs)
        self.adapter.spi_transfer([0x00] * self.pulse_bytes)
        time.sleep(self.boot_delay)

    def reset_into_application(self):
        raise BootloaderError(
            "BOOT0 is on a chip select, which cannot be held low while NRST "
            "is pulsed; the Go command starts the application instead")

    def release(self):
        pass


# --------------------------------------------------------------------------
# Bootloader command layer
#
# The command opcodes and the address encoding are common to both buses. How a
# status byte is collected, and how memory transfers are framed, are not, so
# those live in the per-bus subclasses.
# --------------------------------------------------------------------------

class Bootloader:
    """Common part of the STM32 system bootloader protocol."""

    bus = "?"

    # Only the I2C bootloader offers the no-stretch command variants, but the
    # erase paths are shared, so the base has to answer for the buses that do
    # not. I3C reports status out of band and never stretches, so it is always
    # the plain commands there.
    no_stretch = False

    def __init__(self, adapter, verbose=False):
        self.adapter = adapter
        self.verbose = verbose
        self.max_chunk = 256

    # -- to be provided per bus ------------------------------------------

    def connect(self):
        raise NotImplementedError

    def _write(self, data):
        raise NotImplementedError

    def _read(self, length):
        raise NotImplementedError

    def _status(self, what, timeout=3.0):
        raise NotImplementedError

    # -- shared ------------------------------------------------------------

    @staticmethod
    def _xor(data):
        checksum = 0
        for byte in data:
            checksum ^= byte
        return checksum

    def _expect_ack(self, what, timeout=3.0):
        status = self._status(what, timeout=timeout)
        # A no-stretch command reports progress with BUSY bytes; keep reading
        # until the target settles on ACK or NACK.
        deadline = time.monotonic() + timeout
        while status == BUSY_BYTE and time.monotonic() < deadline:
            status = self._status(what, timeout=timeout)
        if status != ACK_BYTE:
            name = STATUS_NAMES.get(status, f"0x{status:02X}")
            raise BootloaderError(f"{what}: bootloader replied {name}")
        return status

    def _command(self, opcode, timeout=3.0):
        """Every command is the opcode followed by its bitwise complement."""
        self._write([opcode, opcode ^ 0xFF])
        self._expect_ack(f"command {COMMAND_NAMES.get(opcode, hex(opcode))}", timeout=timeout)

    def _send_address(self, address, what="address"):
        payload = [(address >> 24) & 0xFF, (address >> 16) & 0xFF,
                   (address >> 8) & 0xFF, address & 0xFF]
        payload.append(self._xor(payload))
        self._write(payload)
        self._expect_ack(what)

    def erase_mass(self, timeout=30.0):
        self._command(CMD_NS_ERASE if self.no_stretch else CMD_EXT_ERASE)
        payload = [(ERASE_MASS >> 8) & 0xFF, ERASE_MASS & 0xFF]
        payload.append(self._xor(payload))
        self._write(payload)
        self._expect_ack("mass erase", timeout=timeout)

    def go(self, address):
        self._command(CMD_GO)
        self._send_address(address, "go address")

    # How many pages one Extended Erase command may carry. Unlimited as far as
    # anything here is concerned unless a bus says otherwise.
    max_pages_per_erase = None

    def erase_pages(self, pages, timeout=30.0):
        """Erase an explicit list of pages.

        Many short operations instead of one long one. That matters on a
        controller that cannot tolerate the target going unresponsive for the
        whole duration of a mass erase.

        The page count is encoded differently on the two buses: I2C sends
        count - 1, I3C sends the count itself.
        """
        if not pages:
            return
        limit = self.max_pages_per_erase
        if limit and len(pages) > limit:
            for start in range(0, len(pages), limit):
                self._erase_page_group(pages[start:start + limit], timeout)
            return
        self._erase_page_group(pages, timeout)

    def _erase_page_group(self, pages, timeout=30.0):
        self._command(CMD_EXT_ERASE)
        count = len(pages) - 1 if self.bus == "i2c" else len(pages)
        head = [(count >> 8) & 0xFF, count & 0xFF]
        head.append(self._xor(head))
        self._write(head)
        self._expect_ack("erase page count")

        body = []
        for page in pages:
            body += [(page >> 8) & 0xFF, page & 0xFF]
        body.append(self._xor(body))
        self._write(body)
        self._expect_ack("erase pages", timeout=timeout)


ERASE_BANK1 = 0xFFFE
ERASE_BANK2 = 0xFFFD


class I2cBootloader(Bootloader):
    """STM32 system bootloader over I2C (AN4221).

    Status is a byte the host reads back, so every step is a write followed by
    a one-byte read. There is no synchronisation byte: the session begins with
    the first command.
    """

    bus = "i2c"

    def __init__(self, adapter, address=DEFAULT_I2C_ADDRESS, verbose=False,
                 no_stretch=False):
        super().__init__(adapter, verbose)
        # How many pages one erase command may carry is a property of the
        # controller, not of the bootloader. A Pulsar on its dedicated I2C port
        # accepts long lists and a mass erase; the Supernova driving I2C from
        # its I3C port does not, and says so with a NACK.
        self.max_pages_per_erase = getattr(adapter, "i2c_max_pages_per_erase", None)
        self.address = address
        self.no_stretch = no_stretch

    def connect(self):
        # Nothing to open. Confirm the target answers before going further.
        self.get_version()

    def erase_mass(self, timeout=30.0):
        """Erase everything.

        An STM32H503 driven from a Supernova's I3C port rejects the mass erase
        value while accepting the bank one value, which on a single-bank part
        covers the same ground. The same target accepts a mass erase from a
        Pulsar on its dedicated I2C port and over I3C, so the refusal belongs
        to that controller path rather than to the device or the bus.

        Mass erase is tried first so a part that supports it is unaffected. On
        a multi-bank device the fallback clears only the first bank, so it says
        what it did rather than reporting a mass erase it did not perform.
        """
        try:
            super().erase_mass(timeout=timeout)
            return
        except BootloaderError as exc:
            if "NACK" not in str(exc):
                raise
        self._command(CMD_NS_ERASE if self.no_stretch else CMD_EXT_ERASE)
        payload = [(ERASE_BANK1 >> 8) & 0xFF, ERASE_BANK1 & 0xFF]
        payload.append(self._xor(payload))
        self._write(payload)
        self._expect_ack("bank 1 erase", timeout=timeout)
        # Always say so. The caller asked to erase everything and got something
        # narrower, which on a multi-bank part is not the same thing.
        print("    mass erase refused by the target; erased bank 1 instead")

    # While the target is erasing or programming it stops acknowledging its
    # address altogether, so a transaction is refused rather than stretched. A
    # NACKed address means nothing was delivered, so retrying is safe.
    def _retry(self, action, what, timeout, aborts_are_busy=False):
        retryable = ["NACK_ADDRESS"]
        if aborts_are_busy:
            retryable.append("BUS_TIMEOUT")
        deadline = time.monotonic() + timeout
        while True:
            try:
                return action()
            except BootloaderError as exc:
                if (not any(r in str(exc) for r in retryable)
                        or time.monotonic() > deadline):
                    raise
                time.sleep(0.002)

    def _write(self, data, timeout=3.0):
        if self.verbose:
            print(f"    -> [{len(data)}] " + " ".join(f"{b:02X}" for b in list(data)[:16]))
        self._retry(lambda: self.adapter.i2c_write(self.address, list(data)),
                    "write", timeout)

    def _read(self, length, timeout=3.0):
        data = self._retry(lambda: self.adapter.i2c_read(self.address, length),
                           "read", timeout)
        if len(data) != length:
            raise BootloaderError(f"short read: wanted {length} bytes, got {len(data)}")
        if self.verbose:
            print(f"    <- [{len(data)}] " + " ".join(f"{b:02X}" for b in data[:16]))
        return data

    def _status(self, what, timeout=3.0):
        # A target that is mid-erase may accept the address but return nothing
        # yet. Treat an empty reply the same as a busy one and keep asking.
        deadline = time.monotonic() + timeout
        while True:
            if self.adapter.status_settle_s:
                time.sleep(self.adapter.status_settle_s)
            data = self._retry(lambda: self.adapter.i2c_read(self.address, 1),
                               what, timeout, aborts_are_busy=True)
            if data:
                return data[0]
            if time.monotonic() > deadline:
                raise BootloaderError(f"{what}: no status byte from the bootloader")
            time.sleep(0.002)

    def get(self):
        """Get, in two passes.

        The reply is one frame of 2 + N bytes and the host cannot know N in
        advance. Reading past the end of the frame times the bus out, so the
        first pass reads two bytes to learn N, and the second reads the frame.
        Reading short is harmless: the target abandons the rest of the frame
        and moves on to its closing ACK.
        """
        self._command(CMD_GET)
        head = self._read(2)
        count = head[0]
        self._expect_ack("Get trailing")

        self._command(CMD_GET)
        frame = self._read(2 + count)
        self._expect_ack("Get trailing")
        return frame[1], list(frame[2:])

    def get_version(self):
        self._command(CMD_GET_VERSION)
        version = self._read(1)[0]
        self._expect_ack("Get Version trailing")
        return version

    def get_id(self):
        self._command(CMD_GET_ID)
        frame = self._read(3)  # count byte, then two identifier bytes
        self._expect_ack("Get ID trailing")
        return (frame[1] << 8) | frame[2]

    def read_memory(self, address, length, progress=None):
        out = bytearray()
        while len(out) < length:
            want = min(self.max_chunk, length - len(out))
            self._command(CMD_READ_MEMORY)
            self._send_address(address + len(out), "read address")
            count = want - 1
            self._write([count, count ^ 0xFF])
            self._expect_ack("read length")
            out.extend(self._read(want))
            if progress:
                progress(len(out), length)
        return bytes(out)

    def write_memory(self, address, data, progress=None):
        if not data:
            return
        offset, total = 0, len(data)
        while offset < total:
            chunk = data[offset:offset + self.max_chunk]
            self._command(CMD_NS_WRITE_MEMORY if self.no_stretch else CMD_WRITE_MEMORY)
            self._send_address(address + offset, "write address")
            count = len(chunk) - 1
            frame = [count] + list(chunk)
            frame.append(self._xor(frame))
            self._write(frame)
            self._expect_ack("write block", timeout=5.0)
            offset += len(chunk)
            if progress:
                progress(offset, total)


class I3cBootloader(Bootloader):
    """STM32 system bootloader over I3C (AN5927).

    Status arrives as an in-band interrupt carrying one payload byte, so every
    step is a write followed by waiting for an interrupt. Memory transfers use
    the streaming loop: the address is sent once and the target advances it.
    """

    bus = "i3c"

    def __init__(self, adapter, address, verbose=False):
        super().__init__(adapter, verbose)
        self.address = address

    def connect(self):
        self.adapter.drain_ibis()
        self._write([I3C_SYNC_BYTE])
        self._expect_ack("sync")

    def _write(self, data):
        if self.verbose:
            print(f"    -> [{len(data)}] " + " ".join(f"{b:02X}" for b in list(data)[:16]))
        self.adapter.i3c_write(self.address, list(data))

    def _read(self, length):
        data = self.adapter.i3c_read(self.address, length)
        if len(data) != length:
            raise BootloaderError(f"short read: wanted {length} bytes, got {len(data)}")
        return data

    def _status(self, what, timeout=3.0):
        ibi = self.adapter.wait_ibi(timeout=timeout)
        if ibi is None:
            raise BootloaderError(f"{what}: no status IBI from the bootloader")
        payload = ibi.get("payload") or []
        if not payload:
            raise BootloaderError(f"{what}: status IBI carried no payload")
        return payload[0]

    def _command(self, opcode, timeout=3.0):
        self.adapter.drain_ibis()
        super()._command(opcode, timeout=timeout)

    def get(self):
        self._command(CMD_GET)
        count = self._read(1)[0]
        version = self._read(1)[0]
        commands = list(self._read(count))
        self._expect_ack("Get trailing")
        return version, commands

    def get_version(self):
        self._command(CMD_GET_VERSION)
        version = self._read(1)[0]
        self._expect_ack("Get Version trailing")
        return version

    def get_id(self):
        self._command(CMD_GET_ID)
        length = self._read(1)[0]
        data = self._read(length)
        self._expect_ack("Get ID trailing")
        return int.from_bytes(data, "big")

    def read_memory(self, address, length, progress=None):
        if length <= 0:
            return b""
        out = bytearray()
        self._command(CMD_READ_MEMORY)
        self._send_address(address, "read address")
        while len(out) < length:
            want = min(self.max_chunk, length - len(out))
            more = 1 if (len(out) + want) < length else 0
            size_field = (want << 1) | more
            header = [(size_field >> 8) & 0xFF, size_field & 0xFF]
            header.append(self._xor(header))
            self._write(header)
            self._expect_ack("read block header")
            out.extend(self._read(want))
            if progress:
                progress(len(out), length)
        return bytes(out)

    def write_memory(self, address, data, progress=None):
        if not data:
            return
        self._command(CMD_WRITE_MEMORY)
        self._send_address(address, "write address")
        offset, total = 0, len(data)
        while offset < total:
            chunk = data[offset:offset + self.max_chunk]
            offset += len(chunk)
            more = 1 if offset < total else 0
            size_field = (len(chunk) << 1) | more
            header = [(size_field >> 8) & 0xFF, size_field & 0xFF]
            header.append(self._xor(header))
            self._write(header)
            self._expect_ack("write block header")
            body = bytes(chunk) + bytes([self._xor(chunk)])
            self._write(body)
            self._expect_ack("write block data", timeout=5.0)
            if progress:
                progress(offset, total)


# --------------------------------------------------------------------------
# Session setup
# --------------------------------------------------------------------------

class _SpiOvershoot(Exception):
    """The adaptive lead ran past an acknowledgement; redo the step."""


class SpiBootloader(Bootloader):
    """STM32 system bootloader over SPI (AN4286).

    Three things separate this from the other two buses:

    The synchronisation byte prefixes every command, not only the first. The
    target discards anything that is not 0x5A while it waits for one, so
    clocking dummy bytes between commands is harmless.

    Status is read by clocking dummy bytes until the ACK appears, and the host
    must then send the ACK back. The target blocks until it arrives, so a host
    that skips this hangs the session with no error reported.

    The bus is full duplex, so the transfer that sends that acknowledgement
    also clocks a byte in, and that byte is already part of the reply. It has
    to be kept or the reply silently loses its head.
    """

    bus = "spi"

    def __init__(self, adapter, verbose=False):
        super().__init__(adapter, verbose)
        self._rx = []
        self._reply_started = False
        self._ack_lead = 0

    def connect(self):
        self._xfer([SPI_SYNC_BYTE])
        self._wait_ack("session start")

    def _xfer(self, payload):
        if self.verbose:
            print(f"    -> [{len(payload)}] "
                  + " ".join(f"{b:02X}" for b in list(payload)[:16]))
        got = self.adapter.spi_transfer(payload)
        if self.verbose:
            print(f"    <- [{len(got)}] " + " ".join(f"{b:02X}" for b in got[:16]))
        return got

    # Time to let the target load its first reply byte once it has
    # acknowledged. Measured at roughly 38 byte periods at 1 MHz, so a
    # millisecond is generous, and overshooting costs nothing because a target
    # that is already waiting simply holds the byte until it is clocked.
    reply_settle_s = 0.002

    def _wait_ack(self, what, reply_follows=True, limit=4096, timeout=5.0):
        """Clock until the target acknowledges, then release it.

        The target answers roughly thirty to forty byte periods after a step,
        and that latency is counted in clocks rather than in wall time. Spent
        one byte per USB round trip it costs milliseconds; spent as one burst
        it costs microseconds, so how the clocks are delivered decides the
        throughput of the whole bus.

        Bursting is safe wherever nothing the host needs follows the
        acknowledgement, because the target sends only busy padding until it
        acknowledges, so anything clocked before the ACK can be discarded. It
        is not safe where a reply follows: the reply begins straight after the
        ACK with no delimiter, and the busy value is an ordinary data byte, so
        an overshoot eats reply bytes that cannot be told apart or recovered.

        Those two cases are what `reply_follows` selects between.
        """
        deadline = time.monotonic() + timeout

        if not reply_follows:
            while time.monotonic() < deadline:
                got = self._xfer([0x00] * 32)
                if ACK_BYTE in got:
                    self._xfer([ACK_BYTE])
                    self._reply_started = False
                    return
                if NACK_BYTE in got:
                    raise BootloaderError(f"{what}: bootloader replied NACK")
            raise BootloaderError(f"{what}: no acknowledgement from the bootloader")

        # A reply follows, so approach the ACK without stepping past it. The
        # lead is the part of the wait already known to be padding from the
        # last time round, kept a margin short of where the ACK actually fell.
        lead = self._ack_lead
        if lead:
            got = self._xfer([0x00] * lead)
            if ACK_BYTE in got or NACK_BYTE in got:
                self._ack_lead = 0          # too far; creep up again next time
                raise _SpiOvershoot(what)

        polls = 0
        while polls < limit:
            byte = self._xfer([0x00])[0]
            polls += 1
            if byte == ACK_BYTE:
                self._xfer([ACK_BYTE])      # the target blocks until it sees this
                self._reply_started = False
                self._ack_lead = max(0, lead + polls - 8)
                return
            if byte == NACK_BYTE:
                raise BootloaderError(f"{what}: bootloader replied NACK")
            if time.monotonic() > deadline:
                break
        raise BootloaderError(f"{what}: no acknowledgement from the bootloader")

    def _take(self, count, timeout=5.0):
        """Return exactly `count` reply bytes, positionally.

        Nothing is skipped or filtered. The target cannot send until it is
        clocked, so pausing after the acknowledgement leaves the first reply
        byte sitting in its transmit register, and the next clocked byte is
        that byte whatever its value. Searching for the start of the data
        instead would corrupt any reply whose first byte happens to be the
        busy value, which for a flash read is an ordinary byte.
        """
        time.sleep(self.reply_settle_s)
        # AN4286's leading dummy byte, once per reply rather than once per
        # read. The target's shift register still holds the busy pattern when
        # the first reply byte is queued behind it, so that byte clocks out
        # first. Once the reply is flowing the rest is contiguous, and asking
        # for another dummy mid-reply would drop a real byte.
        lead = 0 if self._reply_started else 1
        got = self._xfer([0x00] * (count + lead))
        self._reply_started = True
        return bytes(got[lead:])

    def _command(self, opcode, timeout=3.0, reply_follows=True):
        self._rx = []
        self._xfer([SPI_SYNC_BYTE, opcode, opcode ^ 0xFF])
        self._wait_ack(f"command 0x{opcode:02X}", reply_follows=reply_follows,
                       timeout=timeout)

    def _write(self, data, timeout=3.0):
        self._xfer(list(data))

    def _expect_ack(self, what, timeout=3.0):
        # The shared erase and go paths reach the bus through here, and none
        # of their acknowledgements is followed by a reply the host reads.
        self._wait_ack(what, reply_follows=False, timeout=timeout)

    def _send_address(self, address, what="address", reply_follows=False):
        head = [(address >> 24) & 0xFF, (address >> 16) & 0xFF,
                (address >> 8) & 0xFF, address & 0xFF]
        head.append(self._xor(head))
        self._xfer(head)
        self._wait_ack(what, reply_follows=reply_follows)

    def get(self):
        # Two settled reads rather than one long one. Pausing between them is
        # safe: the target queues its next byte and waits to be clocked, so it
        # never pads mid-reply. Over-reading is what does damage, because it
        # eats the trailing acknowledgement.
        self._command(CMD_GET)
        count, version = self._take(2)
        commands = list(self._take(count))
        self._wait_ack("Get trailing", reply_follows=False)
        return version, commands

    def get_version(self):
        self._command(CMD_GET_VERSION)
        version = self._take(1)[0]
        self._wait_ack("Get Version trailing", reply_follows=False)
        return version

    def get_id(self):
        self._command(CMD_GET_ID)
        reply = self._take(3)
        self._wait_ack("Get ID trailing", reply_follows=False)
        return (reply[1] << 8) | reply[2]

    def read_memory(self, address, length, progress=None):
        out = bytearray()
        while len(out) < length:
            chunk = min(self.max_chunk, length - len(out))
            for attempt in range(4):
                try:
                    self._command(CMD_READ_MEMORY, reply_follows=False)
                    self._send_address(address + len(out))
                    self._xfer([chunk - 1, (chunk - 1) ^ 0xFF])
                    self._wait_ack("read length", reply_follows=True)
                    break
                except _SpiOvershoot:
                    continue        # the lead has reset itself; try again
            else:
                raise BootloaderError("read: could not line up on the reply")
            out += self._take(chunk, timeout=10.0)
            if progress:
                progress(len(out), length)
        return bytes(out)

    def write_memory(self, address, data, progress=None):
        written = 0
        while written < len(data):
            chunk = data[written:written + self.max_chunk]
            self._command(CMD_WRITE_MEMORY, reply_follows=False)
            self._send_address(address + written)
            body = [len(chunk) - 1] + list(chunk)
            body.append(self._xor(body))
            self._xfer(body)
            self._wait_ack("write block", reply_follows=False, timeout=10.0)
            written += len(chunk)
            if progress:
                progress(written, len(data))


def choose_adapter(args, log):
    """Pick the adapter to use, or explain why the choice is ambiguous."""
    found = detect_adapters()
    if args.adapter:
        found = [(k, d) for k, d in found if k == args.adapter]
        if not found:
            raise BootloaderError(f"no {args.adapter} is connected")
    if not found:
        raise BootloaderError(
            "no Binho adapter found.\n"
            "  Install the SDK for the adapter in use:\n"
            "    pip install binhosupernova     (Supernova)\n"
            "    pip install binhopulsar        (Pulsar)")
    if args.serial:
        found = [(k, d) for k, d in found if d.get("serial_number") == args.serial]
        if not found:
            raise BootloaderError(f"no adapter with serial {args.serial}")
    if len(found) > 1:
        kinds = ", ".join(f"{k} ({d.get('serial_number')})" for k, d in found)
        raise BootloaderError(
            f"several adapters are connected: {kinds}.\n"
            "  Choose one with --adapter or --serial.")
    kind, dev = found[0]
    log(f"  Adapter: {kind}  serial {dev.get('serial_number')}  fw {dev.get('firmware_version')}")
    return kind, dev


def choose_bus(args, kind, log):
    cls = ADAPTERS[kind]
    if args.bus:
        if args.bus not in cls.buses:
            raise BootloaderError(
                f"the {kind} has no {args.bus.upper()} interface; "
                f"it supports: {', '.join(b.upper() for b in cls.buses)}")
        return args.bus
    if len(cls.buses) == 1:
        return cls.buses[0]
    raise BootloaderError(
        f"the {kind} supports {', '.join(b.upper() for b in cls.buses)}; "
        "say which to use with --bus")


def build_session(args, log):
    """Open the adapter, reset the target into the bootloader, open a session."""
    kind, _dev = choose_adapter(args, log)
    bus = choose_bus(args, kind, log)
    log(f"  Bus: {bus.upper()}")

    kwargs = {"serial": args.serial, "verbose": args.verbose, "i2c_bus": args.i2c_bus}
    if kind == "supernova":
        kwargs["i2c_port"] = args.i2c_port
    adapter = ADAPTERS[kind](**kwargs)
    adapter.open()

    resetter = None
    try:
        if args.manual:
            input("  Put the target into bootloader mode (BOOT0 high, then reset), "
                  "then press Enter... ")
        elif bus == "spi" and args.reset_via == "cs":
            resetter = SpiResetController(adapter, args.nrst_cs, args.boot0_cs,
                                          bus_cs=args.spi_cs,
                                          reset_hold=args.reset_hold,
                                          boot_delay=args.boot_delay)
            log(f"  Resetting target into bootloader "
                f"(NRST=CS{args.nrst_cs}, BOOT0=CS{args.boot0_cs})")
            resetter.reset_into_bootloader()
        else:
            resetter = ResetController(adapter, args.nrst_pin, args.boot0_pin,
                                       reset_hold=args.reset_hold,
                                       boot_delay=args.boot_delay)
            log(f"  Resetting target into bootloader "
                f"(NRST=GPIO{args.nrst_pin}, BOOT0=GPIO{args.boot0_pin})")
            resetter.reset_into_bootloader()

        if bus == "spi":
            log(f"  SPI: {args.spi_speed // 1000} kHz, mode {args.spi_mode}, "
                f"chip select {args.spi_cs}")
            adapter.spi_init(args.spi_speed, mode=args.spi_mode,
                             chip_select=args.spi_cs)
            bootloader = SpiBootloader(adapter, verbose=args.verbose)
            bootloader.connect()
        elif bus == "i2c":
            where = (" over the I3C port" if kind == "supernova"
                     and args.i2c_port == "i3c" else "")
            adapter.i2c_init(args.i2c_speed, args.pullup)
            actual = getattr(adapter, "i2c_actual_hz", None) or args.i2c_speed
            note = ("" if actual == args.i2c_speed
                    else f" (nearest available to {args.i2c_speed // 1000} kHz)")
            log(f"  I2C{where}: {actual // 1000} kHz{note}, "
                f"target 0x{args.i2c_address:02X}")
            # The I3C controller signalling I2C cannot hold off a stretching
            # target, so the no-stretch commands are required on that path.
            no_stretch = args.no_stretch or (kind == "supernova" and args.i2c_port == "i3c")
            if no_stretch:
                log("  Using the no-stretch command variants")
            bootloader = I2cBootloader(adapter, args.i2c_address, verbose=args.verbose,
                                       no_stretch=no_stretch)
            bootloader.connect()
        else:
            log(f"  I3C: {args.voltage} mV, push-pull {args.push_pull}, "
                f"open-drain {args.open_drain}")
            adapter.i3c_init(args.push_pull, args.open_drain, args.voltage)
            log("  Assigning dynamic addresses (ENTDAA)")
            targets = adapter.i3c_discover()
            if not targets:
                raise BootloaderError(
                    "no I3C targets responded to ENTDAA.\n"
                    "  Check bus wiring and ground, pull-ups, bus voltage, and that the\n"
                    "  target entered its bootloader (BOOT0 high at reset).")
            if args.target_address is not None:
                match = [t for t in targets if t["dynamic_address"] == args.target_address]
                if not match:
                    raise BootloaderError(
                        f"no target at 0x{args.target_address:02X}; found "
                        f"{[hex(t['dynamic_address']) for t in targets]}")
                target = match[0]
            else:
                if len(targets) > 1:
                    log(f"  note: {len(targets)} targets on the bus, using the first")
                target = targets[0]
            addr = target["dynamic_address"]
            pid = " ".join(f"{b:02X}" for b in target.get("pid", []))
            log(f"  Target at 0x{addr:02X}  PID {pid}  "
                f"BCR 0x{target.get('bcr', 0):02X}  DCR 0x{target.get('dcr', 0):02X}")
            adapter.i3c_accept_ibis(addr)
            bootloader = I3cBootloader(adapter, addr, verbose=args.verbose)
            bootloader.connect()

        log("  Bootloader session established")
        return adapter, bootloader, resetter
    except Exception:
        adapter.close()
        raise


# --------------------------------------------------------------------------
# Commands
# --------------------------------------------------------------------------

def describe_target(bootloader, log):
    version, commands = bootloader.get()
    device_id = bootloader.get_id()
    log(f"  Protocol version : {version >> 4}.{version & 0xF}")
    log(f"  Device ID        : 0x{device_id:03X}  "
        f"({DEVICE_IDS.get(device_id, 'unknown device')})")
    log(f"  Commands         : {len(commands)}")
    for opcode in commands:
        log(f"      0x{opcode:02X}  {COMMAND_NAMES.get(opcode, 'unknown')}")
    return version, commands, device_id


def make_progress(enabled):
    if not enabled or not sys.stdout.isatty():
        return None
    state = {"last": -1}

    def report(done, total):
        percent = int(done * 100 / total)
        if percent != state["last"]:
            state["last"] = percent
            width = 32
            filled = int(width * done / total)
            sys.stdout.write(f"\r    [{'#' * filled}{'-' * (width - filled)}] "
                             f"{percent:3d}%  {done}/{total} bytes")
            sys.stdout.flush()
            if done >= total:
                sys.stdout.write("\n")
    return report


def cmd_list(args, log):
    found = detect_adapters()
    if not found:
        log("  No Binho adapters found.")
        return 1
    for kind, dev in found:
        log(f"  {kind:9s} {dev.get('serial_number')}  hw {dev.get('hardware_version')}  "
            f"fw {dev.get('firmware_version')}  buses: "
            f"{', '.join(b.upper() for b in ADAPTERS[kind].buses)}")
    return 0


def cmd_info(args, log):
    adapter, bootloader, resetter = build_session(args, log)
    try:
        log("")
        describe_target(bootloader, log)
        if resetter and args.run:
            log("\n  Resetting into application")
            resetter.reset_into_application()
        if resetter:
            resetter.release()
    finally:
        adapter.close()
    return 0


def cmd_flash(args, log):
    segments = load_firmware(args.file, args.address)
    total = sum(len(d) for _, d in segments)
    log(f"  Image: {args.file}")
    for addr, data in segments:
        log(f"    0x{addr:08X} - 0x{addr + len(data) - 1:08X}  ({len(data)} bytes)")
    log(f"    total {total} bytes in {len(segments)} segment(s)\n")

    adapter, bootloader, resetter = build_session(args, log)
    try:
        log("")
        describe_target(bootloader, log)
        if args.chunk_size:
            bootloader.max_chunk = args.chunk_size
        log(f"\n  Block size: {bootloader.max_chunk} bytes")

        if args.erase == "mass":
            log("\n  Mass erasing")
            start = time.monotonic()
            bootloader.erase_mass(timeout=args.erase_timeout)
            log(f"    done in {time.monotonic() - start:.2f} s")
        elif args.erase == "pages":
            pages = sorted({
                page
                for addr, data in segments
                for page in range((addr - FLASH_BASE) // args.page_size,
                                  (addr - FLASH_BASE + len(data) - 1) // args.page_size + 1)
            })
            log(f"\n  Erasing {len(pages)} page(s) of {args.page_size} bytes: "
                f"{pages[0]}..{pages[-1]}")
            start = time.monotonic()
            bootloader.erase_pages(pages, timeout=args.erase_timeout)
            log(f"    done in {time.monotonic() - start:.2f} s")
        else:
            log("\n  Skipping erase")

        log("\n  Writing")
        start = time.monotonic()
        for addr, data in segments:
            log(f"    segment at 0x{addr:08X}")
            bootloader.write_memory(addr, data, progress=make_progress(not args.quiet))
        elapsed = time.monotonic() - start
        log(f"    wrote {total} bytes in {elapsed:.2f} s "
            f"({total / elapsed / 1024:.1f} KiB/s)" if elapsed else "")

        if args.verify:
            log("\n  Verifying")
            start = time.monotonic()
            for addr, data in segments:
                back = bootloader.read_memory(addr, len(data),
                                              progress=make_progress(not args.quiet))
                if back != data:
                    for i, (want, got) in enumerate(zip(data, back)):
                        if want != got:
                            raise BootloaderError(
                                f"verification failed at 0x{addr + i:08X}: "
                                f"wrote 0x{want:02X}, read 0x{got:02X}")
                    raise BootloaderError("verification failed: length mismatch")
            log(f"    all {total} bytes match ({time.monotonic() - start:.2f} s)")

        if args.run:
            if resetter and resetter.can_run_application:
                log("\n  Resetting into application")
                resetter.reset_into_application()
            else:
                entry = segments[0][0]
                log(f"\n  Jumping to 0x{entry:08X}")
                bootloader.go(entry)
        if resetter:
            resetter.release()
    finally:
        adapter.close()

    log("\n  Success.")
    return 0


def cmd_read(args, log):
    adapter, bootloader, resetter = build_session(args, log)
    try:
        if args.chunk_size:
            bootloader.max_chunk = args.chunk_size
        log(f"\n  Reading {args.length} bytes from 0x{args.address:08X}")
        start = time.monotonic()
        data = bootloader.read_memory(args.address, args.length,
                                      progress=make_progress(not args.quiet))
        log(f"    done in {time.monotonic() - start:.2f} s")
        with open(args.file, "wb") as fh:
            fh.write(data)
        log(f"    wrote {len(data)} bytes to {args.file}")
        if resetter:
            if args.run:
                resetter.reset_into_application()
            resetter.release()
    finally:
        adapter.close()
    return 0


def cmd_erase(args, log):
    adapter, bootloader, resetter = build_session(args, log)
    try:
        log("\n  Mass erasing")
        start = time.monotonic()
        bootloader.erase_mass(timeout=args.erase_timeout)
        log(f"    done in {time.monotonic() - start:.2f} s")
        if resetter:
            if args.run:
                resetter.reset_into_application()
            resetter.release()
    finally:
        adapter.close()
    return 0


def cmd_reset(args, log):
    kind, _dev = choose_adapter(args, log)
    kwargs = {"serial": args.serial, "verbose": args.verbose, "i2c_bus": args.i2c_bus}
    if kind == "supernova":
        kwargs["i2c_port"] = args.i2c_port
    adapter = ADAPTERS[kind](**kwargs)
    with adapter:
        resetter = ResetController(adapter, args.nrst_pin, args.boot0_pin,
                                   reset_hold=args.reset_hold, boot_delay=args.boot_delay)
        if args.bootloader:
            log("  Resetting into bootloader")
            resetter.reset_into_bootloader()
        else:
            log("  Resetting into application")
            resetter.reset_into_application()
        resetter.release()
    return 0


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------

def auto_int(value):
    return int(value, 0)


# Options that describe the session rather than one subcommand. They are
# attached to the top-level parser and to every subcommand, so they work on
# either side of the subcommand name; argparse alone accepts them only before
# it, which is a trap because the two halves of a command line look alike.
#
# Defaults live here rather than on the actions, and are applied after parsing.
# A subparser copy of an option would otherwise write its own default over a
# value the user gave before the subcommand.
SESSION_DEFAULTS = {
    "adapter": None,
    "bus": None,
    "serial": None,
    "verbose": False,
    "quiet": False,
    "i2c_address": DEFAULT_I2C_ADDRESS,
    "i2c_speed": 100000,
    "i2c_bus": "A",
    "i2c_port": "dedicated",
    "no_stretch": False,
    "spi_speed": 1000000,
    "spi_mode": 0,
    "spi_cs": 0,
    "nrst_cs": 1,
    "boot0_cs": 2,
    "reset_via": "gpio",
    "pullup": 1000,
    "voltage": 3300,
    "push_pull": "PUSH_PULL_2_5_MHZ_25_DC",
    "open_drain": "OPEN_DRAIN_1_MHZ",
    "target_address": None,
    "manual": False,
    "nrst_pin": 1,
    "boot0_pin": 2,
    "reset_hold": 0.05,
    "boot_delay": 0.25,
}

SESSION_EPILOG = ("session options such as --bus, --i2c-port and --nrst-pin may be given\n"
                  "here as well as before the subcommand; see 'stm32_flash --help'")


def _session_options(visible):
    """Build the session options, shown in the top-level help and hidden in each
    subcommand's help so the per-subcommand listing stays short."""
    parser = argparse.ArgumentParser(add_help=False)
    S = argparse.SUPPRESS

    def h(text):
        return text if visible else S

    parser.add_argument("--adapter", choices=sorted(ADAPTERS), default=S,
                        help=h("force the adapter to use"))
    parser.add_argument("--bus", choices=("i2c", "i3c", "spi"), default=S,
                        help=h("force the bus to use"))
    parser.add_argument("--serial", default=S, help=h("adapter serial number"))
    parser.add_argument("-v", "--verbose", action="store_true", default=S,
                        help=h("log every transfer"))
    parser.add_argument("-q", "--quiet", action="store_true", default=S,
                        help=h("suppress progress output"))

    i2c = parser.add_argument_group("I2C bus" if visible else S)
    i2c.add_argument("--i2c-address", type=auto_int, default=S, metavar="ADDR",
                     help=h("bootloader address, per AN2606 (default 0x67)"))
    i2c.add_argument("--i2c-speed", type=int, default=S, metavar="HZ",
                     help=h("I2C clock in Hz (default 100000)"))
    i2c.add_argument("--i2c-bus", choices=("A", "B"), default=S,
                     help=h("which I2C bus on a Pulsar (default A)"))
    i2c.add_argument("--i2c-port", choices=("dedicated", "i3c"), default=S,
                     help=h("which Supernova connector carries I2C: its dedicated I2C "
                            "port, or the I3C port using I2C signalling (default dedicated)"))
    i2c.add_argument("--no-stretch", action="store_true", default=S,
                     help=h("use the no-stretch command variants, for a controller "
                            "that cannot tolerate clock stretching"))
    i2c.add_argument("--pullup", type=int, default=S, metavar="OHM",
                     help=h("bus pull-up value in ohms (default 1000)"))

    spi = parser.add_argument_group("SPI bus" if visible else S)
    spi.add_argument("--spi-speed", type=int, default=S, metavar="HZ",
                     help=h("SPI clock in Hz, 10000 to 50000000 (default 1000000)"))
    spi.add_argument("--spi-mode", type=int, choices=(0, 1, 2, 3), default=S,
                     help=h("SPI mode (default 0)"))
    spi.add_argument("--spi-cs", type=int, choices=(0, 1, 2, 3), default=S,
                     help=h("chip select carrying NSS (default 0)"))
    spi.add_argument("--reset-via", choices=("gpio", "cs"), default=S,
                     help=h("drive NRST and BOOT0 from GPIO pins or from spare "
                            "chip selects (default gpio)"))
    spi.add_argument("--nrst-cs", type=int, choices=(0, 1, 2, 3), default=S,
                     help=h("chip select wired to NRST, with --reset-via cs"))
    spi.add_argument("--boot0-cs", type=int, choices=(0, 1, 2, 3), default=S,
                     help=h("chip select wired to BOOT0, with --reset-via cs"))

    i3c = parser.add_argument_group("I3C bus (Supernova only)" if visible else S)
    i3c.add_argument("--voltage", type=int, default=S, metavar="MV",
                     help=h("I3C bus voltage in mV (default 3300)"))
    i3c.add_argument("--push-pull", default=S, help=h("push-pull rate constant"))
    i3c.add_argument("--open-drain", default=S, help=h("open-drain rate constant"))
    i3c.add_argument("--target-address", type=auto_int, default=S, metavar="ADDR",
                     help=h("dynamic address of the target"))

    entry = parser.add_argument_group("bootloader entry" if visible else S)
    entry.add_argument("--manual", action="store_true", default=S,
                       help=h("prompt instead of driving NRST/BOOT0 over GPIO"))
    entry.add_argument("--nrst-pin", type=int, default=S, metavar="N", help=h(None))
    entry.add_argument("--boot0-pin", type=int, default=S, metavar="N", help=h(None))
    entry.add_argument("--reset-hold", type=float, default=S, metavar="S", help=h(None))
    entry.add_argument("--boot-delay", type=float, default=S, metavar="S", help=h(None))
    return parser


def apply_session_defaults(args):
    """Fill in every session option the user did not give in either position."""
    for name, value in SESSION_DEFAULTS.items():
        if not hasattr(args, name):
            setattr(args, name, value)
    return args


def build_parser():
    parser = argparse.ArgumentParser(
        prog="stm32_flash",
        description="Flash an STM32 over I2C or I3C using a Binho USB host adapter.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""examples:
  %(prog)s list
  %(prog)s info
  %(prog)s flash firmware.hex
  %(prog)s flash app.bin --address 0x08000000
  %(prog)s --bus i3c info                     (Supernova only)
  %(prog)s read dump.bin --address 0x08000000 --length 0x20000
""")

    common = _session_options(visible=True)
    for action in common._actions:
        parser._add_action(action)

    sub = parser.add_subparsers(dest="command", required=True)
    hidden = _session_options(visible=False)
    sub_kw = dict(parents=[hidden], epilog=SESSION_EPILOG,
                  formatter_class=argparse.RawDescriptionHelpFormatter)

    p = sub.add_parser("list", help="list connected Binho adapters", **sub_kw)
    p.set_defaults(func=cmd_list)

    p = sub.add_parser("info", help="identify the target and report capabilities", **sub_kw)
    p.add_argument("--run", action="store_true", help="reset into the application afterwards")
    p.set_defaults(func=cmd_info)

    p = sub.add_parser("flash", help="erase, program and verify an image", **sub_kw)
    p.add_argument("file")
    p.add_argument("--address", type=auto_int, default=FLASH_BASE, metavar="ADDR")
    p.add_argument("--erase", choices=("mass", "pages", "none"), default="mass",
                   help="mass erase, erase only the pages the image covers, or skip")
    p.add_argument("--page-size", type=auto_int, default=8192, metavar="N",
                   help="flash page size for --erase pages (default 8192)")
    p.add_argument("--erase-timeout", type=float, default=30.0, metavar="S")
    p.add_argument("--chunk-size", type=int, metavar="N")
    p.add_argument("--no-verify", dest="verify", action="store_false")
    p.add_argument("--no-run", dest="run", action="store_false")
    p.set_defaults(func=cmd_flash, verify=True, run=True)

    p = sub.add_parser("read", help="read memory into a file", **sub_kw)
    p.add_argument("file")
    p.add_argument("--address", type=auto_int, default=FLASH_BASE, metavar="ADDR")
    p.add_argument("--length", type=auto_int, required=True, metavar="N")
    p.add_argument("--chunk-size", type=int, metavar="N")
    p.add_argument("--run", action="store_true")
    p.set_defaults(func=cmd_read)

    p = sub.add_parser("erase", help="mass-erase user flash", **sub_kw)
    p.add_argument("--erase-timeout", type=float, default=30.0, metavar="S")
    p.add_argument("--run", action="store_true")
    p.set_defaults(func=cmd_erase)

    p = sub.add_parser("reset", help="reset the target via GPIO", **sub_kw)
    p.add_argument("--bootloader", action="store_true",
                   help="reset into the bootloader instead of the application")
    p.set_defaults(func=cmd_reset)

    return parser


def main(argv=None):
    args = apply_session_defaults(build_parser().parse_args(argv))

    def log(message=""):
        if not args.quiet:
            print(message)

    try:
        return args.func(args, log)
    except BootloaderError as exc:
        print(f"\nerror: {exc}", file=sys.stderr)
        return 1
    except (FileNotFoundError, ValueError) as exc:
        print(f"\nerror: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("\ninterrupted", file=sys.stderr)
        return 130


if __name__ == "__main__":
    sys.exit(main())
