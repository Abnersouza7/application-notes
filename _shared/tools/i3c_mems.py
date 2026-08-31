#!/usr/bin/env python3
"""Exercise the I3C target in a MEMS sensor from a Binho Supernova.

This is not a sensor driver. Vendor drivers exist and are better at that job.
What this does is drive the I3C command surface and establish, from observable
effects, which parts of it a given target actually implements.

That distinction exists because of one measured fact: a target ACKs its
dynamic address and silently discards commands it does not implement, which
the I3C specification permits. So a command completing successfully is not
evidence that the target supports it. Support has to be established by
observing a change: set a value and read it back, move an address and confirm
the old one stops answering, enable an interrupt and count what arrives.

    python i3c_mems.py scan
    python i3c_mems.py identify --device bmp585
    python i3c_mems.py features --device bmp585
    python i3c_mems.py stream   --device bmi323 --seconds 5
    python i3c_mems.py ibi      --device bmi323 --seconds 5
    python i3c_mems.py rates    --device bmp585

Adding a device is one entry in PROFILES. The parts on the I3C Target Board
that are not listed here have not been exercised on hardware, and a profile
written from a datasheet alone would be exactly the guess this tool exists to
avoid.

Requires the binhosupernova package and a Supernova with an I3C interface.
"""

import argparse
import queue
import sys
import time
from collections import Counter

TOOL_VERSION = "1.0"

# --------------------------------------------------------------------------
# Errors
# --------------------------------------------------------------------------


class MemsError(RuntimeError):
    """Anything that should stop the command with a readable message."""


# --------------------------------------------------------------------------
# Session options
#
# These are accepted both before and after the subcommand. argparse resolves
# a shared parent parser's defaults in favor of whichever parser saw the
# option last, so the parent uses SUPPRESS defaults and the real defaults are
# applied afterwards. Without this, options only work in one position and the
# other silently does nothing.
# --------------------------------------------------------------------------

SESSION_DEFAULTS = {
    "serial": None,
    "voltage": 3300,
    "push_pull": "PUSH_PULL_5_MHZ_50_DC",
    "open_drain": "OPEN_DRAIN_1_MHZ",
    "drive": "FAST_MODE",
    "verbose": False,
}


def add_session_options(parser, visible=True):
    hide = None if visible else argparse.SUPPRESS
    group = parser.add_argument_group("session options" if visible else None)
    group.add_argument("--serial", default=argparse.SUPPRESS,
                       help=hide or "serial number of the Supernova to open")
    group.add_argument("--voltage", type=int, default=argparse.SUPPRESS,
                       help=hide or "bus voltage in mV (default 3300)")
    group.add_argument("--push-pull", default=argparse.SUPPRESS,
                       help=hide or "push-pull rate name (default 5 MHz 50%% DC)")
    group.add_argument("--open-drain", default=argparse.SUPPRESS,
                       help=hide or "open-drain rate name (default 1 MHz)")
    group.add_argument("--drive", default=argparse.SUPPRESS,
                       help=hide or "drive strength: STANDARD_MODE or FAST_MODE")
    group.add_argument("-v", "--verbose", action="store_true",
                       default=argparse.SUPPRESS,
                       help=hide or "print every adapter response")


def apply_session_defaults(args):
    for name, value in SESSION_DEFAULTS.items():
        if not hasattr(args, name):
            setattr(args, name, value)
    return args


# --------------------------------------------------------------------------
# The bus
# --------------------------------------------------------------------------


class Bus:
    """A Supernova acting as I3C controller, with IBIs delivered to a queue."""

    def __init__(self, serial=None, verbose=False):
        from binhosupernova.supernova import Supernova
        self.device = Supernova()
        self.serial = serial
        self.verbose = verbose
        self._responses = queue.Queue()
        self.ibis = queue.Queue()
        self._next_id = 0
        self._opened = False

    # -- lifecycle ---------------------------------------------------------

    def open(self):
        import binhosupernova
        if not binhosupernova.getConnectedSupernovaDevicesList():
            raise MemsError("no Supernova found on USB")
        result = (self.device.open(serial=self.serial) if self.serial
                  else self.device.open())
        if result.get("opcode") != 0:
            raise MemsError(f"could not open the Supernova: {result.get('message')}")
        self._opened = True
        result = self.device.onEvent(self._on_event)
        if result.get("opcode") != 0:
            raise MemsError(f"could not register the callback: {result.get('message')}")

    def close(self):
        if self._opened:
            try:
                self.device.close()
            finally:
                self._opened = False

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *exc):
        self.close()
        return False

    # -- plumbing ----------------------------------------------------------

    def _on_event(self, response, system_message):
        # Called from the SDK receive thread, so it must return promptly.
        if response is None:
            return
        if isinstance(response, dict) and response.get("id") == 0:
            response["_t"] = time.monotonic()
            self.ibis.put(response)
        else:
            self._responses.put(response)

    def call(self, method, *args, timeout=5.0, allowed=(), **kwargs):
        """Send a request and block for the response carrying the same id."""
        self._next_id = (self._next_id % 65534) + 1
        request_id = self._next_id
        submission = method(request_id, *args, **kwargs)
        if submission.get("opcode") != 0:
            raise MemsError(f"{method.__name__} rejected: {submission.get('message')}")

        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise MemsError(f"{method.__name__}: no response in {timeout:g} s")
            try:
                response = self._responses.get(timeout=min(remaining, 0.2))
            except queue.Empty:
                continue
            if response.get("id") != request_id:
                continue          # a stale reply to an earlier, timed-out call
            if self.verbose:
                print(f"    <- {response}")
            result = response.get("result")
            if result not in (None, "SUCCESS") and result not in allowed:
                raise MemsError(f"{method.__name__} -> {result}")
            return response

    def try_call(self, method, *args, **kwargs):
        """Return (ok, response_or_message) instead of raising.

        Every "does this target implement X" question goes through here,
        because the interesting cases are the ones that fail.
        """
        try:
            return True, self.call(method, *args, **kwargs)
        except MemsError as exc:
            return False, str(exc)

    # -- setup -------------------------------------------------------------

    def configure(self, voltage_mv=3300, push_pull="PUSH_PULL_5_MHZ_50_DC",
                  open_drain="OPEN_DRAIN_1_MHZ", drive="FAST_MODE"):
        from binhosupernova.commands.i3c.definitions import (
            I3cPushPullTransferRate, I3cOpenDrainTransferRate, I2cTransferRate,
            I3cDriveStrength)
        try:
            rates = (getattr(I3cPushPullTransferRate, push_pull),
                     getattr(I3cOpenDrainTransferRate, open_drain),
                     I2cTransferRate._100KHz,
                     getattr(I3cDriveStrength, drive))
        except AttributeError as exc:
            raise MemsError(f"unknown rate or drive strength: {exc}")

        self.call(self.device.setI3cVoltage, voltage_mv)
        response = self.call(self.device.i3cControllerInit, *rates,
                             allowed=("INTERFACE_ALREADY_INITIALIZED",))
        if response.get("result") != "SUCCESS":
            # Init discards its arguments when the interface is already up, so
            # the settings have to be sent again or the previous run's rates
            # stay in force. This cost three bench sessions on AN0003.
            self.call(self.device.i3cControllerSetParameters, *rates)
        self.rates = (push_pull, open_drain, drive, voltage_mv)

    def init_bus(self):
        """RSTDAA then ENTDAA. Returns the target table."""
        self.call(self.device.i3cControllerInitBus, timeout=10.0,
                  allowed=("I3C_BUS_INIT_NACK_RSTDAA", "I3C_BUS_INIT_NACK_SETDASA",
                           "I3C_BUS_INIT_NACK_SETAASA", "I3C_BUS_INIT_NACK_ENTDAA"))
        return self.table()

    def table(self):
        response = self.call(self.device.i3cControllerGetTargetDevicesTable,
                             timeout=10.0)
        return response.get("table") or []

    def accept_ibis(self, address, payload_length=8):
        from binhosupernova.commands.i3c.definitions import (
            TargetType, TargetInterruptRequest, ControllerRoleRequest,
            SetdasaConfiguration, SetaasaConfiguration, EntdaaConfiguration,
            IBiTimestamp, PendingReadCapability)
        return self.try_call(
            self.device.i3cControllerSetTargetDeviceConfiguration, address, {
                "targetType": TargetType.I3C_DEVICE,
                "IBIRequest": TargetInterruptRequest.ACCEPT_IBI,
                "CRRequest": ControllerRoleRequest.REJECT_CRR,
                "daaUseSETDASA": SetdasaConfiguration.DO_NOT_USE_SETDASA,
                "daaUseSETAASA": SetaasaConfiguration.DO_NOT_USE_SETAASA,
                "daaUseENTDAA": EntdaaConfiguration.USE_ENTDAA,
                "ibiTimestampEnable": IBiTimestamp.DISABLE_IBIT,
                "pendingReadCapability": PendingReadCapability.DISABLE_AUTOMATIC_READ,
                "maxIbiPayloadLength": payload_length,
            })

    def stop_ibis(self, address, attempts=4, settle=0.2, window=0.5):
        """Disable the target's IBI and confirm it actually stopped.

        A single direct DISEC is not always enough. Measured over repeated
        runs of the feature battery, roughly one attempt in six left a 50 Hz
        stream still arriving, while 45 consecutive attempts at 25, 50 and
        100 Hz with no other traffic never failed once. So it is not simply a
        function of interrupt rate, and rather than assume the command took,
        this verifies and retries.

        Returns the number of attempts used, or None if the stream never
        stopped, so callers can report the retry rather than hide it.
        """
        from binhosupernova.commands.i3c.definitions import DISEC
        for attempt in range(1, attempts + 1):
            self.try_call(self.device.i3cDirectDISEC, address, [DISEC.DISINT])
            time.sleep(settle)
            self.drain_ibis()
            if not self.collect_ibis(window):
                return attempt
        return None

    def quiesce(self, address):
        """Put the target in a known state before a run starts.

        ENEC state survives the host program exiting, so without this a second
        run of an example sees interrupts it never asked for.
        """
        self.stop_ibis(address)
        self.drain_ibis()

    # -- register access ---------------------------------------------------

    def read_regs(self, address, register, count, profile):
        """Read `count` register files, honouring the profile's framing."""
        from binhosupernova.commands.i3c.definitions import TransferMode
        width = profile.data_width
        wanted = profile.read_dummy + width * count
        response = self.call(self.device.i3cControllerRead, address,
                             TransferMode.I3C_SDR, [register], wanted)
        raw = bytes(response.get("payload") or b"")
        if len(raw) < wanted:
            raise MemsError(f"short read at 0x{register:02X}: "
                            f"{len(raw)} of {wanted} bytes")
        body = raw[profile.read_dummy:]
        values = [int.from_bytes(body[i * width:(i + 1) * width], "little")
                  for i in range(count)]
        return values, raw

    def read_reg(self, address, register, profile):
        values, raw = self.read_regs(address, register, 1, profile)
        return values[0], raw

    def write_reg(self, address, register, value, profile):
        from binhosupernova.commands.i3c.definitions import TransferMode
        data = list(int(value).to_bytes(profile.data_width, "little"))
        return self.call(self.device.i3cControllerWrite, address,
                         TransferMode.I3C_SDR, [register], data)

    # -- IBI collection ----------------------------------------------------

    def drain_ibis(self):
        drained = 0
        while True:
            try:
                self.ibis.get_nowait()
                drained += 1
            except queue.Empty:
                return drained

    def collect_ibis(self, seconds, on_each=None):
        end = time.monotonic() + seconds
        got = []
        while time.monotonic() < end:
            try:
                notification = self.ibis.get(timeout=0.02)
            except queue.Empty:
                continue
            got.append(notification)
            if on_each is not None:
                on_each(notification)
        return got


# --------------------------------------------------------------------------
# Device profiles
#
# A profile carries the part's framing, its expected identity, the writes that
# start a data stream, the writes that route an interrupt onto the bus, and a
# decoder for the mandatory data byte. Framing and MDB layout are per part and
# not per vendor: the two Bosch families here differ in both.
# --------------------------------------------------------------------------


class Profile:
    name = "?"
    vendor = "?"
    kind = "?"

    # framing
    data_width = 1          # bytes per register file
    read_dummy = 0          # dummy bytes the target inserts before a read payload

    # identity
    chip_id_register = 0x00
    chip_id_expected = None
    chip_id_mask = 0xFF     # BMI323's CHIP_ID word carries a reserved upper byte
    device_id_expected = None   # PID bits 31:16
    bcr_expected = None
    dcr_expected = None

    # what the reader can see happen
    observable = ""

    # registers worth dumping in the note's reference section
    registers = ()

    def start_stream(self, bus, address, route_ibi=False):
        """Configure the part and put it into its measurement mode.

        Interrupt routing is a parameter rather than a separate call the
        caller makes afterwards, because on the BMP58x the order matters and
        getting it wrong fails silently. See Bmp58x.start_stream.
        """
        raise NotImplementedError

    def stop_stream(self, bus, address):
        raise NotImplementedError

    def read_sample(self, bus, address):
        """Return an ordered list of (label, value, unit) tuples."""
        raise NotImplementedError

    def clear_interrupt(self, bus, address):
        """Called after each IBI. Only some parts need it; default is nothing."""
        return None

    def interrupt_mode(self):
        """How this part's interrupt is configured, for reporting.

        Latched and pulsed are BMP58x concepts. The BMI323 has no equivalent
        setting, so saying "pulsed" about it would be inventing a mode.
        """
        return "the part's default configuration"

    # Command register and soft-reset value. A soft reset is the only way back
    # from some latched modes: SETXTIME engages the BMI323's I3C timing
    # control synchronous feature, which changes the IBI payload from one byte
    # to four and survives both a host restart and a bus reset.
    command_register = None
    soft_reset_value = None

    def soft_reset(self, bus, address):
        if self.command_register is None:
            return False
        bus.write_reg(address, self.command_register, self.soft_reset_value, self)
        time.sleep(0.1)
        return True

    def expected_ibi_rate(self):
        return None

    def decode_mdb(self, byte):
        return {}


class Bmi323(Profile):
    name = "bmi323"
    vendor = "Bosch Sensortec"
    kind = "6-axis IMU"

    data_width = 2
    read_dummy = 2          # "for the I3C read operation, two dummy bytes are inserted"

    chip_id_register = 0x00
    chip_id_expected = 0x43
    chip_id_mask = 0x00FF   # the word reads 0x1143; the upper byte is reserved
    device_id_expected = 0x1043
    bcr_expected = 0x06
    dcr_expected = 0xEF

    observable = ("tilt the board and the accelerometer axes change sign; "
                  "at rest the magnitude is about 1 g")

    STATUS = 0x02
    ACC_DATA_X = 0x03
    INT_STATUS_IBI = 0x0F
    ACC_CONF = 0x20
    INT_MAP2 = 0x3B
    command_register = 0x7E                  # CMD
    soft_reset_value = 0xDEAF                # "largely equivalent to a power cycle"

    # acc_mode = 0b011, acc_range = 2 (+/-8 g), acc_odr = 7 (50 Hz)
    ACC_CONF_RUN = 0x3127
    ACC_ODR_HZ = 50.0
    ACC_LSB_PER_G = 4096.0

    # INT_MAP2.acc_drdy_int occupies bits 11:10 and takes 0b11 for the I3C IBI
    MAP_ACC_DRDY_TO_IBI = 0b11 << 10

    registers = ((0x00, "CHIP_ID"), (0x01, "ERR_REG"), (0x02, "STATUS"),
                 (0x03, "ACC_DATA_X"), (0x04, "ACC_DATA_Y"), (0x05, "ACC_DATA_Z"),
                 (0x09, "TEMP_DATA"), (0x0F, "INT_STATUS_IBI"),
                 (0x20, "ACC_CONF"), (0x21, "GYR_CONF"), (0x3B, "INT_MAP2"))

    def start_stream(self, bus, address, route_ibi=False):
        if route_ibi:
            self.route_interrupt_to_ibi(bus, address)
        bus.write_reg(address, self.ACC_CONF, self.ACC_CONF_RUN, self)
        readback, _ = bus.read_reg(address, self.ACC_CONF, self)
        if readback != self.ACC_CONF_RUN:
            raise MemsError(f"ACC_CONF did not take: wrote 0x{self.ACC_CONF_RUN:04X}, "
                            f"read 0x{readback:04X}")
        time.sleep(0.05)

    def stop_stream(self, bus, address):
        bus.write_reg(address, self.ACC_CONF, 0x0028, self)   # reset value
        bus.write_reg(address, self.INT_MAP2, 0x0000, self)

    def read_sample(self, bus, address):
        words, _ = bus.read_regs(address, self.ACC_DATA_X, 3, self)
        axes = [(w - 0x10000 if w & 0x8000 else w) / self.ACC_LSB_PER_G
                for w in words]
        magnitude = sum(a * a for a in axes) ** 0.5
        return [("acc x", axes[0], "g"), ("acc y", axes[1], "g"),
                ("acc z", axes[2], "g"), ("|acc|", magnitude, "g")]

    def route_interrupt_to_ibi(self, bus, address):
        bus.write_reg(address, self.INT_MAP2, self.MAP_ACC_DRDY_TO_IBI, self)
        readback, _ = bus.read_reg(address, self.INT_MAP2, self)
        if readback != self.MAP_ACC_DRDY_TO_IBI:
            raise MemsError(f"INT_MAP2 did not take: read 0x{readback:04X}")

    def expected_ibi_rate(self):
        return self.ACC_ODR_HZ

    def decode_mdb(self, byte):
        # Table 48, I3C In-band Interrupt Mandatory Byte Payload
        return {
            "FIFO watermark or full": bool(byte & 0x01),
            "sample ready (acc, gyr, temp)": bool(byte & 0x02),
            "feature interrupts": bool(byte & 0x04),
            "interrupt group id": (byte >> 5) & 0x03,
        }


class Bmp58x(Profile):
    """BMP581 and BMP585.

    The register maps are identical, including INT_CONFIG's latched reset
    value. Only CHIP_ID and the device ID field of the PID differ, so this is
    one profile parameterised by chip ID rather than two profiles. That was
    verified by running one code path against both parts, not inferred from
    the datasheets.
    """
    vendor = "Bosch Sensortec"
    kind = "barometric pressure sensor"

    data_width = 1
    read_dummy = 0          # unlike the BMI323, which inserts two

    chip_id_register = 0x01
    bcr_expected = 0x06
    dcr_expected = 0x62     # MIPI DCR registry: pressure sensor

    CHIP_STATUS = 0x11
    INT_CONFIG = 0x14
    INT_SOURCE = 0x15
    TEMP_DATA_XLSB = 0x1D
    INT_STATUS = 0x27
    STATUS = 0x28
    OSR_CONFIG = 0x36
    ODR_CONFIG = 0x37
    OSR_EFF = 0x38
    CMD = 0x7E
    command_register = 0x7E
    soft_reset_value = 0xB6

    PWR_STANDBY, PWR_NORMAL = 0b00, 0b01
    ODR_10HZ, ODR_30HZ = 0x17, 0x13         # both listed with Error = 0.00
    ODR_HZ = 10.0
    DRDY_EN = 1 << 0
    INT_MODE_LATCHED = 1 << 0
    INT_EN = 1 << 3
    INT_CONFIG_RESET = 0x35                 # int_mode = 1, so latched by default

    observable = ("breathe on it or lift it and the pressure changes; "
                  "about 12 Pa per meter of altitude")

    registers = ((0x01, "CHIP_ID"), (0x02, "REV_ID"), (0x11, "CHIP_STATUS"),
                 (0x14, "INT_CONFIG"), (0x15, "INT_SOURCE"),
                 (0x1D, "TEMP_DATA_XLSB"), (0x20, "PRESS_DATA_XLSB"),
                 (0x27, "INT_STATUS"), (0x28, "STATUS"),
                 (0x36, "OSR_CONFIG"), (0x37, "ODR_CONFIG"), (0x38, "OSR_EFF"))

    def __init__(self, latched=False):
        # False puts the interrupt in pulsed mode, which streams IBIs with no
        # host action. True keeps the part's own default and relies on
        # clear_interrupt to re-arm it. Both are documented; both work.
        self.latched = latched

    def start_stream(self, bus, address, route_ibi=False):
        """Everything is configured in standby, and only then does the part
        enter its measurement mode.

        The order is not cosmetic. Measured on the BMP585: writing INT_CONFIG
        while the part is in normal mode is accepted by the register, which
        reads back the new value, but the interrupt block keeps the old mode
        until the next standby-to-normal transition. Configure in normal mode
        and interrupts stop arriving with every register looking correct.

            config written in normal mode          0 interrupts in 2.0 s
            same registers, then standby-to-normal 21 interrupts in 2.0 s
            configured in standby, then normal     20 interrupts in 2.0 s

        The datasheet's own advice covers it: "It is generally recommended to
        write configurations before switching into the measurement mode."
        """
        bus.write_reg(address, self.ODR_CONFIG,
                      (self.ODR_10HZ << 2) | self.PWR_STANDBY, self)
        time.sleep(0.05)
        if route_ibi:
            self.route_interrupt_to_ibi(bus, address)
        bus.write_reg(address, self.OSR_CONFIG, 1 << 6, self)   # press_en
        bus.write_reg(address, self.ODR_CONFIG,
                      (self.ODR_10HZ << 2) | self.PWR_NORMAL, self)
        time.sleep(0.25)
        effective, _ = bus.read_reg(address, self.OSR_EFF, self)
        if not effective & 0x80:
            raise MemsError(f"odr_is_valid clear (OSR_EFF 0x{effective:02X}): the "
                            f"ODR and OSR combination was rejected")

    def stop_stream(self, bus, address):
        bus.write_reg(address, self.ODR_CONFIG,
                      (self.ODR_10HZ << 2) | self.PWR_STANDBY, self)
        # Leave the interrupt configuration as the datasheet's reset value, so
        # the next run does not inherit a mode it did not ask for. Without this
        # a latched run poisons the following pulsed run.
        bus.write_reg(address, self.INT_SOURCE, 0x00, self)
        bus.write_reg(address, self.INT_CONFIG, self.INT_CONFIG_RESET, self)

    def read_sample(self, bus, address):
        body, _ = bus.read_regs(address, self.TEMP_DATA_XLSB, 6, self)
        temp_raw = body[0] | (body[1] << 8) | (body[2] << 16)
        press_raw = body[3] | (body[4] << 8) | (body[5] << 16)
        if temp_raw & 0x800000:
            temp_raw -= 0x1000000
        return [("temperature", temp_raw / 65536.0, "C"),
                ("pressure", press_raw / 64.0, "Pa"),
                ("pressure", press_raw / 6400.0, "hPa")]

    def route_interrupt_to_ibi(self, bus, address):
        bus.write_reg(address, self.INT_SOURCE, self.DRDY_EN, self)
        readback, _ = bus.read_reg(address, self.INT_SOURCE, self)
        if readback != self.DRDY_EN:
            raise MemsError(f"INT_SOURCE did not take: read 0x{readback:02X}")
        mode = self.INT_CONFIG_RESET
        if not self.latched:
            mode &= ~self.INT_MODE_LATCHED
        bus.write_reg(address, self.INT_CONFIG, mode, self)
        # INT_CONFIG.int_en enables the physical INT pin only. It does not gate
        # the IBI, which arrives either way.
        bus.read_reg(address, self.INT_STATUS, self)     # clear anything pending

    def clear_interrupt(self, bus, address):
        # In latched mode the interrupt stays asserted and no further IBI is
        # generated until INT_STATUS, which is clear-on-read, has been read.
        # Without this the part delivers exactly one IBI and then goes quiet.
        if self.latched:
            value, _ = bus.read_reg(address, self.INT_STATUS, self)
            return value
        return None

    def interrupt_mode(self):
        return ("latched, re-armed by the host reading INT_STATUS"
                if self.latched else "pulsed")

    def expected_ibi_rate(self):
        return self.ODR_HZ

    def decode_mdb(self, byte):
        # Table 26 on the BMP581, Table 24 on the BMP585, identical content.
        # Note the BMI323 puts data-ready on bit 1 instead of bit 0.
        return {
            "data ready": bool(byte & 0x01),
            "FIFO full": bool(byte & 0x02),
            "FIFO threshold": bool(byte & 0x04),
            "pressure out of range": bool(byte & 0x08),
        }


class Bmp581(Bmp58x):
    name = "bmp581"
    chip_id_expected = 0x50
    device_id_expected = 0x1050


class Bmp585(Bmp58x):
    name = "bmp585"
    chip_id_expected = 0x51
    device_id_expected = 0x1051
    observable = (Bmp58x.observable +
                  "; this is the media-resistant part, so a drop of water on "
                  "the gel is survivable where the BMP581 would not be")


PROFILES = {
    "bmi323": Bmi323,
    "bmp581": Bmp581,
    "bmp585": Bmp585,
}


def make_profile(name, latched=False):
    try:
        cls = PROFILES[name.lower()]
    except KeyError:
        raise MemsError(f"unknown device {name!r}. Known: "
                        f"{', '.join(sorted(PROFILES))}")
    if issubclass(cls, Bmp58x):
        return cls(latched=latched)
    return cls()


# --------------------------------------------------------------------------
# Identity
# --------------------------------------------------------------------------

SUPPORTED = "supported"
NOT_IMPLEMENTED = "not implemented"
UNDETERMINED = "undetermined"


def hex_bytes(data):
    return " ".join(f"{b:02X}" for b in data) if data else "(none)"


def payload_of(response):
    if not isinstance(response, dict):
        return []
    return list(response.get("payload") or [])


def decode_bcr(value):
    roles = {0: "I3C target", 1: "controller capable",
             2: "reserved", 3: "reserved"}
    return [
        ("device role", roles[(value >> 6) & 3]),
        ("advanced capabilities (bit 5)",
         "yes" if value & 0x20 else "no, so SDR only"),
        ("virtual target support", "yes" if value & 0x10 else "no"),
        ("offline capable", "no, always responds" if value & 0x08 else "yes"),
        ("IBI mandatory payload", "yes" if value & 0x04 else "no"),
        ("IBI capable", "yes" if value & 0x02 else "no"),
        ("max data speed limit", "yes" if value & 0x01 else "no limit"),
    ]


def decode_pid(pid):
    value = int.from_bytes(bytes(pid), "big")
    return [
        ("MIPI member id (47:33)", f"0x{(value >> 33) & 0x7FFF:04X}"),
        ("id type selector (32)", (value >> 32) & 1),
        ("device id (31:16)", f"0x{(value >> 16) & 0xFFFF:04X}"),
        ("instance id (15:12)", f"0b{(value >> 12) & 0xF:04b}"),
        ("reserved (11:0)", f"0x{value & 0xFFF:03X}"),
    ]


def settle_after_enumeration(bus, address, profile):
    """Discard the first reads after ENTDAA, because they are not settled.

    Measured on the BMP585 over eight enumerations: GETPID reports PID bit 12
    as 0 on read 1 every time, and as 1 from read 3 onward every time. Bit 12
    is the SDO pin level, and ENTDAA itself captures the unsettled value, so
    the enumeration table's cached PID is permanently the wrong one. Only that
    bit moves; the device id field is stable throughout.

    The datasheets ask for this directly: "Depending on the interface
    configuration, a dummy read should be the first access to the device."
    Two reads, not one, because read 2 was still unsettled in 2 of 8 trials.
    """
    for _ in range(2):
        try:
            bus.read_reg(address, profile.chip_id_register, profile)
        except MemsError:
            return


def find_target(bus, profile=None):
    """Enumerate and return (address, entry). Identifies by chip ID when asked."""
    table = bus.init_bus()
    if not table:
        raise MemsError("nothing enumerated: no I3C target answered ENTDAA")
    if profile is None:
        return table[0]["dynamic_address"], table[0]
    settle_after_enumeration(bus, table[0]["dynamic_address"], profile)
    for entry in table:
        address = entry["dynamic_address"]
        try:
            value, _ = bus.read_reg(address, profile.chip_id_register, profile)
        except MemsError:
            continue
        if value & profile.chip_id_mask == profile.chip_id_expected:
            return address, entry
    found = ", ".join(f"0x{e['dynamic_address']:02X}" for e in table)
    raise MemsError(f"no {profile.name} found. Enumerated: {found}. "
                    f"Expected CHIP_ID 0x{profile.chip_id_expected:02X} at "
                    f"register 0x{profile.chip_id_register:02X}")


# --------------------------------------------------------------------------
# Feature probing
#
# Each probe returns (verdict, detail). A probe may only return SUPPORTED if
# it observed a change, and only NOT_IMPLEMENTED if it observed the absence of
# one. Anything else is UNDETERMINED, which is a real answer and is reported
# as such rather than being quietly rounded to a no.
# --------------------------------------------------------------------------


def probe_identity(bus, address, profile):
    results = []
    ok, response = bus.try_call(bus.device.i3cGETPID, address)
    pid = payload_of(response) if ok else []
    ok_bcr, response_bcr = bus.try_call(bus.device.i3cGETBCR, address)
    bcr = (payload_of(response_bcr) or [None])[0]
    ok_dcr, response_dcr = bus.try_call(bus.device.i3cGETDCR, address)
    dcr = (payload_of(response_dcr) or [None])[0]

    if pid and profile.device_id_expected is not None:
        device_id = (int.from_bytes(bytes(pid), "big") >> 16) & 0xFFFF
        match = device_id == profile.device_id_expected
        results.append(("GETPID", SUPPORTED if match else UNDETERMINED,
                        f"{hex_bytes(pid)}, device id 0x{device_id:04X}"
                        + ("" if match else
                           f", expected 0x{profile.device_id_expected:04X}")))
    if bcr is not None:
        match = profile.bcr_expected in (None, bcr)
        results.append(("GETBCR", SUPPORTED if match else UNDETERMINED,
                        f"0x{bcr:02X}" + ("" if match else
                                          f", expected 0x{profile.bcr_expected:02X}")))
    if dcr is not None:
        match = profile.dcr_expected in (None, dcr)
        results.append(("GETDCR", SUPPORTED if match else UNDETERMINED,
                        f"0x{dcr:02X}" + ("" if match else
                                          f", expected 0x{profile.dcr_expected:02X}")))
    return results, bcr


def probe_length_limits(bus, address):
    """SETMWL / SETMRL: set a value and read it back."""
    out = []
    for label, setter, getter, value in (
            ("SETMWL / GETMWL", bus.device.i3cDirectSETMWL,
             bus.device.i3cGETMWL, 64),
            ("SETMRL / GETMRL", bus.device.i3cDirectSETMRL,
             bus.device.i3cGETMRL, 32)):
        ok_before, before = bus.try_call(getter, address)
        bus.try_call(setter, address, value)
        ok_after, after = bus.try_call(getter, address)
        if not (ok_before and ok_after):
            out.append((label, UNDETERMINED, "the getter did not answer"))
            continue
        b, a = payload_of(before), payload_of(after)
        if a != b:
            out.append((label, SUPPORTED, f"{b} then set {value} then {a}"))
        else:
            out.append((label, NOT_IMPLEMENTED,
                        f"{b} unchanged after setting {value}"))
    return out


def probe_group_address(bus, address, profile, group=0x20):
    """SETGRPA: if the group address were assigned, the target would answer to it."""
    def answers(at):
        try:
            bus.read_reg(at, profile.chip_id_register, profile)
            return True
        except MemsError:
            return False

    if answers(group):
        return [("SETGRPA", UNDETERMINED,
                 f"0x{group:02X} already answered before the command")]
    bus.try_call(bus.device.i3cDirectSETGRPA, address, group)
    got = answers(group)
    bus.try_call(bus.device.i3cDirectRSTGRPA, address)
    if got:
        return [("SETGRPA / RSTGRPA", SUPPORTED,
                 f"the target answered at group address 0x{group:02X}")]
    return [("SETGRPA / RSTGRPA", NOT_IMPLEMENTED,
             f"0x{group:02X} still does not answer after the command")]


def probe_new_address(bus, address, profile):
    """SETNEWDA, with the old address as the negative control."""
    new = 0x0A if address != 0x0A else 0x0C
    ok, response = bus.try_call(bus.device.i3cSETNEWDA, address, new)
    if not ok:
        return [("SETNEWDA", UNDETERMINED, str(response))], address

    def answers(at):
        try:
            value, _ = bus.read_reg(at, profile.chip_id_register, profile)
            return value & profile.chip_id_mask == profile.chip_id_expected
        except MemsError:
            return False

    moved, old_gone = answers(new), not answers(address)
    if moved and old_gone:
        return [("SETNEWDA", SUPPORTED,
                 f"moved 0x{address:02X} to 0x{new:02X} and the old address "
                 f"stopped answering")], new
    if moved:
        return [("SETNEWDA", UNDETERMINED,
                 f"0x{new:02X} answers but so does 0x{address:02X}")], new
    return [("SETNEWDA", NOT_IMPLEMENTED,
             f"0x{new:02X} does not answer")], address


def probe_hdr(bus, address, profile, bcr):
    """The BCR settles this. Attempting HDR does not."""
    out = []
    if bcr is None:
        out.append(("HDR modes", UNDETERMINED, "no BCR to read"))
    elif bcr & 0x20:
        out.append(("HDR modes", UNDETERMINED,
                    "BCR bit 5 set, so advanced capabilities are claimed"))
    else:
        out.append(("HDR modes", NOT_IMPLEMENTED,
                    "BCR bit 5 clear, so the target declares SDR only"))

    ok, response = bus.try_call(bus.device.i3cGETCAPS, address)
    caps = payload_of(response) if ok else []
    if caps:
        out.append(("GETHDRCAP (0x95)",
                    NOT_IMPLEMENTED if caps == [0] else UNDETERMINED,
                    f"answers {hex_bytes(caps)}"
                    + (", so no HDR modes" if caps == [0] else "")))

    ok, response = bus.try_call(bus.device.i3cControllerHdrDdrRead,
                                address, 0x80, 4)
    verdict = response.get("result") if ok else str(response)
    bus.try_call(bus.device.i3cControllerTriggerHdrExitPattern)
    intact = True
    try:
        bus.read_reg(address, profile.chip_id_register, profile)
    except MemsError:
        intact = False
    out.append(("HDR-DDR read attempt", UNDETERMINED,
                f"returned {verdict}, which is not diagnostic; "
                f"bus {'intact' if intact else 'DISTURBED'} afterwards"))
    return out


def probe_timing_exchange(bus, address):
    """SETXTIME changes a byte that GETXTIME reports, when it is implemented.

    The sub-commands select modes and the bits latch, so re-sending one the
    part is already in is correctly a no-op. Measured on the BMI323: from a
    GETXTIME of 03 00 0D 78, sub-command 0x3F moves byte 1 to 0x01 and then
    never moves it again, while 0xDF moves it to 0x03. So a probe that sends
    one sub-command can only prove support once per power cycle. Try several,
    and if none of them move the value say why rather than calling it no
    observable effect.
    """
    SUBCOMMANDS = (0x3F, 0xDF, 0x1F, 0x5F, 0x7F)
    ok_before, before = bus.try_call(bus.device.i3cGETXTIME, address)
    if not ok_before:
        return [("SETXTIME / GETXTIME", UNDETERMINED, "GETXTIME did not answer")]
    baseline = payload_of(before)
    current = baseline
    for subcommand in SUBCOMMANDS:
        bus.try_call(bus.device.i3cDirectSETXTIME, address, subcommand, [])
        ok_after, after = bus.try_call(bus.device.i3cGETXTIME, address)
        if not ok_after:
            continue
        value = payload_of(after)
        if value and value != current:
            return [("SETXTIME / GETXTIME", SUPPORTED,
                     f"{hex_bytes(current)} then {hex_bytes(value)} after "
                     f"SETXTIME 0x{subcommand:02X}")]
        current = value or current
    return [("SETXTIME / GETXTIME", UNDETERMINED,
             f"{hex_bytes(baseline)} unmoved by sub-commands "
             f"{', '.join(f'0x{s:02X}' for s in SUBCOMMANDS)}; the part may "
             f"already be in the state they select, which a power cycle would "
             f"reset")]


def probe_no_observable(bus, address):
    """Commands that complete but change nothing we can measure here."""
    from binhosupernova.commands.i3c.definitions import (
        TransferDirection, I3cTargetResetDefByte)
    out = []
    for label, method, args in (
            ("ENTAS0", bus.device.i3cDirectENTAS0, (address,)),
            ("ENTAS1", bus.device.i3cDirectENTAS1, (address,)),
            ("ENTAS2", bus.device.i3cDirectENTAS2, (address,)),
            ("ENTAS3", bus.device.i3cDirectENTAS3, (address,))):
        ok, response = bus.try_call(method, *args)
        out.append((label, UNDETERMINED,
                    f"returned {response.get('result') if ok else response}; "
                    f"proving an activity state needs a current measurement"))
    ok, response = bus.try_call(bus.device.i3cDirectRSTACT, address,
                                I3cTargetResetDefByte.NO_RESET,
                                TransferDirection.WRITE)
    out.append(("RSTACT", UNDETERMINED,
                f"returned {response.get('result') if ok else response}; "
                f"no observable effect from the host side"))
    ok, response = bus.try_call(bus.device.i3cGETMXDS, address)
    if ok:
        out.append(("GETMXDS", UNDETERMINED,
                    f"answers {hex_bytes(payload_of(response))}; a part that "
                    f"implements it reports real limits"))
    return out


def probe_control(bus, address, profile, table):
    """The control: the adapter must report a refusal when nothing answers.

    Without this, every SUCCESS above could be the adapter swallowing errors
    rather than the target accepting and discarding.
    """
    occupied = {entry["dynamic_address"] for entry in table} | {address}
    empty = next((a for a in range(0x0B, 0x70) if a not in occupied), None)
    if empty is None:
        return [("control: refusal reporting", UNDETERMINED,
                 "no free address to test against")]
    checks = []
    ok, _ = bus.try_call(bus.device.i3cGETPID, empty)
    checks.append(not ok)
    ok, _ = bus.try_call(bus.device.i3cDirectSETMWL, empty, 64)
    checks.append(not ok)
    try:
        bus.read_reg(empty, profile.chip_id_register, profile)
        checks.append(False)
    except MemsError:
        checks.append(True)
    if all(checks):
        return [("control: refusal reporting", SUPPORTED,
                 f"GETPID, SETMWL and a private read at the unoccupied address "
                 f"0x{empty:02X} were all refused, so a SUCCESS above means the "
                 f"target accepted the command")]
    return [("control: refusal reporting", UNDETERMINED,
             f"some commands to the unoccupied address 0x{empty:02X} did not "
             f"report a refusal, so the results above are weaker evidence")]


def probe_ibi(bus, address, profile, seconds=3.0):
    """Enable, count, decode, disable. The rate is checked against the ODR."""
    from binhosupernova.commands.i3c.definitions import ENEC, DISEC
    out = []
    bus.quiesce(address)
    bus.accept_ibis(address)
    try:
        profile.start_stream(bus, address, route_ibi=True)
    except MemsError as exc:
        return [("IBI", UNDETERMINED, f"could not set the part up: {exc}")]

    bus.drain_ibis()
    before = len(bus.collect_ibis(0.7))
    out.append(("IBI before ENEC",
                NOT_IMPLEMENTED if before == 0 else UNDETERMINED,
                f"{before} in 0.7 s"
                + ("" if before == 0 else ", so something enabled them early")))

    ok, response = bus.try_call(bus.device.i3cDirectENEC, address, [ENEC.ENINT])
    bus.drain_ibis()
    started = time.monotonic()
    got = bus.collect_ibis(seconds,
                           on_each=lambda _n: profile.clear_interrupt(bus, address))
    elapsed = time.monotonic() - started
    rate = len(got) / elapsed if elapsed else 0.0
    expected = profile.expected_ibi_rate()

    if got:
        detail = f"{len(got)} in {elapsed:.2f} s = {rate:.2f}/s"
        if expected:
            error = 100.0 * (rate - expected) / expected
            detail += f", against a configured {expected:g} Hz ({error:+.1f}%)"
        out.append(("ENEC and IBI delivery", SUPPORTED, detail))
        payloads = Counter(tuple(n.get("payload") or []) for n in got)
        first = payload_of(got[0])
        if first:
            bits = ", ".join(f"{k}={v}" for k, v in
                             profile.decode_mdb(first[0]).items())
            detail = f"0x{first[0]:02X}: {bits}"
            if len(first) > 1:
                detail += (f"; followed by {len(first) - 1} further byte(s) "
                           f"{hex_bytes(first[1:])}, not decoded here")
            out.append(("IBI mandatory data byte", SUPPORTED, detail))
        out.append(("IBI payloads seen", SUPPORTED, str(dict(payloads))))
    else:
        out.append(("ENEC and IBI delivery", UNDETERMINED,
                    f"no IBIs in {elapsed:.2f} s after "
                    f"{response.get('result') if ok else response}"))

    attempts = bus.stop_ibis(address)
    if attempts == 1:
        out.append(("DISEC stops them", SUPPORTED,
                    "the stream stopped after one DISEC"))
    elif attempts:
        out.append(("DISEC stops them", SUPPORTED,
                    f"the stream stopped, but only after {attempts} DISEC "
                    f"attempts; a single one is not always enough"))
    else:
        out.append(("DISEC stops them", UNDETERMINED,
                    "the stream was still arriving after four DISEC attempts"))

    hot_join = any(n.get("command", "").upper().find("HJ") >= 0
                   for n in got)
    out.append(("hot-join", UNDETERMINED if hot_join else NOT_IMPLEMENTED,
                "a hot-join notification arrived" if hot_join
                else "no hot-join notification at any point"))

    try:
        profile.stop_stream(bus, address)
    except MemsError:
        pass
    return out


def probe_reset(bus, address, profile):
    """RSTDAA, preceded by DISEC because the BMI323 datasheet asks for it."""
    from binhosupernova.commands.i3c.definitions import DISEC
    bus.try_call(bus.device.i3cDirectDISEC, address, [DISEC.DISINT])
    ok, response = bus.try_call(bus.device.i3cRSTDAA)
    if not ok:
        return [("RSTDAA", UNDETERMINED, str(response))]
    table = bus.table()
    addresses = [entry["dynamic_address"] for entry in table]
    gone = True
    try:
        bus.read_reg(address, profile.chip_id_register, profile)
        gone = False
    except MemsError:
        pass
    if gone and all(a == 0 for a in addresses):
        return [("RSTDAA", SUPPORTED,
                 f"the table dropped to {addresses} and 0x{address:02X} "
                 f"stopped answering")]
    return [("RSTDAA", UNDETERMINED, f"table now {addresses}")]


# --------------------------------------------------------------------------
# Commands
# --------------------------------------------------------------------------


def open_bus(args):
    bus = Bus(serial=args.serial, verbose=args.verbose)
    bus.open()
    bus.configure(voltage_mv=args.voltage, push_pull=args.push_pull,
                  open_drain=args.open_drain, drive=args.drive)
    return bus


def cmd_scan(args):
    with open_bus(args) as bus:
        table = bus.init_bus()
        if not table:
            print("no I3C target answered ENTDAA")
            return 1
        print(f"{len(table)} target(s) on the bus at "
              f"{args.voltage} mV, {args.push_pull}, {args.open_drain}\n")
        for entry in table:
            address = entry["dynamic_address"]
            pid = entry["pid"]
            device_id = (int.from_bytes(bytes(pid), "big") >> 16) & 0xFFFF
            guess = next((name for name, cls in PROFILES.items()
                          if cls.device_id_expected == device_id), None)
            print(f"  dynamic address 0x{address:02X}")
            print(f"    PID  {hex_bytes(pid)}   device id 0x{device_id:04X}")
            print(f"    BCR  0x{entry['bcr']:02X}   DCR 0x{entry['dcr']:02X}")
            print(f"    HDR  {'claimed' if entry['bcr'] & 0x20 else 'SDR only'}"
                  f"   IBI {'capable' if entry['bcr'] & 0x02 else 'not capable'}")
            print(f"    profile  {guess or 'none in this tool'}")
    return 0


def cmd_identify(args):
    profile = make_profile(args.device, latched=args.latched)
    with open_bus(args) as bus:
        address, entry = find_target(bus, profile)
        print(f"{profile.name}  ({profile.vendor}, {profile.kind})")
        print(f"  dynamic address 0x{address:02X}\n")

        ok, response = bus.try_call(bus.device.i3cGETPID, address)
        pid = payload_of(response)
        print(f"  PID {hex_bytes(pid)}")
        for label, value in decode_pid(pid):
            print(f"    {label:28s} {value}")

        ok, response = bus.try_call(bus.device.i3cGETBCR, address)
        bcr = (payload_of(response) or [0])[0]
        print(f"\n  BCR 0x{bcr:02X}")
        for label, value in decode_bcr(bcr):
            print(f"    {label:32s} {value}")

        ok, response = bus.try_call(bus.device.i3cGETDCR, address)
        dcr = (payload_of(response) or [0])[0]
        print(f"\n  DCR 0x{dcr:02X}"
              + ("   pressure sensor, per the MIPI DCR registry"
                 if dcr == 0x62 else ""))

        value, raw = bus.read_reg(address, profile.chip_id_register, profile)
        masked = value & profile.chip_id_mask
        print(f"\n  CHIP_ID register 0x{profile.chip_id_register:02X} "
              f"reads 0x{value:0{profile.data_width * 2}X}, "
              f"raw {hex_bytes(raw)}")
        print(f"    chip id field 0x{masked:02X}, expected "
              f"0x{profile.chip_id_expected:02X}: "
              f"{'match' if masked == profile.chip_id_expected else 'MISMATCH'}")
        if profile.chip_id_mask != 0xFF * profile.data_width:
            print(f"    compared under mask 0x{profile.chip_id_mask:04X}, "
                  f"because the register carries other fields")

        print(f"\n  register access: {profile.data_width}-byte data, "
              f"{profile.read_dummy} dummy byte(s) before a read payload")
        print(f"  observable: {profile.observable}")

        if args.registers:
            print("\n  registers")
            for register, name in profile.registers:
                try:
                    value, _ = bus.read_reg(address, register, profile)
                    print(f"    0x{register:02X} {name:16s} "
                          f"0x{value:0{profile.data_width * 2}X}")
                except MemsError as exc:
                    print(f"    0x{register:02X} {name:16s} {exc}")
    return 0


def cmd_read(args):
    profile = make_profile(args.device, latched=args.latched)
    with open_bus(args) as bus:
        address, _ = find_target(bus, profile)
        values, raw = bus.read_regs(address, args.register, args.count, profile)
        print(f"raw {hex_bytes(raw)}")
        for index, value in enumerate(values):
            print(f"  0x{args.register + index:02X}  "
                  f"0x{value:0{profile.data_width * 2}X}")
    return 0


def cmd_write(args):
    profile = make_profile(args.device, latched=args.latched)
    with open_bus(args) as bus:
        address, _ = find_target(bus, profile)
        before, _ = bus.read_reg(address, args.register, profile)
        bus.write_reg(address, args.register, args.value, profile)
        after, _ = bus.read_reg(address, args.register, profile)
        width = profile.data_width * 2
        print(f"0x{args.register:02X}: 0x{before:0{width}X} -> "
              f"wrote 0x{args.value:0{width}X} -> reads 0x{after:0{width}X}")
        if after != args.value:
            print("  the value did not take. Some registers only accept writes "
                  "in a particular mode, and writes made otherwise are lost.")
    return 0


def cmd_stream(args):
    profile = make_profile(args.device, latched=args.latched)
    with open_bus(args) as bus:
        address, _ = find_target(bus, profile)
        bus.quiesce(address)
        profile.start_stream(bus, address)
        print(f"{profile.name} at 0x{address:02X}, {args.seconds:g} s")
        print(f"  {profile.observable}\n")
        end = time.monotonic() + args.seconds
        try:
            while time.monotonic() < end:
                sample = profile.read_sample(bus, address)
                print("  " + "   ".join(
                    f"{label} {value:9.3f} {unit}"
                    for label, value, unit in sample))
                time.sleep(args.interval)
        except KeyboardInterrupt:
            print("\n  interrupted")
        finally:
            profile.stop_stream(bus, address)
    return 0


def cmd_ibi(args):
    from binhosupernova.commands.i3c.definitions import ENEC, DISEC
    profile = make_profile(args.device, latched=args.latched)
    with open_bus(args) as bus:
        address, _ = find_target(bus, profile)
        bus.quiesce(address)
        bus.accept_ibis(address)
        profile.start_stream(bus, address, route_ibi=True)
        print(f"{profile.name} at 0x{address:02X}: interrupt routed to the I3C "
              f"IBI, {profile.interrupt_mode()}")
        bus.drain_ibis()
        bus.try_call(bus.device.i3cDirectENEC, address, [ENEC.ENINT])

        shown = [0]

        def each(notification):
            profile.clear_interrupt(bus, address)
            if shown[0] < args.show:
                payload = payload_of(notification)
                bits = (", ".join(f"{k}={v}" for k, v in
                                  profile.decode_mdb(payload[0]).items())
                        if payload else "no payload")
                extra = (f"  extra {hex_bytes(payload[1:])}"
                         if len(payload) > 1 else "")
                print(f"  IBI from 0x{notification.get('target_address', 0):02X}"
                      f"  MDB {hex_bytes(payload[:1])}{extra}  {bits}")
                shown[0] += 1

        started = time.monotonic()
        try:
            got = bus.collect_ibis(args.seconds, on_each=each)
        except KeyboardInterrupt:
            got = []
        elapsed = time.monotonic() - started
        rate = len(got) / elapsed if elapsed else 0.0
        print(f"\n  {len(got)} interrupts in {elapsed:.2f} s = {rate:.2f}/s")
        expected = profile.expected_ibi_rate()
        if expected:
            print(f"  configured output data rate {expected:g} Hz "
                  f"({100.0 * (rate - expected) / expected:+.1f}%)")
        print("  the rate is comparable with the configured rate; individual "
              "arrival times are not, because USB coalesces them")

        attempts = bus.stop_ibis(address)
        if attempts is None:
            print("  warning: the interrupts were still arriving after four "
                  "DISEC attempts")
        elif attempts > 1:
            print(f"  the interrupts stopped after {attempts} DISEC attempts, "
                  f"not one")
        profile.stop_stream(bus, address)
    return 0


def cmd_reset(args):
    """Soft reset the target, which is the only way out of some latched modes."""
    profile = make_profile(args.device, latched=args.latched)
    with open_bus(args) as bus:
        address, _ = find_target(bus, profile)
        before, _ = bus.read_reg(address, profile.chip_id_register, profile)
        print(f"{profile.name} at 0x{address:02X}, chip id 0x"
              f"{before & profile.chip_id_mask:02X}")
        if not profile.soft_reset(bus, address):
            print("  this profile does not define a soft reset")
            return 1
        print(f"  wrote 0x{profile.soft_reset_value:02X} to register "
              f"0x{profile.command_register:02X}")
        table = bus.init_bus()
        if not table:
            print("  nothing enumerated after the reset")
            return 1
        address = table[0]["dynamic_address"]
        settle_after_enumeration(bus, address, profile)
        after, _ = bus.read_reg(address, profile.chip_id_register, profile)
        print(f"  re-enumerated at 0x{address:02X}, chip id 0x"
              f"{after & profile.chip_id_mask:02X}")
        print("  configuration registers are back to their reset values")
    return 0


def cmd_features(args):
    profile = make_profile(args.device, latched=args.latched)
    with open_bus(args) as bus:
        address, _ = find_target(bus, profile)
        table = bus.table()
        print(f"{profile.name} at 0x{address:02X}, adapter rates "
              f"{args.push_pull} / {args.open_drain} at {args.voltage} mV\n")

        rows = []
        identity, bcr = probe_identity(bus, address, profile)
        rows += identity
        rows += probe_control(bus, address, profile, table)
        rows += probe_length_limits(bus, address)
        rows += probe_group_address(bus, address, profile)
        rows += probe_hdr(bus, address, profile, bcr)
        rows += probe_no_observable(bus, address)
        rows += probe_ibi(bus, address, profile, seconds=args.seconds)
        # SETXTIME latches a mode that changes the IBI payload from one byte to
        # four and sets mandatory-byte bit 7, so it has to run after the
        # interrupt measurement rather than before it. Ordering probes by what
        # they leave behind matters as much as what they test.
        rows += probe_timing_exchange(bus, address)
        moved, address = probe_new_address(bus, address, profile)
        rows += moved
        rows += probe_reset(bus, address, profile)

        width = max(len(name) for name, _, _ in rows)
        print(f"  {'feature'.ljust(width)}  verdict           evidence")
        print(f"  {'-' * width}  ----------------  --------")
        for name, verdict, detail in rows:
            print(f"  {name.ljust(width)}  {verdict:16s}  {detail}")

        # Leave the part as it was found. Without this the battery is not
        # repeatable: the SETXTIME probe latches a mode that changes the IBI
        # payload, so a second run measures a different device than the first.
        # RSTDAA has just removed the dynamic address, so the bus has to be
        # brought back up before the part can be written to at all.
        restored = False
        table = bus.init_bus()
        if table:
            address = table[0]["dynamic_address"]
            settle_after_enumeration(bus, address, profile)
            try:
                restored = profile.soft_reset(bus, address)
            except MemsError as exc:
                print(f"  could not soft reset the part afterwards: {exc}")
        if restored:
            print("  the part was soft reset afterwards, to undo the modes "
                  "this battery latched\n")
        counts = Counter(verdict for _, verdict, _ in rows)
        print(f"\n  {counts[SUPPORTED]} supported, "
              f"{counts[NOT_IMPLEMENTED]} not implemented, "
              f"{counts[UNDETERMINED]} undetermined")
        print("  a verdict of undetermined means the command completed and "
              "nothing measurable changed.")
        print("  it is not a no: on this bus a target accepts and discards "
              "commands it does not implement,")
        print("  so a result code alone is never evidence of support.")

        bus.init_bus()
    return 0


def cmd_rates(args):
    """Find the highest rate a fixed read loop survives with zero errors."""
    profile = make_profile(args.device, latched=args.latched)
    push_pull_names = [
        "PUSH_PULL_2_5_MHZ_25_DC", "PUSH_PULL_3_125_MHZ_31_25_DC",
        "PUSH_PULL_5_MHZ_50_DC", "PUSH_PULL_6_25_MHZ_50_DC",
        "PUSH_PULL_7_5_MHZ_45_DC", "PUSH_PULL_10_MHZ_40_DC",
        "PUSH_PULL_12_5_MHZ_50_DC",
    ]
    open_drain_names = ["OPEN_DRAIN_100_KHZ", "OPEN_DRAIN_400_KHZ",
                        "OPEN_DRAIN_1_MHZ", "OPEN_DRAIN_2_MHZ",
                        "OPEN_DRAIN_4_17_MHZ"]

    print(f"method: {args.iterations} consecutive reads of the chip id "
          f"register must all return the expected value with no error.")
    print("a rate passes only if every iteration succeeds.")
    print("the two rates are not independent: the adapter rejects some "
          "combinations as an")
    print("invalid frequency pair, so the open-drain sweep is run against the "
          "fastest")
    print("push-pull rate that passed rather than against the default.\n")

    def sweep(group, names, settings_for):
        results = {}
        print(f"  {group}")
        for name in names:
            settings = settings_for(name)
            errors = bad = 0
            try:
                with Bus(serial=args.serial, verbose=args.verbose) as bus:
                    bus.configure(voltage_mv=args.voltage, **settings)
                    address, _ = find_target(bus, profile)
                    for _ in range(args.iterations):
                        try:
                            value, _ = bus.read_reg(
                                address, profile.chip_id_register, profile)
                            if value & profile.chip_id_mask != profile.chip_id_expected:
                                bad += 1
                        except MemsError:
                            errors += 1
            except MemsError as exc:
                message = str(exc)
                if "frequency pair" in message:
                    results[name] = "not tested"
                    print(f"    {name:32s} not tested: the adapter rejected this "
                          f"pairing")
                else:
                    results[name] = "not tested"
                    print(f"    {name:32s} not tested: {message[:60]}")
                continue
            if errors == 0 and bad == 0:
                results[name] = "pass"
                print(f"    {name:32s} pass")
            else:
                results[name] = "fail"
                print(f"    {name:32s} fail ({errors} errors, {bad} wrong values)")
        print()
        return results

    push_pull_results = sweep(
        "push-pull", push_pull_names,
        lambda name: dict(push_pull=name, open_drain="OPEN_DRAIN_100_KHZ",
                          drive=args.drive))
    best_push_pull = next((name for name in reversed(push_pull_names)
                           if push_pull_results.get(name) == "pass"),
                          args.push_pull)

    open_drain_results = sweep(
        "open-drain", open_drain_names,
        lambda name: dict(push_pull=best_push_pull, open_drain=name,
                          drive=args.drive))

    print(f"  method detail: push-pull swept at open drain 100 kHz; "
          f"open-drain swept at push-pull {best_push_pull}")
    for group, results in (("push-pull", push_pull_results),
                           ("open-drain", open_drain_results)):
        passed = [n for n, r in results.items() if r == "pass"]
        untested = [n for n, r in results.items() if r == "not tested"]
        if passed:
            print(f"  highest {group} rate passing {args.iterations} "
                  f"iterations: {passed[-1]}")
        else:
            print(f"  no {group} rate passed, which is a result as it stands")
        if untested:
            # Never let a bounded sweep read as full coverage.
            print(f"    not tested at all: {', '.join(untested)}")
    return 0


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------


def build_parser():
    parent = argparse.ArgumentParser(add_help=False)
    add_session_options(parent, visible=False)

    parser = argparse.ArgumentParser(
        prog="i3c_mems.py",
        description="Exercise the I3C target in a MEMS sensor from a Supernova.",
        epilog="Session options are accepted before or after the subcommand.")
    parser.add_argument("--version", action="version",
                        version=f"i3c_mems.py {TOOL_VERSION}")
    add_session_options(parser)
    subparsers = parser.add_subparsers(dest="command", metavar="command")

    def add(name, function, help_text):
        sub = subparsers.add_parser(name, parents=[parent], help=help_text)
        sub.set_defaults(function=function)
        return sub

    def add_device(sub, required=True):
        sub.add_argument("--device", required=required,
                         choices=sorted(PROFILES),
                         help="which device profile to use")
        sub.add_argument("--latched", action="store_true",
                         help="keep the part's latched interrupt default and "
                              "clear it from the host, instead of using pulsed "
                              "mode (BMP58x only)")

    add("scan", cmd_scan, "enumerate the bus and report what answered")

    sub = add("identify", cmd_identify, "decode a target's identity registers")
    add_device(sub)
    sub.add_argument("--registers", action="store_true",
                     help="also dump the profile's register list")

    sub = add("read", cmd_read, "read one or more registers")
    add_device(sub)
    sub.add_argument("register", type=lambda s: int(s, 0))
    sub.add_argument("--count", type=int, default=1)

    sub = add("write", cmd_write, "write a register and read it back")
    add_device(sub)
    sub.add_argument("register", type=lambda s: int(s, 0))
    sub.add_argument("value", type=lambda s: int(s, 0))

    sub = add("stream", cmd_stream, "print decoded samples by polling")
    add_device(sub)
    sub.add_argument("--seconds", type=float, default=5.0)
    sub.add_argument("--interval", type=float, default=0.2)

    sub = add("ibi", cmd_ibi, "route the sensor's interrupt onto the bus and "
                              "count what arrives")
    add_device(sub)
    sub.add_argument("--seconds", type=float, default=5.0)
    sub.add_argument("--show", type=int, default=5,
                     help="how many individual interrupts to print")

    sub = add("features", cmd_features,
              "establish what the target implements, from observable effects")
    add_device(sub)
    sub.add_argument("--seconds", type=float, default=3.0,
                     help="how long to count interrupts for")

    sub = add("rates", cmd_rates, "find the highest error-free bus rate")
    add_device(sub)
    sub.add_argument("--iterations", type=int, default=200)

    sub = add("reset", cmd_reset, "soft reset the part and re-enumerate")
    add_device(sub)

    return parser


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    if not getattr(args, "command", None):
        parser.print_help()
        return 2
    apply_session_defaults(args)
    try:
        return args.function(args)
    except MemsError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("\ninterrupted", file=sys.stderr)
        return 130


if __name__ == "__main__":
    sys.exit(main())
