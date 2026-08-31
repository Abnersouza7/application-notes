#!/usr/bin/env python3
"""Run exactly the commands an application note prints, and check them.

The notes in this series share one utility, so a change made for one note can
break another silently. AN0003 added SPI to the shared STM32 tool, which made
--bus a required option, which invalidated every command line printed in
AN0002. Nobody noticed until the archives were rebuilt.

This runs the documented command set against real hardware and checks two
different things, because a plain diff is the wrong instrument here:

  shape      the output, with volatile values masked, must match a saved
             golden. This catches renamed labels, dropped rows, reordered
             sections, changed argument names.

  invariants named regular expressions that must appear. These are the claims
             the note actually makes, so they are checked literally rather
             than through the mask: a chip id that matches, a CCC that is not
             implemented, an interrupt rate close to the configured one.

Masking without invariants would pass a run where every number went wrong.
Invariants without masking would fail on every run, because pressure and
sample counts are not reproducible. Both together are the useful test.

    python run_documented_commands.py --device bmi323
    python run_documented_commands.py --device bmi323 --update
    python run_documented_commands.py --device bmp585 --slow
    python run_documented_commands.py --list

One device is on the target board at a time, so --device is required unless
--list is given. Goldens live in documented/ beside this file.
"""

import argparse
import difflib
import os
import re
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
TOOL = os.path.join(HERE, "i3c_mems.py")
GOLDEN_DIR = os.path.join(HERE, "documented")

# --------------------------------------------------------------------------
# The documented command set
#
# This is the contract with the notes: every entry here is a command the note
# prints, and every command the note prints is here. Adding one to a note
# without adding it here is how AN0002 broke.
# --------------------------------------------------------------------------

# The measurement windows the commands below use. The rate invariants are
# derived from these, because the achievable tolerance depends on how many
# interrupts fit in the window.
FEATURES_IBI_WINDOW_S = 2.0
IBI_COMMAND_WINDOW_S = 3.0

def rate_within(configured_hz, window_s, clock_margin_pct=2.0):
    """Build a check that the reported rate error is within the achievable bound.

    A count of N interrupts over a window T at rate R is N = R*T plus or minus
    one, so the smallest error the measurement can resolve is 100/(R*T) per
    cent. At 10 Hz over 2 s that is 5 per cent, which is why a fixed 5 per cent
    tolerance failed the BMP581 while passing the BMI323: 21 interrupts in
    2.00 s is +5.0 per cent and is also exactly one interrupt. The tolerance
    has to come from the measurement, not from a round number.
    """
    quantization = 100.0 / (configured_hz * window_s)
    limit = quantization + clock_margin_pct

    def check(text):
        # Two commands phrase this differently: features says "against a
        # configured 10 Hz (+5.0%)" and ibi says "configured output data rate
        # 10 Hz (+3.3%)". Anchor on the rate and the parenthesised error only.
        found = re.findall(r"%g Hz \(([-+]\d+\.\d)%%\)" % configured_hz, text)
        if not found:
            return False
        return all(abs(float(value)) <= limit for value in found)

    check.description_suffix = (f"within {limit:.1f}% "
                                f"({quantization:.1f}% of that is one interrupt "
                                f"over a {window_s:g} s window)")
    return check


# Invariants shared by every device, expressed as (description, pattern).
# A pattern may be a regular expression, or a callable taking the output
# and returning True, for checks that are numeric rather than textual.
COMMON_FEATURE_INVARIANTS = [
    ("the refusal control passes",
     r"control: refusal reporting\s+supported"),
    ("MWL is measured, not assumed",
     r"SETMWL / GETMWL\s+(not implemented|supported)"),
    ("HDR is settled from the BCR",
     r"HDR modes\s+not implemented\s+BCR bit 5 clear"),
    ("the HDR attempt is called out as not diagnostic",
     r"HDR-DDR read attempt\s+undetermined.*not diagnostic"),
    ("group addressing is measured",
     r"SETGRPA / RSTGRPA\s+(not implemented|supported)"),
    ("SETNEWDA is proven with a negative control",
     r"SETNEWDA\s+supported\s+moved 0x[0-9A-F]{2} to 0x[0-9A-F]{2} and the old "
     r"address stopped answering"),
    ("RSTDAA is proven",
     r"RSTDAA\s+supported"),
    ("interrupts are delivered and counted",
     r"ENEC and IBI delivery\s+supported"),
    ("DISEC stops the stream",
     r"DISEC stops them\s+supported"),
    ("hot-join is reported as not observed",
     r"hot-join\s+not implemented"),
    ("the undetermined verdict is explained",
     r"a verdict of undetermined means the command completed and nothing "
     r"measurable changed"),
]


def device_commands(device):
    """Return a list of (name, argv, invariants, slow) for one device."""
    profile = DEVICE_FACTS[device]
    chip_register = profile["chip_id_register"]
    chip_id = profile["chip_id"]
    rate = profile["ibi_rate_hz"]

    commands = [
        ("scan", ["scan"], [
            ("one target enumerates", r"1 target\(s\) on the bus"),
            ("the PID device id is reported",
             r"device id 0x" + profile["device_id"]),
            ("the profile is recognized", r"profile\s+" + device),
            ("HDR is reported as SDR only", r"HDR\s+SDR only"),
        ], False),

        ("identify", ["identify", "--device", device], [
            ("the chip id matches",
             r"chip id field 0x%02X, expected 0x%02X: match" % (chip_id, chip_id)),
            ("the BCR decodes to SDR only",
             r"advanced capabilities \(bit 5\)\s+no, so SDR only"),
            ("the DCR is reported", r"DCR 0x%02X" % profile["dcr"]),
            ("the framing is stated",
             r"register access: %d-byte data, %d dummy byte" %
             (profile["data_width"], profile["read_dummy"])),
        ], False),

        ("identify-registers",
         ["identify", "--device", device, "--registers"], [
             ("the register dump appears", r"\n  registers\n"),
             ("the chip id register is listed",
              r"0x%02X CHIP_ID" % chip_register),
         ], False),

        ("read-chip-id",
         ["read", "--device", device, hex(chip_register)], [
             ("a raw payload is shown", r"^raw ([0-9A-F]{2} ?)+$"),
         ], False),

        ("stream", ["stream", "--device", device, "--seconds", "2",
                    "--interval", "0.4"], profile["stream_invariants"], False),

        ("ibi", ["ibi", "--device", device, "--seconds", "3", "--show", "2"], [
            # Match the mandatory byte alone. SETXTIME can latch a mode that
            # appends further bytes and sets bit 7, so anchoring on the whole
            # payload made this invariant depend on session history.
            ("interrupts arrive with the expected mandatory byte",
             r"MDB %02X(\s|$)" % profile["mdb"]),
            ("the rate is reported against the configured rate",
             r"configured output data rate %g Hz" % rate),
            ("the host timing caveat is printed",
             r"individual arrival times are not, because USB coalesces them"),
        ], False),

        ("features", ["features", "--device", device, "--seconds", "2"],
         COMMON_FEATURE_INVARIANTS + profile["feature_invariants"], False),

        ("rates", ["rates", "--device", device, "--iterations", "100"], [
            ("the method is stated",
             r"consecutive reads of the chip id register must all return"),
            ("the pairing constraint is explained", r"invalid frequency pair"),
            ("a push-pull ceiling is reported",
             r"highest push-pull rate passing 100 iterations: PUSH_PULL_"),
            ("an open-drain ceiling is reported",
             r"highest open-drain rate passing 100 iterations: OPEN_DRAIN_"),
        ], True),
    ]

    if profile.get("has_latched_mode"):
        commands.insert(6, (
            "ibi-latched",
            ["ibi", "--device", device, "--seconds", "3", "--show", "2",
             "--latched"],
            [("the latched mode is named",
              r"latched, re-armed by the host reading INT_STATUS"),
             ("interrupts still arrive in latched mode",
              r"MDB %02X" % profile["mdb"]),
             ("the rate is still close to the configured rate",
              rate_within(rate, IBI_COMMAND_WINDOW_S))],
            False))
    return commands


DEVICE_FACTS = {
    "bmi323": {
        "chip_id_register": 0x00,
        "chip_id": 0x43,
        "device_id": "1043",
        "dcr": 0xEF,
        "data_width": 2,
        "read_dummy": 2,
        "mdb": 0x02,
        "ibi_rate_hz": 50,
        "has_latched_mode": False,
        "stream_invariants": [
            ("acceleration is decoded in g", r"acc x\s+-?\d+\.\d+ g"),
            ("the magnitude is about 1 g at rest",
             r"\|acc\|\s+(0\.9[5-9]\d|1\.0[0-4]\d) g"),
        ],
        "feature_invariants": [
            # Whether this can be proven depends on session history: the
            # sub-commands latch, so once the part is in the state they
            # select nothing moves until a power cycle. Accept either
            # verdict, but require the reason to be stated either way.
            ("SETXTIME is either proven or explained",
             r"SETXTIME / GETXTIME\s+(supported\s+.* after SETXTIME"
             r"|undetermined\s+.*already be in the state they select)"),
            ("the interrupt rate matches the configured rate",
             rate_within(50, FEATURES_IBI_WINDOW_S)),
        ],
    },
    "bmp581": {
        "chip_id_register": 0x01,
        "chip_id": 0x50,
        "device_id": "1050",
        "dcr": 0x62,
        "data_width": 1,
        "read_dummy": 0,
        "mdb": 0x01,
        "ibi_rate_hz": 10,
        "has_latched_mode": True,
        "stream_invariants": [
            ("temperature is decoded", r"temperature\s+\d+\.\d+ C"),
            ("pressure is plausible for a room",
             r"pressure\s+(8[0-9]|9[0-9]|10[0-9])\d{3}\.\d+ Pa"),
        ],
        "feature_invariants": [
            ("the DCR identifies a pressure sensor", r"GETDCR\s+supported\s+0x62"),
            ("the interrupt rate matches the configured rate",
             rate_within(10, FEATURES_IBI_WINDOW_S)),
        ],
    },
    "bmp585": {
        "chip_id_register": 0x01,
        "chip_id": 0x51,
        "device_id": "1051",
        "dcr": 0x62,
        "data_width": 1,
        "read_dummy": 0,
        "mdb": 0x01,
        "ibi_rate_hz": 10,
        "has_latched_mode": True,
        "stream_invariants": [
            ("temperature is decoded", r"temperature\s+\d+\.\d+ C"),
            ("pressure is plausible for a room",
             r"pressure\s+(8[0-9]|9[0-9]|10[0-9])\d{3}\.\d+ Pa"),
        ],
        "feature_invariants": [
            ("the DCR identifies a pressure sensor", r"GETDCR\s+supported\s+0x62"),
            ("the interrupt rate matches the configured rate",
             rate_within(10, FEATURES_IBI_WINDOW_S)),
        ],
    },
}


# --------------------------------------------------------------------------
# Masking
#
# Everything that legitimately differs between two correct runs is replaced
# with a token, so the shape comparison is about structure rather than values.
# The values themselves are covered by the invariants.
# --------------------------------------------------------------------------

MASKS = (
    # elapsed times and rates
    (re.compile(r"\b\d+\.\d+ s\b"), "<seconds>"),
    (re.compile(r"\b\d+\.\d+/s\b"), "<rate>"),
    (re.compile(r"\([-+]\d+\.\d%\)"), "(<error>)"),
    # counted things
    (re.compile(r"\b\d+ (interrupts|IBIs|in)\b"), r"<n> \1"),
    (re.compile(r"\{\(\d+,\): \d+\}"), "{(<mdb>,): <n>}"),
    (re.compile(r"^\s*\d+ supported, \d+ not implemented, \d+ undetermined",
                re.M), "  <counts>"),
    # measured physical values
    (re.compile(r"-?\d+\.\d+ (g|C|Pa|hPa)\b"), r"<value> \1"),
    # anything that is a live register or payload value
    (re.compile(r"\braw ([0-9A-F]{2} ?)+"), "raw <bytes>"),
    (re.compile(r"payload_length': \d+"), "payload_length': <n>"),
    (re.compile(r"\bserial [0-9A-F]+\b"), "serial <serial>"),
    # The register dump is a snapshot of live state: acceleration, temperature,
    # status and the mode registers all read differently depending on what the
    # part happens to be doing. Mask the whole line, address and value, and do
    # it before any address rule below can reach into the value column.
    (re.compile(r"^(\s+)0x[0-9A-F]{2} (\w+\s+)0x[0-9A-F]{2,4}\s*$", re.M),
     r"\g<1><reg-addr> \g<2><reg-value>"),
    # Dynamic addresses move when SETNEWDA runs, so they are masked, but only
    # where an address is actually what is being printed. An earlier version
    # matched any 0x0X token and silently ate register values of 0x00 to 0x0F
    # while leaving 0x10 alone, which made the mask depend on the data.
    (re.compile(r"\b(at|from|to|address) 0x[0-9A-F]{2}\b"), r"\1 <addr>"),
    (re.compile(r"\b(dynamic|static)=0x[0-9A-F]{2}\b"), r"\1=<addr>"),
    (re.compile(r"\btable(:| dropped to) \[[^\]]*\]"), r"table\1 [<addrs>]"),
    (re.compile(r"\bfinal table \[[^\]]*\]"), "final table [<addrs>]"),
    (re.compile(r"\bre-enumerated at <addr>, chip id 0x[0-9A-F]{2}"),
     "re-enumerated at <addr>, chip id <chip>"),
    # The DISEC retry count varies, so the whole clause is normalized at once.
    # Two overlapping rules previously left the trailing explanation behind.
    (re.compile(r"the stream stopped(,? but only)? after (one|\d+) DISEC"
                r"( attempts)?(; a single one is not always enough)?"),
     "the stream stopped after <n> DISEC attempts"),
    # Which SETXTIME sub-command moves the value depends on which mode bits
    # are already latched, so the evidence bytes are masked and the invariant
    # checks the shape of the claim instead.
    (re.compile(r"(SETXTIME / GETXTIME\s+supported\s+)"
                r"([0-9A-F ]+) then ([0-9A-F ]+) after SETXTIME 0x[0-9A-F]{2}"),
     r"\1<before> then <after> after SETXTIME <subcommand>"),
)


def mask_fingerprint():
    """A short hash of the mask set, stamped into every golden.

    A golden captured under a different set of masks is not comparable with a
    run under the current set, and the resulting diff looks like a regression
    in the tool when it is nothing of the kind. Only the device that is in the
    socket can have its goldens regenerated, so a mask change necessarily
    leaves the other devices' goldens stale. Stamping the fingerprint lets the
    harness say so plainly instead of printing a misleading diff.
    """
    import hashlib
    material = "\n".join(f"{pattern.pattern}=>{replacement}"
                         for pattern, replacement in MASKS)
    return hashlib.sha256(material.encode()).hexdigest()[:12]


FINGERPRINT_PREFIX = "# mask-fingerprint: "


def mask(text):
    for pattern, replacement in MASKS:
        text = pattern.sub(replacement, text)
    # trailing whitespace differences are never interesting
    body = "\n".join(line.rstrip() for line in text.splitlines()) + "\n"
    return f"{FINGERPRINT_PREFIX}{mask_fingerprint()}\n{body}"


def split_fingerprint(stored):
    """Return (fingerprint, body) for a golden file's contents."""
    if stored.startswith(FINGERPRINT_PREFIX):
        first, _, rest = stored.partition("\n")
        return first[len(FINGERPRINT_PREFIX):].strip(), rest
    return None, stored


# --------------------------------------------------------------------------
# Running
# --------------------------------------------------------------------------


def run_one(argv, timeout=240):
    started = time.monotonic()
    completed = subprocess.run(
        [sys.executable, TOOL] + argv,
        capture_output=True, text=True, timeout=timeout,
        env={**os.environ, "PYTHONIOENCODING": "utf-8"})
    elapsed = time.monotonic() - started
    return completed.returncode, completed.stdout + completed.stderr, elapsed


def golden_path(device, name):
    return os.path.join(GOLDEN_DIR, f"{device}-{name}.txt")


def check_invariants(text, invariants):
    missing = []
    for description, pattern in invariants:
        if callable(pattern):
            if not pattern(text):
                missing.append((description, pattern))
            continue
        if not re.search(pattern, text, re.M):
            missing.append((description, pattern))
    return missing


def main():
    parser = argparse.ArgumentParser(
        description="Run the documented command set and check it.")
    parser.add_argument("--device", choices=sorted(DEVICE_FACTS))
    parser.add_argument("--update", action="store_true",
                        help="rewrite the goldens from this run")
    parser.add_argument("--slow", action="store_true",
                        help="include the rate sweep, which takes minutes")
    parser.add_argument("--list", action="store_true",
                        help="print the documented command set and exit")
    parser.add_argument("--only", help="run only commands whose name contains this")
    args = parser.parse_args()

    if args.list:
        for device in sorted(DEVICE_FACTS):
            print(f"{device}")
            for name, argv, invariants, slow in device_commands(device):
                marker = "  (slow)" if slow else ""
                print(f"  {name:20s} i3c_mems.py {' '.join(argv)}{marker}")
                for description, _ in invariants:
                    print(f"    - {description}")
        return 0

    if not args.device:
        parser.error("--device is required (one part is on the board at a time)")

    os.makedirs(GOLDEN_DIR, exist_ok=True)
    commands = device_commands(args.device)
    if args.only:
        commands = [c for c in commands if args.only in c[0]]
    if not args.slow:
        commands = [c for c in commands if not c[3]]

    # Confirm the requested part is the one in the socket. Without this, every
    # identity invariant fails one by one and the output reads like a broken
    # tool rather than the wrong board being plugged in.
    code, scan_output, _ = run_one(["scan"])
    if code != 0:
        print(f"  could not enumerate the bus:\n{scan_output.strip()}")
        return 1
    wanted = DEVICE_FACTS[args.device]["device_id"]
    if f"device id 0x{wanted}" not in scan_output:
        found = re.findall(r"device id 0x([0-9A-F]{4})", scan_output)
        names = {facts["device_id"]: name
                 for name, facts in DEVICE_FACTS.items()}
        installed = ", ".join(f"0x{value} ({names.get(value, 'unknown')})"
                              for value in found) or "nothing"
        print(f"  {args.device} is not on the target board.")
        print(f"    asked for device id 0x{wanted}, the bus has {installed}")
        print(f"    one part is in the socket at a time, so install "
              f"{args.device} or pass --device for what is fitted")
        return 1

    print(f"{args.device}: {len(commands)} documented command(s)"
          f"{' (updating goldens)' if args.update else ''}\n")

    failures = []
    for name, argv, invariants, _slow in commands:
        code, output, elapsed = run_one(argv)
        masked = mask(output)
        path = golden_path(args.device, name)
        problems = []

        if code != 0:
            problems.append(f"exit code {code}")

        missing = check_invariants(output, invariants)
        for description, pattern in missing:
            problems.append(f"invariant not met: {description}")

        if args.update:
            with open(path, "w", encoding="utf-8", newline="\n") as handle:
                handle.write(masked)
            status = "written"
        elif not os.path.exists(path):
            problems.append("no golden on disk; run with --update first")
            status = "no golden"
        else:
            with open(path, encoding="utf-8") as handle:
                expected = handle.read()
            stored_print, _ = split_fingerprint(expected)
            if stored_print != mask_fingerprint():
                # Not a regression: the mask set moved on since this golden was
                # captured, and only the installed device can be regenerated.
                problems.append(
                    f"golden is stale: captured under mask set "
                    f"{stored_print or 'unstamped'}, current set is "
                    f"{mask_fingerprint()}. Install this device and rerun with "
                    f"--update")
                status = "stale"
            elif expected != masked:
                problems.append("shape differs from the golden")
                status = "differs"
            else:
                status = "matches"

        mark = "ok  " if not problems else "FAIL"
        print(f"  {mark}  {name:20s} {elapsed:5.1f} s  {status}"
              f"  {len(invariants) - len(missing)}/{len(invariants)} invariants")
        for problem in problems:
            print(f"          {problem}")
        if problems and "shape differs from the golden" in problems:
            with open(path, encoding="utf-8") as handle:
                expected = handle.read()
            diff = list(difflib.unified_diff(
                expected.splitlines(), masked.splitlines(),
                fromfile="golden", tofile="this run", lineterm="", n=1))
            for line in diff[:24]:
                print(f"          {line}")
            if len(diff) > 24:
                print(f"          ... {len(diff) - 24} more diff lines")
        if problems:
            failures.append(name)

    print()
    if args.update:
        print(f"  goldens written for {len(commands)} command(s)")
        print("  review the diff before committing: a golden is only as good "
              "as the run that made it")
        return 0
    if failures:
        print(f"  FAILED: {', '.join(failures)}")
        return 1
    print(f"  all {len(commands)} documented command(s) pass")
    if not args.slow:
        print("  the rate sweep was skipped; add --slow to include it")
    return 0


if __name__ == "__main__":
    sys.exit(main())
