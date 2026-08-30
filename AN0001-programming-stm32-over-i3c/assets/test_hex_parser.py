#!/usr/bin/env python3
"""Tests for the Intel HEX parser in stm32_flash.

These cover the only part of the tool that can be tested without hardware.
Run with:  python test_hex_parser.py
"""

import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from stm32_flash import load_intel_hex, _merge_segments  # noqa: E402

EOF_RECORD = ":00000001FF"

failures = 0


def record(count, addr, rectype, payload):
    body = [count, (addr >> 8) & 0xFF, addr & 0xFF, rectype] + list(payload)
    body.append((-sum(body)) & 0xFF)
    return ":" + "".join("%02X" % b for b in body)


def data_record(addr, payload):
    return record(len(payload), addr, 0x00, payload)


def linear_record(upper):
    return record(2, 0, 0x04, [(upper >> 8) & 0xFF, upper & 0xFF])


def report(name, passed, detail=""):
    global failures
    if not passed:
        failures += 1
    print(f"  {'PASS' if passed else 'FAIL'}  {name}{'  ' + detail if detail else ''}")


def check(name, lines, expect=None, expect_error=None):
    path = os.path.join(tempfile.gettempdir(), "stm32_flash_hextest.hex")
    with open(path, "w") as fh:
        fh.write("\n".join(lines) + "\n")
    try:
        segments = load_intel_hex(path)
    except ValueError as exc:
        message = str(exc).split(": ", 1)[-1]
        report(name, bool(expect_error) and expect_error in message, f"-> {message}")
        return
    finally:
        try:
            os.unlink(path)
        except OSError:
            pass
    if expect_error:
        report(name, False, f"expected error {expect_error!r}, parsed {segments}")
    else:
        report(name, segments == expect, f"{[(hex(a), d.hex()) for a, d in segments]}")


def main():
    print("Intel HEX parsing:")
    check("simple data record",
          [data_record(0, b"\xde\xad\xbe\xef"), EOF_RECORD],
          expect=[(0, b"\xde\xad\xbe\xef")])
    check("extended linear address applies",
          [linear_record(0x0800), data_record(0, b"\xde\xad\xbe\xef"), EOF_RECORD],
          expect=[(0x08000000, b"\xde\xad\xbe\xef")])
    check("contiguous records merge",
          [linear_record(0x0800), data_record(0, b"\x12\x34"),
           data_record(2, b"\x56\x78"), EOF_RECORD],
          expect=[(0x08000000, b"\x12\x34\x56\x78")])
    check("a gap keeps segments separate",
          [linear_record(0x0800), data_record(0, b"\x12\x34"),
           data_record(0x10, b"\x56\x78"), EOF_RECORD],
          expect=[(0x08000000, b"\x12\x34"), (0x08000010, b"\x56\x78")])
    check("out-of-order records are sorted",
          [linear_record(0x0800), data_record(0x10, b"\x56\x78"),
           data_record(0, b"\x12\x34"), EOF_RECORD],
          expect=[(0x08000000, b"\x12\x34"), (0x08000010, b"\x56\x78")])
    check("records after EOF are ignored",
          [data_record(0, b"\x12\x34"), EOF_RECORD, data_record(0x20, b"\xff\xff")],
          expect=[(0, b"\x12\x34")])
    check("start-address record is skipped",
          [data_record(0, b"\x12\x34"),
           record(4, 0, 0x05, [0x08, 0x00, 0x0A, 0x79]), EOF_RECORD],
          expect=[(0, b"\x12\x34")])

    print("\nMalformed input is rejected cleanly:")
    check("truncated record", [":10000000FFFFFFFF"],
          expect_error="does not match record")
    check("bad checksum", [":04000000DEADBEEF00"],
          expect_error="checksum mismatch")
    check("missing leading colon", ["04000000DEADBEEF35"],
          expect_error="does not start")
    check("odd-length hex", [":04000000DEADBEEF3"],
          expect_error="invalid hex")
    check("no data records", [""],
          expect_error="no data records")
    check("unsupported record type", [record(4, 0, 0x0A, b"\xde\xad\xbe\xef")],
          expect_error="unsupported record type")

    print("\nSegment merging:")
    for name, given, want in [
        ("sorts by address", [(0x10, b"cd"), (0x00, b"ab")],
         [(0x00, b"ab"), (0x10, b"cd")]),
        ("merges adjacent segments", [(0x00, b"ab"), (0x02, b"cd")],
         [(0x00, b"abcd")]),
        ("handles empty input", [], []),
    ]:
        report(name, _merge_segments(list(given)) == want)

    print()
    if failures:
        print(f"{failures} test(s) failed")
        return 1
    print("all tests passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
