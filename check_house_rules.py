#!/usr/bin/env python3
"""Check the APPNOTE-PLAYBOOK house rules that the real build enforces.

The build and publish tooling lives in Binho's internal content repo, so a note
written here cannot be built before review. This checks the rules that are
checkable from the source alone, so the review is not the only gate:

  - front matter carries every required key, and doc_id matches its own path
  - assets/<tool>.py is a byte copy of _shared/tools/<tool>.py
  - no em-dashes, LF line endings only
  - every markdown table is captioned with a "Table: ..." line
  - section headings are numbered without gaps

Run from the repository root:  python3 check_house_rules.py [AN0008 AN0009 ...]
Exits non-zero if anything fails, so it can gate a commit.
"""
import hashlib
import pathlib
import re
import sys

REQUIRED_KEYS = ("doc_id", "title", "rev", "date", "kind", "company", "website",
                 "keywords", "trademarks", "abstract")

failures = []
checks = 0


def fail(note, msg):
    failures.append(f"{note}: {msg}")


def check_front_matter(note, text, path):
    global checks
    checks += 1
    if not text.startswith("---\n"):
        fail(note, "no front matter block")
        return
    fm = text.split("---\n", 2)[1]
    for key in REQUIRED_KEYS:
        if not re.search(rf"^{key}:", fm, re.M):
            fail(note, f"front matter missing '{key}:'")
    m = re.search(r"^doc_id:\s*(\S+)", fm, re.M)
    if m and m.group(1) != note:
        fail(note, f"doc_id is {m.group(1)} but the file is {path}")


def check_characters(note, text):
    global checks
    checks += 1
    if "—" in text:
        n = text.count("—")
        fail(note, f"{n} em-dash(es); the playbook forbids them")
    if "\r" in text:
        fail(note, "CRLF line endings; write LF only")


def check_tables(note, text):
    """Every table needs a 'Table: <caption>' line, per the playbook."""
    global checks
    checks += 1
    lines = text.split("\n")
    in_fence = False
    for i, line in enumerate(lines):
        if line.startswith("```"):
            in_fence = not in_fence
        if in_fence:
            continue
        # A table is a header row followed by a |---|---| separator.
        if not re.match(r"^\s*\|?[\s-]*\|[\s|:-]*$", line):
            continue
        if "|" not in line or "-" not in line:
            continue
        if i == 0 or "|" not in lines[i - 1]:
            continue
        # Walk back over the header row and any blank line to find the caption.
        j = i - 2
        while j >= 0 and not lines[j].strip():
            j -= 1
        if j < 0 or not lines[j].strip().startswith("Table:"):
            fail(note, f"table at line {i + 1} has no 'Table:' caption")


def check_sections(note, text):
    """Numbered headings must run 1, 2, 3 with no gaps or repeats."""
    global checks
    checks += 1
    seen = []
    for line in text.split("\n"):
        m = re.match(r"^##\s+(\d+)\s+\S", line)
        if m:
            seen.append(int(m.group(1)))
    if not seen:
        return
    expected = list(range(1, len(seen) + 1))
    if seen != expected:
        fail(note, f"section numbers are {seen}, expected {expected}")


def check_asset_copies(root, note, folder):
    global checks
    shared = root / "_shared" / "tools"
    assets = folder / "assets"
    if not assets.is_dir():
        return
    for tool in sorted(assets.glob("*.py")):
        canonical = shared / tool.name
        if not canonical.exists():
            # A note-local helper with no shared counterpart is fine; the
            # byte-copy rule only governs the shared series utility.
            continue
        checks += 1
        a = hashlib.md5(tool.read_bytes()).hexdigest()
        b = hashlib.md5(canonical.read_bytes()).hexdigest()
        if a != b:
            fail(note, f"assets/{tool.name} is not a byte copy of _shared/tools/{tool.name}")


def main(argv):
    root = pathlib.Path(__file__).resolve().parent
    wanted = set(argv[1:])
    folders = sorted(p for p in root.glob("AN[0-9][0-9][0-9][0-9]-*") if p.is_dir())
    if wanted:
        folders = [f for f in folders if f.name.split("-")[0] in wanted]
    if not folders:
        print("no notes matched")
        return 2

    for folder in folders:
        note = folder.name.split("-")[0]
        md = folder / f"{note}.md"
        if not md.exists():
            fail(note, f"expected {md.name} in {folder.name}")
            continue
        text = md.read_text(encoding="utf-8")
        check_front_matter(note, text, md.name)
        check_characters(note, text)
        check_tables(note, text)
        check_sections(note, text)
        check_asset_copies(root, note, folder)

    print(f"checked {len(folders)} notes, {checks} checks")
    for f in failures:
        print(f"  FAIL {f}")
    print("OK" if not failures else f"{len(failures)} failure(s)")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
