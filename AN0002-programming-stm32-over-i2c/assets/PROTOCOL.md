# STM32 I2C System Bootloader Protocol

Wire-level notes for the I2C interface of the STM32 system bootloader, as
observed on an STM32H503 (NUCLEO-H503RB). ST documents the protocol in AN4221
and the per-part interface list in AN2606; this file records what was actually
seen on the bus, including the parts that differ from the obvious reading.

Everything marked **verified** was exercised on hardware with a Binho Pulsar
and a Binho Supernova. Everything else is marked.

## Constants

| Name | Value | Note |
|---|---|---|
| Target address | `0x67` | 7-bit. Per-part; AN2606 gives it as `0b1100111x` |
| ACK | `0x79` | |
| NACK | `0x1F` | |
| BUSY | `0x76` | Defined by ST; see "What is never seen" below |
| Flash base | `0x08000000` | |
| Max transfer | 256 bytes | Read Memory and Write Memory |

The bootloader's I2C pins are **not** the same as its I3C pins. On the
STM32H503 the bootloader uses I2C2 on `PB3` (SCL) and `PB4` (SDA), per AN2606
Table 105. Neither the pins nor the address are guessable; check AN2606 for the
part in hand.

## Command opcodes

Reported by the STM32H503 in response to Get. **Verified.**

| Opcode | Command |
|---|---|
| `0x00` | Get |
| `0x01` | Get Version |
| `0x02` | Get ID |
| `0x11` | Read Memory |
| `0x21` | Go |
| `0x31` | Write Memory |
| `0x44` | Extended Erase |
| `0x50` | Special Command |
| `0x63` | Write Protect |
| `0x73` | Write Unprotect |
| `0x32` | No-Stretch Write Memory |
| `0x45` | No-Stretch Erase |
| `0x64` | No-Stretch Write Protect |
| `0x74` | No-Stretch Write Unprotect |
| `0xA1` | Checksum |

Fifteen commands. Note what is **absent**: Readout Protect (`0x82`) and Readout
Unprotect (`0x92`). ST's Open Bootloader reference implementation includes them,
but the ROM bootloader on this part does not offer them over I2C. The reference
implementation and the ROM are not the same program, and the ROM is the
authority. Trusting the reference here produces a command the target rejects.

## Transport model

Unlike the I3C interface, which reports status through an in-band interrupt,
I2C uses the shape everyone expects: **write the command, then read one status
byte.** There is no synchronisation byte. The session begins with the first
command; there is no equivalent of the I3C `0x5A`.

Each read is its own I2C transaction, addressed to `0x67`.

## Command framing

### Opcode

Every command is a two-byte write: the opcode and its complement.

```
write 0x67  [op, op ^ 0xFF]
read  0x67  1 byte           -> 0x79 ACK, or 0x1F NACK
```

### Address

Read Memory, Write Memory and Go take a four-byte big-endian address followed
by a one-byte XOR checksum of those four bytes.

```
write 0x67  [a31_24, a23_16, a15_8, a7_0, xor]
read  0x67  1 byte           -> ACK
```

### Get (0x00) -- read it in two passes

The reply is a single frame of `2 + N` bytes: a count, a version, then `N`
opcodes. The host cannot know `N` in advance, and **reading past the end of the
frame times the bus out.**

The way through is two passes. Read two bytes to learn `N`, take the trailing
ACK, then reissue Get and read the whole `2 + N` frame. Reading short is
harmless: the target abandons the rest of the frame and moves on to its closing
ACK. Reading long is not. **Verified.**

```
cmd 0x00
read 2            -> [N, version]
read 1            -> ACK
cmd 0x00
read 2 + N        -> [N, version, op0 .. opN-1]
read 1            -> ACK
```

### Get Version (0x01), Get ID (0x02)

```
cmd 0x01 ; read 1 -> version ; read 1 -> ACK
cmd 0x02 ; read 3 -> [count, id_msb, id_lsb] ; read 1 -> ACK
```

An STM32H503 reports version `0x20` (protocol 2.0) and device ID `0x474`.

### Write Memory (0x31 / 0x32)

```
cmd    ; address+xor ; read 1 -> ACK
write  [len-1, data[0..len-1], xor]
read 1 -> ACK          (after the write completes)
```

`len-1` is the byte count minus one, so a 256-byte block sends `0xFF`. The XOR
covers the length byte and every data byte.

### Read Memory (0x11)

```
cmd 0x11 ; address+xor ; read 1 -> ACK
write    [len-1, (len-1) ^ 0xFF]
read 1   -> ACK
read len -> data
```

Unlike the I3C interface, each block needs its own Read Memory command; the
address does not auto-increment across commands. Reads are capped at 256 bytes.

### Extended Erase (0x44 / 0x45) -- the page count is off by one

```
cmd    ; read 1 -> ACK
write  [count_msb, count_lsb, page0_msb, page0_lsb, ..., xor]
read 1 -> ACK          (after the erase completes)
```

The count field is **the number of pages minus one**, not the number of pages.
Erasing a single page 0 sends `00 00 00 00` plus checksum. This differs from the
I3C interface on the same part, where the field is the count itself. Sending the
plain count over I2C erases one page too many. **Verified.**

### What this part accepts, and from which controller

Measured on an STM32H503, none of it stated in AN4221. The bootloader is the
same program in both columns; only the host controller changed.

| Request | Pulsar, dedicated I2C | Supernova, via I3C port |
|---|---|---|
| `0xFFFF`, mass erase | accepted, 0.05 s | **rejected** |
| `0xFFFE`, bank 1 erase | accepted | accepted; clears 128 KB in 0.02 s |
| `0xFFFD`, bank 2 erase | accepted | **rejected** |
| page list of 1 or 2 | accepted | accepted; clears exactly those pages |
| page list of 3 or 4 | accepted | **rejected** |

So these are not limits of the bootloader. They appear only when legacy I2C is
driven from the I3C controller, which is the same distinction the clock
stretching section draws, showing up in a different place.

On the constrained path, erasing everything means the bank one value rather
than a mass erase, and a page list must be split into pairs, so a 128 KB erase
becomes eight commands taking about 0.19 s.

All of it was found by trying and checking coverage page by page, not by
reading it anywhere.

On this part a flash page is 8 KB, determined empirically.

## Signalling that the target is busy

This is the one part worth reading carefully.

While the bootloader erases or programs, it cannot answer. It signals this in
one of two ways depending on which command variant was used:

- **Plain commands** (`0x31`, `0x44`) hold SCL low -- ordinary I2C clock
  stretching -- until the operation finishes.
- **No-stretch commands** (`0x32`, `0x45`) do not stretch. Instead the busy
  target **stops acknowledging its own address**. The host polls by retrying the
  transaction until the address is acknowledged again.

A NACKed address means nothing was delivered, so retrying is always safe. This
is what the reference utility does, and it is why the no-stretch variants are
the default there.

### What is never seen

ST defines a BUSY byte, `0x76`. On this part it **is never transmitted.** Every
rejection observed came back as `0x1F`, and a busy target refuses its address
rather than answering with a status byte. The same was true of the I3C
interface in AN0001. Code that waits for `0x76` will wait forever; code that
polls a NACKed address works.

## Clock stretching and I3C controllers

**Verified, and worth knowing before choosing a host adapter.**

Immediately after a command, and before it settles into NACKing its address,
the target holds SCL low briefly. A conventional I2C controller rides this out.

An I3C controller in legacy I2C mode does not have to: I3C targets never
stretch the clock, so an I3C peripheral may enforce a bus timeout and abort the
transfer instead of waiting. On a Binho Supernova driving the bus from its I3C
port, this appears as `BUS_TIMEOUT` on the status read after the erase
page-list write, roughly a quarter of the time.

The fix is not a longer timeout. Wait about 2 ms before the status read, which
skips the stretch window entirely and lets the target present a clean NACK that
the ordinary busy-poll already handles. With that settle in place the timeout
does not occur at all. The reference utility applies it automatically, and only
on that path; a Pulsar driving its dedicated I2C port needs nothing.

## Bootloader entry

Hold `BOOT0` high, pulse `NRST` low, release. Release `NRST` by switching the
pin to an input rather than driving it high, so the on-board reset circuit is
not fought.

To return to the application, drive `BOOT0` low and pulse `NRST` again. Where
those pins are not wired, the Go command jumps to the image without a reset.

## Verified on hardware

Against an STM32H503 on a NUCLEO-H503RB:

- Full erase, program, read-back and verify cycles at 100 kHz, 400 kHz and 1 MHz
- A 128 KB read at all three speeds, byte-identical across them
- Repeated alternating-image cycles from a Pulsar on its dedicated I2C port,
  and from a Supernova driving I2C from its I3C port

## Still unconfirmed

- Behaviour with readout protection active, since the ROM does not offer the
  commands over this interface
- The Special Command (`0x50`) and Checksum (`0xA1`) opcodes, which the target
  reports but which the reference utility does not use
- Whether page size is uniform across the whole H5 family; 8 KB was measured on
  the H503 only
