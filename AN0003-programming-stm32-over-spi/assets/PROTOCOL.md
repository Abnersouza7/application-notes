# STM32 SPI System Bootloader Protocol

Wire-level notes for the SPI interface of the STM32 system bootloader, as
observed on an STM32H503 (NUCLEO-H503RB). ST documents the protocol in AN4286
and the per-part interface list in AN2606; this file records what was actually
seen on the bus, including the parts that differ from the obvious reading.

Everything marked **verified** was exercised on hardware with a Binho Pulsar, and the
protocol behaviour was reproduced with a Binho Supernova on the same wiring.

## Constants

| Name | Value | Note |
|---|---|---|
| Sync | `0x5A` | Prefixes **every** command, not only the first |
| Busy | `0xA5` | The target's underrun pattern; also a valid data byte |
| Dummy | `0x00` | What the host clocks when it only wants to read |
| ACK | `0x79` | |
| NACK | `0x1F` | |
| Flash base | `0x08000000` | |
| Max transfer | 256 bytes | Read Memory and Write Memory |

Bus configuration: **mode 0** (CPOL 0, CPHA 0), MSB first, 8-bit words. The
STM32 is the SPI target. Clock rate is a host choice; see "Throughput" below
for why raising it buys less than expected.

The bootloader's SPI pins are fixed in ROM and differ per part. On the
STM32H503, AN2606 offers three instances and no alternates:

| Instance | MOSI | MISO | SCK | NSS |
|---|---|---|---|---|
| SPI1 | `PA7` | `PA0` | `PA8` | `PB8` |
| SPI2 | `PB1` | `PB14` | `PB10` | `PB12` |
| SPI3 | `PC12` | `PC11` | `PC10` | `PD2` |

All four lines are configured push-pull with **no pull**, so NSS must be driven
and never left floating. Note these are not the pins a board routes to an
Arduino-style SPI header: on the NUCLEO-H503RB that header carries SPI1 on
`PA5`/`PA6`/`PA7`/`PC9`, which is the application mapping, not the ROM's.

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

Ten commands, and the list is **identical to the one this part offers over
I3C**. It is five short of what the same part offers over I2C, which adds the
four no-stretch variants and Checksum (`0xA1`). Neither Readout Protect
(`0x82`) nor Readout Unprotect (`0x92`) is offered on any of the three buses,
whatever ST's Open Bootloader sources suggest.

## Session start

The host clocks `0x5A`. The target replies with a sync byte and then an
acknowledgement, and the session is open. There is no separate handshake.

## Command framing

Every command, including the first, is prefixed with the sync byte:

```
host   5A <op> <~op>
target ... 79            (after some busy padding)
host   79                (release, see below)
```

Commands taking an address then send four bytes big-endian followed by a
one-byte XOR of those four, and take another acknowledgement.

## The three things that will catch you

### 1. The host has to acknowledge the acknowledgement

After sending its ACK the target **blocks until the host sends `0x79` back**.
A host that reads the ACK and moves on leaves the target waiting forever, and
nothing on the bus indicates why. **Verified.**

### 2. The busy byte is also a data byte

`0xA5` is the SPI peripheral's underrun pattern, emitted whenever the target
has nothing queued. It is not a status the bootloader chooses to send, and it
is indistinguishable from flash contents that happen to be `0xA5`.

This rules out the obvious implementation. A host that skips `0xA5` looking for
the start of a reply works perfectly on Get and Get ID, and then silently
corrupts any flash read whose data begins with `0xA5`, or hangs forever on a
page that is entirely `0xA5`. Both were reproduced deliberately.

The framing that works is positional rather than searched:

1. Stop clocking as soon as the ACK is seen.
2. Let the target load its first reply byte.
3. Clock exactly the number of bytes expected, filtering nothing.

The target cannot send without being clocked, so pausing leaves the reply
waiting in its transmit register. Pausing mid-reply is equally safe: the target
queues the next byte and waits, and never inserts padding into a reply it has
already started.

### 3. One dummy byte leads each reply

The first byte clocked out after an acknowledgement is the stale busy pattern,
because the target's shift register still holds it when the reply is queued
behind. AN4286 states this as sending a dummy byte before a read. It happens
**once per reply, not once per transfer**: asking for another dummy partway
through a reply drops a real byte.

## Read Memory (0x11)

```
cmd 5A 11 EE            ; ACK
address + xor           ; ACK
[len-1, ~(len-1)]       ; ACK   <- the reply begins straight after this one
<dummy>                 ; the stale busy byte
<len bytes>             ; the data, contiguous
```

Each block needs its own Read Memory command; the address does not
auto-increment. Blocks are capped at 256 bytes.

## Write Memory (0x31)

```
cmd 5A 31 CE            ; ACK
address + xor           ; ACK
[len-1, data..., xor]   ; ACK, after the write completes
```

`len-1` is the byte count minus one, so a 256-byte block sends `0xFF`. The XOR
covers the length byte and every data byte.

## Extended Erase (0x44)

```
cmd 5A 44 BB            ; ACK
[count_msb, count_lsb, pages..., xor]   ; ACK, after the erase completes
```

Mass erase is the special value `0xFFFF` with no page list. For a page list the
count field carries **the number of pages**, as on I3C, not the number of pages
minus one as on I2C. A flash page on this part is 8 KB.

## Throughput

The target answers roughly thirty to forty byte periods after each step, and
that latency is counted **in clocks, not in wall time**. How the host delivers
those clocks therefore decides the throughput of the whole bus. Spending one
USB round trip per clocked byte costs milliseconds per acknowledgement; sending
the same clocks as one burst costs microseconds.

Bursting is safe wherever nothing the host needs follows the acknowledgement,
because the target sends only busy padding until it acknowledges, so any byte
clocked before the ACK can be discarded. It is not safe on the one
acknowledgement that is directly followed by a reply, for the reason in point 2
above.

The consequence is measurable and counter-intuitive: **writing is faster than
reading**, 25.8 KiB/s against 14.7 KiB/s on a 122,912-byte image at 4 MHz,
because every acknowledgement in a write may be waited for in bursts and one in
each read may not.

Raising the clock helps very little. A 128 KiB read took 10.31 s at 1 MHz and
8.91 s at 8 MHz. The bus time for a 256-byte block is 2.05 ms at 1 MHz and
0.26 ms at 8 MHz, against a measured 20.14 ms and 17.39 ms per block, so
between 17 and 18 ms of every block is host and USB latency regardless of
clock.

## Host adapter initialisation

Not part of the wire protocol, but it costs more time than anything that is.

An adapter whose SPI interface is already initialised answers `spiControllerInit` with
`INTERFACE_ALREADY_INITIALIZED` and **discards the settings passed with it**. Clock rate,
mode and chip select stay as they were. Every subsequent transfer goes out misconfigured
while the call that set it up reported no error.

Re-apply the same settings with `spiControllerSetParameters` whenever init returns anything
other than success. This was observed on a Supernova, which reported the interface as already
initialised on every attempt including the first after a power cycle. A freshly attached
Pulsar reported success and applied the settings, so the fault presents as a difference
between the adapters when it is really a difference in what init does once an interface is
up.

The symptom is a bus that reads back a constant, before and after a target reset alike. That
is indistinguishable from a disconnected data line, and sends you to check wiring that is
perfectly correct.

## Bootloader entry

Hold `BOOT0` high, pulse `NRST` low, release. Release `NRST` by switching the
pin to an input rather than driving it high.

Where NRST and BOOT0 are wired to spare chip selects rather than GPIO, an
active-low select idles high and is driven low only for the duration of a
transfer, so a slow transfer on the NRST line is a reset pulse of a known
width, and the BOOT0 line sits high while it is not the selected one. This
cannot select the application, which needs BOOT0 held low while NRST is pulsed,
and only one line can be selected at a time; the Go command covers that case.

## Verified on hardware

Against an STM32H503 on a NUCLEO-H503RB on SPI2, with a Binho Pulsar, and repeated with a
Binho Supernova on the same wiring:

- Session start, Get, Get Version, Get ID
- Page erase, program, read back and verify, repeated over alternating images
- A 128 KiB read at 1 MHz, 4 MHz and 8 MHz, byte-identical across all three
- A deliberately hostile image in which every 256-byte block begins with `0xA5`
  and one block is entirely `0xA5`, programmed and verified byte for byte

## Still unconfirmed

- The Special Command (`0x50`) opcode, which the target reports but which the
  reference utility does not use
- Behaviour with readout protection active, since the ROM does not offer the
  commands over this interface
- Whether the one-byte reply lag is constant across SPI clock rates on other
  parts; it was constant at 1, 4 and 8 MHz here
