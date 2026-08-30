# STM32 I3C System Bootloader Protocol

Derived from STMicroelectronics' own Open Bootloader reference implementation
(`STMicroelectronics/stm32-mw-openbl`, files `Modules/I3C/openbl_i3c_cmd.c` and
`Interfaces/Patterns/I3C/i3c_interface.c`), cross-checked against AN5927
("I3C protocol used in the STM32 bootloader").

## Constants

| Name  | Value | Meaning |
|-------|-------|---------|
| ACK   | 0x79  | Acknowledge |
| NACK  | 0x1F  | Not acknowledged |
| SYNC  | 0x5A  | I3C bootloader synchronization byte (note: 0xA5 is the *UART/other* sync byte; I3C uses 0x5A) |
| BUSY  | 0x76  | Defined by the bootloader core, but **never sent over I3C** |
| ERROR | 0xEC  | Internal sentinel only, **never transmitted** |

Only ACK and NACK are ever placed in an IBI payload. In ST's Open Bootloader the
I3C module calls `SendAcknowledgeByte` with `ACK_BYTE` 36 times and `NACK_BYTE`
31 times, and with `BUSY_BYTE` never. `ERROR_COMMAND` (0xEC) is the value
`OPENBL_I3C_GetCommandOpcode` returns internally when the opcode integrity check
fails; it falls through to the dispatcher's `default:` case, which answers NACK.

Confirmed on an STM32H503: a malformed opcode, an unsupported opcode, an invalid
address and an oversized transfer block all return an IBI carrying 0x1F, and the
session stays usable afterwards.

## Command opcodes

| Command                  | Code | Supported over I3C |
|--------------------------|------|--------------------|
| Get                      | 0x00 | yes |
| Get Version              | 0x01 | yes |
| Get ID                   | 0x02 | yes |
| Read Memory              | 0x11 | yes |
| Go                       | 0x21 | yes |
| Write Memory             | 0x31 | yes |
| Erase (extended)         | 0x44 | yes |
| Write Protect            | 0x63 | yes |
| Write Unprotect          | 0x73 | yes |
| Readout Protect          | 0x82 | NOT wired over I3C (NULL in command table) |
| Readout Unprotect        | 0x92 | NOT wired over I3C (NULL in command table) |
| Special Command          | 0x50 | yes |
| Extended Special Command | 0x51 | yes |

## Transport model - THE KEY DETAIL

The STM32 is an **I3C target**. The Supernova is the **I3C controller**.

**The bootloader signals ACK/NACK/BUSY by raising an IBI (In-Band Interrupt)
carrying a 1-byte payload** - it does NOT expose a pollable status register.

From `i3c_interface.c`:
- `LL_I3C_EnableIBI()`, `LL_I3C_ConfigNbIBIAddData(LL_I3C_PAYLOAD_1_BYTE)`
- `OPENBL_I3C_SendAcknowledgeByte()` sets the IBI payload to the ACK/NACK byte
  and issues `LL_I3C_TargetHandleMessage(..., LL_I3C_TARGET_MTYPE_IBI, 1)`

So the host must, for every step: issue a private write/read, then **wait for an
IBI and read its 1-byte payload** to learn ACK vs NACK. This is the single most
important design constraint on the host implementation.

## Connection / discovery sequence

1. Controller issues broadcast **ENTDAA**; the bootloader participates and
   receives a dynamic address. Payload returned is the standard 8 bytes:
   48-bit unique ID (PID) + BCR + DCR.
2. Controller sends a single byte **0x5A** (SYNC) to that dynamic address as a
   **private write**.
3. Bootloader's RX interrupt sees 0x5A, sets `I3cDetected = 1`, and replies with
   an **IBI carrying ACK (0x79)**.
4. Session is established; the bootloader now waits for command opcodes.

## Command framing

### Opcode
Every command is sent as **2 bytes**: `[opcode, opcode ^ 0xFF]`.
The target validates `buffer[0] ^ buffer[1] == 0xFF`, then replies with an IBI
carrying ACK or NACK. On a failed check the opcode is rejected with NACK (0x1F);
0xEC is an internal marker in the ST sources and is never transmitted.

### Address (used by Read Memory / Write Memory / Go)
**5 bytes**, big-endian: `[A31:24, A23:16, A15:8, A7:0, XOR]`
where `XOR = a[0]^a[1]^a[2]^a[3]`. Target replies IBI ACK/NACK.

### Get (0x00)
-> opcode. <- IBI ACK.
<- 1 byte: number of supported commands (N)
<- 1 byte: protocol version (OPENBL_I3C_VERSION = 0x10, i.e. v1.0)
<- N bytes: list of supported command opcodes
<- IBI ACK

### Get Version (0x01)
-> opcode. <- IBI ACK. <- 1 byte version. <- IBI ACK.

### Get ID (0x02)
-> opcode. <- IBI ACK. <- 1 byte length (0x02). <- 2 bytes device ID (MSB,LSB). <- IBI ACK.

### Write Memory (0x31)  -- note the streaming loop
-> opcode.            <- IBI ACK (or NACK if protected)
-> address (5 bytes). <- IBI ACK
Then repeat per chunk:
-> 3 bytes: `[size_field >> 8, size_field & 0xFF, xor]`
   where `size_field = (size << 1) | loop_bit`, `xor = byte0 ^ byte1`,
   `loop_bit = 1` means "more chunks follow", `0` means "this is the last chunk".
   Constraints: `size != 0`, `size <= I3C_RAM_BUFFER_SIZE - 1`.
   <- IBI ACK/NACK
-> `size + 1` bytes: the data followed by its XOR byte (all in ONE I3C frame)
   <- IBI ACK/NACK
The target auto-increments the address by `size` after each accepted chunk, so
the address is sent only once for the whole streamed sequence.

### Read Memory (0x11)  -- same streaming loop shape
-> opcode.            <- IBI ACK
-> address (5 bytes). <- IBI ACK
Then repeat:
-> 3 bytes `[size_field >> 8, size_field & 0xFF, xor]` as above
   (here `size <= I3C_RAM_BUFFER_SIZE`)
   <- IBI ACK/NACK
   <- `size` bytes of data
NOTE: the address auto-increments exactly as it does on write. The target
advances `address` as it reads each byte, so successive loop iterations continue
from where the previous one stopped and the address is sent only once for the
whole streamed sequence. Confirmed empirically: six consecutive 128 KB reads
driven this way returned byte-identical data.

### Erase (0x44)
-> opcode. <- IBI ACK (or NACK if protected)
-> 2 bytes: number of pages, MSB first
-> 1 byte: XOR of those 2 bytes
Special range `0xFFF0`-`0xFFFF`:
  - `0xFFFF` = mass erase, `0xFFFE` = bank 1 erase, `0xFFFD` = bank 2 erase
  - <- IBI ACK/NACK (after the erase completes)
Otherwise (N pages, `0 < N < I3C_RAM_BUFFER_SIZE/2`):
  <- IBI ACK
  -> `2*N` bytes of page numbers (each MSB first) + 1 XOR byte over all of them
  <- IBI ACK/NACK

### Go (0x21)
-> opcode. <- IBI ACK. -> address (5 bytes). <- IBI ACK then jumps.

## Verified on hardware

Measured against a NUCLEO-H503RB (STM32H503RB) with a Binho Supernova rev B
(firmware 4.4.0), binhosupernova 4.2.0:

- Device ID `0x474`, protocol version `0x10` (v1.0).
- Ten commands reported, exactly matching the table above: `00 01 02 11 21 31
  44 50 63 73`. Readout protect/unprotect are absent, confirming they are NULL
  in the I3C command table.
- The bootloader joins via **ENTDAA** and is assigned a dynamic address
  (`0x08` in a single-target bus). It reports PID `02 08 13 81 B0 00`,
  BCR `0x2E`, DCR `0x01`. SETDASA was not needed.
- Status really is delivered as an IBI with a 1-byte payload; the Supernova
  surfaces it as an `I3C CONTROLLER IBI REQUEST NOTIFICATION` with
  `result = IBI_REQUEST_ACCEPTED_WITH_PAYLOAD`.
- A **256-byte** chunk works for both read and write. This was probed rather
  than read from a datasheet, so treat it as the largest verified value, not a
  proven ceiling.

### Turnaround hazard (important)

Issuing a fresh Read Memory command per chunk intermittently loses the command
opcode: the target is still returning through its command dispatcher when the
next private write lands, so the write is dropped and no status IBI ever
arrives. Measured over 32 KB reads, 3 runs each:

| Approach | Inter-command delay | Result | Avg |
|----------|--------------------|--------|-----|
| one command per chunk | 0 ms | 1/3 | 2.17 s |
| one command per chunk | 1 ms | 2/3 | 2.38 s |
| one command per chunk | 2 ms | 0/3 | - |
| one command per chunk | 5 ms | 3/3 | 2.91 s |
| streaming loop | 0 ms | 3/3 | 1.39 s |
| streaming loop | 1 ms | 3/3 | 1.57 s |

Use the streaming loop (loop bit set) for both read and write. It is both
reliable and faster. Six consecutive full 128 KB reads using it produced
byte-identical output.

### Still unconfirmed

- The exact value of `I3C_RAM_BUFFER_SIZE` in the ROM bootloader (256 is
  verified to work; larger values were not explored).
- Page erase (`0x44` with an explicit page list) is described above from ST's
  source but was not exercised on hardware -- only mass erase (`0xFFFF`) was.
