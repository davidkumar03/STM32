# Low-Level Register-Accurate Protocol Specification
## UART · I²C · SPI · CAN — ARM Cortex-M4 / STM32F4 Reference Platform

**Reference silicon:** STM32F407/417 (Cortex-M4, up to 168 MHz SYSCLK)
**Clock domains assumed:** HCLK = 168 MHz, APB1 (PCLK1, max 42 MHz) hosts USART2/3, I2C1/2/3, SPI2/3, CAN1/2. APB2 (PCLK2, max 84 MHz) hosts USART1/6, SPI1.
**Convention:** All register bit-field names match ST's RM0090 Reference Manual. All numeric examples are computed step-by-step so you can reproduce them on paper.

---

## 0. How to use this document

Each protocol section follows this skeleton:
1. Physical layer (electrical/mechanical)
2. Frame format + ASCII timing diagram
3. **How the transmitter and receiver actually work** — the field-by-field mechanics of how hardware knows "this bit belongs to field X" with no external labels in the bitstream
4. Clock/baud-rate derivation (symbolic formula → worked numeric example)
5. Register map (only the bits that matter operationally)
6. Hardware error flags and what sets/clears them

Section 5 (bottom) is a side-by-side comparative matrix. Section 6 collects every worked numeric example. Section 7 is a pitfalls list.

**The core idea running through all four protocols:** none of them transmit an explicit "this is the address field" or "this is byte 3 of 5" tag inside the wire signal. Every protocol instead relies on one (or a mix) of three mechanisms to know what a given bit means:
- **Position counting** — both sides agree in advance on a fixed field order and just count bits/clocks from a known reference edge (UART's start bit, SPI's NSS edge, CAN's SOF).
- **Configuration agreement** — both sides are pre-configured identically (baud rate, CPOL/CPHA, word length) so the *receiver* can replay the same bit-timing the transmitter used, without any of that configuration being sent over the wire itself.
- **Self-describing length** — a small fixed field near the start of the frame tells the receiver how long a variable part will be (I²C's implicit byte-by-byte software-controlled length, CAN's DLC field for the data payload).

Keep that framework in mind — it explains *why* each protocol is shaped the way it is.

---

## 1. UART / USART

### 1.1 Physical Layer

- **Signals:** TX (output), RX (input); hardware flow control variants add RTS/CTS. Optional SCLK pin for synchronous mode (rarely used).
- **Levels:** Full CMOS logic levels at VDD (typically 3.3 V) — this is TTL/CMOS UART, *not* RS-232 (which is bipolar ±3–15 V and needs a level shifter such as MAX3232).
- **Idle state:** Line held HIGH (mark) when idle. A frame begins with a falling edge (start bit).
- **Topology:** Point-to-point, full duplex, no shared bus, no arbitration — the simplest protocol here electrically.

### 1.2 Frame Format & Timing Diagram

A UART character frame consists of: **1 start bit (dominant LOW) → 5–9 data bits (LSB first) → optional parity bit → 1–2 stop bits (HIGH)**.

```
Idle    Start  D0  D1  D2  D3  D4  D5  D6  D7  Parity Stop  Idle
 ────┐   ___                                            ___  ┌────
     │  |   |__ __ __ __ __ __ __ __ __ __ __ __ __ __ |   | │
     └──┘   0  b0 b1 b2 b3 b4 b5 b6 b7  P   1   1       └───┘
        1 bit  <----------- N data bits ----------->  1-2 bits
        (LOW)                                          (HIGH)
```

- Bit time `T_bit = 1 / BaudRate`.
- Total frame time (8N1, no parity) = `10 × T_bit`.
- STM32 USART transmits **LSB first** by default.

### 1.3 How the Transmitter Actually Works

There is no "framing logic" that inspects data to decide what field it is — the frame shape is entirely fixed by configuration bits (`CR1.M` for word length, `CR1.PCE`/`PS` for parity, `CR2.STOP` for stop-bit count), decided once at init time. The transmit path is a simple two-stage pipeline:

1. Software (or DMA) writes a byte into `USART_DR`. This is really a **shadow/holding register** called TDR internally.
2. Whenever the internal **Transmit Shift Register (TSR)** becomes empty (flagged by `TXE`), hardware automatically copies TDR into TSR and starts shifting.
3. The shift sequencer is a small hardware state machine with states `START → DATA[0..N-1] → (PARITY) → STOP[1..2] → IDLE/next-START`. It does **not** look at the data value to decide field boundaries — it is purely a bit-position counter driven by the baud clock: bit position 0 is always forced LOW (start), positions 1..N are the payload shifted out LSB-first from TSR, the next position (if `PCE=1`) outputs the running XOR-parity of the N data bits just sent, and the final 1–2 positions are forced HIGH (stop).
4. `TC` (Transmission Complete) sets only when TSR is empty **and** nothing new was loaded into TDR — i.e., truly idle, not just "ready for the next byte" (that's what `TXE` means).

**Key insight:** the transmitter doesn't need to "know" where fields are because it *generates* the field boundaries itself, on a fixed schedule, every single frame — there's nothing to discover.

### 1.4 How the Receiver Actually Works — Field Recognition With No Shared Clock

This is the genuinely interesting half, because RX has no clock line to tell it when a bit starts — it must **reconstruct bit timing purely from signal edges**, and this is exactly how it "knows" which field it's looking at:

1. **Idle scan:** RX line is normally HIGH. A dedicated edge-detector continuously watches for a HIGH→LOW transition.
2. **Start-bit validation (not blind trust):** the moment a falling edge appears, hardware doesn't assume it's a real start bit — it could be a glitch. It starts an internal ×16 (or ×8, if `OVER8=1`) sample-clock counter and waits until the counter reaches the *middle* of the suspected start bit period (count 8 of 16). At that midpoint it takes **three consecutive oversample readings (counts 7, 8, 9)** and majority-votes them:
   - Majority LOW → confirmed genuine start bit. Proceed.
   - Majority HIGH → was noise, not a real start. Abort and return to idle scanning (this path never even reaches the "noise error" flag, because no frame was accepted at all).
3. **Timing lock-on:** the instant a valid start bit is confirmed, the receiver now has a precise reference point — "bit 0 center occurred at this oversample count." Every subsequent bit's center is simply `+16 oversample-clock ticks` (one full bit period) from the previous one. **This is the entire trick**: the receiver never re-synchronizes to new edges within the frame; it trusts its own local oversampling clock to stay accurate enough for ~10 bit periods, which is why UART tolerates only a few percent of baud mismatch — errors accumulate linearly across the frame and eventually miss the true bit center.
4. **Field boundary = bit counter, not content:** hardware maintains a simple down-counter initialized from the configured word length `M` (8 or 9). As each subsequent bit-center is sampled (again via 7/8/9 majority vote — this is also where **Noise Error (NE)** gets flagged, if the three samples disagree), the counter decrements. While counter > 0, sampled bits are shifted into the RX shift register as **data bits** — purely because of their position, not because they "look like" data.
5. **Parity field:** if `PCE=1`, hardware knows in advance (from configuration, not from the bitstream) that exactly one more bit position follows the last data bit, and that this position is parity, not data. It's captured and XOR-compared against the running parity of the received data bits — mismatch sets **PE**.
6. **Stop field:** the receiver knows, purely from its bit-position counter reaching zero (data+parity accounted for), that the *next* 1–2 bit positions **must** sample HIGH. This expectation is what makes stop-bit checking possible at all — there's no "stop pattern" being searched for, it's "whatever is at this specific counted position should be recessive; if it's dominant, that's a **Framing Error (FE)**."
7. **Completion & reset:** once the stop bit(s) are validated, `RXNE` sets, the shift register's data bits move into `DR`, and the entire state machine resets to step 1 (idle scanning) — critically, **every frame re-derives its own timing from scratch** from its own start bit. There is no persistent shared clock state carried between frames.

**So, to directly answer "how do we know which field a bit belongs to":** in UART, it is 100% positional — counted in fixed-size steps from the validated start-bit edge, using a locally regenerated timing grid, with word length/parity/stop-count all pre-agreed by configuration rather than signaled in the frame itself.

### 1.5 Baud Rate Generation — Clock Equations & Derivation

```
Baud = f_CK / (8 × (2 − OVER8) × USARTDIV)
USARTDIV = f_CK / (8 × (2 − OVER8) × Baud)
```

- `OVER8 = 0` → 16× oversampling, 4-bit fraction, max baud = f_CK/16.
- `OVER8 = 1` → 8× oversampling, 3-bit fraction, up to double max baud, coarser resolution.

`USART_BRR` layout:
```
Bit:   15 14 13 12 | 11 10 9 8 7 6 5 4 | 3 2 1 0
       ---- DIV_Mantissa[11:0] ----     DIV_Fraction[3:0]   (OVER8=0)
```

**Worked example** — USART2 on APB1, `f_CK = 16 MHz`, target `Baud = 115200`, `OVER8 = 0`:

```
Step 1: USARTDIV = 16,000,000 / (16 × 115200) = 16,000,000 / 1,843,200 = 8.68055...
Step 2: Mantissa = floor(8.68055) = 8
Step 3: Fraction_raw = 0.68055 × 16 = 10.888 → round = 11 (0xB)
Step 4: No carry (11 < 16)
Step 5: BRR = (8 << 4) | 11 = 128 + 11 = 139 = 0x008B
Step 6: USARTDIV_actual = 8 + 11/16 = 8.6875
        Baud_actual = 16,000,000 / (16 × 8.6875) = 16,000,000 / 139 = 115,107.9
Step 7: Error = (115107.9 − 115200)/115200 = −0.08%  (well within tolerance)
```

**Second example** — USART1 on APB2, `f_CK = 84 MHz`, target `921600`, `OVER8 = 1`:

```
USARTDIV = 84,000,000 / (8 × 921600) = 84,000,000 / 7,372,800 = 11.393...
Mantissa = 11
Fraction_raw = 0.393 × 8 = 3.15 → round = 3
BRR = (11 << 4) | 3 = 176 + 3 = 179 = 0x00B3
Actual USARTDIV = 11.375 → Actual Baud = 84,000,000/(8×11.375) = 923,076
Error = +0.16%
```

### 1.6 Register Map (USARTx)

| Register | Key Bits | Function |
|---|---|---|
| `USART_SR` | TXE, TC, RXNE, IDLE, ORE, NE, FE, PE, CTS, LBD | Status flags (§1.7) |
| `USART_DR` | DR[8:0] | Read = RX buffer, Write = TX shadow register |
| `USART_BRR` | DIV_Mantissa[11:0], DIV_Fraction[3:0] | Baud divisor (§1.5) |
| `USART_CR1` | UE, M, WAKE, PCE, PS, PEIE, TXEIE, TCIE, RXNEIE, IDLEIE, TE, RE, RWU, SBK, OVER8 | Enable, word length, parity, IRQ enables |
| `USART_CR2` | STOP[1:0], CLKEN, CPOL, CPHA, LBCL, LINEN | Stop bits, clock output (sync mode), LIN |
| `USART_CR3` | CTSE, RTSE, DMAT, DMAR, SCEN, HDSEL, IRLP, IREN, EIE | Flow control, DMA, IrDA, Smartcard |
| `USART_GTPR` | GT[7:0], PSC[7:0] | Guard time & prescaler (Smartcard/IrDA) |

### 1.7 Hardware Error Flags (`USART_SR`)

| Flag | Meaning | Set condition | Cleared by |
|---|---|---|---|
| **ORE** (Overrun) | New byte fully received while `RXNE` still set | RX shift register full, DR not read in time | Read `SR` then read `DR` |
| **NE** (Noise Error) | Majority-vote samples disagreed during a *received* bit (post start-bit validation) | Noisy line / baud mismatch | Read `SR` then read `DR` |
| **FE** (Framing Error) | Stop bit sampled as 0 instead of 1 | Baud mismatch, break condition, or line disturbance | Read `SR` then read `DR` |
| **PE** (Parity Error) | Computed parity ≠ received parity bit | Corrupted data or misconfigured parity | Read `SR` then read `DR` |
| **LBD** (LIN Break) | Break frame detected (LIN mode only) | 11+ dominant bits in LIN mode | Software write 0 |
| **IDLE** | Idle line detected after a frame | Line held high ≥1 frame duration | Read `SR` then read `DR` |

> **Gotcha:** ORE/FE/NE/PE all share the same clear sequence (`SR` read → `DR` read). Forgetting the `SR` read before `DR` read means the flag never clears and the RX interrupt fires forever.

---

## 2. I²C (Inter-Integrated Circuit)

### 2.1 Physical Layer

- **Signals:** SDA (data), SCL (clock) — both **open-drain**, requiring external pull-up resistors.
- **Bus topology:** Multi-master, multi-slave, shared 2-wire bus. Wired-AND logic: any device can pull LOW; only the pull-up brings a line HIGH.
- **Speed grades:** Standard Mode (Sm) 100 kHz, Fast Mode (Fm) 400 kHz. STM32F4 I2C peripheral tops out at Fast Mode (no Fm+/HS).
- **START/STOP** are electrical events, not data bits: SDA falling while SCL high = START; SDA rising while SCL high = STOP.

### 2.2 Frame Format & Addressing

```
S  [ 7-bit ADDR | R/W ]  ACK  [ 8-bit DATA ]  ACK  ...  [ 8-bit DATA ]  NACK  P
```

```
SCL  __   __   __   __   __   __   __   __   __
    |  |_|  |_|  |_|  |_|  |_|  |_|  |_|  |_|  |_
SDA   b7  b6  b5  b4  b3  b2  b1  b0  ACK
      (data stable while SCL high, changes while SCL low)
```

### 2.3 How the Transmitter and Receiver Work — Field Recognition on a Shared Bus

I²C has no dedicated "this is an address" wire and no length prefix for how many data bytes follow — everything is derived from **position relative to START** plus a fixed byte-boundary of exactly 8 bits + 1 ACK, repeated indefinitely until a STOP or repeated-START appears. Here is the mechanism step by step:

1. **START detection:** every node's hardware (master and all slaves) continuously runs a comparator watching SDA *while SCL is sampled HIGH*. A HIGH→LOW transition on SDA during this window is unambiguous — it can only be a START, because a genuine data-bit change is only ever allowed while SCL is LOW. This asymmetry (data changes only on SCL-low, START/STOP only happen on SCL-high) is precisely what lets every device on the bus distinguish "this is a control condition" from "this is a data bit," using the exact same wires.
2. **Positional rule — first byte after START is ALWAYS the address:** there is no tag that marks a byte as "address type." Hardware (in every slave listening) simply always treats the *very first* 8 bits clocked in immediately after a START (or repeated-START) condition as `[7-bit ADDR][R/W]`, full stop — this is a hardwired assumption baked into the protocol, not something discovered at runtime.
3. **Address comparison & auto-ACK:** as the shift register fills with these first 8 bits, each slave's hardware compares the top 7 bits against its own `OAR1`/`OAR2` registers (and against the reserved general-call pattern `0000000`). If a slave matches, its hardware — automatically, without software intervention — pulls SDA LOW during the 9th clock pulse (ACK). Non-matching slaves do nothing (leave SDA to float high = implicit NACK from their perspective). The **master** then samples that 9th bit: dominant(low)=ACK seen, so at least one slave matched; still high = **AF (Acknowledge Failure)**.
4. **Positional rule — every byte after the address is DATA**, with the exact same 8-bit-shift + 9th-bit-ACK mechanism repeating. The receiver hardware does not know in advance how many data bytes are coming; **that length is not sent in the frame at all** — it is purely a runtime decision made by software (the master keeps clocking bytes for as long as it wants, and signals "this is the last byte" by having the *receiving* side issue a NACK instead of ACK on the final byte, then the master generates STOP).
5. **Direction switch, not a new "field":** the R/W bit captured as part of the address byte tells every listening slave which direction data will flow for the *rest* of this transaction — this is why the address byte is special: it's the only field whose content (not just position) changes how subsequent bytes are interpreted (as master-transmits-data vs. master-reads-data).
6. **Clock stretching as hardware flow control:** if the CPU hasn't serviced `TXE`/`RXNE` fast enough, hardware **automatically** holds SCL LOW right after the ACK/NACK bit of the byte just completed — this is invisible at the protocol-field level (it doesn't add or remove any bits) but it does mean "one clock period" can take arbitrarily long in wall-clock time; a receiver must therefore treat SCL-high as authoritative for "next bit is now presented," never assume a fixed period.
7. **STOP detection:** mirror of START — a LOW→HIGH SDA transition while SCL is high, again distinguishable from a data-bit change only because of the SCL level at that instant. Every slave resets its internal byte-boundary counter back to "expect an address next" the moment STOP (or a new START) is seen.

**Answer in one line:** I²C fields are known purely by counting bytes from the START condition (byte 1 = address, always; every following byte = data) combined with the 8-bits-then-ACK rhythm that both transmitter and every listening receiver replay in lock-step off the shared SCL line.

### 2.4 Clock Generation — CCR / TRISE Derivation

**Standard Mode (≤100 kHz):**
```
CCR = f_PCLK1 / (2 × f_SCL)
```
**Fast Mode, duty=2:**
```
CCR = f_PCLK1 / (3 × f_SCL)
```
**Fast Mode, duty=16/9:**
```
CCR = f_PCLK1 / (25 × f_SCL)
```
**Rise time:**
```
TRISE = (T_rise_max × f_PCLK1) + 1
```
(`T_rise_max` = 1000 ns Sm / 300 ns Fm)

**Worked example A — Standard Mode 100 kHz, `f_PCLK1 = 42 MHz`:**
```
CCR = 42,000,000 / (2 × 100,000) = 210 = 0x00D2
TRISE = (1000ns × 42MHz) + 1 = 42 + 1 = 43 = 0x2B
Verify: SCL period = 2×210/42MHz = 10.0µs → 100.0 kHz exact
```

**Worked example B — Fast Mode 400 kHz, duty=2, `f_PCLK1 = 42 MHz`:**
```
CCR = 42,000,000 / (3 × 400,000) = 35
TRISE = (300ns × 42MHz) + 1 = 12.6 + 1 = 13.6 → 14 = 0x0E
Verify: T_low = 2×35/42MHz = 1.667µs, T_high = 35/42MHz = 0.833µs
        Period = 2.5µs → 400.0 kHz exact
```

> **Design rule:** `f_PCLK1` must be ≥2 MHz (Sm) / ≥4 MHz (Fm) — otherwise `CCR` rounds to an invalid value.

### 2.5 Register Map (I2Cx)

| Register | Key Bits | Function |
|---|---|---|
| `I2C_CR1` | PE, SMBUS, ENPEC, ENARP, ENGC, NOSTRETCH, START, STOP, ACK, POS, SWRST | Enable, START/STOP generation, ACK control |
| `I2C_CR2` | FREQ[5:0], ITERREN, ITEVTEN, ITBUFEN, DMAEN, LAST | Input clock (MHz), IRQ/DMA |
| `I2C_OAR1`/`OAR2` | ADD[9:0], ADDMODE | Own slave address(es) |
| `I2C_DR` | DR[7:0] | TX/RX data |
| `I2C_SR1` | SB, ADDR, BTF, ADD10, STOPF, RXNE, TXE, BERR, ARLO, AF, OVR, PECERR, TIMEOUT | Event + error flags (§2.6) |
| `I2C_SR2` | MSL, BUSY, TRA, GENCALL, DUALF | Bus state flags |
| `I2C_CCR` | CCR[11:0], DUTY, F/S | Clock control (§2.4) |
| `I2C_TRISE` | TRISE[5:0] | Max rise time (§2.4) |

### 2.6 Hardware Error Flags (`I2C_SR1`)

| Flag | Meaning | Typical Cause |
|---|---|---|
| **BERR** | Misplaced START/STOP mid-byte | Glitch or protocol-violating device |
| **ARLO** | Arbitration lost (multi-master) | Bus contention |
| **AF** | Expected ACK not received | No device at address, or rejection |
| **OVR** | Byte lost due to timing (no stretching) | `NOSTRETCH=1` + slow software |
| **PECERR** | CRC (PEC) mismatch (SMBus) | Data corruption |
| **TIMEOUT** | SCL held low too long | Stuck slave/bus lock-up |
| **SMBALERT** | SMBus Alert asserted | SMBus-specific |

> **Bus lock-up recovery:** if SDA is stuck LOW, bit-bang SCL as GPIO output 9 times to force release, then issue STOP and re-init.

---

## 3. SPI (Serial Peripheral Interface)

### 3.1 Physical Layer

- **Signals:** `MOSI`, `MISO`, `SCK` (master-driven), `NSS`/`CS` (active-low).
- **Topology:** Single master, star topology (1 CS per slave typically).
- **Full duplex, push-pull drivers** — no shared-bus signaling requirement, no open-drain.

### 3.2 Clock Modes — CPOL/CPHA

| Mode | CPOL | CPHA | Clock idle | Data sampled on |
|---|---|---|---|---|
| 0 | 0 | 0 | LOW | Rising edge (1st edge) |
| 1 | 0 | 1 | LOW | Falling edge (2nd edge) |
| 2 | 1 | 0 | HIGH | Falling edge (1st edge) |
| 3 | 1 | 1 | HIGH | Rising edge (2nd edge) |

Mode 0:
```
SCK   ___     ___     ___     ___
    _|   |___|   |___|   |___|   |___
NSS  ‾\_________________________/‾
MOSI  X  b7 X  b6 X  b5 X ...  X
              ↑ sampled on each RISING edge
```

### 3.3 How the Transmitter and Receiver Work — There Are No Fields, Only a Shared Shift Register

SPI is the one protocol here with **no start condition, no address field, no length field, and no delimiter of any kind in the electrical signal.** Understanding *why* this works requires seeing master and slave as two halves of one physical circuit:

1. **Conceptually, MOSI+MISO+SCK connect the master's shift register and the slave's shift register into a single ring.** On every active clock edge, the master's shift register shifts one bit out on MOSI (into the slave's shift register input) *and simultaneously* shifts one bit in from MISO (out of the slave's shift register) — both happen on the exact same physical clock edge. After exactly `N` clock edges (N = 8 or 16, set by `CR1.DFF`), the two shift registers have **completely swapped their original contents.**
2. **"Field" = clock count, fixed by prior agreement, not discovered:** neither side inspects the data to figure out how many bits constitute a frame. Both were configured (usually by firmware written once at init) to use the same `DFF` (data frame format), and the master simply stops toggling SCK after exactly that many edges. If master and slave disagree on `DFF`, the receiver simply captures the wrong number of bits — there's no hardware mechanism that would ever detect this mismatch, because there's no length field to check against.
3. **NSS is the *only* framing signal that exists**, and it doesn't mark data field boundaries within a byte — it marks the *transaction* boundary: falling edge means "the following clock pulses belong to a transfer addressed to you"; rising edge means "transfer over, reset your bit counter for next time." This is how multiple slaves share one SCK/MOSI/MISO bus without any logical addressing at all — the *wire itself* (which NSS line is low) is the address.
4. **CPOL/CPHA determine WHICH edge is the "sample" edge vs the "shift/setup" edge** — this is a timing agreement, not a signaled field. On the sampling edge, both sides latch the incoming bit; on the other (setup) edge, both sides present their *next* outgoing bit on the line so it has half a clock period to stabilize before the next sampling edge. If CPOL/CPHA differ between the two chips, every bit gets sampled half a cycle early or late — the receiver still faithfully captures *something* every edge, it's just capturing the wrong instant, which is why CPOL/CPHA mismatches look exactly like random garbage rather than a detectable error (there's no error flag for this — SPI has no way to know a "field" was misread, because it never had explicit fields to check against in the first place).
5. **Multi-byte transfers are just N single-byte shifts back-to-back** while NSS stays low the whole time — again, purely a count agreed by software (e.g., "read 6 registers" = master toggles NSS low, clocks out 1 command byte + clocks in/out 6 data bytes = 7 total byte-shifts, then raises NSS) with no header describing this length; the *software protocol layered on top of SPI* (e.g. a sensor's register map convention) is what gives meaning to byte position 1 vs byte 2, not SPI itself.

**Answer in one line:** SPI has no fields to "detect" at the hardware level at all — every bit's meaning is 100% pre-agreed by configuration (word length, CPOL/CPHA, and whatever higher-level command structure your firmware defines), and the wire only ever communicates "value now" on the clock edge and "who's involved" via NSS.

### 3.4 Baud Rate Derivation

```
f_SCK = f_PCLK / 2^(BR+1)
```

| BR[2:0] | Prescaler |
|---|---|
| 000 | ÷2 |
| 001 | ÷4 |
| 010 | ÷8 |
| 011 | ÷16 |
| 100 | ÷32 |
| 101 | ÷64 |
| 110 | ÷128 |
| 111 | ÷256 |

**Worked example** — SPI1 on APB2, `f_PCLK2 = 84 MHz`, target ≈10 MHz (sensor max 10.5 MHz):
```
BR=011 (÷16): f_SCK = 84,000,000/16 = 5.25 MHz  (safe margin)
BR=010 (÷8):  f_SCK = 84,000,000/8  = 10.5 MHz  (at absolute max, no margin)
```

### 3.5 Register Map (SPIx)

| Register | Key Bits | Function |
|---|---|---|
| `SPI_CR1` | BIDIMODE, CRCEN, DFF, RXONLY, SSM, SSI, LSBFIRST, SPE, BR[2:0], MSTR, CPOL, CPHA | Mode config, baud, enable |
| `SPI_CR2` | TXEIE, RXNEIE, ERRIE, SSOE, FRF, TXDMAEN, RXDMAEN | IRQ/DMA, NSS output enable |
| `SPI_SR` | RXNE, TXE, CHSIDE, UDR, CRCERR, MODF, OVR, BSY, FRE | Status/error (§3.6) |
| `SPI_DR` | DR[15:0] or DR[7:0] | TX/RX data |
| `SPI_CRCPR` | CRCPOLY[15:0] | CRC polynomial (optional) |
| `SPI_RXCRCR`/`TXCRCR` | — | Running CRC values |

### 3.6 Hardware Error Flags (`SPI_SR`)

| Flag | Meaning | Cause |
|---|---|---|
| **MODF** | NSS driven low externally while master, `SSM=0` | Multi-master conflict / floating NSS |
| **OVR** | New data received before previous `DR` read | Software/DMA too slow |
| **CRCERR** | Hardware CRC mismatch | Line corruption (`CRCEN=1` only) |
| **UDR** | TI mode: no data ready to transmit | Slave software too slow |
| **FRE** | TI mode frame format violation | NSS timing violation |

---

## 4. CAN (Controller Area Network) — bxCAN peripheral

### 4.1 Physical Layer

- **Signals:** `CAN_H`, `CAN_L` — differential pair via external transceiver.
- **Recessive (1):** CAN_H≈CAN_L≈2.5V (diff≈0V). **Dominant (0):** CAN_H≈3.5V, CAN_L≈1.5V (diff≈2V) — dominant wins if both driven.
- **Termination:** 120 Ω at each physical bus end.
- **Multi-master, non-destructive priority arbitration:** lowest ID wins with zero collision cost.

### 4.2 Frame Formats

**Standard Data Frame (11-bit ID):**
```
SOF | ID[10:0] | RTR | IDE | r0 | DLC[3:0] | DATA[0..8 bytes] | CRC[15:0] | CRC_DEL | ACK | ACK_DEL | EOF[7] | IFS[3]
 1        11      1    1    1      4          0-64 bits          15          1        1       1        7        3
```

**Extended Data Frame (29-bit ID):** inserts `SRR`, `IDE=1`, and 18 additional ID bits between the base ID and control field — 29 identifier bits total.

- **RTR:** 0=data frame, 1=remote frame (request only, no payload).
- **IDE:** 0=standard, 1=extended.
- **DLC:** payload length code, 0–8 bytes (classic CAN).

### 4.3 Bit Stuffing Rules

A stuff bit of **opposite polarity** is inserted after every **5 consecutive identical bits**, applied from **SOF through the CRC field** (not to CRC delimiter, ACK, EOF, IFS — those are fixed-form).

```
Raw:      1 1 1 1 1 0 0 1 ...
Stuffed:  1 1 1 1 1 [0]stuff 0 0 1 ...
```
Receivers destuff the mirror way: after counting 5 identical received bits, discard the next bit rather than treating it as data. A 6th identical bit where a stuff bit was expected = **Stuff Error**.

### 4.4 How the Transmitter and Receiver Work — Field Recognition on a Broadcast Bus

CAN is the most self-describing of the four protocols at the bit level, because it has to be: **every node hears every frame** (it's a broadcast bus), and there's no NSS-style physical selection or address-byte convention — the entire frame boundary and internal structure must be derivable by pure bit-position counting plus one dynamically-sized field. Here's the mechanism:

1. **SOF detection = "first dominant bit after sustained idle":** the bus idles recessive. Every node's hardware watches for a dominant bit following an idle period and, purely by that event, starts a **fixed bit-position counter** — there is no separate "start of frame marker pattern," the SOF *is* just bit position 0, one single dominant bit, and its presence after idle is what triggers everyone to start counting.
2. **The fixed-position fields are counted, not detected:** every node (transmitter and every listening receiver simultaneously) advances the same counter: bits 1–11 = Identifier, bit 12 = RTR, bit 13 = IDE, bit 14 = reserved (r0), bits 15–18 = DLC. This is identical in spirit to UART's position-counted fields — nobody is "parsing" a header, everyone just knows position N means field X because the standard defines it that way and all compliant hardware implements the same counter.
3. **DLC is the pivot — the one field whose VALUE changes what comes next:** once the 4-bit DLC value (0–8) is fully shifted in, hardware now knows dynamically how many more 8-bit groups constitute the Data field. This is the one place CAN departs from pure fixed-position counting — the *length* of an upcoming field is carried in an earlier field's content, exactly the same principle as (e.g.) an IP packet's length field, just far simpler.
4. **Destuffing runs concurrently with field counting, not before it:** a separate hardware counter tracks consecutive identical *received bus levels* (before destuffing). The moment it reaches 5, the next incoming bit is consumed as a stuff bit and is **not** advanced into the field-position counter from step 2 — the two counters (bit-position-in-frame, and consecutive-identical-bits-for-stuffing) run side by side, and only genuine (destuffed) bits feed the field-position logic. This is why a receiver can be electrically fooled by 6 identical bits in a row (a real bus violation): the destuffing counter never expected a 6th identical bit, hence Stuff Error.
5. **CRC computed live, not appended after the fact conceptually:** a CRC-15 polynomial shift register processes every (destuffed) bit from SOF through the end of the Data field as it arrives — by the time the Data field ends, the transmitter's shift register already contains the correct CRC value, which it then simply shifts out as the next 15 bits (the CRC field). Every receiver runs the identical CRC computation over the same bits it received and compares against the transmitted CRC field — this is how corruption is detected without any retransmission-request field: a mismatch alone is enough to cause every correctly-receiving node to simply refuse the ACK below.
6. **ACK slot — a field whose value is written by someone OTHER than the transmitter:** the transmitter always sends the ACK bit position as recessive (it is, after all, asking a question: "did anyone get this right?"). Any *other* node whose live CRC check just passed briefly drives that single bit dominant, overwriting the transmitter's recessive level. The transmitter then reads back its own ACK bit position: dominant = someone acknowledged; still recessive = **Acknowledgement Error** (either it's alone on the bus, or every listener's CRC failed).
7. **EOF recognized by an otherwise-illegal run of 7 recessive bits:** stuffing is only active up through the CRC field, so once CRC delimiter passes, 7 recessive bits in a row is *deliberately* legal here and nowhere else in a well-formed frame — this unique, unambiguous pattern is how every node recognizes "the frame has ended," without needing an explicit length or terminator symbol anywhere else.
8. **Arbitration — the ID field is read AND written simultaneously by every contending transmitter:** if two nodes start transmitting at the same idle→dominant SOF transition, both proceed to drive their own ID bits — but each one *also* monitors the actual bus level while doing so. A node that outputs recessive (1) but reads back dominant (0) instantly knows another node is sending a lower-priority-losing... no — actually a *higher priority* (numerically lower) ID at that bit position, since dominant beat its recessive. That node immediately stops driving (becomes a listener for the rest of this frame) without corrupting anything, while the surviving node never even notices a collision occurred. This bit-by-bit "drive-and-read-back" comparison is the entire arbitration field-recognition mechanism — there's no separate "arbitration flag," the same 11 (or 29) ID bits serve as both the message's field content and the contention-resolution mechanism simultaneously.

**Answer in one line:** CAN fields are known by a shared fixed bit-position counter running identically in every node from the SOF event, with exactly one field (DLC) whose *value* dynamically extends that counting scheme to cover the variable-length Data field, all while a parallel destuffing counter filters out the non-data stuff bits before they ever reach the field counter.

### 4.5 Bit Timing — Segment & Prescaler Derivation

```
t_q = (BRP + 1) / f_PCLK1
```
```
| SYNC_SEG | BS1 (1–16 tq) | BS2 (1–8 tq) |
|   1 tq   |               |  ← Sample Point
```
- `BS1 = TS1[3:0] + 1`, `BS2 = TS2[2:0] + 1`.
- Total bit time (tq) = `1 + BS1 + BS2`.
- `f_BAUD = f_PCLK1 / [(BRP+1) × (1 + BS1 + BS2)]`
- Sample point % = `(1 + BS1) / (1 + BS1 + BS2) × 100`
- `SJW` (1–4 tq) = max resync adjustment per bit.

**Worked example — 500 kbit/s, `f_PCLK1 = 42 MHz`:**
```
Step 1: Target bit time = 1/500,000 = 2.0 µs
Step 2: Try BRP+1 = 6 → t_q = 6/42,000,000 = 142.857 ns
Step 3: Total tq/bit = 2.0µs / 142.857ns = 14 tq exactly
Step 4: 14 = 1(sync) + BS1(9) + BS2(4)  [TS1=8, TS2=3]
Step 5: Sample point = (1+9)/14 = 71.4%
Step 6: Registers: BRP_field=5, TS1=8, TS2=3, SJW=1(example)
Step 7: Verify: 42,000,000/(6×14) = 500,000 exact
```
`CAN_BTR = (SJW<<24)|(TS2<<20)|(TS1<<16)|BRP = (1<<24)|(3<<20)|(8<<16)|5 = 0x0138_0005`

### 4.6 Register Map (bxCAN)

| Register | Key Bits | Function |
|---|---|---|
| `CAN_MCR` | INRQ, SLEEP, TXFP, RFLM, NART, AWUM, ABOM, TTCM, RESET, DBF | Init mode, auto-retransmit, auto bus-off mgmt |
| `CAN_MSR` | INAK, SLAK, ERRI, WKUI, TXM, RXM | Init/sleep ack, error interrupt |
| `CAN_TSR` | TXOK0-2, ABRQ0-2, TERR0-2, ALST0-2, TME0-2, CODE | TX mailbox status |
| `CAN_RF0R`/`RF1R` | FMP, FULL, FOVR, RFOM | RX FIFO pending count, overrun |
| `CAN_IER` | Various *IE | Interrupt enables |
| `CAN_ESR` | EWGF, EPVF, BOFF, LEC[2:0], TEC[7:0], REC[7:0] | Error state (§4.7) |
| `CAN_BTR` | BRP[9:0], TS1[3:0], TS2[2:0], SJW[1:0], LBKM, SILM | Bit timing (§4.5) |
| `CAN_TIxR/TDTxR/TDLxR/TDHxR` | STID/EXID, RTR, IDE, DLC, DATA | TX mailboxes (×3) |
| `CAN_RIxR/RDTxR/RDLxR/RDHxR` | (mirror fields) | RX FIFO mailboxes |
| `CAN_FMR/FM1R/FS1R/FFA1R/FA1R/FiRx` | — | Filter banks |

### 4.7 Error States & Hardware Flags

| State | Condition | Behavior |
|---|---|---|
| **Error Active** | TEC<128 and REC<128 | Normal; transmits Active Error Frames (6 dominant bits) |
| **Error Passive** | TEC≥128 or REC≥128 (≤255) | Transmits Passive Error Frames (6 recessive), extra idle wait |
| **Bus-Off** | TEC>255 | Disconnects; recovery needs 128×11 consecutive recessive bits or software reset |

`CAN_ESR.LEC[2:0]`:

| LEC | Meaning |
|---|---|
| 0 | No error |
| 1 | Stuff Error |
| 2 | Form Error (fixed-form field violated) |
| 3 | Acknowledgement Error |
| 4 | Bit Recessive Error (sent 1, read back 0) |
| 5 | Bit Dominant Error (sent 0, read back 1) |
| 6 | CRC Error |
| 7 | Set by software |

`EWGF` sets at TEC/REC ≥ 96 (early warning). `BOFF` sets on Bus-Off entry.

---

## 5. Comparative Matrix

| Property | UART | I²C | SPI | CAN (bxCAN) |
|---|---|---|---|---|
| **Wire count** | 2 (+flow ctrl) | 2 (shared bus) | ≥4 (1 CS/slave) | 2 (differential bus) |
| **Topology** | Point-to-point | Multi-master bus | Single master, star | Multi-master broadcast bus |
| **Duplex** | Full | Half | Full | Half (broadcast) |
| **Clock** | None (independent, matched) | Shared SCL (stretchable) | Shared SCK (master-driven) | None (self-clocking via stuffing edges) |
| **How fields are known** | Bit-position counting from validated start-edge | Byte-position counting from START (1st byte=addr, rest=data) | Pure configuration agreement; no in-band fields at all | Bit-position counting from SOF + one dynamic field (DLC) |
| **Addressing** | None | 7/10-bit slave address | Physical CS line | Message ID (content-based) |
| **Arbitration** | N/A | Multi-master clock+data arbitration | N/A | Non-destructive, ID-priority |
| **Typical max speed (F4)** | ~10.5 Mbps | 400 kHz | Up to 42 Mbps | 1 Mbps |
| **Error detection** | Parity, framing, overrun | ACK/NACK, bus/arbitration error | None built-in (optional CRC) | CRC-15, ACK, stuff, form, bit-monitor (5 checks) |
| **Error recovery** | Software retry | Software retry/bus reset | Software retry | Automatic hardware retransmit + fault confinement |
| **Bit stuffing** | No | No | No | Yes (every 5 identical bits) |
| **Pull-ups required** | No | Yes (mandatory) | No | No |

---

## 6. Worked Numeric Examples — Quick Reference Table

| Protocol | Target | Peripheral Clock | Key Register Values | Actual Result |
|---|---|---|---|---|
| UART | 115200 baud, OVER8=0 | 16 MHz | BRR = 0x008B (Mantissa=8, Frac=11) | 115,107.9 baud (−0.08%) |
| UART | 921600 baud, OVER8=1 | 84 MHz | BRR = 0x00B3 (Mantissa=11, Frac=3) | 923,076 baud (+0.16%) |
| I²C | 100 kHz Standard Mode | 42 MHz | CCR=210 (0xD2), TRISE=43 (0x2B) | 100.0 kHz exact |
| I²C | 400 kHz Fast Mode, duty=2 | 42 MHz | CCR=35, TRISE=14 (0x0E) | 400.0 kHz exact |
| SPI | ~10 MHz (sensor max 10.5 MHz) | 84 MHz | BR=010 (÷8) or BR=011 (÷16, safer) | 10.5 MHz or 5.25 MHz |
| CAN | 500 kbit/s | 42 MHz | BRP_field=5, TS1=8, TS2=3, SJW=1 → BTR=0x0138_0005 | 500,000 bit/s exact, 71.4% sample point |

---

## 7. Study Notes / Common Pitfalls

1. **UART:** always read `SR` before `DR` to clear ORE/FE/NE/PE.
2. **UART:** because timing is re-derived from scratch every frame from the start-bit edge, baud mismatch tends to look fine for the first few bits and then specifically break the *stop bit* check — that's not a coincidence, it's the accumulated-drift mechanism from §1.4 step 3.
3. **I²C:** `f_PCLK1` has hard minimums (2 MHz Sm / 4 MHz Fm).
4. **I²C:** a bus-lock (SDA stuck low) needs manual GPIO bit-banging (9 clock pulses) to recover.
5. **I²C:** remember the address/data distinction is purely positional (§2.3) — there is no way to "resend just the address" mid-transaction; a repeated-START is required to reset the positional counter back to "expect address."
6. **SPI:** CPOL/CPHA mismatch produces byte-shifted, seemingly-random data with **no hardware error flag**, because SPI never had explicit fields to validate against in the first place (§3.3).
7. **SPI:** there is no flow control or ACK — any "my slave isn't ready yet" handling must be built in software on top of SPI.
8. **CAN:** the sample point is a design choice within a valid (BRP,TS1,TS2) combination, not uniquely determined by bit rate.
9. **CAN:** Bus-Off is a hardware self-disconnect, not a software flag you can ignore.
10. **All protocols:** peripheral clock (`PCLK1`/`PCLK2`) must be computed from your actual `RCC` tree (HSI/HSE→PLL→AHB/APB prescalers) before any baud math above is valid.

---

*Compiled as a study reference against ST RM0090 (STM32F405/415, STM32F407/417, STM32F427/437, STM32F429/439 Reference Manual) register definitions and the Bosch CAN 2.0B / I²C-bus / SPI protocol specifications. Always cross-check exact bit offsets against the specific STM32F4 part's datasheet/reference manual revision before implementing on real hardware.*
