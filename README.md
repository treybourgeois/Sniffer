# OLED Sniffer (Daisy Pod)

Turns a Daisy Pod into a passive I2C sniffer that mirrors an SSD1306 OLED
(128x64 or 128x32) onto a computer screen in real time.

```
Instrument (Daisy Seed) ──I2C──► SSD1306 OLED
                           │
                           └─ tap ─► Pod (sniffer.bin) ──USB──► viewer.html (Chrome / Edge)
```

The Pod only listens: its pins are inputs with no pull-ups, and it never
acknowledges or drives the bus. The instrument runs exactly as before.

## Wiring

Solder three wires at the OLED's header (or anywhere on those traces):

| OLED / instrument | Pod (on the Seed's pins) |
|---|---|
| SCL | **D11** (Seed pin 12) |
| SDA | **D12** (Seed pin 13) |
| GND | **GND** (e.g. Seed pin 40, DGND). Required. |

Do **not** connect VCC. Keep the wires short (under about 15 cm).

## Flashing the Pod

Either:

- **No toolchain:** put the Pod in bootloader mode (hold BOOT, tap RESET),
  open the Electro-Smith web programmer in Chrome, and upload `sniffer.bin`
  (from `build/`).
- **From source:** `make` then `make program-dfu` in this folder. The build expects
  [libDaisy](https://github.com/electro-smith/libDaisy) (built) at `../../libDaisy`,
  e.g. with this repo cloned into `DaisyExamples/pod/sniffer`. Otherwise pass
  `make LIBDAISY_DIR=path/to/libDaisy DAISYSP_DIR=path/to/DaisySP`.

## Using the viewer

1. Plug the Pod into the computer with its micro-USB port.
2. Serve `viewer.html` from a local web server and open it in **Chrome or Edge**,
   for example with VS Code's Live Server extension (`http://127.0.0.1:5500/viewer.html`).
   Opening the file straight from disk (`file:///…`) doesn't work, because browsers only
   allow Web Serial on `https://` or `localhost` pages.
3. Click **Connect to Pod** and pick the serial port that appears
   ("USB Serial Device" on Windows).
4. Use the instrument. The screen appears in the browser.

Controls: I2C address (0x3C is the usual one), panel height, pixel color, flip X/Y,
screenshot (PNG), video recording (MP4, which plays on iPhone; WebM on browsers
without MP4 support), and a decoded command log.

If you connect after the instrument has booted, the viewer has missed the
display's setup commands. It assumes the usual libDaisy settings, and the
picture fills in as soon as the instrument redraws. If it looks mirrored, use Flip X / Flip Y.

## Pod LEDs

- **LED 1 green:** I2C traffic seen.
- **LED 2 red:** bus data may have been lost recently (interrupt serviced too late,
  or internal buffer overflow). The viewer shows the counts.

## USB protocol (for other tools)

Packets are `A5 5A type len_lo len_hi payload[len] checksum`, where the checksum
is the 8-bit sum of type, len and payload.

- `type 0x01`: I2C chunk. `payload[0]` holds the flags (bit0 FIRST = the chunk
  starts a transaction and the next byte is the address byte; bit1 LAST =
  transaction ended; bit2 NACK). The rest are the bus bytes. Transactions
  longer than 1024 bytes are split across several chunks.
- `type 0x02`: stats, every 500 ms. Five u32 values: transactions, bytes,
  ring overflows, USB drops, late IRQs.
