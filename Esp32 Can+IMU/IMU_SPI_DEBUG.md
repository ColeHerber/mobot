# IMU SPI Debug

Use this for SPI signal-integrity troubleshooting (e.g. `WHO_AM_I` reads `0x00`/`0xFF`,
intermittent reads, or anything else that looks like a wiring/timing problem on the
SPI bus itself). The debug code supports both ISM330BX (`WHO_AM_I = 0x71`) and
ISM6HG256X (`WHO_AM_I = 0x73`) builds.

## Resolved: the earlier CS/I2C mystery

Earlier hardware revisions showed I2C ACKing even with CS held low, which should be
impossible if CS correctly gates the bus — that ambiguity is why an I2C fallback path
existed in the driver at all. A 300-sample bit-banged SPI mode-3 scope-loop run on the
current (reworked) board read a clean, consistent `WHO_AM_I` every time, proving SPI
alone is reliable. The driver no longer has an I2C path — this doc (and the debug
builds below) now only cover SPI-only troubleshooting.

## Build

OTA debug build:

```sh
pio run -e esp32-s3-wroom-1-ota-imu-spi-debug -t upload
```

ISM6HG256X maximum-debug serial build:

```sh
pio run -e esp32-s3-wroom-1-ism6hg256x-debug
```

Serial debug build:

```sh
pio run -e esp32-s3-wroom-1-imu-spi-debug -t upload
```

Scope-loop build:

```sh
pio run -e esp32-s3-wroom-1-ota-imu-spi-scope -t upload
```

## What It Runs

The debug firmware runs this sequence at boot:

1. Hardware SPI `WHO_AM_I` sweep at 100 kHz, 400 kHz, and 1 MHz in modes 3 and 0.
2. Slow bit-banged SPI `WHO_AM_I` sweep in modes 3 and 0.
3. Optional scope loop (`IMU_SPI_SCOPE_LOOP=1`): repeating slow bit-banged mode-3
   `WHO_AM_I` reads, meant for capturing the transaction on an oscilloscope/logic
   analyzer. `IMU_SPI_SCOPE_LOOP_COUNT` controls how many samples (0 = endless).

The expected SPI read transaction is:

- CS low
- MOSI first byte: `0x8F`
- 16 clocks total
- MISO second byte: `0x71` for ISM330BX or `0x73` for ISM6HG256X

## Interpret Results

- Hardware and bit-banged sweeps agreeing across all speeds/modes with the expected
  `WHO_AM_I` byte means the SPI bus is solid.
- Reads of `0x00`: likely MISO not reaching the host, or the IMU not powered.
- Reads of `0xFF`: likely MISO floating/pulled high with nothing driving it
  (chip not selected, CS not reaching the IMU, or a wiring fault).
- Inconsistent/flaky reads across the scope loop's samples: suspect a marginal
  solder joint or a signal integrity issue (try lowering SPI speed first).

## Important

After any test that changes the SPI peripheral's GPIO routing (the bit-bang tests
do this), the debug pass re-attaches the hardware SPI peripheral before returning —
but if you're experimenting further by hand, physically power-cycle the board before
trusting subsequent results.
