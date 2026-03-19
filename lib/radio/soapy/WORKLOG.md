# SoapySDR Radio Driver — Work Log

## Session 2026-03-10

All implementation tasks (1–9) completed in a single session.

---

### Task 1 — CMakeLists + build wiring

**Files:** `lib/radio/soapy/CMakeLists.txt`, `lib/radio/CMakeLists.txt`, `lib/radio/radio_factory.cpp`

- Created `soapy/CMakeLists.txt` defining `ocudu_radio_soapy` static library with sources
  `radio_config_soapy_validator.cpp`, `radio_soapy_impl.cpp`, `radio_soapy_rx_stream.cpp`,
  `radio_soapy_tx_stream.cpp`. Links against `${SOAPYSDR_LIBRARIES}`, `fmt`, `ocudulog`,
  `ocudu_support`.
- Added `SOAPYSDR_FOUND AND ENABLE_SOAPY` guard block in `lib/radio/CMakeLists.txt` to
  conditionally build the soapy subdirectory and append definitions/libraries.
- Registered `{"soapy", ...}` entry in `radio_factory.cpp` factory map under `#ifdef ENABLE_SOAPY`.

---

### Task 2 — `radio_soapy_exception_handler.h`

**File:** `lib/radio/soapy/radio_soapy_exception_handler.h`

- Header-only `soapy_exception_handler` base class.
- `safe_execution<F>(F&&)` template: calls the callable in a `try/catch(std::exception&)`, stores
  the exception message in `last_error`, returns `bool`.
- `get_error_message()` accessor. No Boost dependency (unlike the UHD version).

---

### Task 3 — `radio_soapy_device.h`

**File:** `lib/radio/soapy/radio_soapy_device.h`

- Thin wrapper around `SoapySDR::Device*`, inheriting `soapy_exception_handler`.
- Methods: `make`, `unmake`, `is_valid`, `set_sample_rate`, `set_frequency`, `set_gain`,
  `get_hardware_time`, `setup_stream` (always CS16), `activate_stream`, `deactivate_stream`,
  `close_stream`, `get_stream_mtu`, `acquire_write_buffer`, `release_write_buffer`,
  `acquire_read_buffer`, `release_read_buffer`, `read_stream_status`.
- Each method is wrapped in `safe_execution` to handle SoapySDR exceptions gracefully.
- Destructor calls `unmake()`.

---

### Task 4 — `radio_config_soapy_validator`

**Files:** `lib/radio/soapy/radio_config_soapy_validator.h`, `radio_config_soapy_validator.cpp`

- Validates: exactly 1 TX and 1 RX stream; 1–2 channels each; TX freq 47 MHz–6 GHz; RX freq
  70 MHz–6 GHz; TX gain −89–0 dB; RX gain 0–76 dB; sample rate 0.5–61.44 MSPS; OTW format
  SC16 or DEFAULT only; no `continuous` TX mode (M2SDR TimedTX requires burst mode).

---

### Task 5 — `radio_soapy_tx_stream_fsm.h`

**File:** `lib/radio/soapy/radio_soapy_tx_stream_fsm.h`

- Mirrors `radio_uhd_tx_stream_fsm.h` adapted for SoapySDR.
- States: `UNINITIALIZED → START_BURST → IN_BURST → END_OF_BURST → WAIT_END_OF_BURST`.
- Uses `long long time_ns` and integer SoapySDR flags (`SOAPY_SDR_HAS_TIME`,
  `SOAPY_SDR_END_BURST`) instead of `uhd::tx_metadata_t`.
- `WAIT_EOB_ACK_TIMEOUT_NS = 10 ms`. `on_stop()` returns true if burst was open (so caller
  sends a zero-length EOB flush).

---

### Task 6 — `radio_soapy_tx_stream`

**Files:** `lib/radio/soapy/radio_soapy_tx_stream.h`, `radio_soapy_tx_stream.cpp`

- `stream_description`: id, srate_hz, nof_channels, discontinuous_tx, power_ramping_us.
- `transmit()`: converts `ts` → `time_ns`, runs FSM, optionally sends zero-filled
  power-ramping buffers via `acquireWriteBuffer` before burst, then sends data chunks
  MTU-at-a-time using zero-copy DMA path. Notifies SOB / EOB events.
- `recv_async_msg()`: calls `readStreamStatus()`, maps `SOAPY_SDR_TIME_ERROR → LATE`,
  `SOAPY_SDR_UNDERFLOW → UNDERFLOW`, calls `state_fsm.async_event_late_underflow()`.
- `run_recv_async_msg()`: self-rescheduling via `async_executor.defer()` with stop token.
- `start()`: resets stop control, enqueues first `run_recv_async_msg` call.
- `stop()`: signals stop, sends zero-length EOB flush if burst was open.

---

### Task 7 — `radio_soapy_rx_stream`

**Files:** `lib/radio/soapy/radio_soapy_rx_stream.h`, `radio_soapy_rx_stream.cpp`

- `receive()`: loops `acquireReadBuffer` until `nof_samples` filled.
  - Captures timestamp from first block with `SOAPY_SDR_HAS_TIME`; falls back to
    `last_time_ns` continuity reconstruction.
  - Handles `SOAPY_SDR_OVERFLOW`: notifies OVERFLOW event, continues (driver re-syncs).
  - Bails after `MAX_TIMEOUT_COUNT = 10` consecutive empty reads.
  - Updates `last_time_ns` after each chunk for continuity.
- `start(time_ns)` / `stop()`: activate / deactivate the underlying SoapySDR stream.

---

### Task 8 — `radio_soapy_baseband_gateway.h`

**File:** `lib/radio/soapy/radio_soapy_baseband_gateway.h`

- Owns `unique_ptr<radio_soapy_tx_stream>` and `unique_ptr<radio_soapy_rx_stream>`.
- Implements `baseband_gateway`: `get_transmitter()`, `get_receiver()`,
  `get_transmitter_optimal_buffer_size()`, `get_receiver_optimal_buffer_size()`.
- `get_tx_stream()` / `get_rx_stream()` accessors for the session impl.
- `is_successful()`: returns true if both stream pointers are non-null.

---

### Task 9 — `radio_soapy_impl` (session + factory)

**Files:** `lib/radio/soapy/radio_soapy_impl.h`, `radio_soapy_impl.cpp`

- `radio_session_soapy_impl`: constructor flow:
  1. `device.make(args)` — open SoapySDR device.
  2. Per TX channel: `set_sample_rate`, `set_frequency`, `set_gain`; populate `tx_port_map`.
  3. Per RX channel: `set_sample_rate`, `set_frequency`, `set_gain`; populate `rx_port_map`.
  4. Per stream pair: build channel index lists, parse kwargs from `stream.args` via
     `SoapySDR::KwargsFromString`, call `device.setup_stream(TX)` and `device.setup_stream(RX)`,
     construct `radio_soapy_tx_stream` and `radio_soapy_rx_stream`, wrap in
     `radio_soapy_baseband_gateway`.
- `start(init_time)`: starts TX async polling task for each gateway; activates RX streams
  with `init_time_ns = init_time * 1e9 / srate_hz`.
- `stop()`: calls `tx_stream.stop()` then `rx_stream.stop()` for each gateway.
- `read_current_time()`: `device.get_hardware_time()` → `ns * srate_hz / 1e9`.
- `set_tx_freq(stream_id, freq)` / `set_rx_freq(stream_id, freq)`: iterate port map,
  call `device.set_frequency` for all ports of the matching stream.
- `radio_factory_soapy_impl`: returns a `radio_config_soapy_validator` and creates
  `radio_session_soapy_impl`, returning nullptr on failure.

---

### Pending

- **Task 10** — Integration test with M2SDR hardware (streaming, timestamp accuracy,
  late/underflow events, TDD burst mode).
