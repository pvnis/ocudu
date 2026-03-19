# SoapySDR Radio Driver for ocudu-pavo

## Overview

This driver implements the ocudu-pavo `radio_session` interface using SoapySDR as the
backend, targeting the LiteX-M2SDR board via its existing SoapySDR plugin
(`driver=LiteXM2SDR`). It is a structural parallel to `lib/radio/uhd/`, replacing
UHD API calls with SoapySDR API calls and adapting timestamp handling accordingly.

## Directory Layout

```
lib/radio/soapy/
├── CMakeLists.txt
├── PLAN.md                          (this file)
├── radio_config_soapy_validator.h
├── radio_config_soapy_validator.cpp
├── radio_soapy_baseband_gateway.h
├── radio_soapy_device.h
├── radio_soapy_exception_handler.h
├── radio_soapy_impl.h
├── radio_soapy_impl.cpp
├── radio_soapy_rx_stream.h
├── radio_soapy_rx_stream.cpp
├── radio_soapy_tx_stream.h
├── radio_soapy_tx_stream.cpp
└── radio_soapy_tx_stream_fsm.h
```

## Class Hierarchy

```
radio_factory  (abstract, include/ocudu/radio/radio_factory.h)
  └─ radio_factory_soapy_impl          [radio_soapy_impl.h]
       └─ create() → radio_session_soapy_impl

radio_session  (abstract)
  └─ radio_session_soapy_impl          [radio_soapy_impl.h/.cpp]
       ├─ radio_management_plane impl  (gain/freq via SoapySDR)
       ├─ read_current_time()          (hardware time → samples)
       ├─ start(init_time)             (activate TX+RX streams)
       ├─ stop()                       (deactivate, cleanup)
       └─ baseband_gateways[]
            └─ radio_soapy_baseband_gateway   [radio_soapy_baseband_gateway.h]
                 ├─ radio_soapy_tx_stream     [radio_soapy_tx_stream.h/.cpp]
                 └─ radio_soapy_rx_stream     [radio_soapy_rx_stream.h/.cpp]
```

## Timestamp Handling

ocudu-pavo uses **sample-based** timestamps (`uint64_t` in sample counts at the
configured sample rate). The M2SDR SoapySDR driver uses **nanosecond** timestamps
(`long long timeNs`). All conversion happens at the streaming boundary:

```
ocudu timestamp (samples) ──→ timeNs = samples * 1e9 / sample_rate_Hz
timeNs ──→ ocudu timestamp (samples) = timeNs * sample_rate_Hz / 1e9
```

For `read_current_time()`: call `SoapySDR::Device::getHardwareTime("")`, convert to
samples.

## TX Streaming

Uses the zero-copy DMA path: `acquireWriteBuffer` → fill → `releaseWriteBuffer`.

- **Flags**: `SOAPY_SDR_HAS_TIME` when `tx_metadata.ts > 0`, `SOAPY_SDR_END_BURST`
  when `tx_metadata.tx_end` is set.
- **Timestamp**: passed as `timeNs` to `releaseWriteBuffer`. The M2SDR gateware's
  `TimedTXArbiter` holds the burst until `time_gen.time >= timeNs`.
- **Power ramping**: if `radio_config.power_ramping_us > 0`, transmit zero-filled
  buffers (aligned to MTU) with a time adjusted earlier by
  `power_ramping_nof_samples / sample_rate_Hz` before the first data buffer.
- **Burst FSM** (`radio_soapy_tx_stream_fsm.h`): mirrors `radio_uhd_tx_stream_fsm.h`.
  States: `UNINITIALIZED → START_BURST → IN_BURST → END_OF_BURST →
  WAIT_END_OF_BURST`. On late/underflow: force EOB, wait for ACK, restart.
- **Async error polling**: background task via `task_executor.defer()` calling
  `readStreamStatus()` at ~1 kHz. Maps return codes to `radio_event_type`:
  - `SOAPY_SDR_TIME_ERROR` → `radio_event_type::LATE`
  - `SOAPY_SDR_UNDERFLOW` → `radio_event_type::UNDERFLOW`

## RX Streaming

Uses zero-copy DMA path: `acquireReadBuffer` → copy → `releaseReadBuffer`.

- Timestamp from first block's `timeNs` (with `SOAPY_SDR_HAS_TIME` flag), converted
  to samples.
- Loop until `nof_samples` is filled (may span multiple DMA buffers).
- `SOAPY_SDR_OVERFLOW` from `acquireReadBuffer` → notify `radio_event_type::OVERFLOW`.
- Timeout after 10 consecutive empty reads (mirrors UHD behaviour).

## Initialization Sequence

```
SoapySDR::Device::make(config.args)          // e.g. "driver=LiteXM2SDR"
device->setSampleRate(TX, 0, srate_Hz)
device->setSampleRate(RX, 0, srate_Hz)
for each TX channel:
    device->setFrequency(TX, ch, freq_Hz)
    device->setGain(TX, ch, gain_dB)
for each RX channel:
    device->setFrequency(RX, ch, freq_Hz)
    device->setGain(RX, ch, gain_dB)
tx_stream = device->setupStream(TX, "CS16", channels, kwargs)
rx_stream = device->setupStream(RX, "CS16", channels, kwargs)
```

`start(init_time)`:
```
device->activateStream(rx_stream, SOAPY_SDR_HAS_TIME, init_time_ns)
device->activateStream(tx_stream, 0, 0)
launch async TX error polling task
```

`stop()`:
```
stop async polling task
device->deactivateStream(tx_stream, SOAPY_SDR_END_BURST)
device->deactivateStream(rx_stream)
device->closeStream(tx_stream)
device->closeStream(rx_stream)
SoapySDR::Device::unmake(device)
```

## Configuration Validator

Checks performed in `radio_config_soapy_validator`:

| Parameter              | Constraint                                         |
|------------------------|----------------------------------------------------|
| nof TX/RX streams      | Exactly 1 (M2SDR has a single DMA path)            |
| nof channels per stream| 1 or 2 (AD9361 2T2R)                               |
| sample rate            | 0.5–61.44 MSPS                                     |
| TX center frequency    | 47 MHz – 6 GHz                                     |
| RX center frequency    | 70 MHz – 6 GHz                                     |
| TX gain                | −89 – 0 dB                                         |
| RX gain                | 0 – 76 dB                                          |
| OTW format             | SC16 or DEFAULT only                               |
| transmission_mode      | discontinuous or same_port (no continuous TX burst)|

## Build Integration

`lib/radio/CMakeLists.txt` gains:
```cmake
if (SOAPYSDR_FOUND AND ENABLE_SOAPY)
    add_subdirectory(soapy)
    list(APPEND OCUDU_RADIO_DEFINITIONS -DENABLE_SOAPY)
    list(APPEND OCUDU_RADIO_LIBRARIES ocudu_radio_soapy)
endif()
```

`lib/radio/radio_factory.cpp` gains:
```cpp
#ifdef ENABLE_SOAPY
    {"soapy", []() { return std::make_unique<radio_factory_soapy_impl>(); }},
#endif
```

`lib/radio/soapy/CMakeLists.txt`:
```cmake
set(SOURCES_SOAPY
    radio_config_soapy_validator.cpp
    radio_soapy_impl.cpp
    radio_soapy_rx_stream.cpp
    radio_soapy_tx_stream.cpp)

add_library(ocudu_radio_soapy STATIC ${SOURCES_SOAPY})
target_link_libraries(ocudu_radio_soapy ${SOAPYSDR_LIBRARIES} fmt ocudulog ocudu_support)
```

## Key Differences vs UHD Driver

| Aspect              | UHD                                  | SoapySDR                                  |
|---------------------|--------------------------------------|-------------------------------------------|
| Timestamp unit      | `time_spec_t` (s + frac)             | `long long timeNs` (nanoseconds)          |
| TX zero-copy        | UHD `send()` (copies internally)     | `acquireWriteBuffer`/`releaseWriteBuffer` |
| RX zero-copy        | UHD `recv()` (copies internally)     | `acquireReadBuffer`/`releaseReadBuffer`   |
| Async TX errors     | `recv_async_msg()`                   | `readStreamStatus()`                      |
| Clock sync          | set_sync_source + PPS alignment      | M2SDR handles via `m2sdr_rf` args/kwargs  |
| OTW format select   | SC8/SC12/SC16 per device type        | SC16 fixed (M2SDR native)                 |
| RTTI requirement    | Yes (Boost)                          | No                                        |
| Device args example | `"type=x300"`                        | `"driver=LiteXM2SDR"`                     |

## Open Questions / Decisions Needed

- **External 10 MHz / PPS sync**: M2SDR supports `-sync=external` via `m2sdr_rf`. Do we
  expose this as a `clock_sources` option, or leave it to out-of-band init (i.e. user
  runs `m2sdr_rf -sync=external` before starting ocudu)? Recommend: add kwargs passthrough.
- **TDD mode**: M2SDR has `tdd_enable`/`tdd_lead`/`tdd_trail` kwargs. These can be
  forwarded from `radio_configuration::radio::args` as SoapySDR stream kwargs.
- **Power ramping**: The M2SDR `TimedTXArbiter` already holds bursts until the timestamp;
  power ramping zeros are still needed to warm up the RF chain. Keep the UHD approach.
- **`read_current_time()` accuracy**: `getHardwareTime` reads via PCIe CSR; latency is
  ~1–2 µs. Acceptable for gNB scheduling (slot boundary is 500 µs at 30.72 MSPS).
