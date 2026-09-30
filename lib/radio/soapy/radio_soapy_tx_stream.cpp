// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Pavonis Communications
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI

#include "radio_soapy_tx_stream.h"
#include "ocudu/gateways/baseband/buffer/baseband_gateway_buffer_reader_view.h"
#include "ocudu/ocuduvec/zero.h"
#include <SoapySDR/Errors.hpp>
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>

using namespace ocudu;

/// Maximum time a single transmission keeps retrying writeStream timeouts. A timeout is back-pressure from the
/// device TX FIFO (samples queued that are not due yet); like the UHD adapter, which loops on send() until all
/// samples are accepted, the downlink thread must wait here rather than drop the block. Bounded so a dead device
/// cannot hang the thread forever (OCUDU_SOAPY_TX_MAX_BLOCK_MS overrides it).
static std::chrono::milliseconds max_write_block()
{
  static const std::chrono::milliseconds value = []() {
    long ms = 2000;
    if (const char* env = std::getenv("OCUDU_SOAPY_TX_MAX_BLOCK_MS")) {
      ms = std::max(1L, std::strtol(env, nullptr, 10));
    }
    return std::chrono::milliseconds(ms);
  }();
  return value;
}

/// Async status poll timeout in microseconds (1 ms).
static constexpr long RECV_ASYNC_TIMEOUT_US = 1000;

/// The LiteXM2SDR carries 12-bit samples, LSB-aligned in each 16-bit word. OCUDU uses the full 16-bit range.
/// \note The gateware truncates to 12 bits, so out-of-range samples wrap instead of clipping. In 8-bit mode (forced
/// above 61.44 MSps) the shift would have to be 8.
static constexpr unsigned DEVICE_SAMPLE_SHIFT = 4;
static constexpr int16_t  DEVICE_SAMPLE_MAX   = (1 << (15 - DEVICE_SAMPLE_SHIFT)) - 1;
static constexpr int16_t  DEVICE_SAMPLE_MIN   = -(1 << (15 - DEVICE_SAMPLE_SHIFT));

/// Rescales one OCUDU 16-bit sample component to the device sample width, clamping to the device range.
static inline int16_t to_device_sample(int16_t value)
{
  return std::clamp(static_cast<int16_t>(value >> DEVICE_SAMPLE_SHIFT), DEVICE_SAMPLE_MIN, DEVICE_SAMPLE_MAX);
}

/// Samples-to-nanoseconds helper.
static inline long long samples_to_ns(uint64_t samples, double srate_hz)
{
  return soapy_samples_to_api(samples, srate_hz);
}

radio_soapy_tx_stream::radio_soapy_tx_stream(radio_soapy_device&       device_,
                                               SoapySDR::Stream*         stream_,
                                               const stream_description& desc,
                                               task_executor&            async_executor_,
                                               radio_event_notifier&     notifier_) :
  stream_id(desc.id),
  async_executor(async_executor_),
  notifier(notifier_),
  device(device_),
  stream(stream_),
  srate_hz(desc.srate_hz),
  nof_channels(desc.nof_channels),
  discontinuous_tx(desc.discontinuous_tx),
  power_ramping_buffer(desc.nof_channels, 0),
  tx_scaled_buffer(desc.nof_channels, static_cast<unsigned>(desc.srate_hz / 1000)),
  logger(ocudulog::fetch_basic_logger("RF"))
{
  ocudu_assert(std::isnormal(srate_hz) && srate_hz > 0.0, "Invalid sampling rate {}.", srate_hz);
  ocudu_assert(stream != nullptr, "TX stream must not be null.");

  if (const char* env = std::getenv("OCUDU_SOAPY_TX_TRACE")) {
    tx_trace_enabled = std::string_view(env) != "0";
  }
  if (const char* env = std::getenv("OCUDU_SOAPY_TX_TRACE_US")) {
    tx_trace_threshold_us = std::strtol(env, nullptr, 10);
    if (tx_trace_threshold_us <= 0) {
      tx_trace_threshold_us = 100;
    }
  }
  if (const char* env = std::getenv("OCUDU_SOAPY_TX_WRITE_TIMEOUT_US")) {
    write_timeout_us = std::strtol(env, nullptr, 10);
    if (write_timeout_us < 0) {
      write_timeout_us = 0;
    }
  }

  if (const char* env = std::getenv("OCUDU_SOAPY_TX_DUMP")) {
    tx_dump_base = env;
    tx_dump_path = fmt::format("{}.0", tx_dump_base);
    double skip_s = 2.0;
    if (const char* skip_env = std::getenv("OCUDU_SOAPY_TX_DUMP_SKIP_S")) {
      skip_s = std::strtod(skip_env, nullptr);
    }
    tx_dump_skip = static_cast<uint64_t>(srate_hz * skip_s);
  }

  mtu = device.get_stream_mtu(stream);

  if (discontinuous_tx && desc.power_ramping_us > 0.0f) {
    power_ramping_nof_samples =
        static_cast<unsigned>(srate_hz * static_cast<double>(desc.power_ramping_us) / 1e6);
    // Align to MTU.
    power_ramping_nof_samples = (power_ramping_nof_samples / static_cast<unsigned>(mtu)) * static_cast<unsigned>(mtu);

    double aligned_us = static_cast<double>(power_ramping_nof_samples) * 1e6 / srate_hz;
    fmt::print("SoapySDR TX: power ramping guard aligned to {} samples ({:.1f} us).\n",
               power_ramping_nof_samples,
               aligned_us);

    power_ramping_buffer.resize(power_ramping_nof_samples);
    for (unsigned ch = 0; ch != nof_channels; ++ch) {
      ocuduvec::zero(power_ramping_buffer.get_writer()[ch]);
    }
  }

  state_fsm.init_successful();
}

void radio_soapy_tx_stream::recv_async_msg()
{
  size_t    chan_mask = 0;
  int       flags     = 0;
  long long time_ns   = 0;

  int ret = device.read_stream_status(stream, chan_mask, flags, time_ns, RECV_ASYNC_TIMEOUT_US);

  radio_event_notifier::event_description event = {.stream_id  = stream_id,
                                                   .channel_id = std::nullopt,
                                                   .source     = radio_event_source::TRANSMIT,
                                                   .type       = radio_event_type::UNDEFINED,
                                                   .timestamp  = std::nullopt};

  if (ret == SOAPY_SDR_TIME_ERROR) {
    event.type            = radio_event_type::LATE;
    event.timestamp       = static_cast<uint64_t>(soapy_api_to_samples(time_ns, srate_hz));
    state_fsm.async_event_late_underflow(soapy_api_to_ns(time_ns, srate_hz));
  } else if (ret == SOAPY_SDR_UNDERFLOW) {
    event.type            = radio_event_type::UNDERFLOW;
    state_fsm.async_event_late_underflow(soapy_api_to_ns(time_ns, srate_hz));
  }
  // ret == 0 is timeout (no event) — ignore.

  if (event.type != radio_event_type::UNDEFINED) {
    notifier.on_radio_rt_event(event);
  }
}

void radio_soapy_tx_stream::run_recv_async_msg()
{
  auto token = stop_control.get_token();
  if (OCUDU_UNLIKELY(token.is_stop_requested())) {
    return;
  }

  recv_async_msg();

  report_error_if_not(async_executor.defer([this, tk = std::move(token)]() { run_recv_async_msg(); }),
                      "Unable to run SoapySDR async TX stream task");
}

void radio_soapy_tx_stream::transmit(const baseband_gateway_buffer_reader&        data,
                                      const baseband_gateway_transmitter_metadata& tx_md)
{
  const auto tx_start_tp = std::chrono::steady_clock::now();
  auto token = stop_control.get_token();
  if (OCUDU_UNLIKELY(token.is_stop_requested())) {
    return;
  }

  const bool tx_start_padding = tx_md.tx_start.has_value();
  const bool tx_end_padding   = tx_md.tx_end.has_value();

  // Compute TX timestamp in nanoseconds.
  long long time_ns = samples_to_ns(tx_md.ts, srate_hz);
  if (discontinuous_tx && tx_start_padding) {
    time_ns = samples_to_ns(tx_md.ts + static_cast<baseband_gateway_timestamp>(tx_md.tx_start.value()), srate_hz);
  }

  int  flags    = 0;
  bool transmit = false;
  if (discontinuous_tx) {
    transmit = state_fsm.on_transmit(flags, soapy_api_to_ns(time_ns, srate_hz), tx_md.is_empty, tx_end_padding);
  } else {
    transmit = state_fsm.on_transmit(flags, soapy_api_to_ns(time_ns, srate_hz), false, false);
  }

  if (!transmit) {
    return;
  }

  const bool is_sob = (flags & SOAPY_SDR_HAS_TIME) != 0;
  const bool is_eob = (flags & SOAPY_SDR_END_BURST) != 0;

  // Notify start of burst.
  if (is_sob) {
    notifier.on_radio_rt_event({.stream_id  = stream_id,
                                .channel_id = 0,
                                .source     = radio_event_source::TRANSMIT,
                                .type       = radio_event_type::START_OF_BURST,
                                .timestamp  = static_cast<uint64_t>(soapy_api_to_samples(time_ns, srate_hz))});

    // Transmit power-ramping zeros before the burst.
    if (discontinuous_tx && power_ramping_nof_samples > 0) {
      const unsigned tx_gap_samples =
          static_cast<unsigned>(soapy_api_to_samples(time_ns - last_tx_time_ns, srate_hz));
      const unsigned min_gap = static_cast<unsigned>(srate_hz / 100000.0); // 10 µs

      if (tx_gap_samples > min_gap) {
        unsigned nof_pad = std::min(power_ramping_nof_samples, tx_gap_samples - min_gap);
        if (nof_pad > 0) {
          long long pad_time_ns = time_ns - samples_to_ns(nof_pad, srate_hz);
          std::array<const void*, RADIO_MAX_NOF_CHANNELS> pad_buffs = {};
          for (unsigned ch = 0; ch != nof_channels; ++ch) {
            pad_buffs[ch] = power_ramping_buffer.get_reader()[ch].data();
          }
          int pad_flags = SOAPY_SDR_HAS_TIME;
          const int ret = device.write_stream(stream, pad_buffs.data(), nof_pad, pad_flags, pad_time_ns, write_timeout_us);
          if (ret != static_cast<int>(nof_pad)) {
            if (ret == SOAPY_SDR_TIMEOUT) {
              logger.warning("SoapySDR TX: power ramping writeStream timeout after {} us; expected {} samples.",
                             write_timeout_us,
                             nof_pad);
            } else {
              logger.warning("SoapySDR TX: power ramping writeStream failed ret={} expected={}.", ret, nof_pad);
            }
            return;
          }

          // The actual burst is no longer the start-of-burst for SoapySDR
          // (we already opened the burst above with the first padding write).
          flags &= ~SOAPY_SDR_HAS_TIME;
        }
      }
    }
  }

  // Notify end of burst (before sending, so the scheduler knows).
  if (is_eob) {
    notifier.on_radio_rt_event({.stream_id  = stream_id,
                                .channel_id = 0,
                                .source     = radio_event_source::TRANSMIT,
                                .type       = radio_event_type::END_OF_BURST,
                                .timestamp  = static_cast<uint64_t>(soapy_api_to_samples(time_ns, srate_hz))});
  }

  // Determine the sample range within the buffer.
  const unsigned data_start =
      (discontinuous_tx && tx_start_padding) ? tx_md.tx_start.value() : 0;
  const unsigned data_nof_samples =
      (discontinuous_tx && tx_end_padding)
          ? tx_md.tx_end.value() - data_start
          : data.get_nof_samples() - data_start;

  if (tx_md.is_empty && discontinuous_tx) {
    // Nothing to write; burst was already closed via flag.
    return;
  }

  // Rescale the samples from the OCUDU 16-bit range to the device sample width.
  if (tx_scaled_buffer.get_nof_samples() < data_nof_samples) {
    tx_scaled_buffer.resize(data_nof_samples);
  }
  for (unsigned ch = 0; ch != nof_channels; ++ch) {
    span<const ci16_t> src = data[ch].subspan(data_start, data_nof_samples);
    span<ci16_t>       dst = tx_scaled_buffer.get_writer()[ch];
    for (unsigned i = 0; i != data_nof_samples; ++i) {
      dst[i] = {to_device_sample(src[i].real()), to_device_sample(src[i].imag())};
    }
  }

  // SIGUSR1 starts a new capture immediately.
  if (!tx_dump_base.empty() && tx_dump_gen != soapy_debug_dump_generation.load(std::memory_order_relaxed)) {
    tx_dump_gen      = soapy_debug_dump_generation.load(std::memory_order_relaxed);
    tx_dump_path     = fmt::format("{}.{}", tx_dump_base, tx_dump_gen);
    tx_dump_start_ts = 0;
    tx_dump_skip     = 0;
  }
  if (!tx_dump_path.empty()) {
    // Skip the configured time, then capture a timeline with zeros between bursts.
    const uint64_t ts      = tx_md.ts + data_start;
    double         dump_ms = 100.0;
    if (const char* env = std::getenv("OCUDU_SOAPY_TX_DUMP_MS")) {
      dump_ms = std::strtod(env, nullptr);
    }
    const uint64_t skip     = tx_dump_skip;
    const size_t   dump_len = static_cast<size_t>(srate_hz * dump_ms / 1000.0);
    if (tx_dump_start_ts == 0) {
      tx_dump_start_ts = ts + skip;
      tx_dump.assign(dump_len, ci16_t());
    }
    if (ts >= tx_dump_start_ts) {
      const uint64_t offset = ts - tx_dump_start_ts;
      if (offset < dump_len) {
        const size_t n = std::min<size_t>(data_nof_samples, dump_len - offset);
        std::copy_n(tx_scaled_buffer.get_reader()[0].begin(), n, tx_dump.begin() + offset);
        ++tx_dump_count;
      } else {
        if (FILE* f = std::fopen(tx_dump_path.c_str(), "wb")) {
          std::fwrite(tx_dump.data(), sizeof(ci16_t), tx_dump.size(), f);
          std::fclose(f);
        }
        logger.info("Soapy TX dump: wrote {} samples from ts={} ({} bursts) to {}.",
                    tx_dump.size(),
                    tx_dump_start_ts,
                    tx_dump_count,
                    tx_dump_path);
        fmt::print("Soapy TX dump: wrote {} samples from ts={} to {}.\n", tx_dump.size(), tx_dump_start_ts, tx_dump_path);
        tx_dump_path.clear();
        tx_dump_count = 0;
      }
    }
  }

  // Transmit lead: how far ahead of the hardware time a block is handed to the driver. Sampled about 20 times per
  // second and reported every 5 seconds; the first number is the minimum (the margin that matters).
  tx_lead_counter += data_nof_samples;
  if (tx_lead_counter >= static_cast<uint64_t>(srate_hz / 20)) {
    tx_lead_counter = 0;
    long long hw_now = 0;
    if (device.get_hardware_time(hw_now)) {
      const long long lead_us = soapy_api_to_ns(time_ns - hw_now, srate_hz) / 1000;
      tx_lead_min = (tx_lead_count == 0) ? lead_us : std::min(tx_lead_min, lead_us);
      tx_lead_max = (tx_lead_count == 0) ? lead_us : std::max(tx_lead_max, lead_us);
      tx_lead_sum += lead_us;
      if (++tx_lead_count >= 100) {
        fmt::print("Soapy TX lead: stamps are {} us ahead of hardware time at least (avg {} / max {} us over {} "
                   "samples).\n",
                   tx_lead_min,
                   tx_lead_sum / tx_lead_count,
                   tx_lead_max,
                   tx_lead_count);
        tx_lead_count = 0;
        tx_lead_sum   = 0;
      }
    }
  }

  unsigned sent_total      = 0;
  unsigned chunks          = 0;
  unsigned   timeout_retries = 0;
  const auto block_deadline  = std::chrono::steady_clock::now() + max_write_block();
  while (sent_total < data_nof_samples) {
    std::array<const void*, RADIO_MAX_NOF_CHANNELS> rd_buffs = {};
    const unsigned remaining = data_nof_samples - sent_total;
    for (unsigned ch = 0; ch != nof_channels; ++ch) {
      rd_buffs[ch] = tx_scaled_buffer.get_reader()[ch].subspan(sent_total, remaining).data();
    }

    int chunk_flags = 0;
    if (sent_total == 0) {
      chunk_flags |= (flags & SOAPY_SDR_HAS_TIME);
    }
    chunk_flags |= (flags & SOAPY_SDR_END_BURST);

    const long long chunk_time_ns = time_ns + samples_to_ns(sent_total, srate_hz);
    const int ret = device.write_stream(stream, rd_buffs.data(), remaining, chunk_flags, chunk_time_ns, write_timeout_us);
    if (ret <= 0) {
      if (ret == SOAPY_SDR_TIMEOUT || ret == 0) {
        // A timeout is backpressure from a full TX ring, not an error. Retry until the stream is stopped or the wait
        // exceeds the maximum, since dropping samples would break the continuous TX timeline.
        ++timeout_retries;
        if (!token.is_stop_requested() && std::chrono::steady_clock::now() < block_deadline) {
          continue;
        }
        logger.warning("SoapySDR TX: writeStream timed out {} times ({} us each) for stream {}.",
                       timeout_retries,
                       write_timeout_us,
                       stream_id);
      } else {
        logger.warning("SoapySDR TX: writeStream failed ret={} for stream {}.", ret, stream_id);
      }
      return;
    }

    sent_total += static_cast<unsigned>(ret);
    ++chunks;
  }

  last_tx_time_ns = time_ns + samples_to_ns(sent_total, srate_hz);

  if (tx_trace_enabled) {
    const long dt_us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - tx_start_tp)
                           .count();
    if (dt_us >= tx_trace_threshold_us) {
      long long hw_now_ns = 0;
      long long lead_us   = 0;
      bool      hw_ok     = false;
      if (device.get_hardware_time(hw_now_ns)) {
        lead_us = soapy_api_to_ns(time_ns - hw_now_ns, srate_hz) / 1000;
        hw_ok   = true;
      }
      logger.info("Soapy TX trace: stream={} samples={} chunks={} flags=0x{:x} ts={} empty={} dt={}us hw_now_ns={} "
                  "tx_time_ns={} lead_us={}",
                  stream_id,
                  data_nof_samples,
                  chunks,
                  flags,
                  tx_md.ts,
                  tx_md.is_empty,
                  dt_us,
                  hw_ok ? hw_now_ns : -1LL,
                  time_ns,
                  hw_ok ? lead_us : -1LL);
    }
  }
}

void radio_soapy_tx_stream::start()
{
  stop_control.reset();
  if (!device.activate_stream(stream)) {
    logger.error("Error: failed to activate TX stream {}. {}", stream_id, device.get_error_message());
    return;
  }
  report_error_if_not(async_executor.defer([this, token = stop_control.get_token()]() { run_recv_async_msg(); }),
                      "Unable to start SoapySDR TX async task");
}

void radio_soapy_tx_stream::stop()
{
  stop_control.stop();

  if (state_fsm.on_stop()) {
    notifier.on_radio_rt_event({.stream_id  = stream_id,
                                .channel_id = 0,
                                .source     = radio_event_source::TRANSMIT,
                                .type       = radio_event_type::END_OF_BURST,
                                .timestamp  = std::nullopt});

    // Send a zero-length end-of-burst to flush.
    std::array<const void*, RADIO_MAX_NOF_CHANNELS> dummy_buffs = {};
    int flush_flags = SOAPY_SDR_END_BURST;
    (void)device.write_stream(stream, dummy_buffs.data(), 0, flush_flags, 0, write_timeout_us);
  }

  if (!device.deactivate_stream(stream)) {
    logger.error("Error: failed to deactivate TX stream {}. {}", stream_id, device.get_error_message());
  }
}
