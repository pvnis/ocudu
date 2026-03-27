// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Pavonis Communications
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI

#include "radio_soapy_rx_stream.h"
#include "ocudu/ocuduvec/zero.h"
#include <SoapySDR/Errors.hpp>
#include <chrono>
#include <cmath>
#include <cstring>
#include <cstdlib>

using namespace ocudu;

/// Maximum consecutive empty reads before bailing out of receive().
static constexpr unsigned MAX_TIMEOUT_COUNT = 10;

static inline long long samples_to_ns(uint64_t samples, double srate_hz)
{
  return static_cast<long long>(static_cast<double>(samples) * 1e9 / srate_hz);
}

radio_soapy_rx_stream::radio_soapy_rx_stream(radio_soapy_device&       device_,
                                               SoapySDR::Stream*         stream_,
                                               const stream_description& desc,
                                               radio_event_notifier&     notifier_) :
  id(desc.id), srate_hz(desc.srate_hz), nof_channels(desc.nof_channels), notifier(notifier_), device(device_), stream(stream_)
{
  ocudu_assert(std::isnormal(srate_hz) && srate_hz > 0.0, "Invalid sampling rate {}.", srate_hz);
  ocudu_assert(stream != nullptr, "RX stream must not be null.");
  mtu = device.get_stream_mtu(stream);
  if (const char* env = std::getenv("OCUDU_SOAPY_RX_TRACE")) {
    rx_trace_enabled = std::string_view(env) != "0";
  }
  if (const char* env = std::getenv("OCUDU_SOAPY_RX_TRACE_US")) {
    rx_trace_slow_us = std::strtol(env, nullptr, 10);
    if (rx_trace_slow_us <= 0) {
      rx_trace_slow_us = 200;
    }
  }
  if (const char* env = std::getenv("OCUDU_SOAPY_RX_TRACE_LIMIT")) {
    rx_trace_limit = std::strtoull(env, nullptr, 10);
  }
}

bool radio_soapy_rx_stream::start(long long time_ns)
{
  stop_control.reset();
  remainder_handle   = static_cast<size_t>(-1);
  remainder_samps    = 0;
  remainder_offset   = 0;
  remainder_flags    = 0;
  remainder_time_ns  = 0;
  last_sample_ts     = 0;
  last_sample_ts_valid = false;
  remainder_buffs.fill(nullptr);
  const int flags = (time_ns > 0) ? SOAPY_SDR_HAS_TIME : 0;
  if (!device.activate_stream(stream, flags, time_ns)) {
    logger.error("Error: failed to activate RX stream {}. {}", id, device.get_error_message());
    return false;
  }
  return true;
}

bool radio_soapy_rx_stream::stop()
{
  stop_control.stop();
  if (remainder_handle != static_cast<size_t>(-1)) {
    device.release_read_buffer(stream, remainder_handle);
    remainder_handle = static_cast<size_t>(-1);
  }
  remainder_samps    = 0;
  remainder_offset   = 0;
  remainder_flags    = 0;
  remainder_time_ns  = 0;
  last_sample_ts     = 0;
  last_sample_ts_valid = false;
  remainder_buffs.fill(nullptr);
  if (!device.deactivate_stream(stream)) {
    logger.error("Error: failed to deactivate RX stream {}. {}", id, device.get_error_message());
    return false;
  }
  return true;
}

baseband_gateway_receiver::metadata radio_soapy_rx_stream::receive(baseband_gateway_buffer_writer& buffs)
{
  const auto receive_start = std::chrono::steady_clock::now();
  baseband_gateway_receiver::metadata ret = {};

  auto token = stop_control.get_token();
  if (OCUDU_UNLIKELY(token.is_stop_requested())) {
    for (unsigned ch = 0; ch != buffs.get_nof_channels(); ++ch) {
      ocuduvec::zero(buffs[ch]);
    }
    return ret;
  }

  const unsigned nsamples           = buffs.get_nof_samples();
  unsigned       rxd_total          = 0;
  unsigned       timeout_count      = 0;
  unsigned       loops              = 0;
  unsigned       overflow_count     = 0;
  bool           timestamp_captured = false;

  while (rxd_total < nsamples) {
    ++loops;
    size_t handle = remainder_handle;
    std::array<const void*, RADIO_MAX_NOF_CHANNELS> rd_buffs = remainder_buffs;
    int flags = remainder_flags;
    long long time_ns = remainder_time_ns;
    int ret_samps = static_cast<int>(remainder_samps);

    if (remainder_handle == static_cast<size_t>(-1)) {
      handle = 0;
      rd_buffs = {};
      flags = 0;
      time_ns = 0;
      ret_samps = device.acquire_read_buffer(stream, handle, rd_buffs.data(), flags, time_ns);
    }

    if (ret_samps == SOAPY_SDR_OVERFLOW) {
      ++overflow_count;
      logger.debug("Overflow detected on RX stream {}.", id);
      notifier.on_radio_rt_event({.stream_id  = id,
                                  .channel_id = std::nullopt,
                                  .source     = radio_event_source::RECEIVE,
                                  .type       = radio_event_type::OVERFLOW,
                                  .timestamp  = std::nullopt});
      // Keep trying; the driver re-syncs after overflow.
      continue;
    }

    if (ret_samps == SOAPY_SDR_TIMEOUT || ret_samps == 0) {
      ++timeout_count;
      if (timeout_count >= MAX_TIMEOUT_COUNT) {
        logger.error("Error: exceeded maximum timeout count on RX stream {}.", id);
        return ret;
      }
      continue;
    }

    if (ret_samps < 0) {
      logger.error("Error: RX stream {} acquireReadBuffer returned {}.", id, ret_samps);
      return ret;
    }

    timeout_count = 0;
    const unsigned nchunk = static_cast<unsigned>(ret_samps);
    const unsigned source_offset = (remainder_handle == static_cast<size_t>(-1)) ? 0 : remainder_offset;

    // Capture timestamp from first block.
    if (!timestamp_captured) {
      if (flags & SOAPY_SDR_HAS_TIME) {
        const long long first_time_ns =
            time_ns + static_cast<long long>((static_cast<double>(source_offset) * 1e9) / srate_hz);
        last_time_ns       = first_time_ns;
        timestamp_captured = true;
        time_ns            = first_time_ns;
        ret.ts             = static_cast<baseband_gateway_timestamp>(std::llround(time_ns * srate_hz / 1e9));
      } else if (last_time_ns > 0 && std::isnormal(srate_hz)) {
        // Reconstruct from last known time + samples elapsed.
        time_ns            = last_time_ns;
        timestamp_captured = true;
        if (last_sample_ts_valid) {
          ret.ts = last_sample_ts;
        } else {
          ret.ts = static_cast<baseband_gateway_timestamp>(std::llround(time_ns * srate_hz / 1e9));
        }
      }
    }

    // Copy samples into the caller's buffer, clamped to remaining space.
    const unsigned copy_samps = std::min(nchunk, nsamples - rxd_total);
    for (unsigned ch = 0; ch != std::min(nof_channels, buffs.get_nof_channels()); ++ch) {
      const auto* src = reinterpret_cast<const ci16_t*>(rd_buffs[ch]) + source_offset;
      std::memcpy(buffs[ch].subspan(rxd_total, copy_samps).data(),
                  src,
                  copy_samps * sizeof(ci16_t));
    }

    if (copy_samps == nchunk) {
      device.release_read_buffer(stream, handle);
      remainder_handle = static_cast<size_t>(-1);
      remainder_samps = 0;
      remainder_offset = 0;
      remainder_flags = 0;
      remainder_time_ns = 0;
      remainder_buffs.fill(nullptr);
    } else {
      remainder_handle = handle;
      remainder_buffs = rd_buffs;
      remainder_samps = nchunk - copy_samps;
      remainder_offset = source_offset + copy_samps;
      remainder_flags = flags & ~SOAPY_SDR_HAS_TIME;
      remainder_time_ns = time_ns;
    }

    rxd_total += copy_samps;

    // Advance last known time by samples just consumed.
    if (timestamp_captured && std::isnormal(srate_hz)) {
      last_sample_ts = ret.ts + rxd_total;
      last_sample_ts_valid = true;
      last_time_ns = static_cast<long long>(std::llround(static_cast<double>(last_sample_ts) * 1e9 / srate_hz));
    }
  }

  if (rx_trace_enabled) {
    const long us = std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::steady_clock::now() - receive_start)
                        .count();
    const uint64_t seq = ++rx_trace_seq;
    if ((rx_trace_limit == 0 || seq <= rx_trace_limit) && us >= rx_trace_slow_us) {
      baseband_gateway_timestamp dts = 0;
      if (last_ret_ts_valid) {
        dts = ret.ts - last_ret_ts;
      }
      long long hw_now_ns = 0;
      long long lag_us    = 0;
      bool      hw_ok     = false;
      if (device.get_hardware_time(hw_now_ns)) {
        const long long ret_time_ns = samples_to_ns(ret.ts, srate_hz);
        lag_us = (hw_now_ns - ret_time_ns) / 1000;
        hw_ok  = true;
      }
      logger.info("Soapy RX receive: nsamples={} mtu={} loops={} overflows={} dt={}us ts={} dts={} hw_now_ns={} "
                  "lag_us={}",
                  nsamples,
                  mtu,
                  loops,
                  overflow_count,
                  us,
                  ret.ts,
                  dts,
                  hw_ok ? hw_now_ns : -1LL,
                  hw_ok ? lag_us : -1LL);
    }
  }

  if (rxd_total == nsamples) {
    last_ret_ts = ret.ts;
    last_ret_ts_valid = true;
  }

  return ret;
}
