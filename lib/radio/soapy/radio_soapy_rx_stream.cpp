// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Pavonis Communications
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI

#include "radio_soapy_rx_stream.h"
#include "ocudu/ocuduvec/zero.h"
#include <SoapySDR/Errors.hpp>
#include <chrono>
#include <cmath>
#include <cstring>
#include <cstdio>
#include <cstdlib>

using namespace ocudu;

/// Maximum consecutive empty reads before bailing out of receive().
static constexpr unsigned MAX_TIMEOUT_COUNT = 10;

/// The LiteXM2SDR carries 12-bit samples, LSB-aligned in each 16-bit word. OCUDU uses the full 16-bit range.
static constexpr unsigned DEVICE_SAMPLE_SHIFT = 4;

static inline long long samples_to_ns(uint64_t samples, double srate_hz)
{
  return soapy_samples_to_api(samples, srate_hz);
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
  if (const char* env = std::getenv("OCUDU_SOAPY_RX_TS_SHIFT_FILE")) {
    rx_ts_shift_file = env;
  }
  if (const char* env = std::getenv("OCUDU_SOAPY_RX_TS_SHIFT")) {
    // Static RX timestamp shift in samples (label = hardware + shift), for radios whose DL emission
    // offset is a constant (hardware-timed TX): the RX-minus-TX offset measured once by align.py,
    // negated. The runtime file/SIGUSR2 mechanism still applies on top.
    rx_ts_shift = std::strtoll(env, nullptr, 10);
    fmt::print("Soapy RX: static timestamp shift {} samples (OCUDU_SOAPY_RX_TS_SHIFT).\n", rx_ts_shift);
  }
  if (const char* env = std::getenv("OCUDU_SOAPY_RX_DUMP")) {
    // Skip OCUDU_SOAPY_RX_DUMP_SKIP_S seconds (2 s by default), then capture OCUDU_SOAPY_RX_DUMP_MS milliseconds (100 ms by default).
    double dump_ms = 100.0;
    if (const char* ms_env = std::getenv("OCUDU_SOAPY_RX_DUMP_MS")) {
      dump_ms = std::strtod(ms_env, nullptr);
    }
    double skip_s = 2.0;
    if (const char* skip_env = std::getenv("OCUDU_SOAPY_RX_DUMP_SKIP_S")) {
      skip_s = std::strtod(skip_env, nullptr);
    }
    rx_dump_base = env;
    rx_dump_path = fmt::format("{}.0", rx_dump_base);
    rx_dump_skip = static_cast<uint64_t>(srate_hz * skip_s);
    rx_dump.reserve(static_cast<size_t>(srate_hz * dump_ms / 1000.0));
  }
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

  // Drain the samples the device buffered before the session started, so that the delivered timestamps track the
  // hardware time. Otherwise the backlog never drains, since the session consumes samples at the real-time rate, and
  // every uplink indication is delivered late.
  uint64_t  drained = 0;
  long long lag_ns  = 0;
  while (drained < static_cast<uint64_t>(srate_hz * 2)) {
    size_t                                          handle   = 0;
    std::array<const void*, RADIO_MAX_NOF_CHANNELS> rd_buffs = {};
    int                                             rd_flags = 0;
    long long                                       rd_time  = 0;
    long long                                       hw_now   = 0;
    const int n = device.acquire_read_buffer(stream, handle, rd_buffs.data(), rd_flags, rd_time, 20000);
    if (n <= 0) {
      break;
    }
    device.release_read_buffer(stream, handle);
    drained += static_cast<uint64_t>(n);
    if ((rd_flags & SOAPY_SDR_HAS_TIME) && device.get_hardware_time(hw_now)) {
      lag_ns = hw_now - rd_time;
      if (lag_ns < 1000000) {
        break;
      }
    }
  }
  fmt::print("Soapy RX: drained {} samples of startup backlog, lag is now {} us.\n", drained, lag_ns / 1000);
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

void radio_soapy_rx_stream::reload_ts_shift()
{
  long long target = 0;
  if (FILE* f = std::fopen(rx_ts_shift_file.c_str(), "r")) {
    if (std::fscanf(f, "%lld", &target) != 1) {
      target = rx_ts_shift;
    }
    std::fclose(f);
  } else {
    logger.warning("Soapy RX: cannot open timestamp shift file {}.", rx_ts_shift_file);
    return;
  }
  const int64_t delta = static_cast<int64_t>(target) - rx_ts_shift;
  if (delta == 0) {
    return;
  }
  rx_ts_shift = target;
  if (delta < 0) {
    rx_drop_pending += static_cast<uint64_t>(-delta);
  } else {
    rx_zero_pending += static_cast<uint64_t>(delta);
  }
  logger.info("Soapy RX: timestamp shift set to {} samples (delta {}).", rx_ts_shift, delta);
  fmt::print("Soapy RX: timestamp shift set to {} samples ({} samples {}).\n", rx_ts_shift,
             (delta < 0) ? -delta : delta, (delta < 0) ? "dropped" : "zero-filled");
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

  if (rx_ts_shift_gen_seen != soapy_rx_ts_shift_generation.load(std::memory_order_relaxed)) {
    rx_ts_shift_gen_seen = soapy_rx_ts_shift_generation.load(std::memory_order_relaxed);
    reload_ts_shift();
  }
  while (rxd_total < nsamples) {
    ++loops;
    // Positive timestamp shift: insert zeros so the labels stay continuous.
    if (rx_zero_pending > 0) {
      if (!last_sample_ts_valid) {
        rx_zero_pending = 0;
      } else {
        const unsigned z = static_cast<unsigned>(std::min<uint64_t>(rx_zero_pending, nsamples - rxd_total));
        for (unsigned ch = 0; ch != buffs.get_nof_channels(); ++ch) {
          ocuduvec::zero(buffs[ch].subspan(rxd_total, z));
        }
        if (!timestamp_captured) {
          timestamp_captured = true;
          ret.ts             = last_sample_ts;
        }
        rxd_total += z;
        rx_zero_pending -= z;
        last_sample_ts = ret.ts + rxd_total;
        continue;
      }
    }
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
    unsigned nchunk        = static_cast<unsigned>(ret_samps);
    unsigned source_offset = (remainder_handle == static_cast<size_t>(-1)) ? 0 : remainder_offset;
    // Negative timestamp shift: discard hardware samples so the labels stay continuous.
    if (rx_drop_pending > 0) {
      const unsigned drop_now = static_cast<unsigned>(std::min<uint64_t>(rx_drop_pending, nchunk));
      rx_drop_pending -= drop_now;
      source_offset += drop_now;
      nchunk -= drop_now;
      if (nchunk == 0) {
        device.release_read_buffer(stream, handle);
        remainder_handle  = static_cast<size_t>(-1);
        remainder_samps   = 0;
        remainder_offset  = 0;
        remainder_flags   = 0;
        remainder_time_ns = 0;
        remainder_buffs.fill(nullptr);
        continue;
      }
    }

    // Capture timestamp from first block.
    if (!timestamp_captured) {
      if (flags & SOAPY_SDR_HAS_TIME) {
        const long long first_time_ns =
            time_ns + soapy_samples_to_api(source_offset, srate_hz);
        last_time_ns       = first_time_ns;
        timestamp_captured = true;
        time_ns            = first_time_ns;
        ret.ts             = static_cast<baseband_gateway_timestamp>(soapy_api_to_samples(time_ns, srate_hz) + rx_ts_shift);
        // Diagnostic: the hardware timestamp should continue exactly where the previous call ended.
        if (last_sample_ts_valid && rxd_total == 0) {
          const int64_t jump = static_cast<int64_t>(ret.ts) - static_cast<int64_t>(last_sample_ts);
          if (jump != 0) {
            ++rx_ts_small_jumps; // any non-contiguous label, including the +-1 rounding ones
          }
          if (jump > 2 || jump < -2) { // +-1..2 samples is ns-to-sample rounding jitter of the plugin timestamps
            ++rx_ts_jumps;
            logger.warning("Soapy RX: hardware timestamp discontinuity of {} samples ({:+.1f} us), overflows in call={}, total jumps={}.",
                           jump, static_cast<double>(jump) * 1e6 / srate_hz, overflow_count, rx_ts_jumps);
            fmt::print("Soapy RX: hardware timestamp discontinuity of {} samples ({:+.1f} us), overflows in call={}, total jumps={}.\n",
                       jump, static_cast<double>(jump) * 1e6 / srate_hz, overflow_count, rx_ts_jumps);
          }
        }
      } else if (last_time_ns > 0 && std::isnormal(srate_hz)) {
        // Reconstruct from last known time + samples elapsed.
        time_ns            = last_time_ns;
        timestamp_captured = true;
        if (last_sample_ts_valid) {
          ret.ts = last_sample_ts;
        } else {
          ret.ts = static_cast<baseband_gateway_timestamp>(soapy_api_to_samples(time_ns, srate_hz) + rx_ts_shift);
        }
      }
    }

    // Copy samples into the caller's buffer, clamped to remaining space.
    const unsigned copy_samps = std::min(nchunk, nsamples - rxd_total);
    for (unsigned ch = 0; ch != std::min(nof_channels, buffs.get_nof_channels()); ++ch) {
      const auto*  src = reinterpret_cast<const ci16_t*>(rd_buffs[ch]) + source_offset;
      span<ci16_t> dst = buffs[ch].subspan(rxd_total, copy_samps);
      // Rescale the device 12-bit samples to the OCUDU 16-bit range.
      for (unsigned i = 0; i != copy_samps; ++i) {
        dst[i] = {static_cast<int16_t>(src[i].real() * (1 << DEVICE_SAMPLE_SHIFT)),
                  static_cast<int16_t>(src[i].imag() * (1 << DEVICE_SAMPLE_SHIFT))};
      }
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
      last_time_ns = soapy_samples_to_api(last_sample_ts, srate_hz);
    }
  }

  // SIGUSR1 starts a new capture immediately.
  if (!rx_dump_base.empty() && rx_dump_gen != soapy_debug_dump_generation.load(std::memory_order_relaxed)) {
    rx_dump_gen  = soapy_debug_dump_generation.load(std::memory_order_relaxed);
    rx_dump_path = fmt::format("{}.{}", rx_dump_base, rx_dump_gen);
    rx_dump.clear();
    rx_dump_skip = 0;
  }
  if (!rx_dump_path.empty() && buffs.get_nof_channels() > 0) {
    span<const ci16_t> samples = buffs[0].first(rxd_total);
    if (rx_dump_skip >= samples.size()) {
      rx_dump_skip -= samples.size();
    } else {
      if (rx_dump.empty()) {
        rx_dump_start_ts = ret.ts + rx_dump_skip;
      }
      samples = samples.last(samples.size() - rx_dump_skip);
      rx_dump_skip = 0;
      const size_t n = std::min(samples.size(), rx_dump.capacity() - rx_dump.size());
      rx_dump.insert(rx_dump.end(), samples.begin(), samples.begin() + n);
      if (rx_dump.size() == rx_dump.capacity()) {
        if (FILE* f = std::fopen(rx_dump_path.c_str(), "wb")) {
          std::fwrite(rx_dump.data(), sizeof(ci16_t), rx_dump.size(), f);
          std::fclose(f);
        }
        logger.info("Soapy RX dump: wrote {} samples starting at ts={} to {}.", rx_dump.size(), rx_dump_start_ts, rx_dump_path);
        fmt::print("Soapy RX dump: wrote {} samples starting at ts={} to {}.\n", rx_dump.size(), rx_dump_start_ts, rx_dump_path);
        rx_dump_path.clear();
      }
    }
  }

  // Delivery lag: age of the newest received sample when the block reaches the lower PHY. Sampled about 20 times
  // per second (each sample costs a hardware time read) and reported every 5 seconds.
  rx_lag_counter += rxd_total;
  if (rx_lag_counter >= static_cast<uint64_t>(srate_hz / 20)) {
    rx_lag_counter = 0;
    long long hw_now = 0;
    if (device.get_hardware_time(hw_now)) {
      const long long lag_us =
          soapy_api_to_ns(hw_now - samples_to_ns(ret.ts + rxd_total - rx_ts_shift, srate_hz), srate_hz) / 1000;
      rx_lag_min = (rx_lag_count == 0) ? lag_us : std::min(rx_lag_min, lag_us);
      rx_lag_max = (rx_lag_count == 0) ? lag_us : std::max(rx_lag_max, lag_us);
      rx_lag_sum += lag_us;
      if (++rx_lag_count >= 100) {
        fmt::print("Soapy RX lag: {} us behind hardware time (min {} / max {} us over {} samples). non-contiguous "
                   "labels so far: {} (of which >2 samples: {}).\n",
                   rx_lag_sum / rx_lag_count,
                   rx_lag_min,
                   rx_lag_max,
                   rx_lag_count,
                   rx_ts_small_jumps,
                   rx_ts_jumps);
        rx_lag_count = 0;
        rx_lag_sum   = 0;
      }
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
        lag_us = soapy_api_to_ns(hw_now_ns - ret_time_ns, srate_hz) / 1000;
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
