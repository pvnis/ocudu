// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Pavonis Communications
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI

#include "radio_soapy_rx_stream.h"
#include "ocudu/ocuduvec/zero.h"
#include <SoapySDR/Errors.hpp>
#include <cstring>

using namespace ocudu;

/// Maximum consecutive empty reads before bailing out of receive().
static constexpr unsigned MAX_TIMEOUT_COUNT = 10;

radio_soapy_rx_stream::radio_soapy_rx_stream(radio_soapy_device&       device_,
                                               SoapySDR::Stream*         stream_,
                                               const stream_description& desc,
                                               radio_event_notifier&     notifier_) :
  id(desc.id), srate_hz(desc.srate_hz), nof_channels(desc.nof_channels), notifier(notifier_), device(device_), stream(stream_)
{
  ocudu_assert(std::isnormal(srate_hz) && srate_hz > 0.0, "Invalid sampling rate {}.", srate_hz);
  ocudu_assert(stream != nullptr, "RX stream must not be null.");
  mtu = device.get_stream_mtu(stream);
}

bool radio_soapy_rx_stream::start(long long time_ns)
{
  stop_control.reset();
  const int flags = (time_ns > 0) ? SOAPY_SDR_HAS_TIME : 0;
  if (!device.activate_stream(stream, flags, time_ns)) {
    fmt::println("Error: failed to activate RX stream {}. {}", id, device.get_error_message());
    return false;
  }
  return true;
}

bool radio_soapy_rx_stream::stop()
{
  stop_control.stop();
  if (!device.deactivate_stream(stream)) {
    fmt::println("Error: failed to deactivate RX stream {}. {}", id, device.get_error_message());
    return false;
  }
  return true;
}

baseband_gateway_receiver::metadata radio_soapy_rx_stream::receive(baseband_gateway_buffer_writer& buffs)
{
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
  bool           timestamp_captured = false;

  while (rxd_total < nsamples) {
    size_t    handle  = 0;
    const void** rd_buffs = nullptr;
    int       flags   = 0;
    long long time_ns = 0;

    const int ret_samps = device.acquire_read_buffer(stream, handle, rd_buffs, flags, time_ns);

    if (ret_samps == SOAPY_SDR_OVERFLOW) {
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
        fmt::println("Error: exceeded maximum timeout count on RX stream {}.", id);
        return ret;
      }
      continue;
    }

    if (ret_samps < 0) {
      fmt::println("Error: RX stream {} acquireReadBuffer returned {}.", id, ret_samps);
      return ret;
    }

    timeout_count = 0;
    const unsigned nchunk = static_cast<unsigned>(ret_samps);

    // Capture timestamp from first block.
    if (!timestamp_captured) {
      if (flags & SOAPY_SDR_HAS_TIME) {
        last_time_ns       = time_ns;
        timestamp_captured = true;
      } else if (last_time_ns > 0 && std::isnormal(srate_hz)) {
        // Reconstruct from last known time + samples elapsed.
        time_ns            = last_time_ns;
        timestamp_captured = true;
      }
      if (timestamp_captured) {
        ret.ts = static_cast<baseband_gateway_timestamp>(time_ns * srate_hz / 1e9);
      }
    }

    // Copy samples into the caller's buffer, clamped to remaining space.
    const unsigned copy_samps = std::min(nchunk, nsamples - rxd_total);
    for (unsigned ch = 0; ch != std::min(nof_channels, buffs.get_nof_channels()); ++ch) {
      std::memcpy(buffs[ch].subspan(rxd_total, copy_samps).data(),
                  reinterpret_cast<const ci16_t*>(rd_buffs[ch]),
                  copy_samps * sizeof(ci16_t));
    }

    device.release_read_buffer(stream, handle);

    rxd_total += copy_samps;

    // Advance last known time by samples just consumed.
    if (timestamp_captured && std::isnormal(srate_hz)) {
      last_time_ns = static_cast<long long>(ret.ts * 1e9 / srate_hz) +
                     static_cast<long long>(rxd_total * 1e9 / srate_hz);
    }
  }

  return ret;
}
