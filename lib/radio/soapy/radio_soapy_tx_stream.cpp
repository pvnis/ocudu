// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Pavonis Communications
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI

#include "radio_soapy_tx_stream.h"
#include "ocudu/gateways/baseband/buffer/baseband_gateway_buffer_reader_view.h"
#include "ocudu/ocuduvec/zero.h"
#include <SoapySDR/Errors.hpp>
#include <cstring>

using namespace ocudu;

/// Async status poll timeout in microseconds (1 ms).
static constexpr long RECV_ASYNC_TIMEOUT_US = 1000;

/// Samples-to-nanoseconds helper.
static inline long long samples_to_ns(uint64_t samples, double srate_hz)
{
  return static_cast<long long>(static_cast<double>(samples) * 1e9 / srate_hz);
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
  power_ramping_buffer(desc.nof_channels, 0)
{
  ocudu_assert(std::isnormal(srate_hz) && srate_hz > 0.0, "Invalid sampling rate {}.", srate_hz);
  ocudu_assert(stream != nullptr, "TX stream must not be null.");

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
    event.timestamp       = static_cast<uint64_t>(time_ns * srate_hz / 1e9);
    state_fsm.async_event_late_underflow(time_ns);
  } else if (ret == SOAPY_SDR_UNDERFLOW) {
    event.type            = radio_event_type::UNDERFLOW;
    state_fsm.async_event_late_underflow(time_ns);
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
    transmit = state_fsm.on_transmit(flags, time_ns, tx_md.is_empty, tx_end_padding);
  } else {
    transmit = state_fsm.on_transmit(flags, time_ns, false, false);
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
                                .timestamp  = static_cast<uint64_t>(time_ns * srate_hz / 1e9)});

    // Transmit power-ramping zeros before the burst.
    if (discontinuous_tx && power_ramping_nof_samples > 0) {
      const unsigned tx_gap_samples =
          static_cast<unsigned>((time_ns - last_tx_time_ns) * srate_hz / 1e9);
      const unsigned min_gap = static_cast<unsigned>(srate_hz / 100000.0); // 10 µs

      if (tx_gap_samples > min_gap) {
        unsigned nof_pad = std::min(power_ramping_nof_samples, tx_gap_samples - min_gap);
        if (nof_pad > 0) {
          long long pad_time_ns = time_ns - samples_to_ns(nof_pad, srate_hz);
          unsigned  pad_sent    = 0;

          // The ramping buffer holds zeros; send MTU-sized chunks via acquireWriteBuffer.
          while (pad_sent < nof_pad) {
            void**  wr_buffs = nullptr;
            int     handle   = device.acquire_write_buffer(stream, wr_buffs);
            if (handle < 0) {
              break;
            }
            const unsigned chunk = static_cast<unsigned>(mtu);
            for (unsigned ch = 0; ch != nof_channels; ++ch) {
              std::memset(reinterpret_cast<int16_t*>(wr_buffs[ch]), 0, chunk * sizeof(int16_t) * 2);
            }
            int pad_flags = (pad_sent == 0) ? SOAPY_SDR_HAS_TIME : 0;
            device.release_write_buffer(stream, static_cast<size_t>(handle), chunk, pad_flags, pad_time_ns);
            pad_time_ns += samples_to_ns(chunk, srate_hz);
            pad_sent    += chunk;
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
                                .timestamp  = static_cast<uint64_t>(time_ns * srate_hz / 1e9)});
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

  // Send data in MTU-sized chunks using the zero-copy DMA path.
  unsigned sent_total = 0;
  while (sent_total < data_nof_samples) {
    void** wr_buffs = nullptr;
    int    handle   = device.acquire_write_buffer(stream, wr_buffs);
    if (handle < 0) {
      fmt::println(stderr, "SoapySDR TX: acquireWriteBuffer failed for stream {}.", stream_id);
      return;
    }

    const unsigned chunk = std::min(static_cast<unsigned>(mtu), data_nof_samples - sent_total);

    // Copy channel data into the DMA buffer.
    for (unsigned ch = 0; ch != nof_channels; ++ch) {
      const ci16_t* src = data[ch].subspan(data_start + sent_total, chunk).data();
      std::memcpy(wr_buffs[ch], src, chunk * sizeof(ci16_t));
    }

    // Build flags for this chunk.
    int chunk_flags = 0;
    if (sent_total == 0) {
      chunk_flags = flags; // carries HAS_TIME (and maybe END_BURST for single-chunk burst)
    }
    if (sent_total + chunk >= data_nof_samples && is_eob) {
      chunk_flags |= SOAPY_SDR_END_BURST;
    }

    const long long chunk_time_ns = time_ns + samples_to_ns(sent_total, srate_hz);
    device.release_write_buffer(stream, static_cast<size_t>(handle), chunk, chunk_flags, chunk_time_ns);

    sent_total += chunk;
  }

  last_tx_time_ns = time_ns + samples_to_ns(sent_total, srate_hz);
}

void radio_soapy_tx_stream::start()
{
  stop_control.reset();
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
    void** wr_buffs = nullptr;
    int    handle   = device.acquire_write_buffer(stream, wr_buffs, 1000);
    if (handle >= 0) {
      device.release_write_buffer(stream, static_cast<size_t>(handle), 0, SOAPY_SDR_END_BURST, 0);
    }
  }
}
