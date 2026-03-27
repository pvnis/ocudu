// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Pavonis Communications
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI

#pragma once

#include "radio_soapy_device.h"
#include "radio_soapy_exception_handler.h"
#include "ocudu/gateways/baseband/baseband_gateway_receiver.h"
#include "ocudu/gateways/baseband/buffer/baseband_gateway_buffer_writer.h"
#include "ocudu/radio/radio_event_notifier.h"
#include "ocudu/radio/radio_constants.h"
#include "ocudu/support/synchronization/stop_event.h"
#include <SoapySDR/Device.hpp>
#include <atomic>
#include <array>

namespace ocudu {

/// Implements baseband_gateway_receiver using SoapySDR zero-copy read buffers.
class radio_soapy_rx_stream : public baseband_gateway_receiver, public soapy_exception_handler
{
  unsigned              id;
  double                srate_hz;
  unsigned              nof_channels;
  radio_event_notifier& notifier;
  radio_soapy_device&   device;
  SoapySDR::Stream*     stream = nullptr;
  size_t                mtu    = 0;
  rt_stop_event_source  stop_control;
  ocudulog::basic_logger& logger = ocudulog::fetch_basic_logger("RF");

  /// Timestamp of the last received sample (ns), used for continuity tracking.
  long long last_time_ns = 0;
  baseband_gateway_timestamp last_sample_ts = 0;
  bool last_sample_ts_valid = false;
  bool rx_trace_enabled = false;
  long rx_trace_slow_us = 200;
  uint64_t rx_trace_limit = 0;
  std::atomic<uint64_t> rx_trace_seq{0};
  baseband_gateway_timestamp last_ret_ts = 0;
  bool last_ret_ts_valid = false;
  size_t remainder_handle = static_cast<size_t>(-1);
  std::array<const void*, RADIO_MAX_NOF_CHANNELS> remainder_buffs = {};
  unsigned remainder_samps = 0;
  unsigned remainder_offset = 0;
  int remainder_flags = 0;
  long long remainder_time_ns = 0;

public:
  struct stream_description {
    unsigned id;
    double   srate_hz;
    unsigned nof_channels;
  };

  radio_soapy_rx_stream(radio_soapy_device&       device_,
                        SoapySDR::Stream*          stream_,
                        const stream_description&  desc,
                        radio_event_notifier&      notifier_);

  unsigned get_buffer_size() const { return static_cast<unsigned>(mtu); }

  bool start(long long time_ns);
  bool stop();

  // See interface for documentation.
  metadata receive(baseband_gateway_buffer_writer& data) override;
};

} // namespace ocudu
