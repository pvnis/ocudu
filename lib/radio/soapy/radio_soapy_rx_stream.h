// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Pavonis Communications
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI

#pragma once

#include "radio_soapy_device.h"
#include "radio_soapy_exception_handler.h"
#include "ocudu/gateways/baseband/baseband_gateway_receiver.h"
#include "ocudu/gateways/baseband/buffer/baseband_gateway_buffer_writer.h"
#include "ocudu/radio/radio_event_notifier.h"
#include "ocudu/support/synchronization/stop_event.h"
#include <SoapySDR/Device.hpp>

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

  /// Timestamp of the last received sample (ns), used for continuity tracking.
  long long last_time_ns = 0;

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
