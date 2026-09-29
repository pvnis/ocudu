// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Pavonis Communications
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI

#pragma once

#include "radio_soapy_device.h"
#include "radio_soapy_exception_handler.h"
#include "radio_soapy_tx_stream_fsm.h"
#include "ocudu/gateways/baseband/baseband_gateway_transmitter.h"
#include "ocudu/gateways/baseband/buffer/baseband_gateway_buffer_dynamic.h"
#include "ocudu/gateways/baseband/buffer/baseband_gateway_buffer_reader.h"
#include "ocudu/radio/radio_configuration.h"
#include "ocudu/radio/radio_event_notifier.h"
#include "ocudu/support/executors/task_executor.h"
#include "ocudu/support/synchronization/stop_event.h"
#include <SoapySDR/Device.hpp>
#include <chrono>
#include <string>
#include <vector>

namespace ocudu {

/// Implements baseband_gateway_transmitter using SoapySDR zero-copy write buffers.
class radio_soapy_tx_stream : public baseband_gateway_transmitter, public soapy_exception_handler
{
  unsigned              stream_id;
  task_executor&        async_executor;
  radio_event_notifier& notifier;
  radio_soapy_device&   device;
  SoapySDR::Stream*     stream = nullptr;
  size_t                mtu    = 0;
  double                srate_hz;
  unsigned              nof_channels;
  bool                  discontinuous_tx;
  unsigned              power_ramping_nof_samples = 0;
  long long             last_tx_time_ns           = 0;
  long                  write_timeout_us          = 200;
  /// Pre-zeroed power ramping buffer (CI16 samples per channel).
  baseband_gateway_buffer_dynamic power_ramping_buffer;
  /// Staging buffer holding the TX samples rescaled to the device sample width.
  baseband_gateway_buffer_dynamic tx_scaled_buffer;
  /// Debug capture of the channel 0 TX timeline to a file, enabled with OCUDU_SOAPY_TX_DUMP=<path>.
  std::string         tx_dump_path;
  std::string         tx_dump_base;
  unsigned            tx_dump_gen      = 0;
  uint64_t            tx_dump_skip     = 0;
  std::vector<ci16_t> tx_dump;
  uint64_t            tx_dump_start_ts = 0;
  unsigned            tx_dump_count    = 0;
  uint64_t            tx_lead_counter  = 0;
  radio_soapy_tx_stream_fsm       state_fsm;
  rt_stop_event_source            stop_control;
  ocudulog::basic_logger&         logger;
  bool                            tx_trace_enabled      = false;
  long                            tx_trace_threshold_us = 100;

  void recv_async_msg();
  void run_recv_async_msg();

public:
  struct stream_description {
    unsigned   id;
    double     srate_hz;
    unsigned   nof_channels;
    bool       discontinuous_tx;
    float      power_ramping_us;
  };

  radio_soapy_tx_stream(radio_soapy_device&       device_,
                        SoapySDR::Stream*          stream_,
                        const stream_description&  desc,
                        task_executor&             async_executor_,
                        radio_event_notifier&      notifier_);

  unsigned get_buffer_size() const { return static_cast<unsigned>(mtu); }

  // See interface for documentation.
  void transmit(const baseband_gateway_buffer_reader&        data,
                const baseband_gateway_transmitter_metadata& metadata) override;

  void start();
  void stop();
};

} // namespace ocudu
