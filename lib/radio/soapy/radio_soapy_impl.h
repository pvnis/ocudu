// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Pavonis Communications
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI

#pragma once

// These headers (formatter.h, sink.h, stop_event.h, log_channel.h) have
// default no-op virtual/static implementations with unused parameters, and
// log_channel uses an incomplete aggregate initializer. Suppress here so
// translation units that include this header (e.g. radio_factory.cpp) are
// not affected by pre-existing issues in those headers.
#include "radio_config_soapy_validator.h"
#include "radio_soapy_baseband_gateway.h"
#include "radio_soapy_device.h"
#include "ocudu/adt/static_vector.h"
#include "ocudu/radio/radio_constants.h"
#include "ocudu/radio/radio_factory.h"
#include "ocudu/radio/radio_management_plane.h"
#include <memory>
#include <vector>

namespace ocudu {

/// Implements a radio session and management plane using SoapySDR (M2SDR / AD9361).
class radio_session_soapy_impl : public radio_session, private radio_management_plane
{
  /// Maps a global port index to (stream_idx, channel_idx).
  using port_to_stream_channel = std::pair<unsigned, unsigned>;

  bool is_init_successful = false;

  radio_soapy_device device;

  /// Global TX port index → (stream_idx, channel_idx).
  static_vector<port_to_stream_channel, RADIO_MAX_NOF_PORTS> tx_port_map;
  /// Global RX port index → (stream_idx, channel_idx).
  static_vector<port_to_stream_channel, RADIO_MAX_NOF_PORTS> rx_port_map;

  std::vector<std::unique_ptr<radio_soapy_baseband_gateway>> bb_gateways;
  double                                                     actual_sampling_rate_Hz = 0.0;

  bool set_tx_gain_unprotected(unsigned port_idx, double gain_dB);
  bool set_rx_gain_unprotected(unsigned port_idx, double gain_dB);

  /// Set TX center frequency for a specific port (channel).
  bool set_tx_port_freq(unsigned port_idx, double freq_Hz);

  /// Set RX center frequency for a specific port (channel).
  bool set_rx_port_freq(unsigned port_idx, double freq_Hz);

public:
  radio_session_soapy_impl(const radio_configuration::radio& radio_config,
                            task_executor&                    async_executor,
                            radio_event_notifier&             notifier);

  bool is_successful() const { return is_init_successful; }

  // See interface for documentation.
  radio_management_plane& get_management_plane() override { return *this; }

  // See interface for documentation.
  baseband_gateway& get_baseband_gateway(unsigned stream_id) override
  {
    ocudu_assert(stream_id < bb_gateways.size(),
                 "Stream identifier (i.e., {}) exceeds the number of baseband gateways (i.e., {})",
                 stream_id,
                 bb_gateways.size());
    return *bb_gateways[stream_id];
  }

  // See interface for documentation.
  void start(baseband_gateway_timestamp init_time) override;

  // See interface for documentation.
  void stop() override;

  // See interface for documentation.
  bool set_tx_gain(unsigned port_idx, double gain_dB) override { return set_tx_gain_unprotected(port_idx, gain_dB); }

  // See interface for documentation.
  bool set_rx_gain(unsigned port_idx, double gain_dB) override { return set_rx_gain_unprotected(port_idx, gain_dB); }

  // See interface for documentation.
  bool set_tx_freq(unsigned stream_id, double center_freq_Hz) override;

  // See interface for documentation.
  bool set_rx_freq(unsigned stream_id, double center_freq_Hz) override;

  // See interface for documentation.
  baseband_gateway_timestamp read_current_time() override;
};

/// Factory for the SoapySDR radio session.
class radio_factory_soapy_impl : public radio_factory
{
public:
  // See interface for documentation.
  const radio_configuration::validator& get_configuration_validator() const override
  {
    static radio_config_soapy_validator config_validator;
    return config_validator;
  }

  // See interface for documentation.
  std::unique_ptr<radio_session> create(const radio_configuration::radio& config,
                                        task_executor&                    async_task_executor,
                                        radio_event_notifier&             notifier) override;
};

} // namespace ocudu
