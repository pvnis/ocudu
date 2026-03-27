// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Pavonis Communications
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI

#include "radio_soapy_impl.h"
#include "ocudu/ocudulog/ocudulog.h"
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Types.hpp>
#include <SoapySDR/Logger.hpp>
#include <fmt/format.h>

using namespace ocudu;

// ---------------------------------------------------------------------------
// Gain
// ---------------------------------------------------------------------------

bool radio_session_soapy_impl::set_tx_gain_unprotected(unsigned port_idx, double gain_dB)
{
  if (port_idx >= tx_port_map.size()) {
    fmt::print("Error: TX port index ({}) exceeds number of ports ({}).\n", port_idx, tx_port_map.size());
    return false;
  }

  const unsigned channel = tx_port_map[port_idx].second;
  if (!device.set_gain(SOAPY_SDR_TX, channel, gain_dB)) {
    fmt::print("Error: setting TX gain for port {}. {}\n", port_idx, device.get_error_message());
    return false;
  }

  return true;
}

bool radio_session_soapy_impl::set_rx_gain_unprotected(unsigned port_idx, double gain_dB)
{
  if (port_idx >= rx_port_map.size()) {
    fmt::print("Error: RX port index ({}) exceeds number of ports ({}).\n", port_idx, rx_port_map.size());
    return false;
  }

  const unsigned channel = rx_port_map[port_idx].second;
  if (!device.set_gain(SOAPY_SDR_RX, channel, gain_dB)) {
    fmt::print("Error: setting RX gain for port {}. {}\n", port_idx, device.get_error_message());
    return false;
  }

  return true;
}

// ---------------------------------------------------------------------------
// Per-port frequency (private)
// ---------------------------------------------------------------------------

bool radio_session_soapy_impl::set_tx_port_freq(unsigned port_idx, double freq_Hz)
{
  if (port_idx >= tx_port_map.size()) {
    fmt::print("Error: TX port index ({}) exceeds number of ports ({}).\n", port_idx, tx_port_map.size());
    return false;
  }

  const unsigned channel = tx_port_map[port_idx].second;
  if (!device.set_frequency(SOAPY_SDR_TX, channel, freq_Hz)) {
    fmt::print("Error: setting TX frequency for port {}. {}\n", port_idx, device.get_error_message());
    return false;
  }

  return true;
}

bool radio_session_soapy_impl::set_rx_port_freq(unsigned port_idx, double freq_Hz)
{
  if (port_idx >= rx_port_map.size()) {
    fmt::print("Error: RX port index ({}) exceeds number of ports ({}).\n", port_idx, rx_port_map.size());
    return false;
  }

  const unsigned channel = rx_port_map[port_idx].second;
  if (!device.set_frequency(SOAPY_SDR_RX, channel, freq_Hz)) {
    fmt::print("Error: setting RX frequency for port {}. {}\n", port_idx, device.get_error_message());
    return false;
  }

  return true;
}

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

radio_session_soapy_impl::radio_session_soapy_impl(const radio_configuration::radio& radio_config,
                                                    task_executor&                    async_executor,
                                                    radio_event_notifier&             notifier)
{
  actual_sampling_rate_Hz = radio_config.sampling_rate_Hz;

  // Open the SoapySDR device.
  if (!device.make(radio_config.args)) {
    fmt::print("Error: failed to open SoapySDR device with args '{}': {}\n",
               radio_config.args,
               device.get_error_message());
    return;
  }

  if (!device.is_valid()) {
    fmt::print("Error: SoapySDR device is not valid after make.\n");
    return;
  }

  // Count total channels across all TX/RX streams (validator enforces 1 stream pair).
  unsigned total_tx_channels = 0;
  for (const auto& stream : radio_config.tx_streams) {
    total_tx_channels += static_cast<unsigned>(stream.channels.size());
  }
  unsigned total_rx_channels = 0;
  for (const auto& stream : radio_config.rx_streams) {
    total_rx_channels += static_cast<unsigned>(stream.channels.size());
  }

  // Configure TX channels: sample rate, frequency, gain.
  for (unsigned stream_idx = 0, nof_streams = radio_config.tx_streams.size(); stream_idx != nof_streams; ++stream_idx) {
    const radio_configuration::stream& stream = radio_config.tx_streams[stream_idx];

    for (unsigned ch_idx = 0, nof_ch = stream.channels.size(); ch_idx != nof_ch; ++ch_idx) {
      const unsigned global_port = tx_port_map.size();
      tx_port_map.emplace_back(port_to_stream_channel(stream_idx, ch_idx));

      const radio_configuration::channel& ch = stream.channels[ch_idx];

      if (!device.set_sample_rate(SOAPY_SDR_TX, ch_idx, actual_sampling_rate_Hz)) {
        fmt::print("Error: setting TX sample rate for channel {}. {}\n", ch_idx, device.get_error_message());
        return;
      }

      if (!set_tx_port_freq(global_port, ch.freq.center_frequency_Hz)) {
        return;
      }

      if (!set_tx_gain_unprotected(global_port, ch.gain_dB)) {
        return;
      }
    }
  }

  // Configure RX channels: sample rate, frequency, gain.
  for (unsigned stream_idx = 0, nof_streams = radio_config.rx_streams.size(); stream_idx != nof_streams; ++stream_idx) {
    const radio_configuration::stream& stream = radio_config.rx_streams[stream_idx];

    for (unsigned ch_idx = 0, nof_ch = stream.channels.size(); ch_idx != nof_ch; ++ch_idx) {
      const unsigned global_port = rx_port_map.size();
      rx_port_map.emplace_back(port_to_stream_channel(stream_idx, ch_idx));

      const radio_configuration::channel& ch = stream.channels[ch_idx];

      if (!device.set_sample_rate(SOAPY_SDR_RX, ch_idx, actual_sampling_rate_Hz)) {
        fmt::print("Error: setting RX sample rate for channel {}. {}\n", ch_idx, device.get_error_message());
        return;
      }

      if (!set_rx_port_freq(global_port, ch.freq.center_frequency_Hz)) {
        return;
      }

      if (!set_rx_gain_unprotected(global_port, ch.gain_dB)) {
        return;
      }
    }
  }

  // Create one baseband gateway per TX+RX stream pair.
  bb_gateways.reserve(radio_config.tx_streams.size());

  for (unsigned stream_idx = 0, nof_streams = radio_config.tx_streams.size(); stream_idx != nof_streams; ++stream_idx) {
    const radio_configuration::stream& tx_stream_cfg = radio_config.tx_streams[stream_idx];
    const radio_configuration::stream& rx_stream_cfg = radio_config.rx_streams[stream_idx];

    const unsigned nof_tx_ch = static_cast<unsigned>(tx_stream_cfg.channels.size());
    const unsigned nof_rx_ch = static_cast<unsigned>(rx_stream_cfg.channels.size());

    // Build SoapySDR channel index lists.
    std::vector<size_t> tx_channels(nof_tx_ch);
    for (unsigned i = 0; i != nof_tx_ch; ++i) {
      tx_channels[i] = i;
    }

    std::vector<size_t> rx_channels(nof_rx_ch);
    for (unsigned i = 0; i != nof_rx_ch; ++i) {
      rx_channels[i] = i;
    }

    // Parse stream-level kwargs from args string.
    const SoapySDR::Kwargs tx_kwargs = SoapySDR::KwargsFromString(tx_stream_cfg.args);
    const SoapySDR::Kwargs rx_kwargs = SoapySDR::KwargsFromString(rx_stream_cfg.args);

    // Setup underlying SoapySDR streams.
    SoapySDR::Stream* tx_sdr_stream = device.setup_stream(SOAPY_SDR_TX, tx_channels, tx_kwargs);
    if (tx_sdr_stream == nullptr) {
      fmt::print("Error: failed to setup TX SoapySDR stream {}. {}\n", stream_idx, device.get_error_message());
      return;
    }

    SoapySDR::Stream* rx_sdr_stream = device.setup_stream(SOAPY_SDR_RX, rx_channels, rx_kwargs);
    if (rx_sdr_stream == nullptr) {
      fmt::print("Error: failed to setup RX SoapySDR stream {}. {}\n", stream_idx, device.get_error_message());
      return;
    }

    const bool discontinuous_tx = (radio_config.tx_mode != radio_configuration::transmission_mode::continuous);

    // Create stream objects.
    auto tx_stream = std::make_unique<radio_soapy_tx_stream>(
        device,
        tx_sdr_stream,
        radio_soapy_tx_stream::stream_description{.id               = stream_idx,
                                                  .srate_hz         = actual_sampling_rate_Hz,
                                                  .nof_channels     = nof_tx_ch,
                                                  .discontinuous_tx = discontinuous_tx,
                                                  .power_ramping_us = radio_config.power_ramping_us},
        async_executor,
        notifier);

    auto rx_stream = std::make_unique<radio_soapy_rx_stream>(
        device,
        rx_sdr_stream,
        radio_soapy_rx_stream::stream_description{
            .id = stream_idx, .srate_hz = actual_sampling_rate_Hz, .nof_channels = nof_rx_ch},
        notifier);

    auto& gateway =
        bb_gateways.emplace_back(std::make_unique<radio_soapy_baseband_gateway>(std::move(tx_stream), std::move(rx_stream)));

    if (!gateway->is_successful()) {
      return;
    }
  }

  is_init_successful = true;
}

// ---------------------------------------------------------------------------
// Start / stop
// ---------------------------------------------------------------------------

void radio_session_soapy_impl::start(baseband_gateway_timestamp init_time)
{
  // Start TX async status polling for each gateway.
  for (auto& gateway : bb_gateways) {
    gateway->get_tx_stream().start();
  }

  // Activate RX streams with the initial timestamp.
  const long long init_time_ns = static_cast<long long>(static_cast<double>(init_time) * 1e9 / actual_sampling_rate_Hz);
  for (auto& gateway : bb_gateways) {
    if (!gateway->get_rx_stream().start(init_time_ns)) {
      fmt::print("Error: failed to start RX stream.\n");
    }
  }
}

void radio_session_soapy_impl::stop()
{
  for (auto& gateway : bb_gateways) {
    gateway->get_tx_stream().stop();
  }

  for (auto& gateway : bb_gateways) {
    if (!gateway->get_rx_stream().stop()) {
      fmt::print("Warning: failed to stop RX stream.\n");
    }
  }
}

// ---------------------------------------------------------------------------
// Stream-level frequency setters (public management plane)
// ---------------------------------------------------------------------------

bool radio_session_soapy_impl::set_tx_freq(unsigned stream_id, double center_freq_Hz)
{
  for (unsigned i_port = 0, end = tx_port_map.size(); i_port != end; ++i_port) {
    if (tx_port_map[i_port].first != stream_id) {
      continue;
    }

    if (!set_tx_port_freq(i_port, center_freq_Hz)) {
      return false;
    }
  }

  return true;
}

bool radio_session_soapy_impl::set_rx_freq(unsigned stream_id, double center_freq_Hz)
{
  for (unsigned i_port = 0, end = rx_port_map.size(); i_port != end; ++i_port) {
    if (rx_port_map[i_port].first != stream_id) {
      continue;
    }

    if (!set_rx_port_freq(i_port, center_freq_Hz)) {
      return false;
    }
  }

  return true;
}

// ---------------------------------------------------------------------------
// Time
// ---------------------------------------------------------------------------

baseband_gateway_timestamp radio_session_soapy_impl::read_current_time()
{
  long long time_ns = 0;
  if (!device.get_hardware_time(time_ns)) {
    fmt::print("Error: failed to read hardware time. {}\n", device.get_error_message());
    return 0;
  }

  return static_cast<baseband_gateway_timestamp>(static_cast<double>(time_ns) * actual_sampling_rate_Hz / 1e9);
}

// ---------------------------------------------------------------------------
// Factory
// ---------------------------------------------------------------------------

std::unique_ptr<radio_session> radio_factory_soapy_impl::create(const radio_configuration::radio& config,
                                                                  task_executor&                    async_task_executor,
                                                                  radio_event_notifier&             notifier)
{
  auto session = std::make_unique<radio_session_soapy_impl>(config, async_task_executor, notifier);
  if (!session->is_successful()) {
    return nullptr;
  }

  return session;
}
