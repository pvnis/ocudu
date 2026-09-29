// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Pavonis Communications
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI

#include "radio_config_soapy_validator.h"
#include "fmt/base.h"
#include <cmath>

using namespace ocudu;

// AD9361 hardware limits.
static constexpr double SRATE_MIN_HZ    = 0.5e6;
static constexpr double SRATE_MAX_HZ    = 61.44e6;
static constexpr double TX_FREQ_MIN_HZ  = 47e6;
static constexpr double TX_FREQ_MAX_HZ  = 6e9;
static constexpr double RX_FREQ_MIN_HZ  = 70e6;
static constexpr double RX_FREQ_MAX_HZ  = 6e9;
static constexpr double TX_GAIN_MIN_DB  = -89.0;
static constexpr double TX_GAIN_MAX_DB  = 0.0;
static constexpr double RX_GAIN_MIN_DB  = 0.0;
static constexpr double RX_GAIN_MAX_DB  = 76.0;
static constexpr size_t MAX_NOF_STREAMS   = 1;
static constexpr size_t MAX_NOF_CHANNELS  = 2;

static bool validate_channel(const radio_configuration::channel& channel, bool is_tx)
{
  const double freq = channel.freq.center_frequency_Hz;
  if (!std::isnormal(freq) || freq <= 0.0) {
    fmt::print("{} center frequency must be non-zero, finite, and positive.\n", is_tx ? "TX" : "RX");
    return false;
  }

  if (is_tx) {
    if (freq < TX_FREQ_MIN_HZ || freq > TX_FREQ_MAX_HZ) {
      fmt::print("TX frequency {:.3f} MHz is out of range [{:.0f}, {:.0f}] MHz.\n",
                 freq * 1e-6,
                 TX_FREQ_MIN_HZ * 1e-6,
                 TX_FREQ_MAX_HZ * 1e-6);
      return false;
    }
    if (channel.gain_dB < TX_GAIN_MIN_DB || channel.gain_dB > TX_GAIN_MAX_DB) {
      fmt::print("TX gain {:.1f} dB is out of range [{:.0f}, {:.0f}] dB.\n",
                 channel.gain_dB,
                 TX_GAIN_MIN_DB,
                 TX_GAIN_MAX_DB);
      return false;
    }
  } else {
    if (freq < RX_FREQ_MIN_HZ || freq > RX_FREQ_MAX_HZ) {
      fmt::print("RX frequency {:.3f} MHz is out of range [{:.0f}, {:.0f}] MHz.\n",
                 freq * 1e-6,
                 RX_FREQ_MIN_HZ * 1e-6,
                 RX_FREQ_MAX_HZ * 1e-6);
      return false;
    }
    if (channel.gain_dB < RX_GAIN_MIN_DB || channel.gain_dB > RX_GAIN_MAX_DB) {
      fmt::print("RX gain {:.1f} dB is out of range [{:.0f}, {:.0f}] dB.\n",
                 channel.gain_dB,
                 RX_GAIN_MIN_DB,
                 RX_GAIN_MAX_DB);
      return false;
    }
  }

  return true;
}

static bool validate_stream(const radio_configuration::stream& stream, bool is_tx)
{
  if (stream.channels.empty()) {
    fmt::print("Streams must contain at least one channel.\n");
    return false;
  }

  if (stream.channels.size() > MAX_NOF_CHANNELS) {
    fmt::print("M2SDR supports at most {} channels per stream (AD9361 2T2R), got {}.\n",
               MAX_NOF_CHANNELS,
               stream.channels.size());
    return false;
  }

  for (const auto& ch : stream.channels) {
    if (!validate_channel(ch, is_tx)) {
      return false;
    }
  }

  return true;
}

bool radio_config_soapy_validator::is_configuration_valid(const radio_configuration::radio& config) const
{
  // M2SDR has a single DMA path, so exactly one TX and one RX stream.
  if (config.tx_streams.size() != 1 || config.rx_streams.size() != 1) {
    fmt::print("M2SDR requires exactly 1 TX stream and 1 RX stream, got {} TX and {} RX.\n",
               config.tx_streams.size(),
               config.rx_streams.size());
    return false;
  }

  for (const auto& s : config.tx_streams) {
    if (!validate_stream(s, true)) {
      return false;
    }
  }

  for (const auto& s : config.rx_streams) {
    if (!validate_stream(s, false)) {
      return false;
    }
  }

  // TX and RX must have the same channel count (AD9361 constraint).
  if (config.tx_streams[0].channels.size() != config.rx_streams[0].channels.size()) {
    fmt::print("M2SDR requires the same number of TX and RX channels (AD9361 2T2R), got {} TX and {} RX.\n",
               config.tx_streams[0].channels.size(),
               config.rx_streams[0].channels.size());
    return false;
  }

  // Sample rate.
  if (!std::isnormal(config.sampling_rate_Hz) || config.sampling_rate_Hz <= 0.0) {
    fmt::print("Sampling rate must be non-zero, finite, and positive.\n");
    return false;
  }
  if (config.sampling_rate_Hz < SRATE_MIN_HZ || config.sampling_rate_Hz > SRATE_MAX_HZ) {
    fmt::print("Sampling rate {:.3f} MSPS is out of range [{:.3f}, {:.3f}] MSPS.\n",
               config.sampling_rate_Hz * 1e-6,
               SRATE_MIN_HZ * 1e-6,
               SRATE_MAX_HZ * 1e-6);
    return false;
  }

  // OTW format: M2SDR natively uses SC16; SC8/SC12 are not supported.
  if (config.otw_format != radio_configuration::over_the_wire_format::DEFAULT &&
      config.otw_format != radio_configuration::over_the_wire_format::SC16) {
    fmt::print("M2SDR only supports SC16 over-the-wire format.\n");
    return false;
  }

  if (config.power_ramping_us < 0) {
    fmt::print("Power ramping time {:.1f} us must be zero or positive.\n", config.power_ramping_us);
    return false;
  }

  return true;
}
