// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Pavonis Communications
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI

#pragma once

#include "radio_soapy_rx_stream.h"
#include "radio_soapy_tx_stream.h"
#include "ocudu/gateways/baseband/baseband_gateway.h"
#include <memory>

namespace ocudu {

/// Implements baseband_gateway for SoapySDR, combining a TX and RX stream pair.
class radio_soapy_baseband_gateway : public baseband_gateway
{
public:
  radio_soapy_baseband_gateway(std::unique_ptr<radio_soapy_tx_stream> tx,
                                std::unique_ptr<radio_soapy_rx_stream> rx) :
    tx_stream(std::move(tx)), rx_stream(std::move(rx))
  {
  }

  // See interface for documentation.
  baseband_gateway_transmitter& get_transmitter() override { return *tx_stream; }

  // See interface for documentation.
  baseband_gateway_receiver& get_receiver() override { return *rx_stream; }

  // See interface for documentation.
  unsigned get_transmitter_optimal_buffer_size() const override { return tx_stream->get_buffer_size(); }

  // See interface for documentation.
  unsigned get_receiver_optimal_buffer_size() const override { return rx_stream->get_buffer_size(); }

  radio_soapy_tx_stream& get_tx_stream() { return *tx_stream; }
  radio_soapy_rx_stream& get_rx_stream() { return *rx_stream; }

  bool is_successful() const { return tx_stream != nullptr && rx_stream != nullptr; }

private:
  std::unique_ptr<radio_soapy_tx_stream> tx_stream;
  std::unique_ptr<radio_soapy_rx_stream> rx_stream;
};

} // namespace ocudu
