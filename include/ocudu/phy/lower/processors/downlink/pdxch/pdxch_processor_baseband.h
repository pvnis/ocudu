// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

#pragma once

#include "ocudu/gateways/baseband/buffer/baseband_gateway_buffer_pool.h"
#include "ocudu/phy/lower/lower_phy_baseband_metrics.h"
#include "ocudu/ran/slot_point.h"
#include <optional>

namespace ocudu {

/// \brief Lower physical layer PDxCH processor - Baseband interface.
///
/// Processes baseband samples with slot granularity.
class pdxch_processor_baseband
{
public:
  /// Default destructor.
  virtual ~pdxch_processor_baseband() = default;

  /// Describes the context of a newly generated slot.
  struct slot_context {
    /// Slot context.
    slot_point slot;
    /// Radio sector identifier.
    unsigned sector;
  };

  /// Groups baseband metrics and buffer for a slot.
  struct slot_result {
    /// Collected baseband buffer metrics.
    lower_phy_baseband_metrics metrics = {};
    /// Actual baseband buffer. Set to nullptr if there was no transmit request in the given slot.
    baseband_gateway_buffer_ptr buffer = nullptr;
  };

  /// \brief Processes a baseband OFDM slot.
  ///
  /// \param[in] context OFDM Symbol context.
  /// \return Slot downlink baseband results.
  virtual slot_result process_slot(slot_context context) = 0;

  /// \brief Indicates whether a (non-empty) transmission request has been received for the given slot.
  ///
  /// Used by the M2 event-driven downlink busy-wait to tell a DL slot that will carry a grid apart from an idle DL
  /// slot: the busy-wait only keeps spinning for the modulated grid once a request has actually been handed in, so it
  /// does not burn the transmit lead on idle slots. The default returns \c true (assume a grid is coming), preserving
  /// the previous always-wait behavior for implementations that do not provide the hint.
  ///
  /// \param[in] slot Slot being processed.
  /// \return True if a transmission request for \c slot has been received, false otherwise.
  virtual bool request_seen(slot_point slot) { return true; }
};

} // namespace ocudu
