// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Pavonis Communications
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI

#pragma once

#include <mutex>

namespace ocudu {

/// Radio SoapySDR transmit stream finite state machine.
///
/// Mirrors radio_uhd_tx_stream_fsm but uses timeNs (nanoseconds) instead of
/// uhd::time_spec_t, and sets SoapySDR int flags instead of uhd::tx_metadata_t.
class radio_soapy_tx_stream_fsm
{
  /// Timeout in nanoseconds to wait for an end-of-burst acknowledgement.
  static constexpr long long WAIT_EOB_ACK_TIMEOUT_NS = 10000000LL; // 10 ms

  enum class states {
    UNINITIALIZED = 0,
    START_BURST,
    IN_BURST,
    END_OF_BURST,
    WAIT_END_OF_BURST,
  };

  states    state = states::UNINITIALIZED;
  mutable std::mutex mutex;
  long long wait_eob_timeout_ns = 0;

public:
  void init_successful()
  {
    std::scoped_lock lock(mutex);
    state = states::START_BURST;
  }

  /// \brief Notifies a late or underflow event. Transitions to END_OF_BURST if in burst.
  void async_event_late_underflow(long long time_ns)
  {
    std::scoped_lock lock(mutex);
    if (state == states::IN_BURST) {
      state                = states::END_OF_BURST;
      wait_eob_timeout_ns  = time_ns + WAIT_EOB_ACK_TIMEOUT_NS;
    }
  }

  /// \brief Notifies an end-of-burst acknowledgement.
  void async_event_end_of_burst_ack()
  {
    std::scoped_lock lock(mutex);
    if (state == states::WAIT_END_OF_BURST) {
      state = states::START_BURST;
    }
  }

  /// \brief Handles a new transmission request.
  ///
  /// \param[out] flags_out     SoapySDR flags to apply (SOAPY_SDR_HAS_TIME, SOAPY_SDR_END_BURST).
  /// \param[in]  time_ns       Transmission timestamp in nanoseconds.
  /// \param[in]  is_empty      True if the buffer contains no signal (gap/padding).
  /// \param[in]  tail_padding  True if this is the last buffer in the burst (tx_end set).
  /// \return True if samples should be transmitted; false if the block shall be dropped.
  bool on_transmit(int& flags_out, long long time_ns, bool is_empty, bool tail_padding)
  {
    std::scoped_lock lock(mutex);

    switch (state) {
      case states::WAIT_END_OF_BURST:
        // If the EOB timeout has not expired, drop the buffer.
        if (wait_eob_timeout_ns >= time_ns) {
          return false;
        }
        // Timeout expired: fall through to START_BURST.
        state = states::START_BURST;
        [[fallthrough]];
      case states::START_BURST:
        if (!is_empty) {
          flags_out |= SOAPY_SDR_HAS_TIME;
          if (tail_padding) {
            flags_out |= SOAPY_SDR_END_BURST;
            // Single-buffer burst: stay in START_BURST (no IN_BURST entry).
          } else {
            state = states::IN_BURST;
          }
          return true;
        }
        return false;

      case states::IN_BURST:
        if (is_empty || tail_padding) {
          flags_out |= SOAPY_SDR_END_BURST;
          state = states::START_BURST;
        }
        return true;

      case states::END_OF_BURST:
        flags_out |= SOAPY_SDR_END_BURST;
        state = states::WAIT_END_OF_BURST;
        if (wait_eob_timeout_ns == 0) {
          wait_eob_timeout_ns = time_ns + WAIT_EOB_ACK_TIMEOUT_NS;
        }
        return true;

      case states::UNINITIALIZED:
      default:
        return false;
    }
  }

  /// Returns true if an end-of-burst must be sent when stopping.
  bool on_stop() const
  {
    std::scoped_lock lock(mutex);
    return state == states::IN_BURST;
  }
};

} // namespace ocudu
