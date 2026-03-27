// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Pavonis Communications
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI

#pragma once

#include "radio_soapy_exception_handler.h"
#include "ocudu/ocudulog/ocudulog.h"
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Logger.hpp>
#include <SoapySDR/Types.hpp>
#include <chrono>
#include <cstdlib>
#include <string_view>
#include <mutex>
#include <string>
#include <vector>
  
namespace ocudu {

/// Thin wrapper around SoapySDR::Device providing safe_execution semantics.
class radio_soapy_device : public soapy_exception_handler
{
public:
  radio_soapy_device() : logger(ocudulog::fetch_basic_logger("RF"))
  {
    static std::once_flag registered;
    std::call_once(registered, []() {
      SoapySDR::registerLogHandler([](const SoapySDRLogLevel level, const char* message) {
        ocudulog::basic_logger& lg = ocudulog::fetch_basic_logger("RF");
        switch (level) {
          case SOAPY_SDR_FATAL:
          case SOAPY_SDR_CRITICAL:
          case SOAPY_SDR_ERROR:
            lg.error("SoapySDR: {}", message);
            break;
          case SOAPY_SDR_WARNING:
            lg.warning("SoapySDR: {}", message);
            break;
          case SOAPY_SDR_NOTICE:
          case SOAPY_SDR_INFO:
            lg.info("SoapySDR: {}", message);
            break;
          case SOAPY_SDR_DEBUG:
          case SOAPY_SDR_TRACE:
          default:
            lg.debug("SoapySDR: {}", message);
            break;
        }
      });
    });

    if (const char* env = std::getenv("OCUDU_SOAPY_TRACE")) {
      trace_enabled = std::string_view(env) != "0";
    }
    if (const char* env = std::getenv("OCUDU_SOAPY_TRACE_US")) {
      trace_slow_us = std::strtol(env, nullptr, 10);
      if (trace_slow_us <= 0) {
        trace_slow_us = 200;
      }
    }
  }

  ~radio_soapy_device() { unmake(); }

  bool is_valid() const { return device != nullptr; }

  bool make(const std::string& args)
  {
    return safe_execution([this, &args]() { device = SoapySDR::Device::make(args); });
  }

  void unmake()
  {
    if (device) {
      SoapySDR::Device::unmake(device);
      device = nullptr;
    }
  }

  bool set_sample_rate(int direction, size_t channel, double rate)
  {
    logger.debug("Setting {} ch{} sample rate to {:.3f} MHz.", direction == SOAPY_SDR_TX ? "TX" : "RX", channel, to_MHz(rate));
    return safe_execution([this, direction, channel, rate]() { device->setSampleRate(direction, channel, rate); });
  }

  bool set_frequency(int direction, size_t channel, double freq_Hz)
  {
    logger.debug("Setting {} ch{} frequency to {:.3f} MHz.", direction == SOAPY_SDR_TX ? "TX" : "RX", channel, to_MHz(freq_Hz));
    return safe_execution([this, direction, channel, freq_Hz]() { device->setFrequency(direction, channel, freq_Hz); });
  }

  bool set_gain(int direction, size_t channel, double gain_dB)
  {
    logger.debug("Setting {} ch{} gain to {:.2f} dB.", direction == SOAPY_SDR_TX ? "TX" : "RX", channel, gain_dB);
    return safe_execution([this, direction, channel, gain_dB]() { device->setGain(direction, channel, gain_dB); });
  }

  bool get_hardware_time(long long& time_ns, const std::string& what = "")
  {
    return safe_execution([this, &time_ns, &what]() { time_ns = device->getHardwareTime(what); });
  }

  SoapySDR::Stream* setup_stream(int                              direction,
                                 const std::vector<size_t>&       channels,
                                 const SoapySDR::Kwargs&          kwargs = SoapySDR::Kwargs())
  {
    SoapySDR::Stream* stream = nullptr;
    safe_execution([this, direction, &channels, &kwargs, &stream]() {
      stream = device->setupStream(direction, SOAPY_SDR_CS16, channels, kwargs);
    });
    return stream;
  }

  bool activate_stream(SoapySDR::Stream* stream, int flags = 0, long long time_ns = 0)
  {
    return safe_execution([this, stream, flags, time_ns]() { device->activateStream(stream, flags, time_ns); });
  }

  bool deactivate_stream(SoapySDR::Stream* stream, int flags = 0, long long time_ns = 0)
  {
    return safe_execution([this, stream, flags, time_ns]() { device->deactivateStream(stream, flags, time_ns); });
  }

  bool close_stream(SoapySDR::Stream* stream)
  {
    return safe_execution([this, stream]() { device->closeStream(stream); });
  }

  size_t get_stream_mtu(SoapySDR::Stream* stream)
  {
    size_t mtu = 0;
    safe_execution([this, stream, &mtu]() { mtu = device->getStreamMTU(stream); });
    return mtu;
  }

  /// Acquire a zero-copy TX write buffer. Returns the handle, or -1 on error.
  int acquire_write_buffer(SoapySDR::Stream* stream, void** buffs, long timeout_us = 1000)
  {
    size_t handle = 0;
    int    ret    = 0;
    const auto t0 = std::chrono::steady_clock::now();
    safe_execution([this, stream, &buffs, &handle, timeout_us, &ret]() {
      ret = device->acquireWriteBuffer(stream, handle, buffs, timeout_us);
    });
    log_trace("acquireWriteBuffer", std::chrono::steady_clock::now() - t0, ret, timeout_us);
    if (ret < 0) {
      return ret;
    }
    return static_cast<int>(handle);
  }

  bool release_write_buffer(SoapySDR::Stream* stream,
                             size_t            handle,
                             size_t            num_elems,
                             int               flags,
                             long long         time_ns)
  {
    const auto t0 = std::chrono::steady_clock::now();
    const bool ok = safe_execution([this, stream, handle, num_elems, flags, time_ns]() mutable {
      device->releaseWriteBuffer(stream, handle, num_elems, flags, time_ns);
    });
    log_trace("releaseWriteBuffer", std::chrono::steady_clock::now() - t0, ok ? 0 : -1, 0);
    return ok;
  }

  int write_stream(SoapySDR::Stream*        stream,
                   const void* const*       buffs,
                   size_t                   num_elems,
                   int&                     flags,
                   long long                time_ns,
                   long                     timeout_us = 0)
  {
    int ret = 0;
    const auto t0 = std::chrono::steady_clock::now();
    safe_execution([this, stream, &buffs, num_elems, &flags, time_ns, timeout_us, &ret]() {
      ret = device->writeStream(stream, buffs, num_elems, flags, time_ns, timeout_us);
    });
    log_trace("writeStream", std::chrono::steady_clock::now() - t0, ret, timeout_us);
    return ret;
  }

  /// Acquire a zero-copy RX read buffer. Returns num_samples, or negative on error.
  int acquire_read_buffer(SoapySDR::Stream* stream,
                          size_t&           handle,
                          const void**      buffs,
                          int&              flags,
                          long long&        time_ns,
                          long              timeout_us = 200000)
  {
    int ret = 0;
    const auto t0 = std::chrono::steady_clock::now();
    safe_execution([this, stream, &handle, &buffs, &flags, &time_ns, timeout_us, &ret]() {
      ret = device->acquireReadBuffer(stream, handle, buffs, flags, time_ns, timeout_us);
    });
    log_trace("acquireReadBuffer", std::chrono::steady_clock::now() - t0, ret, timeout_us);
    return ret;
  }

  bool release_read_buffer(SoapySDR::Stream* stream, size_t handle)
  {
    return safe_execution([this, stream, handle]() { device->releaseReadBuffer(stream, handle); });
  }

  /// Poll for async TX status. Returns SOAPY_SDR_TIME_ERROR, SOAPY_SDR_UNDERFLOW, 0, or negative on error.
  int read_stream_status(SoapySDR::Stream* stream,
                         size_t&           chan_mask,
                         int&              flags,
                         long long&        time_ns,
                         long              timeout_us = 1000)
  {
    int ret = 0;
    const auto t0 = std::chrono::steady_clock::now();
    safe_execution([this, stream, &chan_mask, &flags, &time_ns, timeout_us, &ret]() {
      ret = device->readStreamStatus(stream, chan_mask, flags, time_ns, timeout_us);
    });
    log_trace("readStreamStatus", std::chrono::steady_clock::now() - t0, ret, timeout_us);
    return ret;
  }

private:
  SoapySDR::Device*       device = nullptr;
  ocudulog::basic_logger& logger;
  bool                    trace_enabled = false;
  long                    trace_slow_us = 200;

  static constexpr double to_MHz(double value_Hz)
  {
    return value_Hz * 1e-6;
  }

  template <typename Duration>
  void log_trace(const char* op, Duration dt, int ret, long timeout_us)
  {
    if (!trace_enabled) {
      return;
    }
    const long us = std::chrono::duration_cast<std::chrono::microseconds>(dt).count();
    if (ret < 0 || ret == SOAPY_SDR_OVERFLOW || ret == SOAPY_SDR_UNDERFLOW || ret == SOAPY_SDR_TIME_ERROR ||
        us >= trace_slow_us) {
      logger.info("Soapy trace: {} ret={} dt={}us timeout={}us", op, ret, us, timeout_us);
    }
  }
};

} // namespace ocudu
