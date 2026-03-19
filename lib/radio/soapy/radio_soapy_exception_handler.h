// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Pavonis Communications
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI

#pragma once

#include "fmt/format.h"
#include <functional>
#include <string>

namespace ocudu {

/// Exception handler for SoapySDR calls. SoapySDR throws std::runtime_error for all errors.
class soapy_exception_handler
{
public:
  template <typename S, typename... Args>
  void on_error(const S& format_str, Args&&... args)
  {
    error_message = fmt::format(format_str, std::forward<Args>(args)...);
  }

  template <typename F>
  bool safe_execution(F task)
  {
    static_assert(std::is_convertible<F, std::function<void()>>::value, "The function signature must be void()");

    error_message.clear();

    try {
      task();
    } catch (const std::exception& e) {
      error_message = e.what();
      return false;
    } catch (...) {
      error_message = "Unrecognized exception caught.";
      return false;
    }

    return is_successful();
  }

  bool               is_successful() const { return error_message.empty(); }
  const std::string& get_error_message() const { return error_message; }

private:
  std::string error_message;
};

} // namespace ocudu
