// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Pavonis Communications
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI

#pragma once

#include "ocudu/radio/radio_configuration.h"

namespace ocudu {

/// \brief Radio configuration validator for the SoapySDR/M2SDR radio driver.
///
/// Validates that parameter values are within the capabilities of the AD9361 RFIC
/// on the LiteX-M2SDR board. Does not query the hardware; bounds are static.
class radio_config_soapy_validator : public radio_configuration::validator
{
public:
  // See interface for documentation.
  bool is_configuration_valid(const radio_configuration::radio& config) const override;
};

} // namespace ocudu
