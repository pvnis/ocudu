# SPDX-FileCopyrightText: Copyright (C) 2021-2026 Pavonis Communications
# SPDX-License-Identifier: BSD-3-Clause-Open-MPI

set(SOURCES_SOAPY
        radio_config_soapy_validator.cpp
        radio_soapy_impl.cpp
        radio_soapy_rx_stream.cpp
        radio_soapy_tx_stream.cpp)

add_library(ocudu_radio_soapy STATIC ${SOURCES_SOAPY})

target_compile_options(ocudu_radio_soapy PRIVATE -Wno-unused-parameter -Wno-missing-field-initializers)

target_link_libraries(ocudu_radio_soapy PRIVATE ${SoapySDR_LIBRARIES} fmt ocudulog ocudu_support)

add_to_exported_libs(ocudu_radio_soapy)
