/*
 * Copyright (c) 2018-2026 REV Robotics
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of REV Robotics nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "rev/SparkMax.h"

#include <wpi/system/Errors.hpp>

#include "rev/CANSparkDriver.h"

using namespace rev::spark;

SparkMax::SparkMax(wpi::CANPort canPort, int deviceID, MotorType type)
    : SparkBase{canPort, deviceID, type, SparkModel::kSparkMax},
      configAccessor{m_sparkMaxHandle},
      m_AltEncoder{*this} {
    c_Spark_SparkModel model;
    c_Spark_GetSparkModel(static_cast<c_Spark_handle>(m_sparkMaxHandle),
                          &model);
    if (model != c_Spark_SparkMax) {
        WPILIB_ReportError(
            wpi::warn::Warning,
            "SparkMax object created for CAN Bus ID {} Device ID {}, which is "
            "not a SPARK MAX (is {}). Some functionalities may not work.",
            static_cast<int>(canPort), deviceID, static_cast<int>(model));
    }
}

rev::REVLibError SparkMax::Configure(SparkBaseConfig& config,
                                     rev::ResetMode resetMode,
                                     rev::PersistMode persistMode) {
    rev::REVLibError status =
        SparkBase::Configure(config, resetMode, persistMode);

    if (m_altEncoderCreated) {
        CheckDataPortAlternateEncoder();
    }

    if (m_absoluteEncoderCreated) {
        CheckDataPortAbsoluteEncoder();
    }

    if (m_forwardLimitSwitchCreated || m_reverseLimitSwitchCreated) {
        CheckDataPortLimitSwitch();
    }

    return status;
}

SparkMaxAlternateEncoder& SparkMax::GetAlternateEncoder() {
    CheckDataPortAlternateEncoder();
    m_altEncoderCreated.exchange(true);
    return m_AltEncoder;
}

SparkAbsoluteEncoder& SparkMax::GetAbsoluteEncoder() {
    CheckDataPortAbsoluteEncoder();
    return SparkBase::GetAbsoluteEncoder();
}

SparkLimitSwitch& SparkMax::GetForwardLimitSwitch() {
    CheckDataPortLimitSwitch();
    return SparkBase::GetForwardLimitSwitch();
}

SparkLimitSwitch& SparkMax::GetReverseLimitSwitch() {
    CheckDataPortLimitSwitch();
    return SparkBase::GetReverseLimitSwitch();
}

void SparkMax::CheckDataPortAlternateEncoder() {
    uint8_t configured;
    c_Spark_IsDataPortConfigured(static_cast<c_Spark_handle>(m_sparkMaxHandle),
                                 &configured);
    if (configured) {
        c_Spark_DataPortConfig config;
        c_Spark_GetDataPortConfig(static_cast<c_Spark_handle>(m_sparkMaxHandle),
                                  &config);
        if (config !=
            c_Spark_DataPortConfig::c_Spark_kDataPortConfigAltEncoder) {
            throw std::runtime_error(
                std::format("The SPARK MAX is not configured to use an "
                            "alternate encoder."));
        }
    }
}

void SparkMax::CheckDataPortAbsoluteEncoder() {
    uint8_t configured;
    c_Spark_IsDataPortConfigured(static_cast<c_Spark_handle>(m_sparkMaxHandle),
                                 &configured);
    if (configured) {
        c_Spark_DataPortConfig config;
        c_Spark_GetDataPortConfig(static_cast<c_Spark_handle>(m_sparkMaxHandle),
                                  &config);
        if (config != c_Spark_DataPortConfig::c_Spark_kDataPortConfigDefault) {
            throw std::runtime_error(std::format(
                "The SPARK MAX is not configured to use an absolute encoder."));
        }
    }
}

void SparkMax::CheckDataPortLimitSwitch() {
    uint8_t configured;
    c_Spark_IsDataPortConfigured(static_cast<c_Spark_handle>(m_sparkMaxHandle),
                                 &configured);
    if (configured) {
        c_Spark_DataPortConfig config;
        c_Spark_GetDataPortConfig(static_cast<c_Spark_handle>(m_sparkMaxHandle),
                                  &config);
        if (config != c_Spark_DataPortConfig::c_Spark_kDataPortConfigDefault) {
            throw std::runtime_error(std::format(
                "The SPARK MAX is not configured to use limit switches."));
        }
    }
}
