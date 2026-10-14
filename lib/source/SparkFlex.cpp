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

#include "rev/SparkFlex.h"

#include <wpi/system/Errors.hpp>

#include "rev/CANSparkDriver.h"

using namespace rev::spark;

SparkFlex::SparkFlex(wpi::CANPort canPort, int deviceID, MotorType type)
    : SparkBase{canPort, deviceID, type, SparkModel::kSparkFlex},
      configAccessor{m_sparkMaxHandle},
      m_ExternalEncoder{*this} {
    c_Spark_SparkModel model;
    c_Spark_GetSparkModel(static_cast<c_Spark_handle>(m_sparkMaxHandle),
                          &model);
    if (model != c_Spark_SparkFlex) {
        WPILIB_ReportError(
            wpi::warn::Warning,
            "SparkFlex object created for CAN Bus ID {} Device ID {}, which is "
            "not a SPARK Flex (is {}). Some functionalities may not work.",
            static_cast<int>(canPort), deviceID, static_cast<int>(model));
    }
}

SparkFlexExternalEncoder& SparkFlex::GetExternalEncoder() {
    return m_ExternalEncoder;
}
