/*
 * Copyright (c) 2025-2026 REV Robotics
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

#include "rev/config/DetachedSignalsConfig.h"

#include <algorithm>

#include "rev/config/DetachedEncoderParameters.h"

using namespace rev::detached;

SignalsConfig& SignalsConfig::Apply(SignalsConfig& config) {
    BaseConfig::Apply(config);
    return *this;
}

SignalsConfig& SignalsConfig::EncoderPositionPeriodMs(int period_ms) {
    SetPeriodMsCore(DetachedEncoderParameter::kStatus3Period, period_ms);
    return *this;
}

SignalsConfig& SignalsConfig::EncoderVelocityPeriodMs(int period_ms) {
    SetPeriodMsCore(DetachedEncoderParameter::kStatus4Period, period_ms);
    return *this;
}

SignalsConfig& SignalsConfig::EncoderAnglePeriodMs(int period_ms) {
    SetPeriodMsCore(DetachedEncoderParameter::kStatus2Period, period_ms);
    return *this;
}

void SignalsConfig::SetPeriodMsCore(int parameterId, int period_ms) {
    auto optValue = GetParameter(parameterId);
    const uint32_t period{static_cast<uint32_t>(period_ms)};

    if (!optValue) {
        PutParameter(parameterId, period);
    } else {
        const uint32_t currentPeriod_ms = std::get<uint32_t>(*optValue);
        PutParameter(parameterId, std::min(currentPeriod_ms, period));
    }
}
