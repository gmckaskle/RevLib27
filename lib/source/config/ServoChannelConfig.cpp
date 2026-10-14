/*
 * Copyright (c) 2024 REV Robotics
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

#include "rev/config/ServoChannelConfig.h"

#include <array>

#include "rev/config/ServoHubParameters.h"

using namespace rev::servohub;

namespace {

using chanParams =
    std::array<ServoHubParameter, ServoChannel::kNumServoChannels>;

chanParams minPulseParams{kChannel0_MinPulseWidth, kChannel1_MinPulseWidth,
                          kChannel2_MinPulseWidth, kChannel3_MinPulseWidth,
                          kChannel4_MinPulseWidth, kChannel5_MinPulseWidth};
chanParams centerPulseParams{
    kChannel0_CenterPulseWidth, kChannel1_CenterPulseWidth,
    kChannel2_CenterPulseWidth, kChannel3_CenterPulseWidth,
    kChannel4_CenterPulseWidth, kChannel5_CenterPulseWidth};
chanParams maxPulseParams{kChannel0_MaxPulseWidth, kChannel1_MaxPulseWidth,
                          kChannel2_MaxPulseWidth, kChannel3_MaxPulseWidth,
                          kChannel4_MaxPulseWidth, kChannel5_MaxPulseWidth};

chanParams disableBehaviorParams{
    kChannel0_DisableBehavior, kChannel1_DisableBehavior,
    kChannel2_DisableBehavior, kChannel3_DisableBehavior,
    kChannel4_DisableBehavior, kChannel5_DisableBehavior};
}  // namespace

ServoChannelConfig::ServoChannelConfig(ServoChannel::ChannelId channelId)
    : m_channelId{channelId} {}

ServoChannelConfig& ServoChannelConfig::Apply(ServoChannelConfig& config) {
    // NOTE: Do Not Call BaseConfig::Apply() as it won't copy the correct
    // parameters. Manually call PutParameter for each parameter present
    // in the input config.

    const size_t srcChanIdx{static_cast<size_t>(config.m_channelId)};
    const size_t destChanIdx{static_cast<size_t>(m_channelId)};

    // Only Apply the input config if parameters have been set on it.
    if (const auto minPulse = config.GetParameter(minPulseParams[srcChanIdx]);
        minPulse) {
        PutParameter(minPulseParams[destChanIdx], *minPulse);
    }
    if (const auto centerPulse =
            config.GetParameter(centerPulseParams[srcChanIdx]);
        centerPulse) {
        PutParameter(centerPulseParams[destChanIdx], *centerPulse);
    }
    if (const auto maxPulse = config.GetParameter(maxPulseParams[srcChanIdx]);
        maxPulse) {
        PutParameter(maxPulseParams[destChanIdx], *maxPulse);
    }

    if (const auto disableBehavior =
            config.GetParameter(disableBehaviorParams[srcChanIdx]);
        disableBehavior) {
        PutParameter(disableBehaviorParams[destChanIdx], *disableBehavior);
    }
    return *this;
}

ServoChannelConfig& ServoChannelConfig::PulseRange(uint32_t minPulse_us,
                                                   uint32_t centerPulse_us,
                                                   uint32_t maxPulse_us) {
    const size_t chanIdx{static_cast<size_t>(m_channelId)};

    PutParameter(minPulseParams[chanIdx], minPulse_us);
    PutParameter(centerPulseParams[chanIdx], centerPulse_us);
    PutParameter(maxPulseParams[chanIdx], maxPulse_us);
    return *this;
}

ServoChannelConfig& ServoChannelConfig::PulseRange(
    const PulseRange_t& pulseRange_us) {
    return PulseRange(pulseRange_us.minPulse_us, pulseRange_us.centerPulse_us,
                      pulseRange_us.maxPulse_us);
}

ServoChannelConfig& ServoChannelConfig::DisableBehavior(
    BehaviorWhenDisabled behavior) {
    const size_t chanIdx{static_cast<size_t>(m_channelId)};

    bool disableBehavior = behavior == BehaviorWhenDisabled::kSupplyPower;

    PutParameter(disableBehaviorParams[chanIdx], disableBehavior);
    return *this;
}
