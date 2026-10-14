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

#include "rev/config/ServoHubConfig.h"

#include <array>
#include <string>

using namespace rev::servohub;

ServoHubConfig& ServoHubConfig::Apply(ServoHubConfig& config) {
    BaseConfig::Apply(config);
    channel0.Apply(config.channel0);
    channel1.Apply(config.channel1);
    channel2.Apply(config.channel2);
    channel3.Apply(config.channel3);
    channel4.Apply(config.channel4);
    channel5.Apply(config.channel5);
    return *this;
}

ServoHubConfig& ServoHubConfig::Apply(ServoChannel::ChannelId channelId,
                                      ServoChannelConfig& config) {
    const size_t chanIdx{static_cast<size_t>(channelId)};
    m_ChannelConfigs[chanIdx]->Apply(config);

    return *this;
}

std::string ServoHubConfig::Flatten() {
    std::string flattenedString{BaseConfig::Flatten() + channel0.Flatten() +
                                channel1.Flatten() + channel2.Flatten() +
                                channel3.Flatten() + channel4.Flatten() +
                                channel5.Flatten()};

    return flattenedString;
}
