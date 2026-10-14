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

#include "rev/config/ServoHubConfigAccessor.h"

#include <array>

using namespace rev::servohub;

ServoHubConfigAccessor::ServoHubConfigAccessor(void* servoHubHandle)
    : channel0(ServoChannel::ChannelId::kChannelId0, servoHubHandle),
      channel1(ServoChannel::ChannelId::kChannelId1, servoHubHandle),
      channel2(ServoChannel::ChannelId::kChannelId2, servoHubHandle),
      channel3(ServoChannel::ChannelId::kChannelId3, servoHubHandle),
      channel4(ServoChannel::ChannelId::kChannelId4, servoHubHandle),
      channel5(ServoChannel::ChannelId::kChannelId5, servoHubHandle) {}

ServoChannelConfigAccessor& ServoHubConfigAccessor::channel(
    ServoChannel::ChannelId channelId) {
    static std::array sChannelAccessors{&channel0, &channel1, &channel2,
                                        &channel3, &channel4, &channel5};
    const size_t idx{static_cast<size_t>(channelId)};
    return *sChannelAccessors[idx];
}
