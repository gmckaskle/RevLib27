/*
 * Copyright (c) 2024-2026 REV Robotics
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

#include "rev/ServoChannel.h"

#include "rev/CANServoHubDriver.h"
#include "rev/ServoHubLowLevel.h"

using namespace rev;
using namespace rev::util;
using namespace rev::servohub;

ServoChannel::ServoChannel(ChannelId channelId, ServoHubLowLevel* device)
    : m_channelId{channelId}, m_device{device} {}

ServoChannel::~ServoChannel() { m_device = nullptr; }

#define TO_SERVO_HUB_CHANNEL(channelId) \
    static_cast<c_ServoHub_Channel>(static_cast<int>(channelId))

#define SERVO_HUB_HANDLE() \
    static_cast<c_ServoHub_handle>(m_device->m_servoHubHandle)

Signal<int> ServoChannel::GetPulseWidth() const {
    if (m_channelId <= ChannelId::kChannelId2) {
        return m_device->GetPeriodicStatus2().Map(
            [channelId = m_channelId](const auto& s) -> int {
                switch (channelId) {
                    case ChannelId::kChannelId0:
                        return s.channel0PulseWidth;
                    case ChannelId::kChannelId1:
                        return s.channel1PulseWidth;
                    case ChannelId::kChannelId2:
                        return s.channel2PulseWidth;
                    default:
                        return 0;
                }
            });
    } else {
        return m_device->GetPeriodicStatus3().Map(
            [channelId = m_channelId](const auto& s) -> int {
                switch (channelId) {
                    case ChannelId::kChannelId3:
                        return s.channel3PulseWidth;
                    case ChannelId::kChannelId4:
                        return s.channel4PulseWidth;
                    case ChannelId::kChannelId5:
                        return s.channel5PulseWidth;
                    default:
                        return 0;
                }
            });
    }
}

rev::util::Signal<bool> rev::servohub::ServoChannel::IsEnabled() const {
    if (m_channelId <= ChannelId::kChannelId2) {
        return m_device->GetPeriodicStatus2().Map(
            [channelId = m_channelId](const auto& s) -> bool {
                switch (channelId) {
                    case ChannelId::kChannelId0:
                        return s.channel0Enabled != 0;
                    case ChannelId::kChannelId1:
                        return s.channel1Enabled != 0;
                    case ChannelId::kChannelId2:
                        return s.channel2Enabled != 0;
                    default:
                        return false;
                }
            });
    } else {
        return m_device->GetPeriodicStatus3().Map(
            [channelId = m_channelId](const auto& s) -> bool {
                switch (channelId) {
                    case ChannelId::kChannelId3:
                        return s.channel3Enabled != 0;
                    case ChannelId::kChannelId4:
                        return s.channel4Enabled != 0;
                    case ChannelId::kChannelId5:
                        return s.channel5Enabled != 0;
                    default:
                        return false;
                }
            });
    }
}

Signal<double> ServoChannel::GetCurrent() const {
    return m_device->GetPeriodicStatus4().Map(
        [channelId = m_channelId](const auto& s) -> double {
            switch (channelId) {
                case ChannelId::kChannelId0:
                    return s.channel0Current;
                case ChannelId::kChannelId1:
                    return s.channel1Current;
                case ChannelId::kChannelId2:
                    return s.channel2Current;
                case ChannelId::kChannelId3:
                    return s.channel3Current;
                case ChannelId::kChannelId4:
                    return s.channel4Current;
                case ChannelId::kChannelId5:
                    return s.channel5Current;
                default:
                    return 0.0;
            }
        });
}

REVLibError ServoChannel::SetPulseWidth(int pulseWidth_us) {
    auto status = c_ServoHub_SetChannelPulseWidth(
        SERVO_HUB_HANDLE(), TO_SERVO_HUB_CHANNEL(m_channelId), pulseWidth_us);
    return static_cast<REVLibError>(status);
}

REVLibError ServoChannel::SetEnabled(bool enabled) {
    auto status = c_ServoHub_SetChannelEnabled(
        SERVO_HUB_HANDLE(), TO_SERVO_HUB_CHANNEL(m_channelId), enabled);
    return static_cast<REVLibError>(status);
}

REVLibError ServoChannel::SetPowered(bool powered) {
    auto status = c_ServoHub_SetChannelPowered(
        SERVO_HUB_HANDLE(), TO_SERVO_HUB_CHANNEL(m_channelId), powered);
    return static_cast<REVLibError>(status);
}
