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

#include <array>
#include <string>

#include "TestUtils.h"
#include "gtest/gtest.h"
#include "rev/config/ServoHubConfig.h"
#include "rev/config/ServoHubParameters.h"

using namespace rev::servohub;

class SubServoChannelConfig : public ServoChannelConfig {
public:
    explicit SubServoChannelConfig(ServoChannel::ChannelId channelId)
        : ServoChannelConfig{channelId} {}

    std::optional<ParameterType_t> GetParam(uint8_t parameterId) {
        return GetParameter(parameterId);
    }
};

struct ChanParams {
    ServoChannel::ChannelId id;
    unsigned int idx;
    uint8_t min;
    uint8_t center;
    uint8_t max;
    uint8_t disable;
};
const std::array<ChanParams, ServoChannel::kNumServoChannels> ChanTestParams{
    {{ServoChannel::ChannelId::kChannelId0, 0u,
      ServoHubParameter::kChannel0_MinPulseWidth,
      ServoHubParameter::kChannel0_CenterPulseWidth,
      ServoHubParameter::kChannel0_MaxPulseWidth,
      ServoHubParameter::kChannel0_DisableBehavior},
     {ServoChannel::ChannelId::kChannelId1, 1u,
      ServoHubParameter::kChannel1_MinPulseWidth,
      ServoHubParameter::kChannel1_CenterPulseWidth,
      ServoHubParameter::kChannel1_MaxPulseWidth,
      ServoHubParameter::kChannel1_DisableBehavior},
     {ServoChannel::ChannelId::kChannelId2, 2u,
      ServoHubParameter::kChannel2_MinPulseWidth,
      ServoHubParameter::kChannel2_CenterPulseWidth,
      ServoHubParameter::kChannel2_MaxPulseWidth,
      ServoHubParameter::kChannel2_DisableBehavior},
     {ServoChannel::ChannelId::kChannelId3, 3u,
      ServoHubParameter::kChannel3_MinPulseWidth,
      ServoHubParameter::kChannel3_CenterPulseWidth,
      ServoHubParameter::kChannel3_MaxPulseWidth,
      ServoHubParameter::kChannel3_DisableBehavior},
     {ServoChannel::ChannelId::kChannelId4, 4u,
      ServoHubParameter::kChannel4_MinPulseWidth,
      ServoHubParameter::kChannel4_CenterPulseWidth,
      ServoHubParameter::kChannel4_MaxPulseWidth,
      ServoHubParameter::kChannel4_DisableBehavior},
     {ServoChannel::ChannelId::kChannelId5, 5u,
      ServoHubParameter::kChannel5_MinPulseWidth,
      ServoHubParameter::kChannel5_CenterPulseWidth,
      ServoHubParameter::kChannel5_MaxPulseWidth,
      ServoHubParameter::kChannel5_DisableBehavior}}};

TEST(ServoChannelConfigTest, SetsPulseRange) {
    for (const auto& chan : ChanTestParams) {
        SubServoChannelConfig config(chan.id);

        config.PulseRange(525 + chan.idx, 1500 + chan.idx, 2475 + chan.idx);
        auto minPulse = config.GetParam(chan.min);
        auto centerPulse = config.GetParam(chan.center);
        auto maxPulse = config.GetParam(chan.max);
        ASSERT_TRUE(minPulse) << chan.idx << ": " << config.Flatten();
        ASSERT_TRUE(centerPulse);
        ASSERT_TRUE(maxPulse);
        EXPECT_EQ(std::get<uint32_t>(*minPulse), 525 + chan.idx);
        EXPECT_EQ(std::get<uint32_t>(*centerPulse), 1500 + chan.idx);
        EXPECT_EQ(std::get<uint32_t>(*maxPulse), 2475 + chan.idx);
    }
}

TEST(ServoChannelConfigTest, SetsPulseRangeStruct) {
    for (const auto& chan : ChanTestParams) {
        SubServoChannelConfig config(chan.id);

        const ServoChannelConfig::PulseRange_t pulseRange{
            525 + chan.idx, 1500 + chan.idx, 2475 + chan.idx};
        config.PulseRange(pulseRange);
        auto minPulse = config.GetParam(chan.min);
        auto centerPulse = config.GetParam(chan.center);
        auto maxPulse = config.GetParam(chan.max);
        ASSERT_TRUE(minPulse) << chan.idx << ": " << config.Flatten();
        ASSERT_TRUE(centerPulse);
        ASSERT_TRUE(maxPulse);
        EXPECT_EQ(std::get<uint32_t>(*minPulse), 525 + chan.idx);
        EXPECT_EQ(std::get<uint32_t>(*centerPulse), 1500 + chan.idx);
        EXPECT_EQ(std::get<uint32_t>(*maxPulse), 2475 + chan.idx);
    }
}

TEST(ServoChannelConfigTest, SetsDisableBehavior) {
    for (const auto& chan : ChanTestParams) {
        SubServoChannelConfig config(chan.id);

        config.DisableBehavior(
            ServoChannelConfig::BehaviorWhenDisabled::kSupplyPower);
        auto disableBehavior = config.GetParam(chan.disable);
        ASSERT_TRUE(disableBehavior) << chan.idx << ": " << config.Flatten();
        EXPECT_EQ(std::get<bool>(*disableBehavior), true);
    }

    for (const auto& chan : ChanTestParams) {
        SubServoChannelConfig config(chan.id);

        config.DisableBehavior(
            ServoChannelConfig::BehaviorWhenDisabled::kDoNotSupplyPower);
        auto disableBehavior = config.GetParam(chan.disable);
        ASSERT_TRUE(disableBehavior) << chan.idx << ": " << config.Flatten();
        EXPECT_EQ(std::get<bool>(*disableBehavior), false);
    }
}

TEST(ServoChannelConfigTest, ServoChannelApply) {
    for (const auto& srcChan : ChanTestParams) {
        SubServoChannelConfig srcConfig(srcChan.id);

        srcConfig.PulseRange(545 + srcChan.idx, 1540 + srcChan.idx,
                             2445 + srcChan.idx);

        for (const auto& destChan : ChanTestParams) {
            if (srcChan.id == destChan.id) continue;

            SubServoChannelConfig destConfig(destChan.id);
            destConfig.Apply(srcConfig);

            auto minPulse = destConfig.GetParam(destChan.min);
            auto centerPulse = destConfig.GetParam(destChan.center);
            auto maxPulse = destConfig.GetParam(destChan.max);
            ASSERT_TRUE(minPulse)
                << static_cast<int>(srcChan.id) << "(" << srcChan.idx
                << "): " << static_cast<int>(srcChan.min) << "."
                << static_cast<int>(srcChan.center) << "."
                << static_cast<int>(srcChan.max) << " -> "
                << static_cast<int>(destChan.id) << "(" << destChan.idx
                << "): " << static_cast<int>(destChan.min) << "."
                << static_cast<int>(destChan.center) << "."
                << static_cast<int>(destChan.max) << "\n"
                << srcConfig.Flatten() << "\n\n"
                << destConfig.Flatten();

            ASSERT_TRUE(centerPulse);
            ASSERT_TRUE(maxPulse);
            EXPECT_EQ(std::get<uint32_t>(*minPulse), 545 + srcChan.idx);
            EXPECT_EQ(std::get<uint32_t>(*centerPulse), 1540 + srcChan.idx);
            EXPECT_EQ(std::get<uint32_t>(*maxPulse), 2445 + srcChan.idx);
        }
    }
}

class SubServoHubConfig : public ServoHubConfig {
public:
    std::optional<ParameterType_t> GetParam(uint8_t parameterId) {
        return GetParameter(parameterId);
    }
};

TEST(ServoHubConfigTest, Flattens) {
    ServoHubConfig config{};

    config.channel0.PulseRange(505, 1234, 1856)
        .DisableBehavior(
            ServoChannelConfig::BehaviorWhenDisabled::kSupplyPower);
    config.channel1.PulseRange(605, 1235, 1956)
        .DisableBehavior(
            ServoChannelConfig::BehaviorWhenDisabled::kDoNotSupplyPower);
    config.channel2.PulseRange(705, 1236, 1766)
        .DisableBehavior(
            ServoChannelConfig::BehaviorWhenDisabled::kSupplyPower);
    config.channel3.PulseRange(805, 1237, 1776)
        .DisableBehavior(
            ServoChannelConfig::BehaviorWhenDisabled::kDoNotSupplyPower);
    config.channel4.PulseRange(555, 1238, 1757)
        .DisableBehavior(
            ServoChannelConfig::BehaviorWhenDisabled::kSupplyPower);
    config.channel5.PulseRange(565, 1239, 1758)
        .DisableBehavior(
            ServoChannelConfig::BehaviorWhenDisabled::kDoNotSupplyPower);

    std::string flattened{config.Flatten()};

    ValidateFlattenedSubString(flattened, "0,1F9\n");
    ValidateFlattenedSubString(flattened, "1,4D2\n");
    ValidateFlattenedSubString(flattened, "2,740\n");
    ValidateFlattenedSubString(flattened, "3,25D\n");
    ValidateFlattenedSubString(flattened, "4,4D3\n");
    ValidateFlattenedSubString(flattened, "5,7A4\n");
    ValidateFlattenedSubString(flattened, "6,2C1\n");
    ValidateFlattenedSubString(flattened, "7,4D4\n");
    ValidateFlattenedSubString(flattened, "8,6E6\n");
    ValidateFlattenedSubString(flattened, "9,325\n");
    ValidateFlattenedSubString(flattened, "10,4D5\n");
    ValidateFlattenedSubString(flattened, "11,6F0\n");
    ValidateFlattenedSubString(flattened, "12,22B\n");
    ValidateFlattenedSubString(flattened, "13,4D6\n");
    ValidateFlattenedSubString(flattened, "14,6DD\n");
    ValidateFlattenedSubString(flattened, "15,235\n");
    ValidateFlattenedSubString(flattened, "16,4D7\n");
    ValidateFlattenedSubString(flattened, "17,6DE\n");
    ValidateFlattenedSubString(flattened, "18,1\n");
    ValidateFlattenedSubString(flattened, "19,0\n");
    ValidateFlattenedSubString(flattened, "20,1\n");
    ValidateFlattenedSubString(flattened, "21,0\n");
    ValidateFlattenedSubString(flattened, "22,1\n");
    ValidateFlattenedSubString(flattened, "23,0\n");
    EXPECT_EQ(flattened.size(), 0u) << flattened;
}

TEST(ServoHubConfigTest, ServoHubApply) {
    ServoHubConfig first;
    ServoHubConfig second;

    first.channel0.PulseRange(505, 1234, 1856)
        .DisableBehavior(
            ServoChannelConfig::BehaviorWhenDisabled::kSupplyPower);
    first.channel1.PulseRange(605, 1235, 1956)
        .DisableBehavior(
            ServoChannelConfig::BehaviorWhenDisabled::kDoNotSupplyPower);
    first.channel2.PulseRange(705, 1236, 1766)
        .DisableBehavior(
            ServoChannelConfig::BehaviorWhenDisabled::kSupplyPower);
    first.channel3.PulseRange(805, 1237, 1776)
        .DisableBehavior(
            ServoChannelConfig::BehaviorWhenDisabled::kDoNotSupplyPower);

    first.channel4.PulseRange(555, 1238, 1757)
        .DisableBehavior(
            ServoChannelConfig::BehaviorWhenDisabled::kSupplyPower);
    first.Apply(ServoChannel::ChannelId::kChannelId5, first.channel4);

    ValidateFlattenedStringsExcludingIds(first.channel4.Flatten(),
                                         first.channel5.Flatten());

    second.Apply(first);

    ValidateFlattenedStrings(first.Flatten(), second.Flatten());
}
