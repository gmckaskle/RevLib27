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

#include "gtest/gtest.h"
#include "rev/ServoHub.h"
#include "rev/config/ServoHubConfig.h"

using namespace rev::servohub;

////////////////////////////////////////////////

class ServoHubTest : public testing::Test {
protected:
    ServoHubTest() : servoHub{0, 4} {}
    ~ServoHubTest() override {}

    ServoHub servoHub;
};

////////////////////////////////////////////////

TEST_F(ServoHubTest, InvalidIdThrows) {
    EXPECT_THROW({ ServoHub hub(0, 123); }, std::runtime_error);
}

TEST_F(ServoHubTest, DuplicateIdThrows) {
    EXPECT_THROW({ ServoHub hub(0, 4); }, std::runtime_error);
}

TEST_F(ServoHubTest, ServoHubSmoke) {
    EXPECT_NO_THROW({
        ServoHubConfig config;
        config.channel0.PulseRange(505, 1505, 2405);

        servoHub.Configure(config, rev::ResetMode::kResetSafeParameters);

        auto chan =
            servoHub.GetServoChannel(ServoChannel::ChannelId::kChannelId2);
        EXPECT_EQ(chan.GetChannelId(), ServoChannel::ChannelId::kChannelId2);
    });
}

TEST_F(ServoHubTest, ServoChannelSmoke) {
    ServoChannel chan0 =
        servoHub.GetServoChannel(ServoChannel::ChannelId::kChannelId0);
    chan0.SetPulseWidth(1025);
}
