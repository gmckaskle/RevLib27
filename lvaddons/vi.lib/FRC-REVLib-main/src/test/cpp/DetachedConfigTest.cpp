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

#include <string>

#include "TestUtils.h"
#include "gtest/gtest.h"
#include "rev/config/BaseConfig.h"
#include "rev/config/DetachedEncoderConfig.h"
#include "rev/config/DetachedEncoderParameters.h"

using namespace rev;
using namespace rev::detached;

////////////////////////////////////////////////

// This class is so we can query the parameters with GetParameter (which is
// protected in BaseConfig)
class SubDetachedEncoderConfig : public DetachedEncoderConfig {
public:
    std::optional<ParameterType_t> GetParam(uint8_t parameterId) {
        return GetParameter(parameterId);
    }
};

TEST(DetachedEncoderConfigTest, Inverts) {
    SubDetachedEncoderConfig config{};

    config.Inverted(true);
    auto isInverted =
        config.GetParam(DetachedEncoderParameter::kEncoderInverted);
    EXPECT_EQ(std::get<bool>(*isInverted), true);

    config.Inverted(false);
    isInverted = config.GetParam(DetachedEncoderParameter::kEncoderInverted);
    EXPECT_EQ(std::get<bool>(*isInverted), false);
}

TEST(DetachedEncoderConfigTest, SetsPositionConversionFactor) {
    SubDetachedEncoderConfig config{};

    config.PositionConversionFactor(42.);
    auto factor =
        config.GetParam(DetachedEncoderParameter::kPositionConversionFactor);
    EXPECT_FLOAT_EQ(std::get<float>(*factor), 42.f);
}

TEST(DetachedEncoderConfigTest, SetsVelocityConversionFactor) {
    SubDetachedEncoderConfig config{};

    config.VelocityConversionFactor(42.);
    auto factor =
        config.GetParam(DetachedEncoderParameter::kVelocityConversionFactor);
    EXPECT_FLOAT_EQ(std::get<float>(*factor), 42.f);
}

TEST(DetachedEncoderConfigTest, SetsAngleConversionFactor) {
    SubDetachedEncoderConfig config{};

    config.AngleConversionFactor(42.);
    auto factor =
        config.GetParam(DetachedEncoderParameter::kAngleConversionFactor);
    EXPECT_FLOAT_EQ(std::get<float>(*factor), 42.f);
}

TEST(DetachedEncoderConfigTest, SetsQuadratureAverageDepth) {
    SubDetachedEncoderConfig config{};

    config.VelocityAverageDepth(42u);
    auto depth =
        config.GetParam(DetachedEncoderParameter::kEncoderAverageDepth);
    EXPECT_EQ(std::get<uint32_t>(*depth), 42u);
}

TEST(DetachedEncoderConfigTest, SetsZeroOffset) {
    SubDetachedEncoderConfig config{};

    config.ZeroOffset(42.);
    auto offset = config.GetParam(DetachedEncoderParameter::kDutyCycleOffset);
    EXPECT_FLOAT_EQ(std::get<float>(*offset), 42.f);
}

TEST(DetachedEncoderConfigTest, SetsZeroCentered) {
    SubDetachedEncoderConfig config{};

    config.ZeroCentered(true);
    auto zeroCentered =
        config.GetParam(DetachedEncoderParameter::kDutyCycleZeroCentered);
    EXPECT_EQ(std::get<bool>(*zeroCentered), true);
}

TEST(DetachedEncoderConfigTest, Flattens) {
    SubDetachedEncoderConfig config{};

    config.Inverted(true)
        .PositionConversionFactor(3.42)
        .VelocityConversionFactor(4.42)
        .VelocityAverageDepth(48)
        .ZeroOffset(0.42)
        .ZeroCentered(true);

    std::string flattened{config.Flatten()};

    ValidateFlattenedSubString(flattened, "0,30\n");
    ValidateFlattenedSubString(flattened, "1,1\n");
    ValidateFlattenedSubString(flattened, "2,405AE148\n");
    ValidateFlattenedSubString(flattened, "3,408D70A4\n");
    ValidateFlattenedSubString(flattened, "4,1\n");
    ValidateFlattenedSubString(flattened, "6,3ED70A3D\n");
    EXPECT_EQ(flattened.size(), 0u) << flattened;
}

////////////////////////////////////////////////
