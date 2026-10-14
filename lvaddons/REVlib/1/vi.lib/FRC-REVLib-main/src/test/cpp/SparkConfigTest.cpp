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

#include <string>

#include "TestUtils.h"
#include "gtest/gtest.h"
#include "rev/SparkMax.h"
#include "rev/SplineEncoder.h"
#include "rev/config/AbsoluteEncoderConfig.h"
#include "rev/config/AlternateEncoderConfig.h"
#include "rev/config/AnalogSensorConfig.h"
#include "rev/config/BaseConfig.h"
#include "rev/config/ClosedLoopConfig.h"
#include "rev/config/EncoderConfig.h"
#include "rev/config/ExternalEncoderConfig.h"
#include "rev/config/FeedForwardConfig.h"
#include "rev/config/LimitSwitchConfig.h"
#include "rev/config/MAXMotionConfig.h"
#include "rev/config/SignalsConfig.h"
#include "rev/config/SoftLimitConfig.h"
#include "rev/config/SparkBaseConfig.h"
#include "rev/config/SparkFlexConfig.h"
#include "rev/config/SparkMaxConfig.h"
#include "rev/config/SparkParameters.h"

using namespace rev;
using namespace rev::detached;
using namespace rev::spark;

////////////////////////////////////////////////

// This class is so we can set/get the parameters (which are protected in
// BaseConfig)
class SubConfig : public BaseConfig {
public:
    template <typename ParameterType>
    void PutParam(uint8_t parameterId, ParameterType value) {
        PutParameter<ParameterType>(parameterId, value);
    }

    std::optional<ParameterType_t> GetParam(uint8_t parameterId) {
        return GetParameter(parameterId);
    }

    void RemoveParam(uint8_t parameterId) { RemoveParameter(parameterId); }
};

TEST(ConfigSmokeTest, ConstructBaseConfig) {
    SubConfig sub{};
    sub.PutParam(42, 42);
    sub.PutParam(43, 42.1f);
    sub.PutParam(44, 42u);
    sub.PutParam(45, true);
    sub.PutParam(46, false);
    // If this compiles then it is working

    auto intParam = sub.GetParam(42);
    EXPECT_TRUE(std::holds_alternative<int32_t>(*intParam));
    EXPECT_EQ(std::get<int32_t>(*intParam), 42);
    EXPECT_EQ(std::get_if<float>(&intParam.value()), nullptr);

    auto notFound = sub.GetParam(123);
    EXPECT_FALSE(notFound);

    sub.RemoveParam(42);
    intParam = sub.GetParam(42);
    EXPECT_FALSE(intParam);

    std::string flattened{sub.Flatten()};

    ValidateFlattenedSubString(flattened, "43,42286666\n");
    ValidateFlattenedSubString(flattened, "44,2A\n");
    ValidateFlattenedSubString(flattened, "45,1\n");
    ValidateFlattenedSubString(flattened, "46,0\n");
    EXPECT_EQ(flattened.size(), 0u) << flattened;
}

TEST(ConfigSmokeTest, ConstructSparkBaseConfig) {
    SparkBaseConfig sparkBaseConfig{};

    ASSERT_TRUE(1);
}

TEST(ConfigSmokeTest, ConstructSparkFlexConfig) {
    SparkFlexConfig sparkFlexConfig{};

    ASSERT_TRUE(1);
}

TEST(ConfigSmokeTest, ConstructSparkMaxConfig) {
    SparkMaxConfig sparkMaxConfig{};

    ASSERT_TRUE(1);
}

TEST(ConfigSmokeTest, ConstructAbsoluteEncoderConfig) {
    AbsoluteEncoderConfig absoluteEncoderConfig{};

    ASSERT_TRUE(1);
}

TEST(ConfigSmokeTest, ConstructAnalogSensorConfig) {
    AnalogSensorConfig analogSensorConfig{};

    ASSERT_TRUE(1);
}

TEST(ConfigSmokeTest, ConstructEncoderConfig) {
    EncoderConfig encoderConfig{};

    ASSERT_TRUE(1);
}

TEST(ConfigSmokeTest, ConstructAlternateEncoderConfig) {
    AlternateEncoderConfig alternateEncoderConfig{};

    ASSERT_TRUE(1);
}

TEST(ConfigSmokeTest, ConstructLimitSwitchConfig) {
    LimitSwitchConfig limitSwitchConfig{};

    ASSERT_TRUE(1);
}

TEST(ConfigSmokeTest, ConstructSignalsConfig) {
    rev::spark::SignalsConfig signalsConfig{};

    ASSERT_TRUE(1);
}

TEST(ConfigSmokeTest, ConstructSoftLimitConfig) {
    SoftLimitConfig softLimitConfig{};

    ASSERT_TRUE(1);
}

////////////////////////////////////////////////

// This class is so we can query the parameters with GetParameter (which is
// protected in BaseConfig)
class SubSparkBaseConfig : public SparkBaseConfig {
public:
    std::optional<ParameterType_t> GetParam(uint8_t parameterId) {
        return GetParameter(parameterId);
    }
};

TEST(SparkBaseConfigTest, SetsIdleMode) {
    SubSparkBaseConfig config{};

    config.SetIdleMode(config.IdleMode::kCoast);
    auto idle = config.GetParam(SparkParameter::kIdleMode);
    EXPECT_EQ(std::get<uint32_t>(*idle), config.IdleMode::kCoast);

    config.SetIdleMode(config.IdleMode::kBrake);
    idle = config.GetParam(SparkParameter::kIdleMode);
    EXPECT_EQ(std::get<uint32_t>(*idle), config.IdleMode::kBrake);
}

TEST(SparkBaseConfigTest, Inverts) {
    SubSparkBaseConfig config{};

    config.Inverted(true);
    auto inverted = config.GetParam(SparkParameter::kInverted);
    EXPECT_EQ(std::get<bool>(*inverted), true);

    config.Inverted(false);
    inverted = config.GetParam(SparkParameter::kInverted);
    EXPECT_EQ(std::get<bool>(*inverted), false);
}

TEST(SparkBaseConfigTest, SetsSmartCurrentLimit) {
    SubSparkBaseConfig config{};

    config.SmartCurrentLimit(42);
    auto stallLim = config.GetParam(SparkParameter::kSmartCurrentStallLimit);
    auto freeLim = config.GetParam(SparkParameter::kSmartCurrentFreeLimit);
    auto rpmLim = config.GetParam(SparkParameter::kSmartCurrentConfig);
    EXPECT_EQ(std::get<uint32_t>(*stallLim), 42u);
    EXPECT_EQ(std::get<uint32_t>(*freeLim), 0u);
    EXPECT_EQ(std::get<uint32_t>(*rpmLim), 20000u);

    config.SmartCurrentLimit(42, 123);
    stallLim = config.GetParam(SparkParameter::kSmartCurrentStallLimit);
    freeLim = config.GetParam(SparkParameter::kSmartCurrentFreeLimit);
    rpmLim = config.GetParam(SparkParameter::kSmartCurrentConfig);
    EXPECT_EQ(std::get<uint32_t>(*stallLim), 42u);
    EXPECT_EQ(std::get<uint32_t>(*freeLim), 123u);
    EXPECT_EQ(std::get<uint32_t>(*rpmLim), 20000u);

    config.SmartCurrentLimit(42, 123, 15430);
    stallLim = config.GetParam(SparkParameter::kSmartCurrentStallLimit);
    freeLim = config.GetParam(SparkParameter::kSmartCurrentFreeLimit);
    rpmLim = config.GetParam(SparkParameter::kSmartCurrentConfig);
    EXPECT_EQ(std::get<uint32_t>(*stallLim), 42u);
    EXPECT_EQ(std::get<uint32_t>(*freeLim), 123u);
    EXPECT_EQ(std::get<uint32_t>(*rpmLim), 15430u);
}

TEST(SparkBaseConfigTest, SetsSecondaryCurrentLimit) {
    SubSparkBaseConfig config{};

    config.SecondaryCurrentLimit(42.f);
    auto chop = config.GetParam(SparkParameter::kCurrentChop);
    auto cycles = config.GetParam(SparkParameter::kCurrentChopCycles);
    EXPECT_FLOAT_EQ(std::get<float>(*chop), 42.f);
    EXPECT_EQ(std::get<uint32_t>(*cycles), 0u);

    config.SecondaryCurrentLimit(42.f, 143);
    chop = config.GetParam(SparkParameter::kCurrentChop);
    cycles = config.GetParam(SparkParameter::kCurrentChopCycles);
    EXPECT_FLOAT_EQ(std::get<float>(*chop), 42.f);
    EXPECT_EQ(std::get<uint32_t>(*cycles), 143u);
}

TEST(SparkBaseConfigTest, SetsOpenLoopRampRate) {
    SubSparkBaseConfig config{};

    config.OpenLoopRampRate(0.0);
    auto rate = config.GetParam(SparkParameter::kOpenLoopRampRate);
    EXPECT_FLOAT_EQ(std::get<float>(*rate), 0.0f);

    config.OpenLoopRampRate(42.0);
    rate = config.GetParam(SparkParameter::kOpenLoopRampRate);
    EXPECT_FLOAT_EQ(std::get<float>(*rate), static_cast<float>(1.0 / 42.0));
}

TEST(SparkBaseConfigTest, SetsClosedLoopRampRate) {
    SubSparkBaseConfig config{};

    config.ClosedLoopRampRate(0.0);
    auto rate = config.GetParam(SparkParameter::kClosedLoopRampRate);
    EXPECT_FLOAT_EQ(std::get<float>(*rate), 0.0f);

    config.ClosedLoopRampRate(42.0);
    rate = config.GetParam(SparkParameter::kClosedLoopRampRate);
    EXPECT_FLOAT_EQ(std::get<float>(*rate), static_cast<float>(1.0 / 42.0));
}

TEST(SparkBaseConfigTest, SetsVoltageCompensation) {
    SubSparkBaseConfig config{};

    config.VoltageCompensation(42.f);
    auto voltage = config.GetParam(SparkParameter::kCompensatedNominalVoltage);
    auto compMode = config.GetParam(SparkParameter::kVoltageCompensationMode);
    EXPECT_FLOAT_EQ(std::get<float>(*voltage), 42.f);
    EXPECT_EQ(std::get<uint32_t>(*compMode), 2u);
}

TEST(SparkBaseConfigTest, DisablesVoltageCompensation) {
    SubSparkBaseConfig config{};

    config.DisableVoltageCompensation();
    auto compMode = config.GetParam(SparkParameter::kVoltageCompensationMode);
    EXPECT_EQ(std::get<uint32_t>(*compMode), 0u);
}

TEST(SparkBaseConfigTest, FollowsLeaderCANId) {
    SubSparkBaseConfig config{};

    config.Follow(42, true);
    auto leaderId = config.GetParam(SparkParameter::kFollowerModeLeaderId);
    auto isInverted = config.GetParam(SparkParameter::kFollowerModeIsInverted);
    EXPECT_EQ(std::get<uint32_t>(*leaderId), 42u);
    EXPECT_EQ(std::get<bool>(*isInverted), true);

    // Test with default value for of false for <invert>
    config.Follow(45);
    leaderId = config.GetParam(SparkParameter::kFollowerModeLeaderId);
    isInverted = config.GetParam(SparkParameter::kFollowerModeIsInverted);
    EXPECT_EQ(std::get<uint32_t>(*leaderId), 45u);
    EXPECT_EQ(std::get<bool>(*isInverted), false);
}

TEST(SparkBaseConfigTest, FollowsLeader) {
    SubSparkBaseConfig config{};
    SparkMax sparkMax{0, 34, SparkMax::MotorType::kBrushless};

    config.Follow(sparkMax, true);
    auto leaderId = config.GetParam(SparkParameter::kFollowerModeLeaderId);
    auto isInverted = config.GetParam(SparkParameter::kFollowerModeIsInverted);
    EXPECT_EQ(std::get<uint32_t>(*leaderId), 34u);
    EXPECT_EQ(std::get<bool>(*isInverted), true);

    // Test with default value for of false for <invert>
    config.Follow(sparkMax);
    leaderId = config.GetParam(SparkParameter::kFollowerModeLeaderId);
    isInverted = config.GetParam(SparkParameter::kFollowerModeIsInverted);
    EXPECT_EQ(std::get<uint32_t>(*leaderId), 34u);
    EXPECT_EQ(std::get<bool>(*isInverted), false);
}

TEST(SparkBaseConfigTest, DisablesFollowerMode) {
    SubSparkBaseConfig config{};

    config.Follow(42, true);
    auto leaderId = config.GetParam(SparkParameter::kFollowerModeLeaderId);
    auto isInverted = config.GetParam(SparkParameter::kFollowerModeIsInverted);
    EXPECT_EQ(std::get<uint32_t>(*leaderId), 42u);
    EXPECT_EQ(std::get<bool>(*isInverted), true);

    // Test with default value for of false for <invert>
    config.DisableFollowerMode();
    leaderId = config.GetParam(SparkParameter::kFollowerModeLeaderId);
    isInverted = config.GetParam(SparkParameter::kFollowerModeIsInverted);
    EXPECT_EQ(std::get<uint32_t>(*leaderId), 0u);
    EXPECT_EQ(std::get<bool>(*isInverted), false);
}

TEST(SparkBaseConfigTest, ChainsConfigMethods) {
    SubSparkBaseConfig config{};

    config.Inverted(true)
        .ClosedLoopRampRate(42.)
        .SetIdleMode(config.IdleMode::kBrake)
        .VoltageCompensation(123.45);

    auto rate = config.GetParam(SparkParameter::kClosedLoopRampRate);
    EXPECT_FLOAT_EQ(std::get<float>(*rate), static_cast<float>(1.0 / 42.));

    auto idle = config.GetParam(SparkParameter::kIdleMode);
    EXPECT_EQ(std::get<uint32_t>(*idle), config.IdleMode::kBrake);

    auto voltage = config.GetParam(SparkParameter::kCompensatedNominalVoltage);
    auto compMode = config.GetParam(SparkParameter::kVoltageCompensationMode);
    EXPECT_FLOAT_EQ(std::get<float>(*voltage), 123.45f);
    EXPECT_EQ(std::get<uint32_t>(*compMode), 2u);
}

TEST(SparkBaseConfigTest, AppliesPresets) {
    SubSparkBaseConfig config{};

    config.Apply(SparkBaseConfig::Presets::REV_NEO());
    auto smartCurrentLimit =
        config.GetParam(SparkParameter::kSmartCurrentStallLimit);
    EXPECT_EQ(std::get<uint32_t>(*smartCurrentLimit), 60u);

    config.Apply(SparkBaseConfig::Presets::REV_NEO_550());
    smartCurrentLimit =
        config.GetParam(SparkParameter::kSmartCurrentStallLimit);
    EXPECT_EQ(std::get<uint32_t>(*smartCurrentLimit), 15u);

    config.Apply(SparkBaseConfig::Presets::REV_NEO_2());
    smartCurrentLimit =
        config.GetParam(SparkParameter::kSmartCurrentStallLimit);
    EXPECT_EQ(std::get<uint32_t>(*smartCurrentLimit), 60u);

    config.Apply(SparkBaseConfig::Presets::REV_Vortex());
    smartCurrentLimit =
        config.GetParam(SparkParameter::kSmartCurrentStallLimit);
    EXPECT_EQ(std::get<uint32_t>(*smartCurrentLimit), 80u);

    config.Apply(SparkBaseConfig::Presets::CTRE_Minion());
    smartCurrentLimit =
        config.GetParam(SparkParameter::kSmartCurrentStallLimit);
    EXPECT_EQ(std::get<uint32_t>(*smartCurrentLimit), 30u);

    auto commutationAdvance =
        config.GetParam(SparkParameter::kCommutationAdvance);
    EXPECT_FLOAT_EQ(std::get<float>(*commutationAdvance),
                    static_cast<float>(120.0));
}

TEST(SparkBaseConfigTest, Flattens) {
    SubSparkBaseConfig config{};

    config.Inverted(true)
        .ClosedLoopRampRate(42.)
        .SetIdleMode(config.IdleMode::kBrake)
        .VoltageCompensation(123.45);

    config.absoluteEncoder.AverageDepth(16);

    config.analogSensor.VelocityConversionFactor(42.3).Inverted(true);

    config.closedLoop.Pid(3.42, 4.42, 5.42, kSlot2)
        .PositionWrappingMaxInput(7.42);

    config.limitSwitch
        .ReverseLimitSwitchTriggerBehavior(
            LimitSwitchConfig::Behavior::kStopMovingMotor)
        .ReverseLimitSwitchType(LimitSwitchConfig::Type::kNormallyOpen)
        .ReverseLimitSwitchPosition(1.234)
        .ForwardLimitSwitchTriggerBehavior(
            LimitSwitchConfig::Behavior::kKeepMovingMotor)
        .ForwardLimitSwitchType(LimitSwitchConfig::Type::kNormallyClosed)
        .ForwardLimitSwitchPosition(99.234)
        .LimitSwitchPositionSensor(FeedbackSensor::kPrimaryEncoder);

    config.signals.AbsoluteEncoderPositionAlwaysOn(false)
        .AnalogPositionPeriodMs(342)
        .FaultsPeriodMs(125);

    config.softLimit.ForwardSoftLimit(8.42).ReverseSoftLimitEnabled(true);

    std::string flattened{config.Flatten()};

    ValidateFlattenedSubString(flattened, "45,1\n");
    ValidateFlattenedSubString(flattened, "74,2\n");
    ValidateFlattenedSubString(flattened, "114,3CC30C31\n");
    ValidateFlattenedSubString(flattened, "6,1\n");
    ValidateFlattenedSubString(flattened, "75,42F6E666\n");
    ValidateFlattenedSubString(flattened, "143,4\n");
    ValidateFlattenedSubString(flattened, "120,42293333\n");
    ValidateFlattenedSubString(flattened, "123,1\n");
    ValidateFlattenedSubString(flattened, "53,1\n");
    ValidateFlattenedSubString(flattened, "52,0\n");
    ValidateFlattenedSubString(flattened, "50,1\n");
    ValidateFlattenedSubString(flattened, "29,405AE148\n");
    ValidateFlattenedSubString(flattened, "30,408D70A4\n");
    ValidateFlattenedSubString(flattened, "159,7D\n");
    ValidateFlattenedSubString(flattened, "191,0\n");
    ValidateFlattenedSubString(flattened, "161,156\n");
    ValidateFlattenedSubString(flattened, "115,4106B852\n");
    ValidateFlattenedSubString(flattened, "55,1\n");
    ValidateFlattenedSubString(flattened, "151,40ED70A4\n");
    ValidateFlattenedSubString(flattened, "31,40AD70A4\n");
    ValidateFlattenedSubString(flattened, "127,0\n");
    ValidateFlattenedSubString(flattened, "127,0\n");
    ValidateFlattenedSubString(flattened, "201,1\n");
    ValidateFlattenedSubString(flattened, "51,0\n");
    ValidateFlattenedSubString(flattened, "203,3F9DF3B6\n");
    ValidateFlattenedSubString(flattened, "202,42C677CF\n");
    EXPECT_EQ(flattened.size(), 0u) << flattened;
}

////////////////////////////////////////////////

// This class is so we can query the parameters with GetParameter (which is
// protected in BaseConfig)
class SubSparkFlexConfig : public SparkFlexConfig {
public:
    std::optional<ParameterType_t> GetParam(uint8_t parameterId) {
        return GetParameter(parameterId);
    }
};

TEST(SparkFlexConfigTest, AppliesPresets) {
    SubSparkFlexConfig config{};

    config.Apply(SparkFlexConfig::Presets::REV_NEO());
    auto smartCurrentLimit =
        config.GetParam(SparkParameter::kSmartCurrentStallLimit);
    EXPECT_EQ(std::get<uint32_t>(*smartCurrentLimit), 60u);

    config.Apply(SparkFlexConfig::Presets::REV_NEO_550());
    smartCurrentLimit =
        config.GetParam(SparkParameter::kSmartCurrentStallLimit);
    EXPECT_EQ(std::get<uint32_t>(*smartCurrentLimit), 15u);

    config.Apply(SparkFlexConfig::Presets::REV_NEO_2());
    smartCurrentLimit =
        config.GetParam(SparkParameter::kSmartCurrentStallLimit);
    EXPECT_EQ(std::get<uint32_t>(*smartCurrentLimit), 60u);

    config.Apply(SparkFlexConfig::Presets::REV_Vortex());
    smartCurrentLimit =
        config.GetParam(SparkParameter::kSmartCurrentStallLimit);
    EXPECT_EQ(std::get<uint32_t>(*smartCurrentLimit), 80u);

    config.Apply(SparkFlexConfig::Presets::CTRE_Minion());
    smartCurrentLimit =
        config.GetParam(SparkParameter::kSmartCurrentStallLimit);
    EXPECT_EQ(std::get<uint32_t>(*smartCurrentLimit), 30u);

    auto commutationAdvance =
        config.GetParam(SparkParameter::kCommutationAdvance);
    EXPECT_FLOAT_EQ(std::get<float>(*commutationAdvance),
                    static_cast<float>(120.0));
}

TEST(SparkFlexConfigTest, Flattens) {
    SparkFlexConfig config{};

    config.Inverted(true)
        .ClosedLoopRampRate(42.)
        .SetIdleMode(config.IdleMode::kBrake)
        .VoltageCompensation(123.45);

    config.absoluteEncoder.AverageDepth(16);

    config.analogSensor.VelocityConversionFactor(42.3).Inverted(true);

    config.closedLoop.Pid(3.42, 4.42, 5.42, kSlot2)
        .PositionWrappingMaxInput(7.42);

    config.externalEncoder.Inverted(true).PositionConversionFactor(7.42);

    config.limitSwitch
        .ReverseLimitSwitchTriggerBehavior(
            LimitSwitchConfig::Behavior::kStopMovingMotor)
        .ReverseLimitSwitchType(LimitSwitchConfig::Type::kNormallyOpen)
        .ReverseLimitSwitchPosition(1.234)
        .ForwardLimitSwitchTriggerBehavior(
            LimitSwitchConfig::Behavior::kKeepMovingMotor)
        .ForwardLimitSwitchType(LimitSwitchConfig::Type::kNormallyClosed)
        .ForwardLimitSwitchPosition(99.234)
        .LimitSwitchPositionSensor(FeedbackSensor::kPrimaryEncoder);

    config.signals.AbsoluteEncoderPositionAlwaysOn(false)
        .AnalogPositionPeriodMs(342)
        .FaultsPeriodMs(125);

    config.softLimit.ForwardSoftLimit(8.42).ReverseSoftLimitEnabled(true);

    std::string flattened{config.Flatten()};

    ValidateFlattenedSubString(flattened, "45,1\n");
    ValidateFlattenedSubString(flattened, "74,2\n");
    ValidateFlattenedSubString(flattened, "114,3CC30C31\n");
    ValidateFlattenedSubString(flattened, "6,1\n");
    ValidateFlattenedSubString(flattened, "75,42F6E666\n");
    ValidateFlattenedSubString(flattened, "143,4\n");
    ValidateFlattenedSubString(flattened, "120,42293333\n");
    ValidateFlattenedSubString(flattened, "123,1\n");
    ValidateFlattenedSubString(flattened, "53,1\n");
    ValidateFlattenedSubString(flattened, "52,0\n");
    ValidateFlattenedSubString(flattened, "50,1\n");
    ValidateFlattenedSubString(flattened, "29,405AE148\n");
    ValidateFlattenedSubString(flattened, "30,408D70A4\n");
    ValidateFlattenedSubString(flattened, "151,40ED70A4\n");
    ValidateFlattenedSubString(flattened, "159,7D\n");
    ValidateFlattenedSubString(flattened, "191,0\n");
    ValidateFlattenedSubString(flattened, "161,156\n");
    ValidateFlattenedSubString(flattened, "131,1\n");
    ValidateFlattenedSubString(flattened, "132,40ED70A4\n");
    ValidateFlattenedSubString(flattened, "115,4106B852\n");
    ValidateFlattenedSubString(flattened, "55,1\n");
    ValidateFlattenedSubString(flattened, "31,40AD70A4\n");
    ValidateFlattenedSubString(flattened, "127,0\n");
    ValidateFlattenedSubString(flattened, "127,0\n");
    ValidateFlattenedSubString(flattened, "201,1\n");
    ValidateFlattenedSubString(flattened, "51,0\n");
    ValidateFlattenedSubString(flattened, "203,3F9DF3B6\n");
    ValidateFlattenedSubString(flattened, "202,42C677CF\n");
    EXPECT_EQ(flattened.size(), 0u) << flattened;
}

////////////////////////////////////////////////

// This class is so we can query the parameters with GetParameter (which is
// protected in BaseConfig)
class SubSparkMaxConfig : public SparkMaxConfig {
public:
    std::optional<ParameterType_t> GetParam(uint8_t parameterId) {
        return GetParameter(parameterId);
    }
};

TEST(SparkMaxConfigTest, AppliesPresets) {
    SubSparkMaxConfig config{};

    config.Apply(SparkMaxConfig::Presets::REV_NEO());
    auto smartCurrentLimit =
        config.GetParam(SparkParameter::kSmartCurrentStallLimit);
    EXPECT_EQ(std::get<uint32_t>(*smartCurrentLimit), 60u);

    config.Apply(SparkMaxConfig::Presets::REV_NEO_550());
    smartCurrentLimit =
        config.GetParam(SparkParameter::kSmartCurrentStallLimit);
    EXPECT_EQ(std::get<uint32_t>(*smartCurrentLimit), 15u);

    config.Apply(SparkMaxConfig::Presets::REV_NEO_2());
    smartCurrentLimit =
        config.GetParam(SparkParameter::kSmartCurrentStallLimit);
    EXPECT_EQ(std::get<uint32_t>(*smartCurrentLimit), 60u);

    config.Apply(SparkMaxConfig::Presets::REV_Vortex());
    smartCurrentLimit =
        config.GetParam(SparkParameter::kSmartCurrentStallLimit);
    EXPECT_EQ(std::get<uint32_t>(*smartCurrentLimit), 80u);

    config.Apply(SparkMaxConfig::Presets::CTRE_Minion());
    smartCurrentLimit =
        config.GetParam(SparkParameter::kSmartCurrentStallLimit);
    EXPECT_EQ(std::get<uint32_t>(*smartCurrentLimit), 30u);

    auto commutationAdvance =
        config.GetParam(SparkParameter::kCommutationAdvance);
    EXPECT_FLOAT_EQ(std::get<float>(*commutationAdvance),
                    static_cast<float>(120.0));
}

TEST(SparkMaxConfigTest, Flattens) {
    SparkMaxConfig config{};

    config.Inverted(true)
        .ClosedLoopRampRate(42.)
        .SetIdleMode(config.IdleMode::kBrake)
        .VoltageCompensation(123.45);

    config.absoluteEncoder.AverageDepth(16);

    config.alternateEncoder.VelocityConversionFactor(9.42).AverageDepth(42);

    config.analogSensor.VelocityConversionFactor(42.3).Inverted(true);

    config.closedLoop.Pid(3.42, 4.42, 5.42, kSlot2)
        .PositionWrappingMaxInput(7.42);

    config.limitSwitch
        .ReverseLimitSwitchTriggerBehavior(
            LimitSwitchConfig::Behavior::kStopMovingMotor)
        .ReverseLimitSwitchType(LimitSwitchConfig::Type::kNormallyOpen)
        .ReverseLimitSwitchPosition(1.234)
        .ForwardLimitSwitchTriggerBehavior(
            LimitSwitchConfig::Behavior::kKeepMovingMotor)
        .ForwardLimitSwitchType(LimitSwitchConfig::Type::kNormallyClosed)
        .ForwardLimitSwitchPosition(99.234)
        .LimitSwitchPositionSensor(FeedbackSensor::kPrimaryEncoder);

    config.signals.AbsoluteEncoderPositionAlwaysOn(false)
        .AnalogPositionPeriodMs(342)
        .FaultsPeriodMs(125);

    config.softLimit.ForwardSoftLimit(8.42).ReverseSoftLimitEnabled(true);

    std::string flattened{config.Flatten()};

    ValidateFlattenedSubString(flattened, "45,1\n");
    ValidateFlattenedSubString(flattened, "74,2\n");
    ValidateFlattenedSubString(flattened, "114,3CC30C31\n");
    ValidateFlattenedSubString(flattened, "6,1\n");
    ValidateFlattenedSubString(flattened, "75,42F6E666\n");
    ValidateFlattenedSubString(flattened, "143,4\n");
    ValidateFlattenedSubString(flattened, "120,42293333\n");
    ValidateFlattenedSubString(flattened, "123,1\n");
    ValidateFlattenedSubString(flattened, "53,1\n");
    ValidateFlattenedSubString(flattened, "52,0\n");
    ValidateFlattenedSubString(flattened, "50,1\n");
    ValidateFlattenedSubString(flattened, "29,405AE148\n");
    ValidateFlattenedSubString(flattened, "30,408D70A4\n");
    ValidateFlattenedSubString(flattened, "151,40ED70A4\n");
    ValidateFlattenedSubString(flattened, "159,7D\n");
    ValidateFlattenedSubString(flattened, "191,0\n");
    ValidateFlattenedSubString(flattened, "161,156\n");
    ValidateFlattenedSubString(flattened, "133,4116B852\n");
    ValidateFlattenedSubString(flattened, "129,2A\n");
    ValidateFlattenedSubString(flattened, "115,4106B852\n");
    ValidateFlattenedSubString(flattened, "55,1\n");
    ValidateFlattenedSubString(flattened, "31,40AD70A4\n");
    ValidateFlattenedSubString(flattened, "127,FFFFFFFF\n");
    ValidateFlattenedSubString(flattened, "201,1\n");
    ValidateFlattenedSubString(flattened, "51,0\n");
    ValidateFlattenedSubString(flattened, "203,3F9DF3B6\n");
    ValidateFlattenedSubString(flattened, "202,42C677CF\n");
    EXPECT_EQ(flattened.size(), 0u) << flattened;
}

////////////////////////////////////////////////

// This class is so we can query the parameters with GetParameter (which is
// protected in BaseConfig)
class SubAbsoluteEncoderConfig : public AbsoluteEncoderConfig {
public:
    std::optional<ParameterType_t> GetParam(uint8_t parameterId) {
        return GetParameter(parameterId);
    }
};

TEST(AbsoluteEncoderConfigTest, Inverts) {
    SubAbsoluteEncoderConfig config{};

    config.Inverted(true);
    auto isInverted = config.GetParam(SparkParameter::kDutyCycleInverted);
    EXPECT_EQ(std::get<bool>(*isInverted), true);

    auto dataPortConfig =
        config.GetParam(SparkParameter::kCompatibilityPortConfig);
    EXPECT_EQ(std::get<int32_t>(*dataPortConfig),
              SparkMaxConfig::DataPortConfig::kLimitSwitchesAndAbsoluteEncoder);

    config.Inverted(false);
    isInverted = config.GetParam(SparkParameter::kDutyCycleInverted);
    EXPECT_EQ(std::get<bool>(*isInverted), false);
}

TEST(AbsoluteEncoderConfigTest, SetsPositionConversionFactor) {
    SubAbsoluteEncoderConfig config{};

    config.PositionConversionFactor(42.);
    auto factor = config.GetParam(SparkParameter::kDutyCyclePositionFactor);
    EXPECT_FLOAT_EQ(std::get<float>(*factor), 42.f);

    auto dataPortConfig =
        config.GetParam(SparkParameter::kCompatibilityPortConfig);
    EXPECT_EQ(std::get<int32_t>(*dataPortConfig),
              SparkMaxConfig::DataPortConfig::kLimitSwitchesAndAbsoluteEncoder);
}

TEST(AbsoluteEncoderConfigTest, SetsVelocityConversionFactor) {
    SubAbsoluteEncoderConfig config{};

    config.VelocityConversionFactor(42.);
    auto factor = config.GetParam(SparkParameter::kDutyCycleVelocityFactor);
    EXPECT_FLOAT_EQ(std::get<float>(*factor), 42.f);

    auto dataPortConfig =
        config.GetParam(SparkParameter::kCompatibilityPortConfig);
    EXPECT_EQ(std::get<int32_t>(*dataPortConfig),
              SparkMaxConfig::DataPortConfig::kLimitSwitchesAndAbsoluteEncoder);
}

TEST(AbsoluteEncoderConfigTest, SetsZeroOffset) {
    SubAbsoluteEncoderConfig config{};

    config.ZeroOffset(42.);
    auto offset = config.GetParam(SparkParameter::kDutyCycleOffset);
    EXPECT_FLOAT_EQ(std::get<float>(*offset), 42.f);

    auto dataPortConfig =
        config.GetParam(SparkParameter::kCompatibilityPortConfig);
    EXPECT_EQ(std::get<int32_t>(*dataPortConfig),
              SparkMaxConfig::DataPortConfig::kLimitSwitchesAndAbsoluteEncoder);
}

TEST(AbsoluteEncoderConfigTest, SetsAverageDepth) {
    SubAbsoluteEncoderConfig config{};

    config.AverageDepth(1);
    auto depthIdx = config.GetParam(SparkParameter::kDutyCycleAverageDepth);
    EXPECT_EQ(std::get<uint32_t>(*depthIdx), 0u);

    auto dataPortConfig =
        config.GetParam(SparkParameter::kCompatibilityPortConfig);
    EXPECT_EQ(std::get<int32_t>(*dataPortConfig),
              SparkMaxConfig::DataPortConfig::kLimitSwitchesAndAbsoluteEncoder);

    config.AverageDepth(2);
    depthIdx = config.GetParam(SparkParameter::kDutyCycleAverageDepth);
    EXPECT_EQ(std::get<uint32_t>(*depthIdx), 1u);

    config.AverageDepth(4);
    depthIdx = config.GetParam(SparkParameter::kDutyCycleAverageDepth);
    EXPECT_EQ(std::get<uint32_t>(*depthIdx), 2u);

    config.AverageDepth(8);
    depthIdx = config.GetParam(SparkParameter::kDutyCycleAverageDepth);
    EXPECT_EQ(std::get<uint32_t>(*depthIdx), 3u);

    config.AverageDepth(16);
    depthIdx = config.GetParam(SparkParameter::kDutyCycleAverageDepth);
    EXPECT_EQ(std::get<uint32_t>(*depthIdx), 4u);

    config.AverageDepth(32);
    depthIdx = config.GetParam(SparkParameter::kDutyCycleAverageDepth);
    EXPECT_EQ(std::get<uint32_t>(*depthIdx), 5u);

    config.AverageDepth(64);
    depthIdx = config.GetParam(SparkParameter::kDutyCycleAverageDepth);
    EXPECT_EQ(std::get<uint32_t>(*depthIdx), 6u);

    config.AverageDepth(128);
    depthIdx = config.GetParam(SparkParameter::kDutyCycleAverageDepth);
    EXPECT_EQ(std::get<uint32_t>(*depthIdx), 7u);

    config.AverageDepth(42);
    depthIdx = config.GetParam(SparkParameter::kDutyCycleAverageDepth);
    EXPECT_EQ(std::get<uint32_t>(*depthIdx), 7u);
}

TEST(AbsoluteEncoderConfigTest, SetsStartPulseUs) {
    SubAbsoluteEncoderConfig config{};

    config.StartPulseUs(42.);
    auto startPulse =
        config.GetParam(SparkParameter::kDutyCycleEncoderStartPulseUs);
    EXPECT_FLOAT_EQ(std::get<float>(*startPulse), 42.f);

    auto dataPortConfig =
        config.GetParam(SparkParameter::kCompatibilityPortConfig);
    EXPECT_EQ(std::get<int32_t>(*dataPortConfig),
              SparkMaxConfig::DataPortConfig::kLimitSwitchesAndAbsoluteEncoder);
}

TEST(AbsoluteEncoderConfigTest, SetsEndPulseUs) {
    SubAbsoluteEncoderConfig config{};

    config.EndPulseUs(42.);
    auto endPulse =
        config.GetParam(SparkParameter::kDutyCycleEncoderEndPulseUs);
    EXPECT_FLOAT_EQ(std::get<float>(*endPulse), 42.f);

    auto dataPortConfig =
        config.GetParam(SparkParameter::kCompatibilityPortConfig);
    EXPECT_EQ(std::get<int32_t>(*dataPortConfig),
              SparkMaxConfig::DataPortConfig::kLimitSwitchesAndAbsoluteEncoder);
}

TEST(AbsoluteEncoderConfigTest, SetsRangeOffset) {
    SubAbsoluteEncoderConfig config{};

    config.RangeOffset(-0.25);
    auto rangeOffset = config.GetParam(SparkParameter::kDutyCycleRangeOffset);
    EXPECT_EQ(std::get<float>(*rangeOffset), -0.25f);

    auto dataPortConfig =
        config.GetParam(SparkParameter::kCompatibilityPortConfig);
    EXPECT_EQ(std::get<int32_t>(*dataPortConfig),
              SparkMaxConfig::DataPortConfig::kLimitSwitchesAndAbsoluteEncoder);
}

////////////////////////////////////////////////

// This class is so we can query the parameters with GetParameter (which is
// protected in BaseConfig)
class SubAnalogSensorConfig : public AnalogSensorConfig {
public:
    std::optional<ParameterType_t> GetParam(uint8_t parameterId) {
        return GetParameter(parameterId);
    }
};

TEST(AnalogSensorConfigTest, Inverts) {
    SubAnalogSensorConfig config{};

    config.Inverted(true);
    auto isInverted = config.GetParam(SparkParameter::kAnalogInverted);
    EXPECT_EQ(std::get<bool>(*isInverted), true);

    config.Inverted(false);
    isInverted = config.GetParam(SparkParameter::kAnalogInverted);
    EXPECT_EQ(std::get<bool>(*isInverted), false);
}

TEST(AnalogSensorConfigTest, SetsPositionConversionFactor) {
    SubAnalogSensorConfig config{};

    config.PositionConversionFactor(42.);
    auto factor = config.GetParam(SparkParameter::kAnalogPositionConversion);
    EXPECT_FLOAT_EQ(std::get<float>(*factor), 42.f);
}

TEST(AnalogSensorConfigTest, SetsVelocityConversionFactor) {
    SubAnalogSensorConfig config{};

    config.VelocityConversionFactor(42.);
    auto factor = config.GetParam(SparkParameter::kAnalogVelocityConversion);
    EXPECT_FLOAT_EQ(std::get<float>(*factor), 42.f);
}

////////////////////////////////////////////////

// This class is so we can query the parameters with GetParameter (which is
// protected in BaseConfig)
class SubEncoderConfig : public EncoderConfig {
public:
    std::optional<ParameterType_t> GetParam(uint8_t parameterId) {
        return GetParameter(parameterId);
    }
};

TEST(EncoderConfigTest, SetsCountsPerRevolution) {
    SubEncoderConfig config{};

    config.CountsPerRevolution(42);
    auto cpr = config.GetParam(SparkParameter::kEncoderCountsPerRev);
    EXPECT_EQ(std::get<uint32_t>(*cpr), 42u);
}

TEST(EncoderConfigTest, Inverts) {
    SubEncoderConfig config{};

    config.Inverted(true);
    auto isInverted = config.GetParam(SparkParameter::kEncoderInverted);
    EXPECT_EQ(std::get<bool>(*isInverted), true);

    config.Inverted(false);
    isInverted = config.GetParam(SparkParameter::kEncoderInverted);
    EXPECT_EQ(std::get<bool>(*isInverted), false);
}

TEST(EncoderConfigTest, SetsPositionConversionFactor) {
    SubEncoderConfig config{};

    config.PositionConversionFactor(42.);
    auto factor = config.GetParam(SparkParameter::kPositionConversionFactor);
    EXPECT_FLOAT_EQ(std::get<float>(*factor), 42.f);
}

TEST(EncoderConfigTest, SetsVelocityConversionFactor) {
    SubEncoderConfig config{};

    config.VelocityConversionFactor(42.);
    auto factor = config.GetParam(SparkParameter::kVelocityConversionFactor);
    EXPECT_FLOAT_EQ(std::get<float>(*factor), 42.f);
}

TEST(EncoderConfigTest, SetsQuadratureAverageDepth) {
    SubEncoderConfig config{};

    config.QuadratureAverageDepth(42);
    auto depth = config.GetParam(SparkParameter::kEncoderAverageDepth);
    EXPECT_EQ(std::get<uint32_t>(*depth), 42u);
}

TEST(EncoderConfigTest, SetsQuadratureMeasurementPeriod) {
    SubEncoderConfig config{};

    config.QuadratureMeasurementPeriod(42);
    auto delta = config.GetParam(SparkParameter::kEncoderSampleDelta);
    EXPECT_EQ(std::get<uint32_t>(*delta), (42u << 1));
}

////////////////////////////////////////////////

// This class is so we can query the parameters with GetParameter (which is
// protected in BaseConfig)
class SubAlternateEncoderConfig : public AlternateEncoderConfig {
public:
    std::optional<ParameterType_t> GetParam(uint8_t parameterId) {
        return GetParameter(parameterId);
    }
};

TEST(AlternateEncoderConfigTest, SetsCountsPerRevolution) {
    SubAlternateEncoderConfig config{};

    config.CountsPerRevolution(42);
    auto cpr = config.GetParam(SparkParameter::kAltEncoderCountsPerRev);
    EXPECT_EQ(std::get<uint32_t>(*cpr), 42u);

    auto dataPortConfig =
        config.GetParam(SparkParameter::kCompatibilityPortConfig);
    EXPECT_EQ(std::get<int32_t>(*dataPortConfig),
              SparkMaxConfig::DataPortConfig::kAlternateEncoder);
}

TEST(AlternateEncoderConfigTest, Inverts) {
    SubAlternateEncoderConfig config{};

    config.Inverted(true);
    auto isInverted = config.GetParam(SparkParameter::kAltEncoderInverted);
    EXPECT_EQ(std::get<bool>(*isInverted), true);

    auto dataPortConfig =
        config.GetParam(SparkParameter::kCompatibilityPortConfig);
    EXPECT_EQ(std::get<int32_t>(*dataPortConfig),
              SparkMaxConfig::DataPortConfig::kAlternateEncoder);

    config.Inverted(false);
    isInverted = config.GetParam(SparkParameter::kAltEncoderInverted);
    EXPECT_EQ(std::get<bool>(*isInverted), false);
}

TEST(AlternateEncoderConfigTest, SetsPositionConversionFactor) {
    SubAlternateEncoderConfig config{};

    config.PositionConversionFactor(42.);
    auto factor =
        config.GetParam(SparkParameter::kAltEncoderPositionConversion);
    EXPECT_FLOAT_EQ(std::get<float>(*factor), 42.f);

    auto dataPortConfig =
        config.GetParam(SparkParameter::kCompatibilityPortConfig);
    EXPECT_EQ(std::get<int32_t>(*dataPortConfig),
              SparkMaxConfig::DataPortConfig::kAlternateEncoder);
}

TEST(AlternateEncoderConfigTest, SetsVelocityConversionFactor) {
    SubAlternateEncoderConfig config{};

    config.VelocityConversionFactor(42.);
    auto factor =
        config.GetParam(SparkParameter::kAltEncoderVelocityConversion);
    EXPECT_FLOAT_EQ(std::get<float>(*factor), 42.f);

    auto dataPortConfig =
        config.GetParam(SparkParameter::kCompatibilityPortConfig);
    EXPECT_EQ(std::get<int32_t>(*dataPortConfig),
              SparkMaxConfig::DataPortConfig::kAlternateEncoder);
}

TEST(AlternateEncoderConfigTest, SetsAverageDepth) {
    SubAlternateEncoderConfig config{};

    config.AverageDepth(42);
    auto depth = config.GetParam(SparkParameter::kAltEncoderAverageDepth);
    EXPECT_EQ(std::get<uint32_t>(*depth), 42u);

    auto dataPortConfig =
        config.GetParam(SparkParameter::kCompatibilityPortConfig);
    EXPECT_EQ(std::get<int32_t>(*dataPortConfig),
              SparkMaxConfig::DataPortConfig::kAlternateEncoder);
}

TEST(AlternateEncoderConfigTest, SetsMeasurementPeriod) {
    SubAlternateEncoderConfig config{};

    config.MeasurementPeriod(42);
    auto delta = config.GetParam(SparkParameter::kAltEncoderSampleDelta);
    EXPECT_EQ(std::get<uint32_t>(*delta), 42u);

    auto dataPortConfig =
        config.GetParam(SparkParameter::kCompatibilityPortConfig);
    EXPECT_EQ(std::get<int32_t>(*dataPortConfig),
              SparkMaxConfig::DataPortConfig::kAlternateEncoder);
}

////////////////////////////////////////////////

// This class is so we can query the parameters with GetParameter (which is
// protected in BaseConfig)
class SubExternalEncoderConfig : public ExternalEncoderConfig {
public:
    std::optional<ParameterType_t> GetParam(uint8_t parameterId) {
        return GetParameter(parameterId);
    }
};

TEST(ExternalEncoderConfigTest, SetsCountsPerRevolution) {
    SubExternalEncoderConfig config{};

    config.CountsPerRevolution(42);
    auto cpr = config.GetParam(SparkParameter::kAltEncoderCountsPerRev);
    EXPECT_EQ(std::get<uint32_t>(*cpr), 42u);
}

TEST(ExternalEncoderConfigTest, Inverts) {
    SubExternalEncoderConfig config{};

    config.Inverted(true);
    auto isInverted = config.GetParam(SparkParameter::kAltEncoderInverted);
    EXPECT_EQ(std::get<bool>(*isInverted), true);

    config.Inverted(false);
    isInverted = config.GetParam(SparkParameter::kAltEncoderInverted);
    EXPECT_EQ(std::get<bool>(*isInverted), false);
}

TEST(ExternalEncoderConfigTest, SetsPositionConversionFactor) {
    SubExternalEncoderConfig config{};

    config.PositionConversionFactor(42.);
    auto factor =
        config.GetParam(SparkParameter::kAltEncoderPositionConversion);
    EXPECT_FLOAT_EQ(std::get<float>(*factor), 42.f);
}

TEST(ExternalEncoderConfigTest, SetsVelocityConversionFactor) {
    SubExternalEncoderConfig config{};

    config.VelocityConversionFactor(42.);
    auto factor =
        config.GetParam(SparkParameter::kAltEncoderVelocityConversion);
    EXPECT_FLOAT_EQ(std::get<float>(*factor), 42.f);
}

TEST(ExternalEncoderConfigTest, SetsAverageDepth) {
    SubExternalEncoderConfig config{};

    config.AverageDepth(42);
    auto depth = config.GetParam(SparkParameter::kAltEncoderAverageDepth);
    EXPECT_EQ(std::get<uint32_t>(*depth), 42u);
}

TEST(ExternalEncoderConfigTest, SetsMeasurementPeriod) {
    SubExternalEncoderConfig config{};

    config.MeasurementPeriod(42);
    auto delta = config.GetParam(SparkParameter::kAltEncoderSampleDelta);
    EXPECT_EQ(std::get<uint32_t>(*delta), 42u);
}

////////////////////////////////////////////////

// This class is so we can query the parameters with GetParameter (which is
// protected in BaseConfig)
class SubLimitSwitchConfig : public LimitSwitchConfig {
public:
    std::optional<ParameterType_t> GetParam(uint8_t parameterId) {
        return GetParameter(parameterId);
    }
};

TEST(LimitSwitchConfigTest, SetsForwardLimitSwitchTriggerBehavior) {
    SubLimitSwitchConfig config{};

    config.ForwardLimitSwitchTriggerBehavior(config.Behavior::kStopMovingMotor);
    auto behavior = config.GetParam(SparkParameter::kHardLimitFwdEn);
    EXPECT_EQ(std::get<uint32_t>(*behavior), config.Behavior::kStopMovingMotor);

    auto dataPortConfig =
        config.GetParam(SparkParameter::kCompatibilityPortConfig);
    EXPECT_EQ(std::get<int32_t>(*dataPortConfig),
              SparkMaxConfig::DataPortConfig::kLimitSwitchesAndAbsoluteEncoder);

    config.ForwardLimitSwitchTriggerBehavior(config.Behavior::kKeepMovingMotor);
    behavior = config.GetParam(SparkParameter::kHardLimitFwdEn);
    EXPECT_EQ(std::get<uint32_t>(*behavior), config.Behavior::kKeepMovingMotor);

    config.ForwardLimitSwitchTriggerBehavior(
        config.Behavior::kKeepMovingMotorAndSetPosition);
    behavior = config.GetParam(SparkParameter::kHardLimitFwdEn);
    EXPECT_EQ(std::get<uint32_t>(*behavior),
              config.Behavior::kKeepMovingMotorAndSetPosition);

    config.ForwardLimitSwitchTriggerBehavior(
        config.Behavior::kStopMovingMotorAndSetPosition);
    behavior = config.GetParam(SparkParameter::kHardLimitFwdEn);
    EXPECT_EQ(std::get<uint32_t>(*behavior),
              config.Behavior::kStopMovingMotorAndSetPosition);
}

TEST(LimitSwitchConfigTest, SetsForwardLimitSwitchType) {
    SubLimitSwitchConfig config{};

    config.ForwardLimitSwitchType(config.Type::kNormallyClosed);
    auto polarity = config.GetParam(SparkParameter::kLimitSwitchFwdPolarity);
    EXPECT_EQ(std::get<bool>(*polarity), true);

    auto dataPortConfig =
        config.GetParam(SparkParameter::kCompatibilityPortConfig);
    EXPECT_EQ(std::get<int32_t>(*dataPortConfig),
              SparkMaxConfig::DataPortConfig::kLimitSwitchesAndAbsoluteEncoder);

    config.ForwardLimitSwitchType(config.Type::kNormallyOpen);
    polarity = config.GetParam(SparkParameter::kLimitSwitchFwdPolarity);
    EXPECT_EQ(std::get<bool>(*polarity), false);
}

TEST(LimitSwitchConfigTest, SetsForwardLimitSwitchPosition) {
    SubLimitSwitchConfig config{};

    config.ForwardLimitSwitchPosition(99.234);
    auto position = config.GetParam(SparkParameter::kLimitSwitchFwdPosition);
    EXPECT_FLOAT_EQ(std::get<float>(*position), 99.234f);

    auto dataPortConfig =
        config.GetParam(SparkParameter::kCompatibilityPortConfig);
    EXPECT_EQ(std::get<int32_t>(*dataPortConfig),
              SparkMaxConfig::DataPortConfig::kLimitSwitchesAndAbsoluteEncoder);
}

TEST(LimitSwitchConfigTest, SetsReverseLimitSwitchTriggerBehavior) {
    SubLimitSwitchConfig config{};

    config.ReverseLimitSwitchTriggerBehavior(
        LimitSwitchConfig::Behavior::kStopMovingMotor);
    auto behavior = config.GetParam(SparkParameter::kHardLimitRevEn);
    EXPECT_EQ(std::get<uint32_t>(*behavior),
              LimitSwitchConfig::Behavior::kStopMovingMotor);

    auto dataPortConfig =
        config.GetParam(SparkParameter::kCompatibilityPortConfig);
    EXPECT_EQ(std::get<int32_t>(*dataPortConfig),
              SparkMaxConfig::DataPortConfig::kLimitSwitchesAndAbsoluteEncoder);

    config.ReverseLimitSwitchTriggerBehavior(
        LimitSwitchConfig::Behavior::kKeepMovingMotor);
    behavior = config.GetParam(SparkParameter::kHardLimitRevEn);
    EXPECT_EQ(std::get<uint32_t>(*behavior),
              LimitSwitchConfig::Behavior::kKeepMovingMotor);

    config.ReverseLimitSwitchTriggerBehavior(
        LimitSwitchConfig::Behavior::kStopMovingMotorAndSetPosition);
    behavior = config.GetParam(SparkParameter::kHardLimitRevEn);
    EXPECT_EQ(std::get<uint32_t>(*behavior),
              LimitSwitchConfig::Behavior::kStopMovingMotorAndSetPosition);

    config.ReverseLimitSwitchTriggerBehavior(
        LimitSwitchConfig::Behavior::kKeepMovingMotorAndSetPosition);
    behavior = config.GetParam(SparkParameter::kHardLimitRevEn);
    EXPECT_EQ(std::get<uint32_t>(*behavior),
              LimitSwitchConfig::Behavior::kKeepMovingMotorAndSetPosition);
}

TEST(LimitSwitchConfigTest, SetsReverseLimitSwitchType) {
    SubLimitSwitchConfig config{};

    config.ReverseLimitSwitchType(config.Type::kNormallyClosed);
    auto polarity = config.GetParam(SparkParameter::kLimitSwitchRevPolarity);
    EXPECT_EQ(std::get<bool>(*polarity), true);

    auto dataPortConfig =
        config.GetParam(SparkParameter::kCompatibilityPortConfig);
    EXPECT_EQ(std::get<int32_t>(*dataPortConfig),
              SparkMaxConfig::DataPortConfig::kLimitSwitchesAndAbsoluteEncoder);

    config.ReverseLimitSwitchType(config.Type::kNormallyOpen);
    polarity = config.GetParam(SparkParameter::kLimitSwitchRevPolarity);
    EXPECT_EQ(std::get<bool>(*polarity), false);
}

TEST(LimitSwitchConfigTest, SetsReverseLimitSwitchPosition) {
    SubLimitSwitchConfig config{};

    config.ReverseLimitSwitchPosition(1.234);
    auto position = config.GetParam(SparkParameter::kLimitSwitchRevPosition);
    EXPECT_FLOAT_EQ(std::get<float>(*position), 1.234f);

    auto dataPortConfig =
        config.GetParam(SparkParameter::kCompatibilityPortConfig);
    EXPECT_EQ(std::get<int32_t>(*dataPortConfig),
              SparkMaxConfig::DataPortConfig::kLimitSwitchesAndAbsoluteEncoder);
}

TEST(LimitSwitchConfigTest, SetsLimitSwitchPositionSensor) {
    SubLimitSwitchConfig config{};

    config.LimitSwitchPositionSensor(FeedbackSensor::kAbsoluteEncoder);
    auto sensor = config.GetParam(SparkParameter::kLimitSwitchPositionSensor);
    EXPECT_EQ(static_cast<FeedbackSensor>(std::get<uint32_t>(*sensor)),
              FeedbackSensor::kAbsoluteEncoder);

    auto dataPortConfig =
        config.GetParam(SparkParameter::kCompatibilityPortConfig);
    EXPECT_EQ(std::get<int32_t>(*dataPortConfig),
              SparkMaxConfig::DataPortConfig::kLimitSwitchesAndAbsoluteEncoder);

    config.LimitSwitchPositionSensor(
        FeedbackSensor::kAlternateOrExternalEncoder);
    sensor = config.GetParam(SparkParameter::kLimitSwitchPositionSensor);
    EXPECT_EQ(static_cast<FeedbackSensor>(std::get<uint32_t>(*sensor)),
              FeedbackSensor::kAlternateOrExternalEncoder);

    config.LimitSwitchPositionSensor(FeedbackSensor::kAnalogSensor);
    sensor = config.GetParam(SparkParameter::kLimitSwitchPositionSensor);
    EXPECT_EQ(static_cast<FeedbackSensor>(std::get<uint32_t>(*sensor)),
              FeedbackSensor::kAnalogSensor);

    config.LimitSwitchPositionSensor(FeedbackSensor::kNoSensor);
    sensor = config.GetParam(SparkParameter::kLimitSwitchPositionSensor);
    EXPECT_EQ(static_cast<FeedbackSensor>(std::get<uint32_t>(*sensor)),
              FeedbackSensor::kNoSensor);

    config.LimitSwitchPositionSensor(FeedbackSensor::kPrimaryEncoder);
    sensor = config.GetParam(SparkParameter::kLimitSwitchPositionSensor);
    EXPECT_EQ(static_cast<FeedbackSensor>(std::get<uint32_t>(*sensor)),
              FeedbackSensor::kPrimaryEncoder);
}

////////////////////////////////////////////////

// This class is so we can query the parameters with GetParameter (which is
// protected in BaseConfig)
class SubMAXMotionConfig : public MAXMotionConfig {
public:
    std::optional<ParameterType_t> GetParam(uint8_t parameterId) {
        return GetParameter(parameterId);
    }
};

TEST(MAXMotionConfigTest, SetsCruiseVelocity) {
    SubMAXMotionConfig config{};

    // Set using default slot = kSlot0
    config.CruiseVelocity(42.);
    auto CruiseVelocity =
        config.GetParam(SparkParameter::kMAXMotionCruiseVelocity_0);
    EXPECT_FLOAT_EQ(std::get<float>(*CruiseVelocity), 42.f);

    config.CruiseVelocity(43., kSlot1);
    CruiseVelocity =
        config.GetParam(SparkParameter::kMAXMotionCruiseVelocity_1);
    EXPECT_FLOAT_EQ(std::get<float>(*CruiseVelocity), 43.f);

    config.CruiseVelocity(44., kSlot2);
    CruiseVelocity =
        config.GetParam(SparkParameter::kMAXMotionCruiseVelocity_2);
    EXPECT_FLOAT_EQ(std::get<float>(*CruiseVelocity), 44.f);

    config.CruiseVelocity(45., kSlot3);
    CruiseVelocity =
        config.GetParam(SparkParameter::kMAXMotionCruiseVelocity_3);
    EXPECT_FLOAT_EQ(std::get<float>(*CruiseVelocity), 45.f);

    // Explicitly set kSlot0
    config.CruiseVelocity(46., kSlot0);
    CruiseVelocity =
        config.GetParam(SparkParameter::kMAXMotionCruiseVelocity_0);
    EXPECT_FLOAT_EQ(std::get<float>(*CruiseVelocity), 46.f);
}

TEST(MAXMotionConfigTest, SetsMaxAcceleration) {
    SubMAXMotionConfig config{};

    // Set using default slot = kSlot0
    config.MaxAcceleration(42.);
    auto maxAcceleration =
        config.GetParam(SparkParameter::kMAXMotionMaxAccel_0);
    EXPECT_FLOAT_EQ(std::get<float>(*maxAcceleration), 42.f);

    config.MaxAcceleration(43., kSlot1);
    maxAcceleration = config.GetParam(SparkParameter::kMAXMotionMaxAccel_1);
    EXPECT_FLOAT_EQ(std::get<float>(*maxAcceleration), 43.f);

    config.MaxAcceleration(44., kSlot2);
    maxAcceleration = config.GetParam(SparkParameter::kMAXMotionMaxAccel_2);
    EXPECT_FLOAT_EQ(std::get<float>(*maxAcceleration), 44.f);

    config.MaxAcceleration(45., kSlot3);
    maxAcceleration = config.GetParam(SparkParameter::kMAXMotionMaxAccel_3);
    EXPECT_FLOAT_EQ(std::get<float>(*maxAcceleration), 45.f);

    // Explicitly set kSlot0
    config.MaxAcceleration(46., kSlot0);
    maxAcceleration = config.GetParam(SparkParameter::kMAXMotionMaxAccel_0);
    EXPECT_FLOAT_EQ(std::get<float>(*maxAcceleration), 46.f);
}

TEST(MAXMotionConfigTest, SetsAllowedProfileError) {
    SubMAXMotionConfig config{};

    // Set using default slot = kSlot0
    config.AllowedProfileError(42.);
    auto allowedError =
        config.GetParam(SparkParameter::kMAXMotionAllowedProfileError_0);
    EXPECT_FLOAT_EQ(std::get<float>(*allowedError), 42.f);

    config.AllowedProfileError(43., kSlot1);
    allowedError =
        config.GetParam(SparkParameter::kMAXMotionAllowedProfileError_1);
    EXPECT_FLOAT_EQ(std::get<float>(*allowedError), 43.f);

    config.AllowedProfileError(44., kSlot2);
    allowedError =
        config.GetParam(SparkParameter::kMAXMotionAllowedProfileError_2);
    EXPECT_FLOAT_EQ(std::get<float>(*allowedError), 44.f);

    config.AllowedProfileError(45., kSlot3);
    allowedError =
        config.GetParam(SparkParameter::kMAXMotionAllowedProfileError_3);
    EXPECT_FLOAT_EQ(std::get<float>(*allowedError), 45.f);

    // Explicitly set kSlot0
    config.AllowedProfileError(46., kSlot0);
    allowedError =
        config.GetParam(SparkParameter::kMAXMotionAllowedProfileError_0);
    EXPECT_FLOAT_EQ(std::get<float>(*allowedError), 46.f);
}

TEST(MAXMotionConfigTest, SetsMAXMotionPositionMode) {
    SubMAXMotionConfig config{};

    // Set using default slot = kSlot0
    config.PositionMode(config.MAXMotionPositionMode::kMAXMotionTrapezoidal);
    auto mode = config.GetParam(SparkParameter::kMAXMotionPositionMode_0);
    EXPECT_EQ(std::get<uint32_t>(*mode),
              config.MAXMotionPositionMode::kMAXMotionTrapezoidal);

#if 0  // TODO(rylan): Add when S-curve is supported
    config.PositionMode(config.MAXMotionPositionMode::kMAXMotionSCurve);
    mode = config.GetParam(SparkParameter::kMAXMotionPositionMode_0);
    EXPECT_EQ(std::get<uint32_t>(*mode), config.MAXMotionPositionMode::kMAXMotionSCurve);
#endif

    config.PositionMode(config.MAXMotionPositionMode::kMAXMotionTrapezoidal,
                        kSlot1);
    mode = config.GetParam(SparkParameter::kMAXMotionPositionMode_1);
    EXPECT_EQ(std::get<uint32_t>(*mode),
              config.MAXMotionPositionMode::kMAXMotionTrapezoidal);

    config.PositionMode(config.MAXMotionPositionMode::kMAXMotionTrapezoidal,
                        kSlot2);
    mode = config.GetParam(SparkParameter::kMAXMotionPositionMode_2);
    EXPECT_EQ(std::get<uint32_t>(*mode),
              config.MAXMotionPositionMode::kMAXMotionTrapezoidal);

    config.PositionMode(config.MAXMotionPositionMode::kMAXMotionTrapezoidal,
                        kSlot3);
    mode = config.GetParam(SparkParameter::kMAXMotionPositionMode_3);
    EXPECT_EQ(std::get<uint32_t>(*mode),
              config.MAXMotionPositionMode::kMAXMotionTrapezoidal);

    // // Explicitly set kSlot0
    config.PositionMode(config.MAXMotionPositionMode::kMAXMotionTrapezoidal,
                        kSlot0);
    mode = config.GetParam(SparkParameter::kMAXMotionPositionMode_0);
    EXPECT_EQ(std::get<uint32_t>(*mode),
              config.MAXMotionPositionMode::kMAXMotionTrapezoidal);
}

////////////////////////////////////////////////

// This class is so we can query the parameters with GetParameter (which is
// protected in BaseConfig)
class SubSignalsConfig : public rev::spark::SignalsConfig {
public:
    std::optional<ParameterType_t> GetParam(uint8_t parameterId) {
        return GetParameter(parameterId);
    }
};

TEST(SignalsConfigTest, SetsAppliedOutputPeriodMs) {
    SubSignalsConfig config{};

    config.AppliedOutputPeriodMs(42);
    auto periodMs = config.GetParam(SparkParameter::kStatus0Period);
    EXPECT_EQ(std::get<uint32_t>(*periodMs), 42u);
}

TEST(SignalsConfigTest, SetsBusVoltagePeriodMs) {
    SubSignalsConfig config{};

    config.BusVoltagePeriodMs(42);
    auto periodMs = config.GetParam(SparkParameter::kStatus0Period);
    EXPECT_EQ(std::get<uint32_t>(*periodMs), 42u);
}

TEST(SignalsConfigTest, SetsOutputCurrentPeriodMs) {
    SubSignalsConfig config{};

    config.OutputCurrentPeriodMs(42);
    auto periodMs = config.GetParam(SparkParameter::kStatus0Period);
    EXPECT_EQ(std::get<uint32_t>(*periodMs), 42u);
}

TEST(SignalsConfigTest, SetsMotorTemperaturePeriodMs) {
    SubSignalsConfig config{};

    config.MotorTemperaturePeriodMs(42);
    auto periodMs = config.GetParam(SparkParameter::kStatus0Period);
    EXPECT_EQ(std::get<uint32_t>(*periodMs), 42u);
}

TEST(SignalsConfigTest, SetsLimitsPeriodMs) {
    SubSignalsConfig config{};

    config.LimitsPeriodMs(42);
    auto periodMs = config.GetParam(SparkParameter::kStatus0Period);
    EXPECT_EQ(std::get<uint32_t>(*periodMs), 42u);
}

TEST(SignalsConfigTest, SetsFaultsPeriodMs) {
    SubSignalsConfig config{};

    config.FaultsPeriodMs(42);
    auto periodMs = config.GetParam(SparkParameter::kStatus1Period);
    EXPECT_EQ(std::get<uint32_t>(*periodMs), 42u);
}

TEST(SignalsConfigTest, SetsFaultsAlwaysOn) {
    SubSignalsConfig config{};

    config.FaultsAlwaysOn(false);
    auto enabled = config.GetParam(SparkParameter::kForceEnableStatus_1);
    EXPECT_EQ(std::get<uint32_t>(*enabled), 0u);

    config.FaultsAlwaysOn(true);
    enabled = config.GetParam(SparkParameter::kForceEnableStatus_1);
    EXPECT_EQ(std::get<uint32_t>(*enabled), 1u);

    config.FaultsAlwaysOn(false);
    enabled = config.GetParam(SparkParameter::kForceEnableStatus_1);
    // Enabled is latched
    EXPECT_EQ(std::get<uint32_t>(*enabled), 1u);
}

TEST(SignalsConfigTest, SetsWarningsPeriodMs) {
    SubSignalsConfig config{};

    config.WarningsPeriodMs(42);
    auto periodMs = config.GetParam(SparkParameter::kStatus1Period);
    EXPECT_EQ(std::get<uint32_t>(*periodMs), 42u);
}

TEST(SignalsConfigTest, SetsWarningsAlwaysOn) {
    SubSignalsConfig config{};

    config.WarningsAlwaysOn(false);
    auto enabled = config.GetParam(SparkParameter::kForceEnableStatus_1);
    EXPECT_EQ(std::get<uint32_t>(*enabled), 0u);

    config.WarningsAlwaysOn(true);
    enabled = config.GetParam(SparkParameter::kForceEnableStatus_1);
    EXPECT_EQ(std::get<uint32_t>(*enabled), 1u);

    config.WarningsAlwaysOn(false);
    enabled = config.GetParam(SparkParameter::kForceEnableStatus_1);
    // Enabled is latched
    EXPECT_EQ(std::get<uint32_t>(*enabled), 1u);
}

TEST(SignalsConfigTest, SetsPrimaryEncoderVelocityPeriodMs) {
    SubSignalsConfig config{};

    config.PrimaryEncoderVelocityPeriodMs(42);
    auto periodMs = config.GetParam(SparkParameter::kStatus2Period);
    EXPECT_EQ(std::get<uint32_t>(*periodMs), 42u);
}

TEST(SignalsConfigTest, SetsPrimaryEncoderVelocityAlwaysOn) {
    SubSignalsConfig config{};

    config.PrimaryEncoderVelocityAlwaysOn(false);
    auto enabled = config.GetParam(SparkParameter::kForceEnableStatus_2);
    EXPECT_EQ(std::get<uint32_t>(*enabled), 0u);

    config.PrimaryEncoderVelocityAlwaysOn(true);
    enabled = config.GetParam(SparkParameter::kForceEnableStatus_2);
    EXPECT_EQ(std::get<uint32_t>(*enabled), 1u);

    config.PrimaryEncoderVelocityAlwaysOn(false);
    enabled = config.GetParam(SparkParameter::kForceEnableStatus_2);
    // Enabled is latched
    EXPECT_EQ(std::get<uint32_t>(*enabled), 1u);
}

TEST(SignalsConfigTest, SetsAnalogVoltagePeriodMs) {
    SubSignalsConfig config{};

    config.AnalogVoltagePeriodMs(42);
    auto periodMs = config.GetParam(SparkParameter::kStatus3Period);
    EXPECT_EQ(std::get<uint32_t>(*periodMs), 42u);
}

TEST(SignalsConfigTest, SetsAnalogVoltageAlwaysOn) {
    SubSignalsConfig config{};

    config.AnalogVoltageAlwaysOn(false);
    auto enabled = config.GetParam(SparkParameter::kForceEnableStatus_3);
    EXPECT_EQ(std::get<uint32_t>(*enabled), 0u);

    config.AnalogVoltageAlwaysOn(true);
    enabled = config.GetParam(SparkParameter::kForceEnableStatus_3);
    EXPECT_EQ(std::get<uint32_t>(*enabled), 1u);

    config.AnalogVoltageAlwaysOn(false);
    enabled = config.GetParam(SparkParameter::kForceEnableStatus_3);
    // Enabled is latched
    EXPECT_EQ(std::get<uint32_t>(*enabled), 1u);
}

TEST(SignalsConfigTest, SetsAnalogVelocityPeriodMs) {
    SubSignalsConfig config{};

    config.AnalogVelocityPeriodMs(42);
    auto periodMs = config.GetParam(SparkParameter::kStatus3Period);
    EXPECT_EQ(std::get<uint32_t>(*periodMs), 42u);
}

TEST(SignalsConfigTest, SetsAnalogVelocityAlwaysOn) {
    SubSignalsConfig config{};

    config.AnalogVelocityAlwaysOn(false);
    auto enabled = config.GetParam(SparkParameter::kForceEnableStatus_3);
    EXPECT_EQ(std::get<uint32_t>(*enabled), 0u);

    config.AnalogVelocityAlwaysOn(true);
    enabled = config.GetParam(SparkParameter::kForceEnableStatus_3);
    EXPECT_EQ(std::get<uint32_t>(*enabled), 1u);

    config.AnalogVelocityAlwaysOn(false);
    enabled = config.GetParam(SparkParameter::kForceEnableStatus_3);
    // Enabled is latched
    EXPECT_EQ(std::get<uint32_t>(*enabled), 1u);
}

TEST(SignalsConfigTest, SetsAnalogPositionPeriodMs) {
    SubSignalsConfig config{};

    config.AnalogPositionPeriodMs(42);
    auto periodMs = config.GetParam(SparkParameter::kStatus3Period);
    EXPECT_EQ(std::get<uint32_t>(*periodMs), 42u);
}

TEST(SignalsConfigTest, SetsAnalogPositionAlwaysOn) {
    SubSignalsConfig config{};

    config.AnalogPositionAlwaysOn(false);
    auto enabled = config.GetParam(SparkParameter::kForceEnableStatus_3);
    EXPECT_EQ(std::get<uint32_t>(*enabled), 0u);

    config.AnalogPositionAlwaysOn(true);
    enabled = config.GetParam(SparkParameter::kForceEnableStatus_3);
    EXPECT_EQ(std::get<uint32_t>(*enabled), 1u);

    config.AnalogPositionAlwaysOn(false);
    enabled = config.GetParam(SparkParameter::kForceEnableStatus_3);
    // Enabled is latched
    EXPECT_EQ(std::get<uint32_t>(*enabled), 1u);
}

TEST(SignalsConfigTest, SetsExternalOrAltEncoderVelocity) {
    SubSignalsConfig config{};

    config.ExternalOrAltEncoderVelocity(42);
    auto periodMs = config.GetParam(SparkParameter::kStatus4Period);
    EXPECT_EQ(std::get<uint32_t>(*periodMs), 42u);
}

TEST(SignalsConfigTest, SetsExternalOrAltEncoderVelocityAlwaysOn) {
    SubSignalsConfig config{};

    config.ExternalOrAltEncoderVelocityAlwaysOn(false);
    auto enabled = config.GetParam(SparkParameter::kForceEnableStatus_4);
    EXPECT_EQ(std::get<uint32_t>(*enabled), 0u);

    config.ExternalOrAltEncoderVelocityAlwaysOn(true);
    enabled = config.GetParam(SparkParameter::kForceEnableStatus_4);
    EXPECT_EQ(std::get<uint32_t>(*enabled), 1u);

    config.ExternalOrAltEncoderVelocityAlwaysOn(false);
    enabled = config.GetParam(SparkParameter::kForceEnableStatus_4);
    // Enabled is latched
    EXPECT_EQ(std::get<uint32_t>(*enabled), 1u);
}

TEST(SignalsConfigTest, SetsExternalOrAltEncoderPosition) {
    SubSignalsConfig config{};

    config.ExternalOrAltEncoderPosition(42);
    auto periodMs = config.GetParam(SparkParameter::kStatus4Period);
    EXPECT_EQ(std::get<uint32_t>(*periodMs), 42u);
}

TEST(SignalsConfigTest, SetsExternalOrAltEncoderPositionAlwaysOn) {
    SubSignalsConfig config{};

    config.ExternalOrAltEncoderPositionAlwaysOn(false);
    auto enabled = config.GetParam(SparkParameter::kForceEnableStatus_4);
    EXPECT_EQ(std::get<uint32_t>(*enabled), 0u);

    config.ExternalOrAltEncoderPositionAlwaysOn(true);
    enabled = config.GetParam(SparkParameter::kForceEnableStatus_4);
    EXPECT_EQ(std::get<uint32_t>(*enabled), 1u);

    config.ExternalOrAltEncoderPositionAlwaysOn(false);
    enabled = config.GetParam(SparkParameter::kForceEnableStatus_4);
    // Enabled is latched
    EXPECT_EQ(std::get<uint32_t>(*enabled), 1u);
}

TEST(SignalsConfigTest, SetsAbsoluteEncoderVelocityPeriodMs) {
    SubSignalsConfig config{};

    config.AbsoluteEncoderVelocityPeriodMs(42);
    auto periodMs = config.GetParam(SparkParameter::kStatus5Period);
    EXPECT_EQ(std::get<uint32_t>(*periodMs), 42u);
}

TEST(SignalsConfigTest, SetsAbsoluteEncoderVelocityAlwaysOn) {
    SubSignalsConfig config{};

    config.AbsoluteEncoderVelocityAlwaysOn(false);
    auto enabled = config.GetParam(SparkParameter::kForceEnableStatus_5);
    EXPECT_EQ(std::get<uint32_t>(*enabled), 0u);

    config.AbsoluteEncoderVelocityAlwaysOn(true);
    enabled = config.GetParam(SparkParameter::kForceEnableStatus_5);
    EXPECT_EQ(std::get<uint32_t>(*enabled), 1u);

    config.AbsoluteEncoderVelocityAlwaysOn(false);
    enabled = config.GetParam(SparkParameter::kForceEnableStatus_5);
    // Enabled is latched
    EXPECT_EQ(std::get<uint32_t>(*enabled), 1u);
}

TEST(SignalsConfigTest, SetsAbsoluteEncoderPositionPeriodMs) {
    SubSignalsConfig config{};

    config.AbsoluteEncoderPositionPeriodMs(42);
    auto periodMs = config.GetParam(SparkParameter::kStatus5Period);
    EXPECT_EQ(std::get<uint32_t>(*periodMs), 42u);
}

TEST(SignalsConfigTest, SetsAbsoluteEncoderPositionAlwaysOn) {
    SubSignalsConfig config{};

    config.AbsoluteEncoderPositionAlwaysOn(false);
    auto enabled = config.GetParam(SparkParameter::kForceEnableStatus_5);
    EXPECT_EQ(std::get<uint32_t>(*enabled), 0u);

    config.AbsoluteEncoderPositionAlwaysOn(true);
    enabled = config.GetParam(SparkParameter::kForceEnableStatus_5);
    EXPECT_EQ(std::get<uint32_t>(*enabled), 1u);

    config.AbsoluteEncoderPositionAlwaysOn(false);
    enabled = config.GetParam(SparkParameter::kForceEnableStatus_5);
    // Enabled is latched
    EXPECT_EQ(std::get<uint32_t>(*enabled), 1u);
}

TEST(SignalsConfigTest, SetsIAccumulationPeriodMs) {
    SubSignalsConfig config{};

    config.IAccumulationPeriodMs(42);
    auto periodMs = config.GetParam(SparkParameter::kStatus7Period);
    EXPECT_EQ(std::get<uint32_t>(*periodMs), 42u);
}

TEST(SignalsConfigTest, SetsIAccumulationAlwaysOn) {
    SubSignalsConfig config{};

    config.IAccumulationAlwaysOn(false);
    auto enabled = config.GetParam(SparkParameter::kForceEnableStatus_7);
    EXPECT_EQ(std::get<uint32_t>(*enabled), 0u);

    config.IAccumulationAlwaysOn(true);
    enabled = config.GetParam(SparkParameter::kForceEnableStatus_7);
    EXPECT_EQ(std::get<uint32_t>(*enabled), 1u);

    config.IAccumulationAlwaysOn(false);
    enabled = config.GetParam(SparkParameter::kForceEnableStatus_7);
    // Enabled is latched
    EXPECT_EQ(std::get<uint32_t>(*enabled), 1u);
}

TEST(SignalsConfigTest, SetsSetpointPeriodMs) {
    SubSignalsConfig config{};

    config.SetpointPeriodMs(42);
    auto periodMs = config.GetParam(SparkParameter::kStatus8Period);
    EXPECT_EQ(std::get<uint32_t>(*periodMs), 42u);
}

TEST(SignalsConfigTest, SetsSetpointAlwaysOn) {
    SubSignalsConfig config{};

    config.SetpointAlwaysOn(false);
    auto enabled = config.GetParam(SparkParameter::kForceEnableStatus_8);
    EXPECT_EQ(std::get<uint32_t>(*enabled), 0u);

    config.SetpointAlwaysOn(true);
    enabled = config.GetParam(SparkParameter::kForceEnableStatus_8);
    EXPECT_EQ(std::get<uint32_t>(*enabled), 1u);

    config.SetpointAlwaysOn(false);
    enabled = config.GetParam(SparkParameter::kForceEnableStatus_8);
    // Enabled is latched
    EXPECT_EQ(std::get<uint32_t>(*enabled), 1u);
}

TEST(SignalsConfigTest, SetsIsAtSetpointPeriodMs) {
    SubSignalsConfig config{};

    config.IsAtSetpointPeriodMs(42);
    auto periodMs = config.GetParam(SparkParameter::kStatus8Period);
    EXPECT_EQ(std::get<uint32_t>(*periodMs), 42u);
}

TEST(SignalsConfigTest, SetsIsAtSetpointAlwaysOn) {
    SubSignalsConfig config{};

    config.IsAtSetpointAlwaysOn(false);
    auto enabled = config.GetParam(SparkParameter::kForceEnableStatus_8);
    EXPECT_EQ(std::get<uint32_t>(*enabled), 0u);

    config.IsAtSetpointAlwaysOn(true);
    enabled = config.GetParam(SparkParameter::kForceEnableStatus_8);
    EXPECT_EQ(std::get<uint32_t>(*enabled), 1u);

    config.IsAtSetpointAlwaysOn(false);
    enabled = config.GetParam(SparkParameter::kForceEnableStatus_8);
    // Enabled is latched
    EXPECT_EQ(std::get<uint32_t>(*enabled), 1u);
}

TEST(SignalsConfigTest, SetsSelectedSlotPeriodMs) {
    SubSignalsConfig config{};

    config.SelectedSlotPeriodMs(42);
    auto periodMs = config.GetParam(SparkParameter::kStatus8Period);
    EXPECT_EQ(std::get<uint32_t>(*periodMs), 42u);
}

TEST(SignalsConfigTest, SetsSelectedSlotAlwaysOn) {
    SubSignalsConfig config{};

    config.SelectedSlotAlwaysOn(false);
    auto enabled = config.GetParam(SparkParameter::kForceEnableStatus_8);
    EXPECT_EQ(std::get<uint32_t>(*enabled), 0u);

    config.SelectedSlotAlwaysOn(true);
    enabled = config.GetParam(SparkParameter::kForceEnableStatus_8);
    EXPECT_EQ(std::get<uint32_t>(*enabled), 1u);

    config.SelectedSlotAlwaysOn(false);
    enabled = config.GetParam(SparkParameter::kForceEnableStatus_8);
    // Enabled is latched
    EXPECT_EQ(std::get<uint32_t>(*enabled), 1u);
}

////////////////////////////////////////////////

class SubSoftLimitConfig : public SoftLimitConfig {
public:
    std::optional<ParameterType_t> GetParam(uint8_t parameterId) {
        return GetParameter(parameterId);
    }
};

TEST(SoftLimitConfigTest, SetsForwardSoftLimit) {
    SubSoftLimitConfig config{};

    config.ForwardSoftLimit(42.);
    auto limit = config.GetParam(SparkParameter::kSoftLimitForward);
    EXPECT_FLOAT_EQ(std::get<float>(*limit), 42.f);
}

TEST(LimitSwitchConfigTest, SetsForwardSoftLimitEnabled) {
    SubSoftLimitConfig config{};

    config.ForwardSoftLimitEnabled(true);
    auto isEnabled = config.GetParam(SparkParameter::kSoftLimitFwdEn);
    EXPECT_EQ(std::get<bool>(*isEnabled), true);

    config.ForwardSoftLimitEnabled(false);
    isEnabled = config.GetParam(SparkParameter::kSoftLimitFwdEn);
    EXPECT_EQ(std::get<bool>(*isEnabled), false);
}

TEST(SoftLimitConfigTest, SetsReverseSoftLimit) {
    SubSoftLimitConfig config{};

    config.ReverseSoftLimit(42.);
    auto limit = config.GetParam(SparkParameter::kSoftLimitReverse);
    EXPECT_FLOAT_EQ(std::get<float>(*limit), 42.f);
}

TEST(LimitSwitchConfigTest, SetsReverseSoftLimitEnabled) {
    SubSoftLimitConfig config{};

    config.ReverseSoftLimitEnabled(true);
    auto isEnabled = config.GetParam(SparkParameter::kSoftLimitRevEn);
    EXPECT_EQ(std::get<bool>(*isEnabled), true);

    config.ReverseSoftLimitEnabled(false);
    isEnabled = config.GetParam(SparkParameter::kSoftLimitRevEn);
    EXPECT_EQ(std::get<bool>(*isEnabled), false);
}

////////////////////////////////////////////////

// This class is so we can query the parameters with GetParameter (which is
// protected in BaseConfig)
class SubFeedForwardConfig : public FeedForwardConfig {
public:
    std::optional<ParameterType_t> GetParam(uint8_t parameterId) {
        return GetParameter(parameterId);
    }
};

TEST(FeedForwardConfigTest, Sets_kS) {
    SubFeedForwardConfig config{};

    // Set using default slot = kSlot0
    config.kS(42.);
    auto kS = config.GetParam(SparkParameter::kS_0);
    EXPECT_FLOAT_EQ(std::get<float>(*kS), 42.f);

    config.kS(43., kSlot1);
    kS = config.GetParam(SparkParameter::kS_1);
    EXPECT_FLOAT_EQ(std::get<float>(*kS), 43.f);

    config.kS(44., kSlot2);
    kS = config.GetParam(SparkParameter::kS_2);
    EXPECT_FLOAT_EQ(std::get<float>(*kS), 44.f);

    config.kS(45., kSlot3);
    kS = config.GetParam(SparkParameter::kS_3);
    EXPECT_FLOAT_EQ(std::get<float>(*kS), 45.f);

    // Explicitly set kSlot0
    config.kS(46., kSlot0);
    kS = config.GetParam(SparkParameter::kS_0);
    EXPECT_FLOAT_EQ(std::get<float>(*kS), 46.f);
}

TEST(FeedForwardConfigTest, Sets_kV) {
    SubFeedForwardConfig config{};

    // Set using default slot = kSlot0
    config.kV(42.);
    auto kV = config.GetParam(SparkParameter::kV_0);
    EXPECT_FLOAT_EQ(std::get<float>(*kV), 42.f);

    config.kV(43., kSlot1);
    kV = config.GetParam(SparkParameter::kV_1);
    EXPECT_FLOAT_EQ(std::get<float>(*kV), 43.f);

    config.kV(44., kSlot2);
    kV = config.GetParam(SparkParameter::kV_2);
    EXPECT_FLOAT_EQ(std::get<float>(*kV), 44.f);

    config.kV(45., kSlot3);
    kV = config.GetParam(SparkParameter::kV_3);
    EXPECT_FLOAT_EQ(std::get<float>(*kV), 45.f);

    // Explicitly set kSlot0
    config.kV(46., kSlot0);
    kV = config.GetParam(SparkParameter::kV_0);
    EXPECT_FLOAT_EQ(std::get<float>(*kV), 46.f);
}

TEST(FeedForwardConfigTest, Sets_kA) {
    SubFeedForwardConfig config{};

    // Set using default slot = kSlot0
    config.kA(42.);
    auto kA = config.GetParam(SparkParameter::kA_0);
    EXPECT_FLOAT_EQ(std::get<float>(*kA), 42.f);

    config.kA(43., kSlot1);
    kA = config.GetParam(SparkParameter::kA_1);
    EXPECT_FLOAT_EQ(std::get<float>(*kA), 43.f);

    config.kA(44., kSlot2);
    kA = config.GetParam(SparkParameter::kA_2);
    EXPECT_FLOAT_EQ(std::get<float>(*kA), 44.f);

    config.kA(45., kSlot3);
    kA = config.GetParam(SparkParameter::kA_3);
    EXPECT_FLOAT_EQ(std::get<float>(*kA), 45.f);

    // Explicitly set kSlot0
    config.kA(46., kSlot0);
    kA = config.GetParam(SparkParameter::kA_0);
    EXPECT_FLOAT_EQ(std::get<float>(*kA), 46.f);
}

TEST(FeedForwardConfigTest, Sets_kG) {
    SubFeedForwardConfig config{};

    // Set using default slot = kSlot0
    config.kG(42.);
    auto kG = config.GetParam(SparkParameter::kG_0);
    EXPECT_FLOAT_EQ(std::get<float>(*kG), 42.f);

    config.kG(43., kSlot1);
    kG = config.GetParam(SparkParameter::kG_1);
    EXPECT_FLOAT_EQ(std::get<float>(*kG), 43.f);

    config.kG(44., kSlot2);
    kG = config.GetParam(SparkParameter::kG_2);
    EXPECT_FLOAT_EQ(std::get<float>(*kG), 44.f);

    config.kG(45., kSlot3);
    kG = config.GetParam(SparkParameter::kG_3);
    EXPECT_FLOAT_EQ(std::get<float>(*kG), 45.f);

    // Explicitly set kSlot0
    config.kG(46., kSlot0);
    kG = config.GetParam(SparkParameter::kG_0);
    EXPECT_FLOAT_EQ(std::get<float>(*kG), 46.f);
}

TEST(FeedForwardConfigTest, Sets_kCos) {
    SubFeedForwardConfig config{};

    // Set using default slot = kSlot0
    config.kCos(42.);
    auto kCos = config.GetParam(SparkParameter::kCos_0);
    EXPECT_FLOAT_EQ(std::get<float>(*kCos), 42.f);

    config.kCos(43., kSlot1);
    kCos = config.GetParam(SparkParameter::kCos_1);
    EXPECT_FLOAT_EQ(std::get<float>(*kCos), 43.f);

    config.kCos(44., kSlot2);
    kCos = config.GetParam(SparkParameter::kCos_2);
    EXPECT_FLOAT_EQ(std::get<float>(*kCos), 44.f);

    config.kCos(45., kSlot3);
    kCos = config.GetParam(SparkParameter::kCos_3);
    EXPECT_FLOAT_EQ(std::get<float>(*kCos), 45.f);

    // Explicitly set kSlot0
    config.kCos(46., kSlot0);
    kCos = config.GetParam(SparkParameter::kCos_0);
    EXPECT_FLOAT_EQ(std::get<float>(*kCos), 46.f);
}

TEST(FeedForwardConfigTest, Sets_kCosRatio) {
    SubFeedForwardConfig config{};

    // Set using default slot = kSlot0
    config.kCosRatio(42.);
    auto kCosRatio = config.GetParam(SparkParameter::kCosRatio_0);
    EXPECT_FLOAT_EQ(std::get<float>(*kCosRatio), 42.f);

    config.kCosRatio(43., kSlot1);
    kCosRatio = config.GetParam(SparkParameter::kCosRatio_1);
    EXPECT_FLOAT_EQ(std::get<float>(*kCosRatio), 43.f);

    config.kCosRatio(44., kSlot2);
    kCosRatio = config.GetParam(SparkParameter::kCosRatio_2);
    EXPECT_FLOAT_EQ(std::get<float>(*kCosRatio), 44.f);

    config.kCosRatio(45., kSlot3);
    kCosRatio = config.GetParam(SparkParameter::kCosRatio_3);
    EXPECT_FLOAT_EQ(std::get<float>(*kCosRatio), 45.f);

    // Explicitly set kSlot0
    config.kCosRatio(46., kSlot0);
    kCosRatio = config.GetParam(SparkParameter::kCosRatio_0);
    EXPECT_FLOAT_EQ(std::get<float>(*kCosRatio), 46.f);
}

TEST(FeedForwardConfigTest, Flattens) {
    SubFeedForwardConfig config{};

    config.kS(42.)
        .kV(43., kSlot1)
        .kA(44., kSlot2)
        .kG(45., kSlot3)
        .kCos(0.123)
        .kCosRatio(0.456, kSlot1);

    std::string flattened{config.Flatten()};

    ValidateFlattenedSubString(flattened, "204,42280000\n");
    ValidateFlattenedSubString(flattened, "213,3EE978D5\n");
    ValidateFlattenedSubString(flattened, "215,42300000\n");
    ValidateFlattenedSubString(flattened, "24,422C0000\n");
    ValidateFlattenedSubString(flattened, "221,42340000\n");
    ValidateFlattenedSubString(flattened, "207,3DFBE76D\n");

    EXPECT_EQ(flattened.size(), 0u) << flattened;
}
////////////////////////////////////////////////

class SubClosedLoopConfig : public ClosedLoopConfig {
public:
    std::optional<ParameterType_t> GetParam(uint8_t parameterId) {
        return GetParameter(parameterId);
    }
};

TEST(ClosedLoopConfigTest, SetsPid) {
    SubClosedLoopConfig config{};

    // Set using default slot = kSlot0
    config.Pid(42., 43., 44.);
    auto p = config.GetParam(SparkParameter::kP_0);
    auto i = config.GetParam(SparkParameter::kI_0);
    auto d = config.GetParam(SparkParameter::kD_0);
    EXPECT_FLOAT_EQ(std::get<float>(*p), 42.f);
    EXPECT_FLOAT_EQ(std::get<float>(*i), 43.f);
    EXPECT_FLOAT_EQ(std::get<float>(*d), 44.f);

    config.Pid(142., 143., 144., kSlot1);
    p = config.GetParam(SparkParameter::kP_1);
    i = config.GetParam(SparkParameter::kI_1);
    d = config.GetParam(SparkParameter::kD_1);
    EXPECT_FLOAT_EQ(std::get<float>(*p), 142.f);
    EXPECT_FLOAT_EQ(std::get<float>(*i), 143.f);
    EXPECT_FLOAT_EQ(std::get<float>(*d), 144.f);

    config.Pid(242., 243., 244., kSlot2);
    p = config.GetParam(SparkParameter::kP_2);
    i = config.GetParam(SparkParameter::kI_2);
    d = config.GetParam(SparkParameter::kD_2);
    EXPECT_FLOAT_EQ(std::get<float>(*p), 242.f);
    EXPECT_FLOAT_EQ(std::get<float>(*i), 243.f);
    EXPECT_FLOAT_EQ(std::get<float>(*d), 244.f);

    config.Pid(342., 343., 344., kSlot3);
    p = config.GetParam(SparkParameter::kP_3);
    i = config.GetParam(SparkParameter::kI_3);
    d = config.GetParam(SparkParameter::kD_3);
    EXPECT_FLOAT_EQ(std::get<float>(*p), 342.f);
    EXPECT_FLOAT_EQ(std::get<float>(*i), 343.f);
    EXPECT_FLOAT_EQ(std::get<float>(*d), 344.f);

    // Explicitly set kSlot0
    config.Pid(42., 43., 44., kSlot0);
    p = config.GetParam(SparkParameter::kP_0);
    i = config.GetParam(SparkParameter::kI_0);
    d = config.GetParam(SparkParameter::kD_0);
    EXPECT_FLOAT_EQ(std::get<float>(*p), 42.f);
    EXPECT_FLOAT_EQ(std::get<float>(*i), 43.f);
    EXPECT_FLOAT_EQ(std::get<float>(*d), 44.f);
}

TEST(ClosedLoopConfigTest, SetsP) {
    SubClosedLoopConfig config{};

    // Set using default slot = kSlot0
    config.P(42.);
    auto p = config.GetParam(SparkParameter::kP_0);
    EXPECT_FLOAT_EQ(std::get<float>(*p), 42.f);

    config.P(142., kSlot1);
    p = config.GetParam(SparkParameter::kP_1);
    EXPECT_FLOAT_EQ(std::get<float>(*p), 142.f);

    config.P(242., kSlot2);
    p = config.GetParam(SparkParameter::kP_2);
    EXPECT_FLOAT_EQ(std::get<float>(*p), 242.f);

    config.P(342., kSlot3);
    p = config.GetParam(SparkParameter::kP_3);
    EXPECT_FLOAT_EQ(std::get<float>(*p), 342.f);

    // Explicitly set kSlot0
    config.P(42., kSlot0);
    p = config.GetParam(SparkParameter::kP_0);
    EXPECT_FLOAT_EQ(std::get<float>(*p), 42.f);
}

TEST(ClosedLoopConfigTest, SetsI) {
    SubClosedLoopConfig config{};

    // Set using default slot = kSlot0
    config.I(42.);
    auto i = config.GetParam(SparkParameter::kI_0);
    EXPECT_FLOAT_EQ(std::get<float>(*i), 42.f);

    config.I(142., kSlot1);
    i = config.GetParam(SparkParameter::kI_1);
    EXPECT_FLOAT_EQ(std::get<float>(*i), 142.f);

    config.I(242., kSlot2);
    i = config.GetParam(SparkParameter::kI_2);
    EXPECT_FLOAT_EQ(std::get<float>(*i), 242.f);

    config.I(342., kSlot3);
    i = config.GetParam(SparkParameter::kI_3);
    EXPECT_FLOAT_EQ(std::get<float>(*i), 342.f);

    // Explicitly set kSlot0
    config.I(42., kSlot0);
    i = config.GetParam(SparkParameter::kI_0);
    EXPECT_FLOAT_EQ(std::get<float>(*i), 42.f);
}

TEST(ClosedLoopConfigTest, SetsD) {
    SubClosedLoopConfig config{};

    // Set using default slot = kSlot0
    config.D(42.);
    auto d = config.GetParam(SparkParameter::kD_0);
    EXPECT_FLOAT_EQ(std::get<float>(*d), 42.f);

    config.D(142., kSlot1);
    d = config.GetParam(SparkParameter::kD_1);
    EXPECT_FLOAT_EQ(std::get<float>(*d), 142.f);

    config.D(242., kSlot2);
    d = config.GetParam(SparkParameter::kD_2);
    EXPECT_FLOAT_EQ(std::get<float>(*d), 242.f);

    config.D(342., kSlot3);
    d = config.GetParam(SparkParameter::kD_3);
    EXPECT_FLOAT_EQ(std::get<float>(*d), 342.f);

    // Explicitly set kSlot0
    config.D(42., kSlot0);
    d = config.GetParam(SparkParameter::kD_0);
    EXPECT_FLOAT_EQ(std::get<float>(*d), 42.f);
}

TEST(ClosedLoopConfigTest, SetsDFilter) {
    SubClosedLoopConfig config{};

    // Set using default slot = kSlot0
    config.DFilter(42.);
    auto dFilter = config.GetParam(SparkParameter::kDFilter_0);
    EXPECT_FLOAT_EQ(std::get<float>(*dFilter), 42.f);

    config.DFilter(142., kSlot1);
    dFilter = config.GetParam(SparkParameter::kDFilter_1);
    EXPECT_FLOAT_EQ(std::get<float>(*dFilter), 142.f);

    config.DFilter(242., kSlot2);
    dFilter = config.GetParam(SparkParameter::kDFilter_2);
    EXPECT_FLOAT_EQ(std::get<float>(*dFilter), 242.f);

    config.DFilter(342., kSlot3);
    dFilter = config.GetParam(SparkParameter::kDFilter_3);
    EXPECT_FLOAT_EQ(std::get<float>(*dFilter), 342.f);

    // Explicitly set kSlot0
    config.DFilter(42., kSlot0);
    dFilter = config.GetParam(SparkParameter::kDFilter_0);
    EXPECT_FLOAT_EQ(std::get<float>(*dFilter), 42.f);
}

TEST(ClosedLoopConfigTest, SetsIZone) {
    SubClosedLoopConfig config{};

    // Set using default slot = kSlot0
    config.IZone(42.);
    auto iZone = config.GetParam(SparkParameter::kIZone_0);
    EXPECT_FLOAT_EQ(std::get<float>(*iZone), 42.f);

    config.IZone(142., kSlot1);
    iZone = config.GetParam(SparkParameter::kIZone_1);
    EXPECT_FLOAT_EQ(std::get<float>(*iZone), 142.f);

    config.IZone(242., kSlot2);
    iZone = config.GetParam(SparkParameter::kIZone_2);
    EXPECT_FLOAT_EQ(std::get<float>(*iZone), 242.f);

    config.IZone(342., kSlot3);
    iZone = config.GetParam(SparkParameter::kIZone_3);
    EXPECT_FLOAT_EQ(std::get<float>(*iZone), 342.f);

    // Explicitly set kSlot0
    config.IZone(42., kSlot0);
    iZone = config.GetParam(SparkParameter::kIZone_0);
    EXPECT_FLOAT_EQ(std::get<float>(*iZone), 42.f);
}

TEST(ClosedLoopConfigTest, SetsMinOutput) {
    SubClosedLoopConfig config{};

    // Set using default slot = kSlot0
    config.MinOutput(42.);
    auto minOutput = config.GetParam(SparkParameter::kOutputMin_0);
    EXPECT_FLOAT_EQ(std::get<float>(*minOutput), 42.f);

    config.MinOutput(142., kSlot1);
    minOutput = config.GetParam(SparkParameter::kOutputMin_1);
    EXPECT_FLOAT_EQ(std::get<float>(*minOutput), 142.f);

    config.MinOutput(242., kSlot2);
    minOutput = config.GetParam(SparkParameter::kOutputMin_2);
    EXPECT_FLOAT_EQ(std::get<float>(*minOutput), 242.f);

    config.MinOutput(342., kSlot3);
    minOutput = config.GetParam(SparkParameter::kOutputMin_3);
    EXPECT_FLOAT_EQ(std::get<float>(*minOutput), 342.f);

    // Explicitly set kSlot0
    config.MinOutput(42., kSlot0);
    minOutput = config.GetParam(SparkParameter::kOutputMin_0);
    EXPECT_FLOAT_EQ(std::get<float>(*minOutput), 42.f);
}

TEST(ClosedLoopConfigTest, SetsMaxOutput) {
    SubClosedLoopConfig config{};

    // Set using default slot = kSlot0
    config.MaxOutput(42.);
    auto maxOutput = config.GetParam(SparkParameter::kOutputMax_0);
    EXPECT_FLOAT_EQ(std::get<float>(*maxOutput), 42.f);

    config.MaxOutput(142., kSlot1);
    maxOutput = config.GetParam(SparkParameter::kOutputMax_1);
    EXPECT_FLOAT_EQ(std::get<float>(*maxOutput), 142.f);

    config.MaxOutput(242., kSlot2);
    maxOutput = config.GetParam(SparkParameter::kOutputMax_2);
    EXPECT_FLOAT_EQ(std::get<float>(*maxOutput), 242.f);

    config.MaxOutput(342., kSlot3);
    maxOutput = config.GetParam(SparkParameter::kOutputMax_3);
    EXPECT_FLOAT_EQ(std::get<float>(*maxOutput), 342.f);

    // Explicitly set kSlot0
    config.MaxOutput(42., kSlot0);
    maxOutput = config.GetParam(SparkParameter::kOutputMax_0);
    EXPECT_FLOAT_EQ(std::get<float>(*maxOutput), 42.f);
}

TEST(ClosedLoopConfigTest, SetsOutputRange) {
    SubClosedLoopConfig config{};

    // Set using default slot = kSlot0
    config.OutputRange(42., 43.);
    auto minOutput = config.GetParam(SparkParameter::kOutputMin_0);
    auto maxOutput = config.GetParam(SparkParameter::kOutputMax_0);
    EXPECT_FLOAT_EQ(std::get<float>(*minOutput), 42.f);
    EXPECT_FLOAT_EQ(std::get<float>(*maxOutput), 43.f);

    config.OutputRange(142., 143., kSlot1);
    minOutput = config.GetParam(SparkParameter::kOutputMin_1);
    maxOutput = config.GetParam(SparkParameter::kOutputMax_1);
    EXPECT_FLOAT_EQ(std::get<float>(*minOutput), 142.f);
    EXPECT_FLOAT_EQ(std::get<float>(*maxOutput), 143.f);

    config.OutputRange(242., 243., kSlot2);
    minOutput = config.GetParam(SparkParameter::kOutputMin_2);
    maxOutput = config.GetParam(SparkParameter::kOutputMax_2);
    EXPECT_FLOAT_EQ(std::get<float>(*minOutput), 242.f);
    EXPECT_FLOAT_EQ(std::get<float>(*maxOutput), 243.f);

    config.OutputRange(342., 343., kSlot3);
    minOutput = config.GetParam(SparkParameter::kOutputMin_3);
    maxOutput = config.GetParam(SparkParameter::kOutputMax_3);
    EXPECT_FLOAT_EQ(std::get<float>(*minOutput), 342.f);
    EXPECT_FLOAT_EQ(std::get<float>(*maxOutput), 343.f);

    // Explicitly set kSlot0
    config.OutputRange(42., 43., kSlot0);
    minOutput = config.GetParam(SparkParameter::kOutputMin_0);
    maxOutput = config.GetParam(SparkParameter::kOutputMax_0);
    EXPECT_FLOAT_EQ(std::get<float>(*minOutput), 42.f);
    EXPECT_FLOAT_EQ(std::get<float>(*maxOutput), 43.f);
}

TEST(ClosedLoopConfigTest, SetsIMaxAccum) {
    SubClosedLoopConfig config{};

    // Set using default slot = kSlot0
    config.IMaxAccum(42.);
    auto iMaxAccum = config.GetParam(SparkParameter::kIMaxAccum_0);
    EXPECT_FLOAT_EQ(std::get<float>(*iMaxAccum), 42.f);

    config.IMaxAccum(142., kSlot1);
    iMaxAccum = config.GetParam(SparkParameter::kIMaxAccum_1);
    EXPECT_FLOAT_EQ(std::get<float>(*iMaxAccum), 142.f);

    config.IMaxAccum(242., kSlot2);
    iMaxAccum = config.GetParam(SparkParameter::kIMaxAccum_2);
    EXPECT_FLOAT_EQ(std::get<float>(*iMaxAccum), 242.f);

    config.IMaxAccum(342., kSlot3);
    iMaxAccum = config.GetParam(SparkParameter::kIMaxAccum_3);
    EXPECT_FLOAT_EQ(std::get<float>(*iMaxAccum), 342.f);

    // Explicitly set kSlot0
    config.IMaxAccum(42., kSlot0);
    iMaxAccum = config.GetParam(SparkParameter::kIMaxAccum_0);
    EXPECT_FLOAT_EQ(std::get<float>(*iMaxAccum), 42.f);
}

TEST(ClosedLoopConfigTest, SetsPositionWrappingEnabled) {
    SubClosedLoopConfig config{};

    config.PositionWrappingEnabled(true);
    auto isEnabled = config.GetParam(SparkParameter::kPositionPIDWrapEnable);
    EXPECT_EQ(std::get<bool>(*isEnabled), true);

    config.PositionWrappingEnabled(false);
    isEnabled = config.GetParam(SparkParameter::kPositionPIDWrapEnable);
    EXPECT_EQ(std::get<bool>(*isEnabled), false);
}

TEST(ClosedLoopConfigTest, SetsPositionWrappingMinInput) {
    SubClosedLoopConfig config{};

    config.PositionWrappingMinInput(42.);
    auto minInput = config.GetParam(SparkParameter::kPositionPIDMinInput);
    EXPECT_FLOAT_EQ(std::get<float>(*minInput), 42.f);
}

TEST(ClosedLoopConfigTest, SetsPositionWrappingMaxInput) {
    SubClosedLoopConfig config{};

    config.PositionWrappingMaxInput(42.);
    auto maxInput = config.GetParam(SparkParameter::kPositionPIDMaxInput);
    EXPECT_FLOAT_EQ(std::get<float>(*maxInput), 42.f);
}

TEST(ClosedLoopConfigTest, SetsPositionWrappingInputRange) {
    SubClosedLoopConfig config{};

    config.PositionWrappingInputRange(42., 43.);
    auto minInput = config.GetParam(SparkParameter::kPositionPIDMinInput);
    auto maxInput = config.GetParam(SparkParameter::kPositionPIDMaxInput);
    EXPECT_FLOAT_EQ(std::get<float>(*minInput), 42.f);
    EXPECT_FLOAT_EQ(std::get<float>(*maxInput), 43.f);
}

TEST(ClosedLoopConfigTest, SetsAttachedFeedbackSensor) {
    SubClosedLoopConfig config{};

    config.SetFeedbackSensor(FeedbackSensor::kNoSensor);
    auto sensor = config.GetParam(SparkParameter::kClosedLoopControlSensor);
    EXPECT_EQ(static_cast<FeedbackSensor>(std::get<uint32_t>(*sensor)),
              FeedbackSensor::kNoSensor);

    config.SetFeedbackSensor(FeedbackSensor::kPrimaryEncoder);
    sensor = config.GetParam(SparkParameter::kClosedLoopControlSensor);
    EXPECT_EQ(static_cast<FeedbackSensor>(std::get<uint32_t>(*sensor)),
              FeedbackSensor::kPrimaryEncoder);

    config.SetFeedbackSensor(FeedbackSensor::kAnalogSensor);
    sensor = config.GetParam(SparkParameter::kClosedLoopControlSensor);
    EXPECT_EQ(static_cast<FeedbackSensor>(std::get<uint32_t>(*sensor)),
              FeedbackSensor::kAnalogSensor);

    config.SetFeedbackSensor(FeedbackSensor::kAlternateOrExternalEncoder);
    sensor = config.GetParam(SparkParameter::kClosedLoopControlSensor);
    EXPECT_EQ(static_cast<FeedbackSensor>(std::get<uint32_t>(*sensor)),
              FeedbackSensor::kAlternateOrExternalEncoder);

    config.SetFeedbackSensor(FeedbackSensor::kAbsoluteEncoder);
    sensor = config.GetParam(SparkParameter::kClosedLoopControlSensor);
    EXPECT_EQ(static_cast<FeedbackSensor>(std::get<uint32_t>(*sensor)),
              FeedbackSensor::kAbsoluteEncoder);
}

TEST(ClosedLoopConfigTest, SetsDetachedFeedbackSensor) {
    SubClosedLoopConfig config{};

    SplineEncoder spline1{0, 13};
    SplineEncoder spline2{1, 15};

    config.SetFeedbackSensor(FeedbackSensor::kDetachedAbsoluteEncoder,
                             spline1.GetDeviceId());
    auto sensor = config.GetParam(SparkParameter::kClosedLoopControlSensor);
    EXPECT_EQ(static_cast<FeedbackSensor>(std::get<uint32_t>(*sensor)),
              FeedbackSensor::kDetachedAbsoluteEncoder);
    auto detachedId = config.GetParam(SparkParameter::kDetachedEncoderDeviceID);
    EXPECT_EQ(std::get<uint32_t>(*detachedId),
              static_cast<uint32_t>(spline1.GetDeviceId()));

    config.SetFeedbackSensor(FeedbackSensor::kPrimaryEncoder);
    sensor = config.GetParam(SparkParameter::kClosedLoopControlSensor);
    EXPECT_EQ(static_cast<FeedbackSensor>(std::get<uint32_t>(*sensor)),
              FeedbackSensor::kPrimaryEncoder);

    config.SetFeedbackSensor(FeedbackSensor::kDetachedRelativeEncoder,
                             spline1.GetDeviceId());
    sensor = config.GetParam(SparkParameter::kClosedLoopControlSensor);
    EXPECT_EQ(static_cast<FeedbackSensor>(std::get<uint32_t>(*sensor)),
              FeedbackSensor::kDetachedRelativeEncoder);
    detachedId = config.GetParam(SparkParameter::kDetachedEncoderDeviceID);
    EXPECT_EQ(std::get<uint32_t>(*detachedId),
              static_cast<uint32_t>(spline1.GetDeviceId()));

    config.SetFeedbackSensor(FeedbackSensor::kPrimaryEncoder);
    sensor = config.GetParam(SparkParameter::kClosedLoopControlSensor);
    EXPECT_EQ(static_cast<FeedbackSensor>(std::get<uint32_t>(*sensor)),
              FeedbackSensor::kPrimaryEncoder);

    config.SetFeedbackSensor(FeedbackSensor::kDetachedAbsoluteEncoder, spline2);
    sensor = config.GetParam(SparkParameter::kClosedLoopControlSensor);
    EXPECT_EQ(static_cast<FeedbackSensor>(std::get<uint32_t>(*sensor)),
              FeedbackSensor::kDetachedAbsoluteEncoder);
    detachedId = config.GetParam(SparkParameter::kDetachedEncoderDeviceID);
    EXPECT_EQ(std::get<uint32_t>(*detachedId),
              static_cast<uint32_t>(spline2.GetDeviceId()));
}

TEST(ClosedLoopConfigTest, Flattens) {
    SubClosedLoopConfig config{};

    config.Pid(10.42, 11.42, 12.42, kSlot1)
        .MinOutput(-0.42)
        .PositionWrappingMaxInput(14.42)
        .IMaxAccum(15.42, kSlot3);

    config.maxMotion.MaxAcceleration(23.42, kSlot1)
        .PositionMode(
            config.maxMotion.MAXMotionPositionMode::kMAXMotionTrapezoidal);

    config.feedForward.kS(42.)
        .kV(43., kSlot1)
        .kA(44., kSlot2)
        .kG(45., kSlot3)
        .kCos(0.123)
        .kCosRatio(0.456, kSlot1);

    std::string flattened{config.Flatten()};

    ValidateFlattenedSubString(flattened, "21,4126B852\n");
    ValidateFlattenedSubString(flattened, "151,4166B852\n");
    ValidateFlattenedSubString(flattened, "23,4146B852\n");
    ValidateFlattenedSubString(flattened, "19,BED70A3D\n");
    ValidateFlattenedSubString(flattened, "108,4176B852\n");
    ValidateFlattenedSubString(flattened, "22,4136B852\n");
    ValidateFlattenedSubString(flattened, "172,41BB5C29\n");
    ValidateFlattenedSubString(flattened, "170,0\n");
    ValidateFlattenedSubString(flattened, "204,42280000\n");
    ValidateFlattenedSubString(flattened, "213,3EE978D5\n");
    ValidateFlattenedSubString(flattened, "215,42300000\n");
    ValidateFlattenedSubString(flattened, "24,422C0000\n");
    ValidateFlattenedSubString(flattened, "221,42340000\n");
    ValidateFlattenedSubString(flattened, "207,3DFBE76D\n");
    EXPECT_EQ(flattened.size(), 0u) << flattened;
}

////////////////////////////////////////////////
