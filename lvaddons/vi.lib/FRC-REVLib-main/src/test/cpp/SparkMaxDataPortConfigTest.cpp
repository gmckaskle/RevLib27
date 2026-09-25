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
#include "rev/SparkFlex.h"
#include "rev/SparkMax.h"
#include "rev/config/AbsoluteEncoderConfig.h"
#include "rev/config/AlternateEncoderConfig.h"
#include "rev/config/LimitSwitchConfig.h"
#include "rev/config/SparkMaxConfig.h"

using namespace rev;
using namespace rev::spark;

////////////////////////////////////////////////

class SparkMaxTest : public testing::Test {
protected:
    SparkMaxTest()
        : sparkFlex{0, 1, SparkFlex::MotorType::kBrushless},
          sparkMax{0, 2, SparkMax::MotorType::kBrushless} {}
    ~SparkMaxTest() override {}

    SparkFlex sparkFlex;
    SparkMax sparkMax;
};

////////////////////////////////////////////////

TEST_F(SparkMaxTest, AlternateAndAbsoluteThrowOnMax) {
    SparkMaxConfig maxConfig;
    maxConfig.alternateEncoder.Inverted(true);
    maxConfig.absoluteEncoder.Inverted(true);

    EXPECT_THROW(
        {
            sparkMax.Configure(maxConfig, ResetMode::kResetSafeParameters,
                               PersistMode::kPersistParameters);
        },
        std::runtime_error);
}

TEST_F(SparkMaxTest, AlternateAndAbsoluteNoThrowOnFlex) {
    SparkMaxConfig maxConfig;
    maxConfig.alternateEncoder.Inverted(true);
    maxConfig.absoluteEncoder.Inverted(true);

    EXPECT_NO_THROW({
        sparkFlex.Configure(maxConfig, ResetMode::kResetSafeParameters,
                            PersistMode::kPersistParameters);
    });
}

TEST_F(SparkMaxTest, AlternateAndForwardLimitThrowOnMax) {
    EXPECT_THROW(
        {
            SparkMaxConfig maxConfig;
            maxConfig.alternateEncoder.Inverted(true);
            maxConfig.limitSwitch.ForwardLimitSwitchTriggerBehavior(
                LimitSwitchConfig::Behavior::kKeepMovingMotor);

            sparkMax.Configure(maxConfig, ResetMode::kResetSafeParameters,
                               PersistMode::kPersistParameters);
        },
        std::runtime_error);
}

TEST_F(SparkMaxTest, AlternateAndReverseLimitThrowOnMax) {
    EXPECT_THROW(
        {
            SparkMaxConfig maxConfig;
            maxConfig.alternateEncoder.Inverted(true);
            maxConfig.limitSwitch.ReverseLimitSwitchTriggerBehavior(
                LimitSwitchConfig::Behavior::kKeepMovingMotor);

            sparkMax.Configure(maxConfig, ResetMode::kResetSafeParameters,
                               PersistMode::kPersistParameters);
        },
        std::runtime_error);
}

TEST_F(SparkMaxTest, AlternateAndLimitsNoThrowOnFlex) {
    EXPECT_NO_THROW({
        SparkMaxConfig maxConfig;
        maxConfig.alternateEncoder.Inverted(true);
        maxConfig.limitSwitch
            .ForwardLimitSwitchTriggerBehavior(
                LimitSwitchConfig::Behavior::kKeepMovingMotor)
            .ReverseLimitSwitchTriggerBehavior(
                LimitSwitchConfig::Behavior::kKeepMovingMotor);

        sparkFlex.Configure(maxConfig, ResetMode::kResetSafeParameters,
                            PersistMode::kPersistParameters);
    });
}

TEST_F(SparkMaxTest, AbsoluteAndLimits) {
    SparkMaxConfig maxConfig;
    maxConfig.absoluteEncoder.Inverted(true);
    maxConfig.limitSwitch
        .ForwardLimitSwitchTriggerBehavior(
            LimitSwitchConfig::Behavior::kKeepMovingMotor)
        .ReverseLimitSwitchTriggerBehavior(
            LimitSwitchConfig::Behavior::kKeepMovingMotor);

    EXPECT_NO_THROW({
        sparkMax.Configure(maxConfig, ResetMode::kResetSafeParameters,
                           PersistMode::kPersistParameters);
    });

    EXPECT_NO_THROW({
        sparkFlex.Configure(maxConfig, ResetMode::kResetSafeParameters,
                            PersistMode::kPersistParameters);
    });
}

TEST_F(SparkMaxTest, AlternateAndAbsoluteAndLimits) {
    SparkMaxConfig maxConfig;
    maxConfig.alternateEncoder.Inverted(true);
    maxConfig.absoluteEncoder.Inverted(true);
    maxConfig.limitSwitch
        .ForwardLimitSwitchTriggerBehavior(
            LimitSwitchConfig::Behavior::kKeepMovingMotor)
        .ReverseLimitSwitchTriggerBehavior(
            LimitSwitchConfig::Behavior::kKeepMovingMotor);

    EXPECT_THROW(
        {
            sparkMax.Configure(maxConfig, ResetMode::kResetSafeParameters,
                               PersistMode::kPersistParameters);
        },
        std::runtime_error);

    EXPECT_NO_THROW({
        sparkFlex.Configure(maxConfig, ResetMode::kResetSafeParameters,
                            PersistMode::kPersistParameters);
    });
}

TEST_F(SparkMaxTest, GetAbsoluteEncoderBeforeValidConfigureNoThrow) {
    EXPECT_NO_THROW({ sparkMax.GetAbsoluteEncoder(); });

    EXPECT_NO_THROW({
        SparkMaxConfig maxConfig;
        maxConfig.absoluteEncoder.SetSparkMaxDataPortConfig();
        sparkMax.Configure(maxConfig, ResetMode::kResetSafeParameters,
                           PersistMode::kPersistParameters);
    });
}

TEST_F(SparkMaxTest, GetAbsoluteEncoderBeforeInvalidConfigureThrow) {
    EXPECT_NO_THROW({ sparkMax.GetAbsoluteEncoder(); });

    EXPECT_THROW(
        {
            SparkMaxConfig maxConfig;
            maxConfig.alternateEncoder.SetSparkMaxDataPortConfig();
            sparkMax.Configure(maxConfig, ResetMode::kResetSafeParameters,
                               PersistMode::kPersistParameters);
        },
        std::runtime_error);
}

TEST_F(SparkMaxTest, GetAbsoluteEncoderAfterValidConfigureNoThrow) {
    EXPECT_NO_THROW({
        SparkMaxConfig maxConfig;
        maxConfig.absoluteEncoder.SetSparkMaxDataPortConfig();
        sparkMax.Configure(maxConfig, ResetMode::kResetSafeParameters,
                           PersistMode::kPersistParameters);
    });

    EXPECT_NO_THROW({ sparkMax.GetAbsoluteEncoder(); });
}

TEST_F(SparkMaxTest, GetAbsoluteEncoderAfterInvalidConfigureThrow) {
    EXPECT_NO_THROW({
        SparkMaxConfig maxConfig;
        maxConfig.alternateEncoder.SetSparkMaxDataPortConfig();
        sparkMax.Configure(maxConfig, ResetMode::kResetSafeParameters,
                           PersistMode::kPersistParameters);
    });

    EXPECT_THROW({ sparkMax.GetAbsoluteEncoder(); }, std::runtime_error);
}

TEST_F(SparkMaxTest, GetAlternateEncoderBeforeValidConfigureNoThrow) {
    EXPECT_NO_THROW({ sparkMax.GetAlternateEncoder(); });

    EXPECT_NO_THROW({
        SparkMaxConfig maxConfig;
        maxConfig.alternateEncoder.SetSparkMaxDataPortConfig();
        sparkMax.Configure(maxConfig, ResetMode::kResetSafeParameters,
                           PersistMode::kPersistParameters);
    });
}

TEST_F(SparkMaxTest, GetAlternateEncoderBeforeInvalidConfigureThrow) {
    EXPECT_NO_THROW({ sparkMax.GetAlternateEncoder(); });

    EXPECT_THROW(
        {
            SparkMaxConfig maxConfig;
            maxConfig.absoluteEncoder.SetSparkMaxDataPortConfig();
            sparkMax.Configure(maxConfig, ResetMode::kResetSafeParameters,
                               PersistMode::kPersistParameters);
        },
        std::runtime_error);

    EXPECT_THROW(
        {
            SparkMaxConfig maxConfig;
            maxConfig.limitSwitch.SetSparkMaxDataPortConfig();
            sparkMax.Configure(maxConfig, ResetMode::kResetSafeParameters,
                               PersistMode::kPersistParameters);
        },
        std::runtime_error);
}

TEST_F(SparkMaxTest, GetAlternateEncoderAfterValidConfigureNoThrow) {
    EXPECT_NO_THROW({
        SparkMaxConfig maxConfig;
        maxConfig.alternateEncoder.SetSparkMaxDataPortConfig();
        sparkMax.Configure(maxConfig, ResetMode::kResetSafeParameters,
                           PersistMode::kPersistParameters);
    });

    EXPECT_NO_THROW({ sparkMax.GetAlternateEncoder(); });
}

TEST_F(SparkMaxTest, GetAlternateEncoderAfterInvalidConfigureThrow) {
    EXPECT_NO_THROW({
        SparkMaxConfig maxConfig;
        maxConfig.absoluteEncoder.SetSparkMaxDataPortConfig();
        maxConfig.limitSwitch.SetSparkMaxDataPortConfig();
        sparkMax.Configure(maxConfig, ResetMode::kResetSafeParameters,
                           PersistMode::kPersistParameters);
    });

    EXPECT_THROW({ sparkMax.GetAlternateEncoder(); }, std::runtime_error);
}

TEST_F(SparkMaxTest, GetLimitSwitchBeforeValidConfigureNoThrow) {
    EXPECT_NO_THROW({
        sparkMax.GetForwardLimitSwitch();
        sparkMax.GetReverseLimitSwitch();
    });

    EXPECT_NO_THROW({
        SparkMaxConfig maxConfig;
        maxConfig.limitSwitch.SetSparkMaxDataPortConfig();
        sparkMax.Configure(maxConfig, ResetMode::kResetSafeParameters,
                           PersistMode::kPersistParameters);
    });
}

TEST_F(SparkMaxTest, GetLimitSwitchBeforeInvalidConfigureThrow) {
    EXPECT_NO_THROW({
        sparkMax.GetForwardLimitSwitch();
        sparkMax.GetReverseLimitSwitch();
    });

    EXPECT_THROW(
        {
            SparkMaxConfig maxConfig;
            maxConfig.alternateEncoder.SetSparkMaxDataPortConfig();
            sparkMax.Configure(maxConfig, ResetMode::kResetSafeParameters,
                               PersistMode::kPersistParameters);
        },
        std::runtime_error);
}

TEST_F(SparkMaxTest, GetLimitSwitchAfterValidConfigureNoThrow) {
    EXPECT_NO_THROW({
        SparkMaxConfig maxConfig;
        maxConfig.limitSwitch.SetSparkMaxDataPortConfig();
        sparkMax.Configure(maxConfig, ResetMode::kResetSafeParameters,
                           PersistMode::kPersistParameters);
    });

    EXPECT_NO_THROW({
        sparkMax.GetForwardLimitSwitch();
        sparkMax.GetReverseLimitSwitch();
    });
}

TEST_F(SparkMaxTest, GetLimitSwitchAfterInvalidConfigureThrow) {
    EXPECT_NO_THROW({
        SparkMaxConfig maxConfig;
        maxConfig.alternateEncoder.SetSparkMaxDataPortConfig();
        sparkMax.Configure(maxConfig, ResetMode::kResetSafeParameters,
                           PersistMode::kPersistParameters);
    });

    EXPECT_THROW({ sparkMax.GetForwardLimitSwitch(); }, std::runtime_error);

    EXPECT_THROW({ sparkMax.GetReverseLimitSwitch(); }, std::runtime_error);
}

TEST_F(SparkMaxTest, FlexGetAllNoThrow) {
    EXPECT_NO_THROW({
        SparkMaxConfig maxConfig;
        maxConfig.absoluteEncoder.SetSparkMaxDataPortConfig();
        maxConfig.alternateEncoder.SetSparkMaxDataPortConfig();
        maxConfig.limitSwitch.SetSparkMaxDataPortConfig();

        sparkFlex.Configure(maxConfig, ResetMode::kResetSafeParameters,
                            PersistMode::kPersistParameters);

        sparkFlex.GetAbsoluteEncoder();
        sparkFlex.GetExternalEncoder();
        sparkMax.GetForwardLimitSwitch();
        sparkMax.GetReverseLimitSwitch();
    });
}
