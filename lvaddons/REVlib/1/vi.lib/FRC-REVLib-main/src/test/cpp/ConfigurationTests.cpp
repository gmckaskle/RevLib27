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

#include "TestUtils.h"
#include "gtest/gtest.h"
#include "rev/config/AbsoluteEncoderConfig.h"
#include "rev/config/AlternateEncoderConfig.h"
#include "rev/config/AnalogSensorConfig.h"
#include "rev/config/ClosedLoopConfig.h"
#include "rev/config/EncoderConfig.h"
#include "rev/config/ExternalEncoderConfig.h"
#include "rev/config/LimitSwitchConfig.h"
#include "rev/config/MAXMotionConfig.h"
#include "rev/config/SignalsConfig.h"
#include "rev/config/SoftLimitConfig.h"
#include "rev/config/SparkFlexConfig.h"
#include "rev/config/SparkMaxConfig.h"

using namespace rev::spark;

////////////////////////////////////////////////

TEST(ConfigurationTest, AbsoluteApply) {
    AbsoluteEncoderConfig first;
    AbsoluteEncoderConfig second;

    first.Inverted(true).PositionConversionFactor(42.).VelocityConversionFactor(
        43.);
    second.Apply(first);

    ValidateFlattenedStrings(first.Flatten(), second.Flatten());
}

TEST(ConfigurationTest, AbsoluteApplyPreset) {
    AbsoluteEncoderConfig first;
    AbsoluteEncoderConfig second;

    first.StartPulseUs(1.0).EndPulseUs(1.0);
    second.Apply(AbsoluteEncoderConfig::Presets::REV_ThroughBoreEncoder());

    ValidateFlattenedStrings(first.Flatten(), second.Flatten());

    first.StartPulseUs(3.88443797).EndPulseUs(1.94221899);
    second.Apply(AbsoluteEncoderConfig::Presets::REV_ThroughBoreEncoderV2());

    ValidateFlattenedStrings(first.Flatten(), second.Flatten());

    first.StartPulseUs(1.0).EndPulseUs(1.0);
    second.Apply(AbsoluteEncoderConfig::Presets::REV_SplineEncoder());

    ValidateFlattenedStrings(first.Flatten(), second.Flatten());
}

TEST(ConfigurationTest, AlternateApply) {
    AlternateEncoderConfig first;
    AlternateEncoderConfig second;

    first.Inverted(true).PositionConversionFactor(42.).VelocityConversionFactor(
        43.);
    second.Apply(first);

    ValidateFlattenedStrings(first.Flatten(), second.Flatten());
}

TEST(ConfigurationTest, AlternateApplyPreset) {
    AlternateEncoderConfig first;
    AlternateEncoderConfig second;

    first.CountsPerRevolution(8192);
    second.Apply(AlternateEncoderConfig::Presets::REV_ThroughBoreEncoder());

    ValidateFlattenedStrings(first.Flatten(), second.Flatten());

    first.CountsPerRevolution(8192);
    second.Apply(AlternateEncoderConfig::Presets::REV_ThroughBoreEncoderV2());

    ValidateFlattenedStrings(first.Flatten(), second.Flatten());

    first.CountsPerRevolution(8192);
    second.Apply(AlternateEncoderConfig::Presets::REV_SplineEncoder());

    ValidateFlattenedStrings(first.Flatten(), second.Flatten());
}

TEST(ConfigurationTest, AnalogApply) {
    AnalogSensorConfig first;
    AnalogSensorConfig second;

    first.Inverted(true).PositionConversionFactor(42.).VelocityConversionFactor(
        43.);
    second.Apply(first);

    ValidateFlattenedStrings(first.Flatten(), second.Flatten());
}

TEST(ConfigurationTest, ClosedLoopApply) {
    ClosedLoopConfig first;
    ClosedLoopConfig second;

    first.Pid(42., 43., 44.);
    first.feedForward.kV(53.);
    first.maxMotion.MaxAcceleration(46).CruiseVelocity(47.);
    second.Apply(first);

    ValidateFlattenedStrings(first.Flatten(), second.Flatten());

    MAXMotionConfig maxMotionConfig;
    maxMotionConfig.MaxAcceleration(50).CruiseVelocity(51.);
    second.Apply(maxMotionConfig);

    ValidateFlattenedStrings(second.maxMotion.Flatten(),
                             maxMotionConfig.Flatten());
}

TEST(ConfigurationTest, EncoderApply) {
    EncoderConfig first;
    EncoderConfig second;

    first.PositionConversionFactor(42.).VelocityConversionFactor(43.);
    second.Apply(first);

    ValidateFlattenedStrings(first.Flatten(), second.Flatten());
}

TEST(ConfigurationTest, EncoderApplyPreset) {
    EncoderConfig first;
    EncoderConfig second;

    first.CountsPerRevolution(8192);
    second.Apply(EncoderConfig::Presets::REV_ThroughBoreEncoder());

    ValidateFlattenedStrings(first.Flatten(), second.Flatten());

    first.CountsPerRevolution(8192);
    second.Apply(EncoderConfig::Presets::REV_ThroughBoreEncoderV2());

    ValidateFlattenedStrings(first.Flatten(), second.Flatten());

    first.CountsPerRevolution(8192);
    second.Apply(EncoderConfig::Presets::REV_SplineEncoder());

    ValidateFlattenedStrings(first.Flatten(), second.Flatten());
}

TEST(ConfigurationTest, ExternalApply) {
    ExternalEncoderConfig first;
    ExternalEncoderConfig second;

    first.PositionConversionFactor(42.).VelocityConversionFactor(43.);
    second.Apply(first);

    ValidateFlattenedStrings(first.Flatten(), second.Flatten());
}

TEST(ConfigurationTest, ExternalApplyPreset) {
    ExternalEncoderConfig first;
    ExternalEncoderConfig second;

    first.CountsPerRevolution(8192);
    second.Apply(ExternalEncoderConfig::Presets::REV_ThroughBoreEncoder());

    ValidateFlattenedStrings(first.Flatten(), second.Flatten());

    first.CountsPerRevolution(8192);
    second.Apply(ExternalEncoderConfig::Presets::REV_ThroughBoreEncoderV2());

    ValidateFlattenedStrings(first.Flatten(), second.Flatten());

    first.CountsPerRevolution(8192);
    second.Apply(ExternalEncoderConfig::Presets::REV_SplineEncoder());

    ValidateFlattenedStrings(first.Flatten(), second.Flatten());
}

TEST(ConfigurationTest, LimitSwitchApply) {
    LimitSwitchConfig first;
    LimitSwitchConfig second;

    first
        .ForwardLimitSwitchTriggerBehavior(
            LimitSwitchConfig::Behavior::kStopMovingMotor)
        .ForwardLimitSwitchType(LimitSwitchConfig::Type::kNormallyClosed)
        .ForwardLimitSwitchPosition(1.234)
        .ReverseLimitSwitchTriggerBehavior(
            LimitSwitchConfig::Behavior::kStopMovingMotorAndSetPosition)
        .ReverseLimitSwitchType(LimitSwitchConfig::Type::kNormallyOpen)
        .ReverseLimitSwitchPosition(100.234)
        .LimitSwitchPositionSensor(FeedbackSensor::kAbsoluteEncoder);
    second.Apply(first);

    ValidateFlattenedStrings(first.Flatten(), second.Flatten());
}

TEST(ConfigurationTest, MaxMotionApply) {
    MAXMotionConfig first;
    MAXMotionConfig second;

    first.MaxAcceleration(42).CruiseVelocity(43.);
    second.Apply(first);

    ValidateFlattenedStrings(first.Flatten(), second.Flatten());
}

TEST(ConfigurationTest, SignalsApply) {
    SignalsConfig first;
    SignalsConfig second;

    first.AbsoluteEncoderPositionAlwaysOn(true)
        .PrimaryEncoderPositionPeriodMs(500)
        .AnalogPositionAlwaysOn(false);
    second.Apply(first);

    ValidateFlattenedStrings(first.Flatten(), second.Flatten());
}

TEST(ConfigurationTest, SoftLimitApply) {
    SoftLimitConfig first;
    SoftLimitConfig second;

    first.ForwardSoftLimit(42.)
        .ReverseSoftLimit(43.)
        .ForwardSoftLimitEnabled(true)
        .ReverseSoftLimitEnabled(true);
    second.Apply(first);

    ValidateFlattenedStrings(first.Flatten(), second.Flatten());
}

TEST(ConfigurationTest, MaxApply) {
    SparkMaxConfig first;
    SparkMaxConfig second;

    first.Inverted(true).SmartCurrentLimit(42);

    first.SetIdleMode(SparkMaxConfig::IdleMode::kBrake).OpenLoopRampRate(43.76);

    first.absoluteEncoder.PositionConversionFactor(43.)
        .VelocityConversionFactor(44.);
    first.alternateEncoder.PositionConversionFactor(45.)
        .VelocityConversionFactor(46.);
    first.analogSensor.PositionConversionFactor(47.).VelocityConversionFactor(
        48.);
    first.encoder.PositionConversionFactor(49.).VelocityConversionFactor(50.);
    first.limitSwitch
        .ForwardLimitSwitchTriggerBehavior(
            LimitSwitchConfig::Behavior::kStopMovingMotor)
        .ForwardLimitSwitchType(LimitSwitchConfig::Type::kNormallyClosed)
        .ForwardLimitSwitchPosition(1.234)
        .ReverseLimitSwitchTriggerBehavior(
            LimitSwitchConfig::Behavior::kStopMovingMotorAndSetPosition)
        .ReverseLimitSwitchType(LimitSwitchConfig::Type::kNormallyOpen)
        .ReverseLimitSwitchPosition(100.234)
        .LimitSwitchPositionSensor(FeedbackSensor::kAbsoluteEncoder);
    first.closedLoop.Pid(49., 50., 51.);
    first.closedLoop.feedForward.kV(52.);
    first.closedLoop.maxMotion.MaxAcceleration(51).CruiseVelocity(52.);
    first.signals.AbsoluteEncoderPositionAlwaysOn(true)
        .PrimaryEncoderPositionPeriodMs(500)
        .AnalogPositionAlwaysOn(false);
    first.softLimit.ForwardSoftLimit(55.)
        .ReverseSoftLimit(56.)
        .ForwardSoftLimitEnabled(true)
        .ReverseSoftLimitEnabled(true);
    second.Apply(first);

    ValidateFlattenedStrings(first.Flatten(), second.Flatten());
}

TEST(ConfigurationTest, MaxApplySubComponents) {
    SparkMaxConfig first;
    SparkMaxConfig second;

    first.Inverted(true).SmartCurrentLimit(42);

    first.SetIdleMode(SparkMaxConfig::IdleMode::kBrake).OpenLoopRampRate(43.76);

    AbsoluteEncoderConfig absConfig;
    absConfig.PositionConversionFactor(43.).VelocityConversionFactor(44.);
    first.Apply(absConfig);

    AlternateEncoderConfig altConfig;
    altConfig.PositionConversionFactor(45.).VelocityConversionFactor(46.);
    first.Apply(altConfig);

    AnalogSensorConfig analogConfig;
    analogConfig.PositionConversionFactor(47.).VelocityConversionFactor(48.);
    first.Apply(analogConfig);

    EncoderConfig encConfig;
    encConfig.PositionConversionFactor(49.).VelocityConversionFactor(50.);
    first.Apply(encConfig);

    LimitSwitchConfig limitConfig;
    limitConfig
        .ForwardLimitSwitchTriggerBehavior(
            LimitSwitchConfig::Behavior::kStopMovingMotor)
        .ForwardLimitSwitchType(LimitSwitchConfig::Type::kNormallyClosed)
        .ForwardLimitSwitchPosition(1.234)
        .ReverseLimitSwitchTriggerBehavior(
            LimitSwitchConfig::Behavior::kStopMovingMotorAndSetPosition)
        .ReverseLimitSwitchType(LimitSwitchConfig::Type::kNormallyOpen)
        .ReverseLimitSwitchPosition(100.234)
        .LimitSwitchPositionSensor(FeedbackSensor::kAbsoluteEncoder);
    first.Apply(limitConfig);

    ClosedLoopConfig closedConfig;
    closedConfig.Pid(49., 50., 51.);
    closedConfig.feedForward.kV(52.);
    closedConfig.maxMotion.MaxAcceleration(51).CruiseVelocity(52.);
    first.Apply(closedConfig);

    SignalsConfig signalsConfig;
    signalsConfig.AbsoluteEncoderPositionAlwaysOn(true)
        .PrimaryEncoderPositionPeriodMs(500)
        .AnalogPositionAlwaysOn(false);
    first.Apply(signalsConfig);

    SoftLimitConfig softConfig;
    softConfig.ForwardSoftLimit(55.)
        .ReverseSoftLimit(56.)
        .ForwardSoftLimitEnabled(true)
        .ReverseSoftLimitEnabled(true);
    first.Apply(softConfig);

    second.Apply(first);

    ValidateFlattenedStrings(first.Flatten(), second.Flatten());
}

TEST(ConfigurationTest, MaxApplyPreset) {
    SparkMaxConfig first;
    SparkMaxConfig second;

    first.SmartCurrentLimit(60);
    second.Apply(SparkMaxConfig::Presets::REV_NEO());

    ValidateFlattenedStrings(first.Flatten(), second.Flatten());

    first.SmartCurrentLimit(15);
    second.Apply(SparkMaxConfig::Presets::REV_NEO_550());

    ValidateFlattenedStrings(first.Flatten(), second.Flatten());
}

TEST(ConfigurationTest, FlexApply) {
    SparkFlexConfig first;
    SparkFlexConfig second;

    first.Inverted(true).SmartCurrentLimit(42);
    first.absoluteEncoder.PositionConversionFactor(43.)
        .VelocityConversionFactor(44.);
    first.closedLoop.Pid(49., 50., 51.);
    first.closedLoop.feedForward.kV(52.);
    first.closedLoop.maxMotion.MaxAcceleration(51).CruiseVelocity(52.);
    first.encoder.PositionConversionFactor(49.).VelocityConversionFactor(50.);
    first.externalEncoder.PositionConversionFactor(45.)
        .VelocityConversionFactor(46.);
    first.limitSwitch
        .ForwardLimitSwitchTriggerBehavior(
            LimitSwitchConfig::Behavior::kStopMovingMotor)
        .ForwardLimitSwitchType(LimitSwitchConfig::Type::kNormallyClosed)
        .ForwardLimitSwitchPosition(1.234)
        .ReverseLimitSwitchTriggerBehavior(
            LimitSwitchConfig::Behavior::kStopMovingMotorAndSetPosition)
        .ReverseLimitSwitchType(LimitSwitchConfig::Type::kNormallyOpen)
        .ReverseLimitSwitchPosition(100.234)
        .LimitSwitchPositionSensor(FeedbackSensor::kAbsoluteEncoder);
    first.signals.AbsoluteEncoderPositionAlwaysOn(true)
        .PrimaryEncoderPositionPeriodMs(500)
        .AnalogPositionAlwaysOn(false);
    first.softLimit.ForwardSoftLimit(55.)
        .ReverseSoftLimit(56.)
        .ForwardSoftLimitEnabled(true)
        .ReverseSoftLimitEnabled(true);
    second.Apply(first);

    ValidateFlattenedStrings(first.Flatten(), second.Flatten());
}

TEST(ConfigurationTest, FlexApplySubComponents) {
    SparkFlexConfig first;
    SparkFlexConfig second;

    first.Inverted(true).SmartCurrentLimit(42);

    first.SetIdleMode(SparkMaxConfig::IdleMode::kBrake).OpenLoopRampRate(43.76);

    AbsoluteEncoderConfig absConfig;
    absConfig.PositionConversionFactor(43.).VelocityConversionFactor(44.);
    first.Apply(absConfig);

    ClosedLoopConfig closedConfig;
    closedConfig.Pid(49., 50., 51.);
    closedConfig.feedForward.kV(52.);
    closedConfig.maxMotion.MaxAcceleration(51).CruiseVelocity(52.);
    first.Apply(closedConfig);

    EncoderConfig encConfig;
    encConfig.PositionConversionFactor(49.).VelocityConversionFactor(50.);
    first.Apply(encConfig);

    ExternalEncoderConfig extConfig;
    extConfig.PositionConversionFactor(45.).VelocityConversionFactor(46.);
    first.Apply(extConfig);

    LimitSwitchConfig limitConfig;
    limitConfig
        .ForwardLimitSwitchTriggerBehavior(
            LimitSwitchConfig::Behavior::kStopMovingMotor)
        .ForwardLimitSwitchType(LimitSwitchConfig::Type::kNormallyClosed)
        .ForwardLimitSwitchPosition(1.234)
        .ReverseLimitSwitchTriggerBehavior(
            LimitSwitchConfig::Behavior::kStopMovingMotorAndSetPosition)
        .ReverseLimitSwitchType(LimitSwitchConfig::Type::kNormallyOpen)
        .ReverseLimitSwitchPosition(100.234)
        .LimitSwitchPositionSensor(FeedbackSensor::kAbsoluteEncoder);
    first.Apply(limitConfig);

    SignalsConfig signalsConfig;
    signalsConfig.AbsoluteEncoderPositionAlwaysOn(true)
        .PrimaryEncoderPositionPeriodMs(500)
        .AnalogPositionAlwaysOn(false);
    first.Apply(signalsConfig);

    SoftLimitConfig softConfig;
    softConfig.ForwardSoftLimit(55.)
        .ReverseSoftLimit(56.)
        .ForwardSoftLimitEnabled(true)
        .ReverseSoftLimitEnabled(true);
    first.Apply(softConfig);

    second.Apply(first);

    ValidateFlattenedStrings(first.Flatten(), second.Flatten());
}

TEST(ConfigurationTest, FlexApplyPreset) {
    SparkFlexConfig first;
    SparkFlexConfig second;

    first.SmartCurrentLimit(60);
    second.Apply(SparkFlexConfig::Presets::REV_NEO());

    ValidateFlattenedStrings(first.Flatten(), second.Flatten());

    first.SmartCurrentLimit(15);
    second.Apply(SparkFlexConfig::Presets::REV_NEO_550());

    ValidateFlattenedStrings(first.Flatten(), second.Flatten());
}
