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

package com.revrobotics;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import com.revrobotics.spark.config.AnalogSensorConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.ExternalEncoderConfig;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SignalsConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import org.junit.jupiter.api.Test;

public class ConfigurationTests {
  @Test
  void absoluteApply() {
    AbsoluteEncoderConfig first = new AbsoluteEncoderConfig();
    AbsoluteEncoderConfig second = new AbsoluteEncoderConfig();

    first.inverted(true).positionConversionFactor(42.).velocityConversionFactor(43.);
    second.apply(first);

    assertEquals(first.flatten(), second.flatten());
  }

  @Test
  void absoluteApplyPreset() {
    AbsoluteEncoderConfig first = new AbsoluteEncoderConfig();
    AbsoluteEncoderConfig second = new AbsoluteEncoderConfig();

    first.startPulseUs(1.0).endPulseUs(1.0);
    second.apply(AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoder);

    assertEquals(first.flatten(), second.flatten());

    first.startPulseUs(3.88443797).endPulseUs(1.94221899);
    second.apply(AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoderV2);

    assertEquals(first.flatten(), second.flatten());

    first.startPulseUs(1.0).endPulseUs(1.0);
    second.apply(AbsoluteEncoderConfig.Presets.REV_SplineEncoder);

    assertEquals(first.flatten(), second.flatten());
  }

  @Test
  void alternateApply() {
    AlternateEncoderConfig first = new AlternateEncoderConfig();
    AlternateEncoderConfig second = new AlternateEncoderConfig();

    first.inverted(true).positionConversionFactor(42.).velocityConversionFactor(43.);
    second.apply(first);

    assertEquals(first.flatten(), second.flatten());
  }

  @Test
  void alternateApplyPreset() {
    AlternateEncoderConfig first = new AlternateEncoderConfig();
    AlternateEncoderConfig second = new AlternateEncoderConfig();

    first.countsPerRevolution(8192);
    second.apply(AlternateEncoderConfig.Presets.REV_ThroughBoreEncoder);

    assertEquals(first.flatten(), second.flatten());

    first.countsPerRevolution(8192);
    second.apply(AlternateEncoderConfig.Presets.REV_ThroughBoreEncoderV2);

    assertEquals(first.flatten(), second.flatten());

    first.countsPerRevolution(8192);
    second.apply(AlternateEncoderConfig.Presets.REV_SplineEncoder);

    assertEquals(first.flatten(), second.flatten());
  }

  @Test
  void analogApply() {
    AnalogSensorConfig first = new AnalogSensorConfig();
    AnalogSensorConfig second = new AnalogSensorConfig();

    first.inverted(true).positionConversionFactor(42.).velocityConversionFactor(43.);
    second.apply(first);

    assertEquals(first.flatten(), second.flatten());
  }

  @Test
  void closedLoopApply() {
    ClosedLoopConfig first = new ClosedLoopConfig();
    ClosedLoopConfig second = new ClosedLoopConfig();

    first.pid(42., 43., 44.).feedForward.kV(45.);
    first.maxMotion.maxAcceleration(46).cruiseVelocity(47.);
    second.apply(first);

    assertEquals(first.flatten(), second.flatten());

    MAXMotionConfig maxMotionConfig = new MAXMotionConfig();
    maxMotionConfig.maxAcceleration(50).cruiseVelocity(51.);
    second.apply(maxMotionConfig);

    assertEquals(second.maxMotion.flatten(), maxMotionConfig.flatten());
  }

  @Test
  void encoderApply() {
    EncoderConfig first = new EncoderConfig();
    EncoderConfig second = new EncoderConfig();

    first.positionConversionFactor(42.).velocityConversionFactor(43.);
    second.apply(first);

    assertEquals(first.flatten(), second.flatten());
  }

  @Test
  void encoderApplyPreset() {
    EncoderConfig first = new EncoderConfig();
    EncoderConfig second = new EncoderConfig();

    first.countsPerRevolution(8192);
    second.apply(EncoderConfig.Presets.REV_ThroughBoreEncoder);

    assertEquals(first.flatten(), second.flatten());

    first.countsPerRevolution(8192);
    second.apply(EncoderConfig.Presets.REV_ThroughBoreEncoderV2);

    assertEquals(first.flatten(), second.flatten());

    first.countsPerRevolution(8192);
    second.apply(EncoderConfig.Presets.REV_SplineEncoder);

    assertEquals(first.flatten(), second.flatten());
  }

  @Test
  void externalApply() {
    ExternalEncoderConfig first = new ExternalEncoderConfig();
    ExternalEncoderConfig second = new ExternalEncoderConfig();

    first.positionConversionFactor(42.).velocityConversionFactor(43.);
    second.apply(first);

    assertEquals(first.flatten(), second.flatten());
  }

  @Test
  void ExternalApplyPreset() {
    ExternalEncoderConfig first = new ExternalEncoderConfig();
    ExternalEncoderConfig second = new ExternalEncoderConfig();

    first.countsPerRevolution(8192);
    second.apply(ExternalEncoderConfig.Presets.REV_ThroughBoreEncoder);

    assertEquals(first.flatten(), second.flatten());

    first.countsPerRevolution(8192);
    second.apply(ExternalEncoderConfig.Presets.REV_ThroughBoreEncoderV2);

    assertEquals(first.flatten(), second.flatten());

    first.countsPerRevolution(8192);
    second.apply(ExternalEncoderConfig.Presets.REV_SplineEncoder);

    assertEquals(first.flatten(), second.flatten());
  }

  @Test
  void limitSwitchApply() {
    LimitSwitchConfig first = new LimitSwitchConfig();
    LimitSwitchConfig second = new LimitSwitchConfig();

    first
        .forwardLimitSwitchTriggerBehavior(Behavior.kStopMovingMotor)
        .forwardLimitSwitchType(Type.kNormallyClosed);
    first
        .reverseLimitSwitchTriggerBehavior(Behavior.kStopMovingMotor)
        .reverseLimitSwitchType(Type.kNormallyOpen);
    second.apply(first);

    assertEquals(first.flatten(), second.flatten());
  }

  @Test
  void maxMotionApply() {
    MAXMotionConfig first = new MAXMotionConfig();
    MAXMotionConfig second = new MAXMotionConfig();

    first.maxAcceleration(42).cruiseVelocity(43.);
    second.apply(first);

    assertEquals(first.flatten(), second.flatten());
  }

  @Test
  void signalsApply() {
    SignalsConfig first = new SignalsConfig();
    SignalsConfig second = new SignalsConfig();

    first
        .absoluteEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(500)
        .analogPositionAlwaysOn(false);
    second.apply(first);

    assertEquals(first.flatten(), second.flatten());
  }

  @Test
  void softLimitApply() {
    SoftLimitConfig first = new SoftLimitConfig();
    SoftLimitConfig second = new SoftLimitConfig();

    first
        .forwardSoftLimit(42.)
        .reverseSoftLimit(43.)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimitEnabled(true);
    second.apply(first);

    assertEquals(first.flatten(), second.flatten());
  }

  @Test
  void maxApply() {
    SparkMaxConfig first = new SparkMaxConfig();
    SparkMaxConfig second = new SparkMaxConfig();

    first.inverted(true).smartCurrentLimit(42);
    first.absoluteEncoder.positionConversionFactor(43.).velocityConversionFactor(44.);
    first.alternateEncoder.positionConversionFactor(45.).velocityConversionFactor(46.);
    first.analogSensor.positionConversionFactor(47.).velocityConversionFactor(48.);
    first.encoder.positionConversionFactor(49.).velocityConversionFactor(50.);
    first
        .limitSwitch
        .forwardLimitSwitchTriggerBehavior(Behavior.kStopMovingMotor)
        .forwardLimitSwitchType(Type.kNormallyClosed)
        .forwardLimitSwitchPosition(1.234)
        .reverseLimitSwitchTriggerBehavior(Behavior.kStopMovingMotorAndSetPosition)
        .reverseLimitSwitchType(Type.kNormallyOpen)
        .reverseLimitSwitchPosition(100.234)
        .limitSwitchPositionSensor(FeedbackSensor.kAbsoluteEncoder);
    first.closedLoop.pid(49., 50., 51.).feedForward.kV(52.);
    first.closedLoop.maxMotion.maxAcceleration(51).cruiseVelocity(52.);
    first
        .signals
        .absoluteEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(500)
        .analogPositionAlwaysOn(false);
    first
        .softLimit
        .forwardSoftLimit(55.)
        .reverseSoftLimit(56.)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimitEnabled(true);
    second.apply(first);

    assertEquals(first.flatten(), second.flatten());
  }

  @Test
  void flexApply() {
    SparkFlexConfig first = new SparkFlexConfig();
    SparkFlexConfig second = new SparkFlexConfig();

    first.inverted(true).smartCurrentLimit(42);
    first.absoluteEncoder.positionConversionFactor(43.).velocityConversionFactor(44.);
    first.closedLoop.pid(49., 50., 51.).feedForward.kV(52.);
    first.closedLoop.maxMotion.maxAcceleration(51).cruiseVelocity(52.);
    first.encoder.positionConversionFactor(49.).velocityConversionFactor(50.);
    first.externalEncoder.positionConversionFactor(45.).velocityConversionFactor(46.);
    first
        .limitSwitch
        .forwardLimitSwitchTriggerBehavior(Behavior.kStopMovingMotor)
        .forwardLimitSwitchType(Type.kNormallyClosed)
        .forwardLimitSwitchPosition(1.234)
        .reverseLimitSwitchTriggerBehavior(Behavior.kStopMovingMotorAndSetPosition)
        .reverseLimitSwitchType(Type.kNormallyOpen)
        .reverseLimitSwitchPosition(100.234)
        .limitSwitchPositionSensor(FeedbackSensor.kAbsoluteEncoder);
    first
        .signals
        .absoluteEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(500)
        .analogPositionAlwaysOn(false);
    first
        .softLimit
        .forwardSoftLimit(55.)
        .reverseSoftLimit(56.)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimitEnabled(true);
    second.apply(first);

    assertEquals(first.flatten(), second.flatten());
  }
}
