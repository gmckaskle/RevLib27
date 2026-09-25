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

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertThrows;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;
import com.revrobotics.spark.config.SparkMaxConfig;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class DataPortConfigTests {
  static SparkMax max;
  static SparkFlex flex;
  static SparkMaxConfig maxConfig;

  @BeforeEach
  void setup() {
    max = new SparkMax(0, 1, MotorType.kBrushless);
    flex = new SparkFlex(1, 2, MotorType.kBrushless);
    maxConfig = new SparkMaxConfig();
  }

  @AfterEach
  void teardown() {
    flex.close();
    max.close();
  }

  @Test
  void alternateAndAbsolute() {
    SparkMaxConfig maxConfig = new SparkMaxConfig();

    maxConfig.alternateEncoder.inverted(true);
    maxConfig.absoluteEncoder.inverted(true);

    Assertions.assertThrows(
        IllegalStateException.class,
        () -> {
          max.configure(
              maxConfig,
              com.revrobotics.ResetMode.kResetSafeParameters,
              com.revrobotics.PersistMode.kPersistParameters);
        },
        "Configuring an absolute encoder and an alternate encoder on MAX should throw.");

    Assertions.assertDoesNotThrow(
        () -> {
          flex.configure(
              maxConfig,
              com.revrobotics.ResetMode.kResetSafeParameters,
              com.revrobotics.PersistMode.kPersistParameters);
        },
        "Configuring an absolute encoder and an external encoder on Flex should NOT throw.");
  }

  @Test
  void alternateAndLimits() {
    Assertions.assertThrows(
        IllegalStateException.class,
        () -> {
          SparkMaxConfig maxConfig = new SparkMaxConfig();
          maxConfig.alternateEncoder.inverted(true);
          maxConfig.limitSwitch.forwardLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor);
          max.configure(
              maxConfig,
              com.revrobotics.ResetMode.kResetSafeParameters,
              com.revrobotics.PersistMode.kPersistParameters);
        },
        "Configuring an alternate encoder and forward limit switch on MAX should throw.");

    Assertions.assertThrows(
        IllegalStateException.class,
        () -> {
          SparkMaxConfig maxConfig = new SparkMaxConfig();
          maxConfig.alternateEncoder.inverted(true);
          maxConfig.limitSwitch.reverseLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor);
          max.configure(
              maxConfig,
              com.revrobotics.ResetMode.kResetSafeParameters,
              com.revrobotics.PersistMode.kPersistParameters);
        },
        "Configuring an alternate encoder and reverse limit switch on MAX should throw.");

    Assertions.assertDoesNotThrow(
        () -> {
          SparkMaxConfig maxConfig = new SparkMaxConfig();
          maxConfig.alternateEncoder.inverted(true);
          maxConfig
              .limitSwitch
              .forwardLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor)
              .reverseLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor);
          flex.configure(
              maxConfig,
              com.revrobotics.ResetMode.kResetSafeParameters,
              com.revrobotics.PersistMode.kPersistParameters);
        },
        "Configuring an alternate encoder and reverse limit switch on Flex should NOT throw.");
  }

  @Test
  void absoluteAndLimits() {
    SparkMaxConfig maxConfig = new SparkMaxConfig();
    maxConfig.absoluteEncoder.inverted(true);
    maxConfig
        .limitSwitch
        .forwardLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor)
        .reverseLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor);
    Assertions.assertDoesNotThrow(
        () -> {
          max.configure(
              maxConfig,
              com.revrobotics.ResetMode.kResetSafeParameters,
              com.revrobotics.PersistMode.kPersistParameters);
        },
        "Configuring an absolute encoder and limit switches on MAX should NOT throw.");

    Assertions.assertDoesNotThrow(
        () -> {
          flex.configure(
              maxConfig,
              com.revrobotics.ResetMode.kResetSafeParameters,
              com.revrobotics.PersistMode.kPersistParameters);
        },
        "Configuring an absolute encoder and limit switches on Flex should NOT throw.");
  }

  @Test
  void alternateAndAbsoluteAndLimits() {
    SparkMaxConfig maxConfig = new SparkMaxConfig();
    maxConfig.absoluteEncoder.inverted(true);
    maxConfig
        .limitSwitch
        .forwardLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor)
        .reverseLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor);
    maxConfig.alternateEncoder.inverted(false);

    Assertions.assertThrows(
        IllegalStateException.class,
        () -> {
          max.configure(
              maxConfig,
              com.revrobotics.ResetMode.kResetSafeParameters,
              com.revrobotics.PersistMode.kPersistParameters);
        },
        "Configuring an alternate encoder, an absolute encoder, and limit switches on MAX should throw.");

    Assertions.assertDoesNotThrow(
        () -> {
          flex.configure(
              maxConfig,
              com.revrobotics.ResetMode.kResetSafeParameters,
              com.revrobotics.PersistMode.kPersistParameters);
        },
        "Configuring an alternate encoder, an absolute encoder, and limit switches on Flex should NOT throw.");
  }

  @Test
  void getAbsoluteEncoderBeforeValidConfigure() {
    assertDoesNotThrow(
        () -> {
          max.getAbsoluteEncoder();
        },
        "Getting an absolute encoder before configuring should NOT throw.");

    assertDoesNotThrow(
        () -> {
          maxConfig.absoluteEncoder.setSparkMaxDataPortConfig();
          max.configure(
              maxConfig,
              com.revrobotics.ResetMode.kResetSafeParameters,
              com.revrobotics.PersistMode.kPersistParameters);
        },
        "Configuring an absolute encoder after getting it should NOT throw.");
  }

  @Test
  void getAbsoluteEncoderBeforeInvalidConfigure() {
    assertDoesNotThrow(
        () -> {
          max.getAbsoluteEncoder();
        },
        "Getting an absolute encoder before configuring should NOT throw.");

    assertThrows(
        IllegalStateException.class,
        () -> {
          maxConfig.alternateEncoder.setSparkMaxDataPortConfig();
          max.configure(
              maxConfig,
              com.revrobotics.ResetMode.kResetSafeParameters,
              com.revrobotics.PersistMode.kPersistParameters);
        },
        "Configuring an alternate encoder after getting an absolute encoder should throw.");
  }

  @Test
  void getAbsoluteEncoderAfterValidConfigure() {
    assertDoesNotThrow(
        () -> {
          maxConfig.absoluteEncoder.setSparkMaxDataPortConfig();
          max.configure(
              maxConfig,
              com.revrobotics.ResetMode.kResetSafeParameters,
              com.revrobotics.PersistMode.kPersistParameters);
        },
        "Configuring an absolute encoder should NOT throw.");

    assertDoesNotThrow(
        () -> {
          max.getAbsoluteEncoder();
        },
        "Getting an absolute encoder after configuring should NOT throw.");
  }

  @Test
  void getAbsoluteEncoderAfterInvalidConfigure() {
    assertDoesNotThrow(
        () -> {
          maxConfig.alternateEncoder.setSparkMaxDataPortConfig();
          max.configure(
              maxConfig,
              com.revrobotics.ResetMode.kResetSafeParameters,
              com.revrobotics.PersistMode.kPersistParameters);
        },
        "Configuring an alternate encoder should NOT throw.");

    assertThrows(
        IllegalStateException.class,
        () -> {
          max.getAbsoluteEncoder();
        },
        "Getting an absolute encoder after configuring an alternate encoder should throw.");
  }

  @Test
  void getAlternateEncoderBeforeValidConfigure() {
    assertDoesNotThrow(
        () -> {
          max.getAlternateEncoder();
        },
        "Getting an alternate encoder before configuring should NOT throw.");

    assertDoesNotThrow(
        () -> {
          maxConfig.alternateEncoder.setSparkMaxDataPortConfig();
          max.configure(
              maxConfig,
              com.revrobotics.ResetMode.kResetSafeParameters,
              com.revrobotics.PersistMode.kPersistParameters);
        },
        "Configuring an alternate encoder after getting it should NOT throw.");
  }

  @Test
  void getAlternateEncoderBeforeInvalidConfigure() {
    assertDoesNotThrow(
        () -> {
          max.getAlternateEncoder();
        },
        "Getting an alternate encoder before configuring should NOT throw.");

    assertThrows(
        IllegalStateException.class,
        () -> {
          maxConfig.absoluteEncoder.setSparkMaxDataPortConfig();
          max.configure(
              maxConfig,
              com.revrobotics.ResetMode.kResetSafeParameters,
              com.revrobotics.PersistMode.kPersistParameters);
        },
        "Configuring an absolute encoder after getting an alternate encoder should throw.");

    assertThrows(
        IllegalStateException.class,
        () -> {
          maxConfig = new SparkMaxConfig();
          maxConfig.limitSwitch.setSparkMaxDataPortConfig();
          max.configure(
              maxConfig,
              com.revrobotics.ResetMode.kResetSafeParameters,
              com.revrobotics.PersistMode.kPersistParameters);
        },
        "Configuring limit switches after getting an alternate encoder should throw.");
  }

  @Test
  void getAlternateEncoderAfterValidConfigure() {
    assertDoesNotThrow(
        () -> {
          maxConfig.alternateEncoder.setSparkMaxDataPortConfig();
          max.configure(
              maxConfig,
              com.revrobotics.ResetMode.kResetSafeParameters,
              com.revrobotics.PersistMode.kPersistParameters);
        },
        "Configuring an alternate encoder should NOT throw.");

    assertDoesNotThrow(
        () -> {
          max.getAlternateEncoder();
        },
        "Getting an alternate encoder after configuring should NOT throw.");
  }

  @Test
  void getAlternateEncoderAfterInvalidConfigure() {
    assertDoesNotThrow(
        () -> {
          maxConfig.absoluteEncoder.setSparkMaxDataPortConfig();
          maxConfig.limitSwitch.setSparkMaxDataPortConfig();
          max.configure(
              maxConfig,
              com.revrobotics.ResetMode.kResetSafeParameters,
              com.revrobotics.PersistMode.kPersistParameters);
        },
        "Configuring an absolute encoder and limit switches should NOT throw.");

    assertThrows(
        IllegalStateException.class,
        () -> {
          max.getAlternateEncoder();
        },
        "Getting an alternate encoder after configuring an absolute encoder or limit switches should throw.");
  }

  @Test
  void getLimitSwitchBeforeValidConfigure() {
    assertDoesNotThrow(
        () -> {
          max.getForwardLimitSwitch();
          max.getReverseLimitSwitch();
        },
        "Getting limit switches before configuring should NOT throw.");

    assertDoesNotThrow(
        () -> {
          maxConfig.limitSwitch.setSparkMaxDataPortConfig();
          max.configure(
              maxConfig,
              com.revrobotics.ResetMode.kResetSafeParameters,
              com.revrobotics.PersistMode.kPersistParameters);
        },
        "Configuring limit switches after getting them should NOT throw.");
  }

  @Test
  void getLimitSwitchBeforeInvalidConfigure() {
    assertDoesNotThrow(
        () -> {
          max.getForwardLimitSwitch();
          max.getReverseLimitSwitch();
        },
        "Getting limit switches before configuring should NOT throw.");

    assertThrows(
        IllegalStateException.class,
        () -> {
          maxConfig.alternateEncoder.setSparkMaxDataPortConfig();
          max.configure(
              maxConfig,
              com.revrobotics.ResetMode.kResetSafeParameters,
              com.revrobotics.PersistMode.kPersistParameters);
        },
        "Configuring an alternate encoder after getting limit switches should throw.");
  }

  @Test
  void getLimitSwitchAfterValidConfigure() {
    assertDoesNotThrow(
        () -> {
          maxConfig.limitSwitch.setSparkMaxDataPortConfig();
          max.configure(
              maxConfig,
              com.revrobotics.ResetMode.kResetSafeParameters,
              com.revrobotics.PersistMode.kPersistParameters);
        },
        "Configuring limit switches should NOT throw.");

    assertDoesNotThrow(
        () -> {
          max.getForwardLimitSwitch();
          max.getReverseLimitSwitch();
        },
        "Getting limit switches after configuring should NOT throw.");
  }

  @Test
  void getLimitSwitchAfterInvalidConfigure() {
    assertDoesNotThrow(
        () -> {
          maxConfig.alternateEncoder.setSparkMaxDataPortConfig();
          max.configure(
              maxConfig,
              com.revrobotics.ResetMode.kResetSafeParameters,
              com.revrobotics.PersistMode.kPersistParameters);
        },
        "Configuring an alternate encoder should NOT throw.");

    assertThrows(
        IllegalStateException.class,
        () -> {
          max.getForwardLimitSwitch();
        },
        "Getting limit switches after configuring an alternate encoder should throw.");

    assertThrows(
        IllegalStateException.class,
        () -> {
          max.getReverseLimitSwitch();
        },
        "Getting limit switches after configuring an alternate encoder should throw.");
  }

  @Test
  void flexGetAll() {
    assertDoesNotThrow(
        () -> {
          SparkMaxConfig maxConfig = new SparkMaxConfig();
          maxConfig.alternateEncoder.setSparkMaxDataPortConfig();
          maxConfig.absoluteEncoder.setSparkMaxDataPortConfig();
          maxConfig.limitSwitch.setSparkMaxDataPortConfig();

          flex.configure(
              maxConfig,
              com.revrobotics.ResetMode.kResetSafeParameters,
              com.revrobotics.PersistMode.kPersistParameters);

          flex.getAbsoluteEncoder();
          flex.getExternalEncoder();
          flex.getForwardLimitSwitch();
          flex.getReverseLimitSwitch();
        },
        "Getting all data port config objects on a Flex should NOT throw.");
  }

  @Test
  void incompatibleFeedbackSensor() {
    SparkMaxConfig maxConfig = new SparkMaxConfig();
    maxConfig.closedLoop.feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder);
    assertThrows(
        IllegalStateException.class,
        () -> {
          max.configure(
              maxConfig,
              com.revrobotics.ResetMode.kResetSafeParameters,
              com.revrobotics.PersistMode.kPersistParameters);
        },
        "Setting the feedback sensor to an alternate encoder without configuring the data port from the default should throw.");
    assertDoesNotThrow(
        () -> {
          flex.configure(
              maxConfig,
              com.revrobotics.ResetMode.kResetSafeParameters,
              com.revrobotics.PersistMode.kPersistParameters);
        });

    maxConfig.alternateEncoder.setSparkMaxDataPortConfig();
    maxConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    assertThrows(
        IllegalStateException.class,
        () -> {
          max.configure(
              maxConfig,
              com.revrobotics.ResetMode.kResetSafeParameters,
              com.revrobotics.PersistMode.kPersistParameters);
        },
        "Setting the feedback sensor to an absolute encoder with the data port configured for an alternate encoder should throw.");
    assertDoesNotThrow(
        () -> {
          flex.configure(
              maxConfig,
              com.revrobotics.ResetMode.kResetSafeParameters,
              com.revrobotics.PersistMode.kPersistParameters);
        });
  }
}
