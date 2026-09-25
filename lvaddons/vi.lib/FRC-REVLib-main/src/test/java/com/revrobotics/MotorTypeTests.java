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

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

public class MotorTypeTests {
  SparkMax max;

  @AfterEach
  void testEnd() {
    max.close();
  }

  @Test
  void brushlessEncoder() {
    max = new SparkMax(0, 1, MotorType.kBrushless);
    assertThrows(
        IllegalStateException.class,
        () -> {
          SparkMaxConfig maxConfig = new SparkMaxConfig();
          maxConfig.encoder.inverted(true);
          max.configure(maxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        },
        "Inverting an encoder while in brushless mode should throw.");
    assertThrows(
        IllegalStateException.class,
        () -> {
          SparkMaxConfig maxConfig = new SparkMaxConfig();
          maxConfig.encoder.countsPerRevolution(42);
          max.configure(maxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        },
        "Setting CPR while in brushless mode should throw.");
  }

  @Test
  void brushedEncoder() {
    max = new SparkMax(0, 1, MotorType.kBrushed);
    assertDoesNotThrow(
        () -> {
          SparkMaxConfig maxConfig = new SparkMaxConfig();
          maxConfig.encoder.inverted(true);
          maxConfig.encoder.countsPerRevolution(42);
          max.configure(maxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        },
        "Inverting an encoder or setting CPR while in brushed mode should NOT throw.");
  }
}
