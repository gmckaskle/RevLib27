/*
 * Copyright (c) 2021-2024 REV Robotics
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

import com.revrobotics.jni.CANSparkJNI;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

// TODO(Noah): Unit tests
// TODO(Noah): Test that closing an object causes its encoder to throw an exception if used
// TODO(Noah): Test that initializing a hall sensor does not call setCountsPerRevolution
// TODO(Noah): Make these changes to the C++ version of the code
public class CANSparkMaxTest {
  @Test
  @DisplayName("Cannot use closed CANSparkMax")
  void closedCANSparkMax() {
    CANSparkJNI.c_Spark_GetAPIBuildRevision();
    /*try (CANSparkMax device = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless)) {
        CANEncoder encoder = device.getEncoder();
        device.getBusVoltage();
        encoder.getCountsPerRevolution();
        device.close();
        Assertions.assertThrows(IllegalStateException.class, device::getBusVoltage);
        Assertions.assertThrows(IllegalStateException.class, encoder::getCountsPerRevolution);
    }*/
  }
}
