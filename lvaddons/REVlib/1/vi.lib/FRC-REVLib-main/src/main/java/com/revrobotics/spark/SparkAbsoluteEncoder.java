/*
 * Copyright (c) 2018-2026 REV Robotics
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

package com.revrobotics.spark;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.jni.CANSparkJNI;
import com.revrobotics.util.Signal;

public class SparkAbsoluteEncoder implements AbsoluteEncoder {
  // package-private to the revrobotics package
  final SparkBase spark;

  SparkAbsoluteEncoder(SparkBase sparkMax) {
    this.spark = sparkMax;

    // let the driver check if sim is running and create sim if necessary
    CANSparkJNI.c_Spark_CreateAbsoluteEncoderSim(sparkMax.sparkHandle);
  }

  @Override
  public Signal<Double> getPosition() {
    spark.throwIfClosed();
    var status = spark.getPeriodicStatus5();
    return status.map(SparkLowLevel.PeriodicStatus5::dutyCycleEncoderPosition);
  }

  @Override
  public Signal<Double> getVelocity() {
    spark.throwIfClosed();
    var status = spark.getPeriodicStatus5();
    return status.map(SparkLowLevel.PeriodicStatus5::dutyCycleEncoderVelocity);
  }
}
