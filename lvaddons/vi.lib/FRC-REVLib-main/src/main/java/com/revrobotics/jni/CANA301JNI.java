/*
 * Copyright (c) 2026 REV Robotics
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

package com.revrobotics.jni;

import com.revrobotics.MutableInt;
import com.revrobotics.spark.A301;
import org.jspecify.annotations.Nullable;

public class CANA301JNI extends RevJNIWrapper {
  public static native int c_A301_RegisterId(
      int busId, int deviceId, MutableInt mutableActualDeviceIdObj);

  public static native long c_A301_Create(int busId, int deviceId, MutableInt mutableStatusObj);

  public static native void c_A301_Close(long handle);

  public static native void c_A301_Destroy(long handle);

  public static native int c_A301_GetFirmwareVersion(long handle);

  public static native int c_A301_SetRelativeEncoderPosition(long handle, float position);

  public static native int c_A301_SetAbsoluteEncoderPosition(long handle, float position);

  public static native int c_A301_SetAbsoluteEncoderRangeOffset(long handle, float offset);

  public static native float c_A301_GetAbsoluteEncoderRangeOffset(long handle);

  public static native int c_A301_SetpointCommand(
      long handle, float value, int ctrlType, float positionSpeed);

  public static native int c_A301_SetIdleMode(long handle, int idleMode);

  public static native int c_A301_GetIdleMode(long handle);

  public static native int c_A301_SetAbsolutePositionContinuousInput(long handle, boolean enabled);

  public static native boolean c_A301_GetAbsolutePositionContinuousInput(long handle);

  public static native int c_A301_SetInverted(long handle, boolean inverted);

  public static native boolean c_A301_GetInverted(long handle);

  public static native int c_A301_ClearFaults(long handle);

  public static native int c_A301_SetStatusFramePeriod(long handle, int frame, int period_ms);

  public static native int c_A301_GetStatusFramePeriod(long handle, int frame);

  public static native A301.@Nullable PeriodicStatus0 c_A301_GetPeriodicStatus0(long handle);

  public static native A301.@Nullable PeriodicStatus1 c_A301_GetPeriodicStatus1(long handle);

  public static native A301.@Nullable PeriodicStatus2 c_A301_GetPeriodicStatus2(long handle);

  public static native A301.@Nullable PeriodicStatus3 c_A301_GetPeriodicStatus3(long handle);
}
