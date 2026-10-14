/*
 * Copyright (c) 2020-2026 REV Robotics
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

#include <fmt/format.h>

#include "com_revrobotics_jni_CANA301JNI.h"
#include "first/CANA301Driver.h"

extern "C" {

static jclass periodicStatus0Clazz;
static jclass periodicStatus1Clazz;
static jclass periodicStatus2Clazz;
static jclass periodicStatus3Clazz;

static jclass loadClassWithGlobalRef(JNIEnv* env, std::string name) {
    jclass localClazz = env->FindClass(name.c_str());

    auto result = reinterpret_cast<jclass>(env->NewGlobalRef(localClazz));
    env->DeleteLocalRef(localClazz);

    return result;
}

void CANA301JNI_OnLoad(JNIEnv* env) {
    periodicStatus0Clazz = loadClassWithGlobalRef(
        env, "com/revrobotics/spark/A301$PeriodicStatus0");
    periodicStatus1Clazz = loadClassWithGlobalRef(
        env, "com/revrobotics/spark/A301$PeriodicStatus1");
    periodicStatus2Clazz = loadClassWithGlobalRef(
        env, "com/revrobotics/spark/A301$PeriodicStatus2");
    periodicStatus3Clazz = loadClassWithGlobalRef(
        env, "com/revrobotics/spark/A301$PeriodicStatus3");
}

void CANA301JNI_OnUnLoad(JNIEnv* env) {
    env->DeleteGlobalRef(periodicStatus0Clazz);
    env->DeleteGlobalRef(periodicStatus1Clazz);
    env->DeleteGlobalRef(periodicStatus2Clazz);
    env->DeleteGlobalRef(periodicStatus3Clazz);
}

/*
 * Class:     com_revrobotics_jni_CANA301JNI_c_1A301
 * Method:    1RegisterId
 * Signature: (IILjava/lang/Object;)I
 */
JNIEXPORT jint JNICALL
Java_com_revrobotics_jni_CANA301JNI_c_1A301_1RegisterId
  (JNIEnv* env, jclass, jint busId, jint deviceId,
   jobject mutableActualDeviceIdObj)
{
    int actualDeviceId;
    jint status = (jint)c_A301_RegisterId(busId, deviceId, &actualDeviceId);

    jclass clazz = env->GetObjectClass(mutableActualDeviceIdObj);
    jfieldID fieldID = env->GetFieldID(clazz, "value", "I");
    env->SetIntField(mutableActualDeviceIdObj, fieldID,
                     static_cast<jint>(actualDeviceId));

    return status;
}

/*
 * Class:     com_revrobotics_jni_CANA301JNI_c_1A301
 * Method:    1Create
 * Signature: (IILjava/lang/Object;)J
 */
JNIEXPORT jlong JNICALL
Java_com_revrobotics_jni_CANA301JNI_c_1A301_1Create
  (JNIEnv* env, jclass, jint busId, jint deviceID, jobject mutableStatusObj)
{
    c_REVLib_ErrorCode status;
    jlong handle = (jlong)c_A301_Create(busId, deviceID, &status);

    jclass clazz = env->GetObjectClass(mutableStatusObj);
    jfieldID fieldID = env->GetFieldID(clazz, "value", "I");
    env->SetIntField(mutableStatusObj, fieldID, static_cast<jint>(status));

    return handle;
}

/*
 * Class:     com_revrobotics_jni_CANA301JNI_c_1A301
 * Method:    1Close
 * Signature: (J)V
 */
JNIEXPORT void JNICALL
Java_com_revrobotics_jni_CANA301JNI_c_1A301_1Close
  (JNIEnv*, jclass, jlong handle)
{
    c_A301_Close((c_A301_handle)handle);
}

/*
 * Class:     com_revrobotics_jni_CANA301JNI_c_1A301
 * Method:    1Destroy
 * Signature: (J)V
 */
JNIEXPORT void JNICALL
Java_com_revrobotics_jni_CANA301JNI_c_1A301_1Destroy
  (JNIEnv*, jclass, jlong handle)
{
    c_A301_Destroy((c_A301_handle)handle);
}

/*
 * Class:     com_revrobotics_jni_CANA301JNI_c_1A301
 * Method:    1GetFirmwareVersion
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_revrobotics_jni_CANA301JNI_c_1A301_1GetFirmwareVersion
  (JNIEnv*, jclass, jlong handle)
{
    c_A301_FirmwareVersion fwVersion;
    c_A301_GetFirmwareVersion((c_A301_handle)handle, &fwVersion);
    return fwVersion.versionRaw;
}

/*
 * Class:     com_revrobotics_jni_CANA301JNI_c_1A301
 * Method:    1SetRelativeEncoderPosition
 * Signature: (JF)I
 */
JNIEXPORT jint JNICALL
Java_com_revrobotics_jni_CANA301JNI_c_1A301_1SetRelativeEncoderPosition
  (JNIEnv*, jclass, jlong handle, jfloat position)
{
    return (jint)c_A301_SetRelativeEncoderPosition((c_A301_handle)handle,
                                                   position);
}

/*
 * Class:     com_revrobotics_jni_CANA301JNI_c_1A301
 * Method:    1SetAbsoluteEncoderPosition
 * Signature: (JF)I
 */
JNIEXPORT jint JNICALL
Java_com_revrobotics_jni_CANA301JNI_c_1A301_1SetAbsoluteEncoderPosition
  (JNIEnv*, jclass, jlong handle, jfloat position)
{
    return (jint)c_A301_SetAbsoluteEncoderPosition((c_A301_handle)handle,
                                                   position);
}

/*
 * Class:     com_revrobotics_jni_CANA301JNI_c_1A301
 * Method:    1SetAbsoluteEncoderRangeOffset
 * Signature: (JF)I
 */
JNIEXPORT jint JNICALL
Java_com_revrobotics_jni_CANA301JNI_c_1A301_1SetAbsoluteEncoderRangeOffset
  (JNIEnv*, jclass, jlong handle, jfloat offset)
{
    return (jint)c_A301_SetAbsoluteEncoderRangeOffset((c_A301_handle)handle,
                                                      offset);
}

/*
 * Class:     com_revrobotics_jni_CANA301JNI_c_1A301
 * Method:    1GetAbsoluteEncoderRangeOffset
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL
Java_com_revrobotics_jni_CANA301JNI_c_1A301_1GetAbsoluteEncoderRangeOffset
  (JNIEnv*, jclass, jlong handle)
{
    float offset;
    c_A301_GetAbsoluteEncoderRangeOffset((c_A301_handle)handle, &offset);
    return (jfloat)offset;
}

/*
 * Class:     com_revrobotics_jni_CANA301JNI_c_1A301
 * Method:    1SetpointCommand
 * Signature: (JFIF)I
 */
JNIEXPORT jint JNICALL
Java_com_revrobotics_jni_CANA301JNI_c_1A301_1SetpointCommand
  (JNIEnv*, jclass, jlong handle, jfloat value, jint ctrl, jfloat positionSpeed)
{
    return (jint)c_A301_SetpointCommand(
        (c_A301_handle)handle, value, (c_A301_ControlType)ctrl, positionSpeed);
}

/*
 * Class:     com_revrobotics_jni_CANA301JNI_c_1A301
 * Method:    1SetIdleMode
 * Signature: (JI)I
 */
JNIEXPORT jint JNICALL
Java_com_revrobotics_jni_CANA301JNI_c_1A301_1SetIdleMode
  (JNIEnv*, jclass, jlong handle, jint idleMode)
{
    return (jint)c_A301_SetIdleMode((c_A301_handle)handle, idleMode);
}

/*
 * Class:     com_revrobotics_jni_CANA301JNI_c_1A301
 * Method:    1GetIdleMode
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_revrobotics_jni_CANA301JNI_c_1A301_1GetIdleMode
  (JNIEnv*, jclass, jlong handle)
{
    uint8_t idleMode;
    c_A301_GetIdleMode((c_A301_handle)handle, &idleMode);
    return (jint)idleMode;
}

/*
 * Class:     com_revrobotics_jni_CANA301JNI_c_1A301
 * Method:    1SetAbsolutePositionContinuousInput
 * Signature: (JZ)I
 */
JNIEXPORT jint JNICALL
Java_com_revrobotics_jni_CANA301JNI_c_1A301_1SetAbsolutePositionContinuousInput
  (JNIEnv*, jclass, jlong handle, jboolean enabled)
{
    return (jint)c_A301_SetAbsolutePositionContinuousInput(
        (c_A301_handle)handle, enabled);
}

/*
 * Class:     com_revrobotics_jni_CANA301JNI_c_1A301
 * Method:    1GetAbsolutePositionContinuousInput
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL
Java_com_revrobotics_jni_CANA301JNI_c_1A301_1GetAbsolutePositionContinuousInput
  (JNIEnv*, jclass, jlong handle)
{
    uint8_t enabled;
    c_A301_GetAbsolutePositionContinuousInput((c_A301_handle)handle, &enabled);
    return (jboolean)enabled;
}

/*
 * Class:     com_revrobotics_jni_CANA301JNI_c_1A301
 * Method:    1SetInverted
 * Signature: (JZ)I
 */
JNIEXPORT jint JNICALL
Java_com_revrobotics_jni_CANA301JNI_c_1A301_1SetInverted
  (JNIEnv*, jclass, jlong handle, jboolean inverted)
{
    return (jint)c_A301_SetInverted((c_A301_handle)handle, inverted ? 1 : 0);
}

/*
 * Class:     com_revrobotics_jni_CANA301JNI_c_1A301
 * Method:    1GetInverted
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL
Java_com_revrobotics_jni_CANA301JNI_c_1A301_1GetInverted
  (JNIEnv*, jclass, jlong handle)
{
    uint8_t inverted;
    c_A301_GetInverted((c_A301_handle)handle, &inverted);
    return (jboolean)inverted;
}

/*
 * Class:     com_revrobotics_jni_CANA301JNI_c_1A301
 * Method:    1ClearFaults
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_revrobotics_jni_CANA301JNI_c_1A301_1ClearFaults
  (JNIEnv*, jclass, jlong handle)
{
    return (jint)c_A301_ClearFaults((c_A301_handle)handle);
}

/*
 * Class:     com_revrobotics_jni_CANA301JNI_c_1A301
 * Method:    1SetStatusFramePeriod
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL
Java_com_revrobotics_jni_CANA301JNI_c_1A301_1SetStatusFramePeriod
  (JNIEnv*, jclass, jlong handle, jint frame, jint period_ms)
{
    return (jint)c_A301_SetStatusFramePeriod(
        (c_A301_handle)handle, (c_A301_PeriodicFrame)frame, period_ms);
}

/*
 * Class:     com_revrobotics_jni_CANA301JNI_c_1A301
 * Method:    1GetStatusFramePeriod
 * Signature: (JI)I
 */
JNIEXPORT jint JNICALL
Java_com_revrobotics_jni_CANA301JNI_c_1A301_1GetStatusFramePeriod
  (JNIEnv*, jclass, jlong handle, jint frame)
{
    uint32_t period_ms{};
    c_A301_GetStatusFramePeriod((c_A301_handle)handle,
                                (c_A301_PeriodicFrame)frame, &period_ms);
    return (jint)period_ms;
}

/*
 * Class:     com_revrobotics_jni_CANA301JNI_c_1A301
 * Method:    1GetPeriodicStatus0
 * Signature: (J)Ljava/lang/Object;
 */
JNIEXPORT jobject JNICALL
Java_com_revrobotics_jni_CANA301JNI_c_1A301_1GetPeriodicStatus0
  (JNIEnv* env, jclass, jlong handle)
{
    if (periodicStatus0Clazz == nullptr) {
        CANA301JNI_OnLoad(env);
    }
    c_A301_PeriodicStatus0 frame;
    c_REVLib_ErrorCode result =
        c_A301_GetPeriodicStatus0((c_A301_handle)handle, &frame);

    const auto constructor =
        env->GetMethodID(periodicStatus0Clazz, "<init>", "(DDDIZZIIJ)V");
    jobject o = env->NewObject(
        periodicStatus0Clazz, constructor, frame.appliedOutput, frame.voltage,
        frame.current, frame.motorTemperature, frame.inverted,
        frame.primaryHeartbeatLock, frame.gearboxRPM, result, frame.timestamp);

    return o;
}

/*
 * Class:     com_revrobotics_jni_CANA301JNI_c_1A301
 * Method:    1GetPeriodicStatus1
 * Signature: (J)Ljava/lang/Object;
 */
JNIEXPORT jobject JNICALL
Java_com_revrobotics_jni_CANA301JNI_c_1A301_1GetPeriodicStatus1
  (JNIEnv* env, jclass, jlong handle)
{
    if (periodicStatus1Clazz == nullptr) {
        CANA301JNI_OnLoad(env);
    }
    c_A301_PeriodicStatus1 frame;
    c_REVLib_ErrorCode result =
        c_A301_GetPeriodicStatus1((c_A301_handle)handle, &frame);

    const auto constructor =
        env->GetMethodID(periodicStatus1Clazz, "<init>",
                         "(ZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZIJ)V");

    jobject o = env->NewObject(
        periodicStatus1Clazz, constructor, frame.otherFault,
        frame.motorTypeFault, frame.sensorFault, frame.canFault,
        frame.temperatureFault, frame.drvFault, frame.escEepromFault,
        frame.firmwareFault, frame.brownoutWarning, frame.overcurrentWarning,
        frame.escEepromWarning, frame.extEepromWarning, frame.sensorWarning,
        frame.stallWarning, frame.hasResetWarning, frame.otherWarning,
        frame.otherStickyFault, frame.motorTypeStickyFault,
        frame.sensorStickyFault, frame.canStickyFault,
        frame.temperatureStickyFault, frame.drvStickyFault,
        frame.escEepromStickyFault, frame.firmwareStickyFault,
        frame.brownoutStickyWarning, frame.overcurrentStickyWarning,
        frame.escEepromStickyWarning, frame.extEepromStickyWarning,
        frame.sensorStickyWarning, frame.stallStickyWarning,
        frame.hasResetStickyWarning, frame.otherStickyWarning, JNI_FALSE,
        result, frame.timestamp);
    return o;
}

/*
 * Class:     com_revrobotics_jni_CANA301JNI_c_1A301
 * Method:    1GetPeriodicStatus2
 * Signature: (J)Ljava/lang/Object;
 */
JNIEXPORT jobject JNICALL
Java_com_revrobotics_jni_CANA301JNI_c_1A301_1GetPeriodicStatus2
  (JNIEnv* env, jclass, jlong handle)
{
    if (periodicStatus2Clazz == nullptr) {
        CANA301JNI_OnLoad(env);
    }
    c_A301_PeriodicStatus2 frame;
    c_REVLib_ErrorCode result =
        c_A301_GetPeriodicStatus2((c_A301_handle)handle, &frame);

    const auto constructor =
        env->GetMethodID(periodicStatus2Clazz, "<init>", "(DDIJ)V");

    jobject o =
        env->NewObject(periodicStatus2Clazz, constructor, frame.encoderVelocity,
                       frame.relativeEncoderPosition, result, frame.timestamp);
    return o;
}

/*
 * Class:     com_revrobotics_jni_CANA301JNI_c_1A301
 * Method:    1GetPeriodicStatus3
 * Signature: (J)Ljava/lang/Object;
 */
JNIEXPORT jobject JNICALL
Java_com_revrobotics_jni_CANA301JNI_c_1A301_1GetPeriodicStatus3
  (JNIEnv* env, jclass, jlong handle)
{
    if (periodicStatus3Clazz == nullptr) {
        CANA301JNI_OnLoad(env);
    }
    c_A301_PeriodicStatus3 frame;
    c_REVLib_ErrorCode result =
        c_A301_GetPeriodicStatus3((c_A301_handle)handle, &frame);

    const auto constructor =
        env->GetMethodID(periodicStatus3Clazz, "<init>", "(DIJ)V");
    jobject o =
        env->NewObject(periodicStatus3Clazz, constructor,
                       frame.absoluteEncoderPosition, result, frame.timestamp);
    return o;
}

}  // extern "C"
