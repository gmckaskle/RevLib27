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

#include "com_revrobotics_jni_CANSparkJNI.h"
#include "rev/CANSparkDriver.h"
#include "rev/REVLibDaemon.h"

// TODO(Noah): Always read REVLibError result, throw checked exception for the
//             high-level API to deal with

extern "C" {

static jclass periodicStatus0Clazz;
static jclass periodicStatus1Clazz;
static jclass periodicStatus2Clazz;
static jclass periodicStatus3Clazz;
static jclass periodicStatus4Clazz;
static jclass periodicStatus5Clazz;
static jclass periodicStatus6Clazz;
static jclass periodicStatus7Clazz;
static jclass periodicStatus8Clazz;
static jclass periodicStatus9Clazz;

static jclass loadClassWithGlobalRef(JNIEnv* env, std::string name) {
    jclass localClazz = env->FindClass(name.c_str());

    auto result = reinterpret_cast<jclass>(env->NewGlobalRef(localClazz));
    env->DeleteLocalRef(localClazz);

    return result;
}

void CANSparkJNI_OnLoad(JNIEnv* env) {
    periodicStatus0Clazz = loadClassWithGlobalRef(
        env, "com/revrobotics/spark/SparkLowLevel$PeriodicStatus0");
    periodicStatus1Clazz = loadClassWithGlobalRef(
        env, "com/revrobotics/spark/SparkLowLevel$PeriodicStatus1");
    periodicStatus2Clazz = loadClassWithGlobalRef(
        env, "com/revrobotics/spark/SparkLowLevel$PeriodicStatus2");
    periodicStatus3Clazz = loadClassWithGlobalRef(
        env, "com/revrobotics/spark/SparkLowLevel$PeriodicStatus3");
    periodicStatus4Clazz = loadClassWithGlobalRef(
        env, "com/revrobotics/spark/SparkLowLevel$PeriodicStatus4");
    periodicStatus5Clazz = loadClassWithGlobalRef(
        env, "com/revrobotics/spark/SparkLowLevel$PeriodicStatus5");
    periodicStatus6Clazz = loadClassWithGlobalRef(
        env, "com/revrobotics/spark/SparkLowLevel$PeriodicStatus6");
    periodicStatus7Clazz = loadClassWithGlobalRef(
        env, "com/revrobotics/spark/SparkLowLevel$PeriodicStatus7");
    periodicStatus8Clazz = loadClassWithGlobalRef(
        env, "com/revrobotics/spark/SparkLowLevel$PeriodicStatus8");
    periodicStatus9Clazz = loadClassWithGlobalRef(
        env, "com/revrobotics/spark/SparkLowLevel$PeriodicStatus9");

    // TODO(Landry): Cache fields globally? That's too much work for right now
    // :/
}

void CANSparkJNI_OnUnLoad(JNIEnv* env) {
    env->DeleteGlobalRef(periodicStatus0Clazz);
    env->DeleteGlobalRef(periodicStatus1Clazz);
    env->DeleteGlobalRef(periodicStatus2Clazz);
    env->DeleteGlobalRef(periodicStatus3Clazz);
    env->DeleteGlobalRef(periodicStatus4Clazz);
    env->DeleteGlobalRef(periodicStatus5Clazz);
    env->DeleteGlobalRef(periodicStatus6Clazz);
    env->DeleteGlobalRef(periodicStatus7Clazz);
    env->DeleteGlobalRef(periodicStatus8Clazz);
    env->DeleteGlobalRef(periodicStatus9Clazz);
}

/*
 * Class:     com_revrobotics_jni_CANSparkJNI_c_1Spark
 * Method:    1RegisterId
 * Signature: (II)I
 */
JNIEXPORT jint JNICALL
Java_com_revrobotics_jni_CANSparkJNI_c_1Spark_1RegisterId
  (JNIEnv*, jclass, jint busId, jint deviceId)
{
    return (jint)c_Spark_RegisterId(busId, deviceId);
}

/*
 * Class:     com_revrobotics_jni_CANSparkJNI_c_1Spark
 * Method:    1Create
 * Signature: (IIIILjava/lang/Object;)J
 */
JNIEXPORT jlong JNICALL
Java_com_revrobotics_jni_CANSparkJNI_c_1Spark_1Create
  (JNIEnv* env, jclass, jint busId, jint deviceID, jint motorType,
   jint sparkModel, jobject mutableStatusObj)
{
    c_REVLib_ErrorCode status;
    jlong handle = (jlong)c_Spark_Create(
        busId, deviceID, static_cast<c_Spark_MotorType>(motorType),
        static_cast<c_Spark_SparkModel>(sparkModel), &status);

    jclass clazz = env->GetObjectClass(mutableStatusObj);
    jfieldID fieldID = env->GetFieldID(clazz, "value", "I");
    env->SetIntField(mutableStatusObj, fieldID, static_cast<jint>(status));

    return handle;
}

/*
 * Class:     com_revrobotics_jni_CANSparkJNI_c_1Spark
 * Method:    1Close
 * Signature: (J)V
 */
JNIEXPORT void JNICALL
Java_com_revrobotics_jni_CANSparkJNI_c_1Spark_1Close
  (JNIEnv*, jclass, jlong handle)
{
    c_Spark_Close((c_Spark_handle)handle);
}

/*
 * Class:     com_revrobotics_jni_CANSparkJNI_c_1Spark
 * Method:    1Destroy
 * Signature: (J)V
 */
JNIEXPORT void JNICALL
Java_com_revrobotics_jni_CANSparkJNI_c_1Spark_1Destroy
  (JNIEnv*, jclass, jlong handle)
{
    c_Spark_Destroy((c_Spark_handle)handle);
}

/*
 * Class:     com_revrobotics_jni_CANSparkJNI_c_1Spark
 * Method:    1GetFirmwareVersion
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_revrobotics_jni_CANSparkJNI_c_1Spark_1GetFirmwareVersion
  (JNIEnv*, jclass, jlong handle)
{
    c_Spark_FirmwareVersion fwVersion;
    c_Spark_GetFirmwareVersion((c_Spark_handle)handle, &fwVersion);
    return fwVersion.versionRaw;
}

/*
 * Class:     com_revrobotics_jni_CANSparkJNI_c_1Spark
 * Method:    1GetDeviceId
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_revrobotics_jni_CANSparkJNI_c_1Spark_1GetDeviceId
  (JNIEnv*, jclass, jlong handle)
{
    int deviceId;
    c_Spark_GetDeviceId((c_Spark_handle)handle, &deviceId);
    return (jint)deviceId;
}

/*
 * Class:     com_revrobotics_jni_CANSparkJNI_c_1Spark
 * Method:    1SetPeriodicFrameTimeout
 * Signature: (JI)V
 */
JNIEXPORT void JNICALL
Java_com_revrobotics_jni_CANSparkJNI_c_1Spark_1SetPeriodicFrameTimeout
  (JNIEnv*, jclass, jlong handle, jint timeout)
{
    c_Spark_SetPeriodicFrameTimeout((c_Spark_handle)handle, timeout);
}

/*
 * Class:     com_revrobotics_jni_CANSparkJNI_c_1Spark
 * Method:    1SetControlFramePeriod
 * Signature: (JI)V
 */
JNIEXPORT void JNICALL
Java_com_revrobotics_jni_CANSparkJNI_c_1Spark_1SetControlFramePeriod
  (JNIEnv*, jclass, jlong handle, jint periodMs)
{
    c_Spark_SetControlFramePeriod((c_Spark_handle)handle, periodMs);
}

/*
 * Class:     com_revrobotics_jni_CANSparkJNI_c_1Spark
 * Method:    1GetControlFramePeriod
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_revrobotics_jni_CANSparkJNI_c_1Spark_1GetControlFramePeriod
  (JNIEnv*, jclass, jlong handle)
{
    return (jint)c_Spark_GetControlFramePeriod((c_Spark_handle)handle);
}

/*
 * Class:     com_revrobotics_jni_CANSparkJNI_c_1Spark
 * Method:    1SetEncoderPosition
 * Signature: (JF)I
 */
JNIEXPORT jint JNICALL
Java_com_revrobotics_jni_CANSparkJNI_c_1Spark_1SetEncoderPosition
  (JNIEnv*, jclass, jlong handle, jfloat position)
{
    return (jint)c_Spark_SetEncoderPosition((c_Spark_handle)handle, position);
}

/*
 * Class:     com_revrobotics_jni_CANSparkJNI_c_1Spark
 * Method:    1GetDataPortConfig
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_revrobotics_jni_CANSparkJNI_c_1Spark_1GetDataPortConfig
  (JNIEnv*, jclass, jlong handle)
{
    c_Spark_DataPortConfig config;
    c_Spark_GetDataPortConfig((c_Spark_handle)handle, &config);
    return (jint)config;
}

/*
 * Class:     com_revrobotics_jni_CANSparkJNI_c_1Spark
 * Method:    1IsDataPortConfigured
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL
Java_com_revrobotics_jni_CANSparkJNI_c_1Spark_1IsDataPortConfigured
  (JNIEnv*, jclass, jlong handle)
{
    uint8_t configured;
    c_Spark_IsDataPortConfigured((c_Spark_handle)handle, &configured);
    return (jboolean)(configured);
}

/*
 * Class:     com_revrobotics_jni_CANSparkJNI_c_1Spark
 * Method:    1SetAltEncoderPosition
 * Signature: (JF)I
 */
JNIEXPORT jint JNICALL
Java_com_revrobotics_jni_CANSparkJNI_c_1Spark_1SetAltEncoderPosition
  (JNIEnv*, jclass, jlong handle, jfloat position)
{
    return (jint)c_Spark_SetAltEncoderPosition((c_Spark_handle)handle,
                                               position);
}

/*
 * Class:     com_revrobotics_jni_CANSparkJNI_c_1Spark
 * Method:    1ResetSafeParameters
 * Signature: (JZ)I
 */
JNIEXPORT jint JNICALL
Java_com_revrobotics_jni_CANSparkJNI_c_1Spark_1ResetSafeParameters
  (JNIEnv*, jclass, jlong handle, jboolean persist)
{
    return (jint)c_Spark_ResetSafeParameters((c_Spark_handle)handle,
                                             persist ? 1 : 0);
}

/*
 * Class:     com_revrobotics_jni_CANSparkJNI_c_1Spark
 * Method:    1SafeFloat
 * Signature: (F)F
 */
JNIEXPORT jfloat JNICALL
Java_com_revrobotics_jni_CANSparkJNI_c_1Spark_1SafeFloat
  (JNIEnv*, jclass, jfloat f)
{
    return (jfloat)c_Spark_SafeFloat(f);
}

/*
 * Class:     com_revrobotics_jni_CANSparkJNI_c_1Spark
 * Method:    1SetpointCommand
 * Signature: (JFIIFI)I
 */
JNIEXPORT jint JNICALL
Java_com_revrobotics_jni_CANSparkJNI_c_1Spark_1SetpointCommand
  (JNIEnv*, jclass, jlong handle, jfloat value, jint ctrl, jint pidSlot,
   jfloat arbFF, jint arbFFUnits)
{
    return (jint)c_Spark_SetpointCommand((c_Spark_handle)handle, value,
                                         (c_Spark_ControlType)ctrl, pidSlot,
                                         arbFF, arbFFUnits);
}

/*
 * Class:     com_revrobotics_jni_CANSparkJNI_c_1Spark
 * Method:    1SetInverted
 * Signature: (JZ)I
 */
JNIEXPORT jint JNICALL
Java_com_revrobotics_jni_CANSparkJNI_c_1Spark_1SetInverted
  (JNIEnv*, jclass, jlong handle, jboolean inverted)
{
    return (jint)c_Spark_SetInverted((c_Spark_handle)handle, inverted ? 1 : 0);
}

/*
 * Class:     com_revrobotics_jni_CANSparkJNI_c_1Spark
 * Method:    1GetInverted
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL
Java_com_revrobotics_jni_CANSparkJNI_c_1Spark_1GetInverted
  (JNIEnv*, jclass, jlong handle)
{
    uint8_t inverted;
    c_Spark_GetInverted((c_Spark_handle)handle, &inverted);
    return (jboolean)inverted;
}

/*
 * Class:     com_revrobotics_jni_CANSparkJNI_c_1Spark
 * Method:    1SetSimAppliedOutput
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL
Java_com_revrobotics_jni_CANSparkJNI_c_1Spark_1SetSimAppliedOutput
  (JNIEnv*, jclass, jlong handle, jfloat value)
{
    c_Spark_SetSimAppliedOutput((c_Spark_handle)handle, value);
}

/*
 * Class:     com_revrobotics_jni_CANSparkJNI_c_1Spark
 * Method:    1ClearFaults
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_revrobotics_jni_CANSparkJNI_c_1Spark_1ClearFaults
  (JNIEnv*, jclass, jlong handle)
{
    return (jint)c_Spark_ClearFaults((c_Spark_handle)handle);
}

/*
 * Class:     com_revrobotics_jni_CANSparkJNI_c_1Spark
 * Method:    1PersistParameters
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_revrobotics_jni_CANSparkJNI_c_1Spark_1PersistParameters
  (JNIEnv*, jclass, jlong handle)
{
    return (jint)c_Spark_PersistParameters((c_Spark_handle)handle);
}

/*
 * Class:     com_revrobotics_jni_CANSparkJNI_c_1Spark
 * Method:    1SetCANTimeout
 * Signature: (JI)I
 */
JNIEXPORT jint JNICALL
Java_com_revrobotics_jni_CANSparkJNI_c_1Spark_1SetCANTimeout
  (JNIEnv*, jclass, jlong handle, jint timeout)
{
    return (jint)c_Spark_SetCANTimeout((c_Spark_handle)handle, timeout);
}

/*
 * Class:     com_revrobotics_jni_CANSparkJNI_c_1Spark
 * Method:    1SetCANMaxRetries
 * Signature: (JI)V
 */
JNIEXPORT void JNICALL
Java_com_revrobotics_jni_CANSparkJNI_c_1Spark_1SetCANMaxRetries
  (JNIEnv*, jclass, jlong handle, jint numRetries)
{
    c_Spark_SetCANMaxRetries((c_Spark_handle)handle, numRetries);
}

/*
 * Class:     com_revrobotics_jni_CANSparkJNI_c_1Spark
 * Method:    1GetMotorInterface
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_revrobotics_jni_CANSparkJNI_c_1Spark_1GetMotorInterface
  (JNIEnv*, jclass, jlong handle)
{
    uint8_t motorInterface;
    c_Spark_GetMotorInterface((c_Spark_handle)handle, &motorInterface);

    return (jint)motorInterface;
}

/*
 * Class:     com_revrobotics_jni_CANSparkJNI_c_1Spark
 * Method:    1GetSoftLimit
 * Signature: (JI)Z
 */
JNIEXPORT jboolean JNICALL
Java_com_revrobotics_jni_CANSparkJNI_c_1Spark_1GetSoftLimit
  (JNIEnv*, jclass, jlong handle, jint limit)
{
    uint8_t tmp;
    c_Spark_GetSoftLimit((c_Spark_handle)handle, (c_Spark_LimitDirection)limit,
                         &tmp);
    return (jboolean)tmp;
}

/*
 * Class:     com_revrobotics_jni_CANSparkJNI_c_1Spark
 * Method:    1SetIAccum
 * Signature: (JF)I
 */
JNIEXPORT jint JNICALL
Java_com_revrobotics_jni_CANSparkJNI_c_1Spark_1SetIAccum
  (JNIEnv*, jclass, jlong handle, jfloat val)
{
    return (jint)c_Spark_SetIAccum((c_Spark_handle)handle, val);
}

/*
 * Class:     com_revrobotics_jni_CANSparkJNI_c_1Spark
 * Method:    1GetAPIMajorRevision
 * Signature: ()I
 */
JNIEXPORT jint JNICALL
Java_com_revrobotics_jni_CANSparkJNI_c_1Spark_1GetAPIMajorRevision
  (JNIEnv*, jclass)
{
    return (jint)c_Spark_GetAPIVersion().Major;
}

/*
 * Class:     com_revrobotics_jni_CANSparkJNI_c_1Spark
 * Method:    1GetAPIMinorRevision
 * Signature: ()I
 */
JNIEXPORT jint JNICALL
Java_com_revrobotics_jni_CANSparkJNI_c_1Spark_1GetAPIMinorRevision
  (JNIEnv*, jclass)
{
    return (jint)c_Spark_GetAPIVersion().Minor;
}

/*
 * Class:     com_revrobotics_jni_CANSparkJNI_c_1Spark
 * Method:    1GetAPIBuildRevision
 * Signature: ()I
 */
JNIEXPORT jint JNICALL
Java_com_revrobotics_jni_CANSparkJNI_c_1Spark_1GetAPIBuildRevision
  (JNIEnv*, jclass)
{
    return (jint)c_Spark_GetAPIVersion().Build;
}

/*
 * Class:     com_revrobotics_jni_CANSparkJNI_c_1Spark
 * Method:    1GetAPIVersion
 * Signature: ()I
 */
JNIEXPORT jint JNICALL
Java_com_revrobotics_jni_CANSparkJNI_c_1Spark_1GetAPIVersion
  (JNIEnv*, jclass)
{
    return (jint)c_Spark_GetAPIVersion().Version;
}

/*
 * Class:     com_revrobotics_jni_CANSparkJNI_c_1Spark
 * Method:    1GetLastError
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_revrobotics_jni_CANSparkJNI_c_1Spark_1GetLastError
  (JNIEnv*, jclass, jlong handle)
{
    auto status = c_Spark_GetLastError((c_Spark_handle)handle);
    return (jint)status;
}

/*
 * Class:     com_revrobotics_jni_CANSparkJNI_c_1Spark
 * Method:    1SetParameterFloat32
 * Signature: (JIF)I
 */
JNIEXPORT jint JNICALL
Java_com_revrobotics_jni_CANSparkJNI_c_1Spark_1SetParameterFloat32
  (JNIEnv*, jclass, jlong handle, jint paramid, jfloat value)
{
    return static_cast<int>(c_Spark_SetParameterFloat32(
        reinterpret_cast<c_Spark_handle>(handle),
        static_cast<c_Spark_ConfigParameter>(paramid),
        static_cast<float>(value)));
}

/*
 * Class:     com_revrobotics_jni_CANSparkJNI_c_1Spark
 * Method:    1SetParameterInt32
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL
Java_com_revrobotics_jni_CANSparkJNI_c_1Spark_1SetParameterInt32
  (JNIEnv*, jclass, jlong handle, jint paramid, jint value)
{
    return static_cast<int>(
        c_Spark_SetParameterInt32(reinterpret_cast<c_Spark_handle>(handle),
                                  static_cast<c_Spark_ConfigParameter>(paramid),
                                  static_cast<int32_t>(value)));
}

/*
 * Class:     com_revrobotics_jni_CANSparkJNI_c_1Spark
 * Method:    1SetParameterUint32
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL
Java_com_revrobotics_jni_CANSparkJNI_c_1Spark_1SetParameterUint32
  (JNIEnv*, jclass, jlong handle, jint paramid, jint value)
{
    return static_cast<int>(c_Spark_SetParameterUint32(
        reinterpret_cast<c_Spark_handle>(handle),
        static_cast<c_Spark_ConfigParameter>(paramid),
        static_cast<uint32_t>(value)));
}

/*
 * Class:     com_revrobotics_jni_CANSparkJNI_c_1Spark
 * Method:    1SetParameterBool
 * Signature: (JIZ)I
 */
JNIEXPORT jint JNICALL
Java_com_revrobotics_jni_CANSparkJNI_c_1Spark_1SetParameterBool
  (JNIEnv*, jclass, jlong handle, jint paramid, jboolean value)
{
    return static_cast<int>(
        c_Spark_SetParameterBool(reinterpret_cast<c_Spark_handle>(handle),
                                 static_cast<c_Spark_ConfigParameter>(paramid),
                                 static_cast<bool>(value)));
}

/*
 * Class:     com_revrobotics_jni_CANSparkJNI_c_1Spark
 * Method:    1GetParameterFloat32
 * Signature: (JI)F
 */
JNIEXPORT jfloat JNICALL
Java_com_revrobotics_jni_CANSparkJNI_c_1Spark_1GetParameterFloat32
  (JNIEnv*, jclass, jlong handle, jint paramid)
{
    float value;
    c_Spark_GetParameterFloat32(reinterpret_cast<c_Spark_handle>(handle),
                                static_cast<c_Spark_ConfigParameter>(paramid),
                                &value);
    return static_cast<jfloat>(value);
}

/*
 * Class:     com_revrobotics_jni_CANSparkJNI_c_1Spark
 * Method:    1GetParameterInt32
 * Signature: (JI)I
 */
JNIEXPORT jint JNICALL
Java_com_revrobotics_jni_CANSparkJNI_c_1Spark_1GetParameterInt32
  (JNIEnv*, jclass, jlong handle, jint paramid)
{
    int32_t value;
    c_Spark_GetParameterInt32(reinterpret_cast<c_Spark_handle>(handle),
                              static_cast<c_Spark_ConfigParameter>(paramid),
                              &value);
    return static_cast<jint>(value);
}

/*
 * Class:     com_revrobotics_jni_CANSparkJNI_c_1Spark
 * Method:    1GetParameterUint32
 * Signature: (JI)I
 */
JNIEXPORT jint JNICALL
Java_com_revrobotics_jni_CANSparkJNI_c_1Spark_1GetParameterUint32
  (JNIEnv*, jclass, jlong handle, jint paramid)
{
    uint32_t value;
    c_Spark_GetParameterUint32(reinterpret_cast<c_Spark_handle>(handle),
                               static_cast<c_Spark_ConfigParameter>(paramid),
                               &value);
    return static_cast<jint>(value);
}

/*
 * Class:     com_revrobotics_jni_CANSparkJNI_c_1Spark
 * Method:    1GetParameterBool
 * Signature: (JI)Z
 */
JNIEXPORT jboolean JNICALL
Java_com_revrobotics_jni_CANSparkJNI_c_1Spark_1GetParameterBool
  (JNIEnv*, jclass, jlong handle, jint paramid)
{
    uint8_t value;
    c_Spark_GetParameterBool(reinterpret_cast<c_Spark_handle>(handle),
                             static_cast<c_Spark_ConfigParameter>(paramid),
                             &value);
    return static_cast<jboolean>(value != 0);
}

/*
 * Class:     com_revrobotics_jni_CANSparkJNI_c_1Spark
 * Method:    1GetParameterType
 * Signature: (I)I
 */
JNIEXPORT jint JNICALL
Java_com_revrobotics_jni_CANSparkJNI_c_1Spark_1GetParameterType
  (JNIEnv*, jclass, jint paramId)
{
    return static_cast<jint>(c_Spark_GetParameterType(
        static_cast<c_Spark_ConfigParameter>(paramId)));
}

/*
 * Class:     com_revrobotics_jni_CANSparkJNI_c_1Spark
 * Method:    1GetSparkModel
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_revrobotics_jni_CANSparkJNI_c_1Spark_1GetSparkModel
  (JNIEnv*, jclass, jlong handle)
{
    c_Spark_SparkModel model;
    c_Spark_GetSparkModel((c_Spark_handle)handle, &model);
    return (jint)model;
}

/*
 * Class:     com_revrobotics_jni_CANSparkJNI_c_1Spark
 * Method:    1Configure
 * Signature: (JLjava/lang/String;ZZ)I
 */
JNIEXPORT jint JNICALL
Java_com_revrobotics_jni_CANSparkJNI_c_1Spark_1Configure
  (JNIEnv* env, jclass, jlong handle, jstring flattenedString,
   jboolean resetSafeParameters, jboolean persistParameters)
{
    const char* str = env->GetStringUTFChars(flattenedString, nullptr);
    char* cstr = nullptr;
    if (str) {
        cstr = strdup(str);
    }

    env->ReleaseStringUTFChars(flattenedString, str);

    return static_cast<jint>(c_Spark_Configure(
        (c_Spark_handle)handle, cstr, static_cast<uint8_t>(resetSafeParameters),
        static_cast<uint8_t>(persistParameters)));
}

/*
 * Class:     com_revrobotics_jni_CANSparkJNI_c_1Spark
 * Method:    1StartFollowerMode
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_revrobotics_jni_CANSparkJNI_c_1Spark_1StartFollowerMode
  (JNIEnv*, jclass, jlong handle)
{
    return (jint)c_Spark_StartFollowerMode((c_Spark_handle)handle);
}

/*
 * Class:     com_revrobotics_jni_CANSparkJNI_c_1Spark
 * Method:    1StopFollowerMode
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_revrobotics_jni_CANSparkJNI_c_1Spark_1StopFollowerMode
  (JNIEnv*, jclass, jlong handle)
{
    return (jint)c_Spark_StopFollowerMode((c_Spark_handle)handle);
}

/*
 * Class:     com_revrobotics_jni_CANSparkJNI_c_1Spark
 * Method:    1GetSimClosedLoopOutput
 * Signature: (JFFF)F
 */
JNIEXPORT jfloat JNICALL
Java_com_revrobotics_jni_CANSparkJNI_c_1Spark_1GetSimClosedLoopOutput
  (JNIEnv*, jclass, jlong handle, jfloat setpoint, jfloat pv, jfloat dt)
{
    float tmp;
    c_SIM_Spark_GetSimPIDOutput((c_Spark_handle)handle, &tmp, setpoint, pv, dt);
    return (jfloat)tmp;
}

/*
 * Class:     com_revrobotics_jni_CANSparkJNI_c_1Spark
 * Method:    1GetSimMAXMotionPositionControlOutput
 * Signature: (JF)F
 */
JNIEXPORT jfloat JNICALL
Java_com_revrobotics_jni_CANSparkJNI_c_1Spark_1GetSimMAXMotionPositionControlOutput
  (JNIEnv*, jclass, jlong handle, jfloat dt)
{
    float tmp;
    c_SIM_Spark_GetSimMAXMotionPositionControlOutput((c_Spark_handle)handle,
                                                     &tmp, dt);
    return (jfloat)tmp;
}

/*
 * Class:     com_revrobotics_jni_CANSparkJNI_c_1Spark
 * Method:    1GetSimMAXMotionVelocityControlOutput
 * Signature: (JF)F
 */
JNIEXPORT jfloat JNICALL
Java_com_revrobotics_jni_CANSparkJNI_c_1Spark_1GetSimMAXMotionVelocityControlOutput
  (JNIEnv*, jclass, jlong handle, jfloat dt)
{
    float tmp;
    c_SIM_Spark_GetSimMAXMotionVelocityControlOutput((c_Spark_handle)handle,
                                                     &tmp, dt);
    return (jfloat)tmp;
}

/*
 * Class:     com_revrobotics_jni_CANSparkJNI_c_1Spark
 * Method:    1GetSimCurrentLimitOutput
 * Signature: (JFF)F
 */
JNIEXPORT jfloat JNICALL
Java_com_revrobotics_jni_CANSparkJNI_c_1Spark_1GetSimCurrentLimitOutput
  (JNIEnv*, jclass, jlong handle, jfloat appliedOutput, jfloat current)
{
    float tmp;
    c_SIM_Spark_GetSimCurrentLimitOutput((c_Spark_handle)handle, &tmp,
                                         appliedOutput, current);
    return (jfloat)tmp;
}

/*
 * Class:     com_revrobotics_jni_CANSparkJNI_c_1Spark
 * Method:    1CreateAbsoluteEncoderSim
 * Signature: (J)V
 */
JNIEXPORT void JNICALL
Java_com_revrobotics_jni_CANSparkJNI_c_1Spark_1CreateAbsoluteEncoderSim
  (JNIEnv*, jclass, jlong handle)
{
    c_SIM_Spark_CreateSimAbsoluteEncoder((c_Spark_handle)handle);
}

/*
 * Class:     com_revrobotics_jni_CANSparkJNI_c_1Spark
 * Method:    1CreateAlternateEncoderSim
 * Signature: (J)V
 */
JNIEXPORT void JNICALL
Java_com_revrobotics_jni_CANSparkJNI_c_1Spark_1CreateAlternateEncoderSim
  (JNIEnv*, jclass, jlong handle)
{
    c_SIM_Spark_CreateSimExtOrAltEncoder((c_Spark_handle)handle);
}

/*
 * Class:     com_revrobotics_jni_CANSparkJNI_c_1Spark
 * Method:    1CreateAnalogSensorSim
 * Signature: (J)V
 */
JNIEXPORT void JNICALL
Java_com_revrobotics_jni_CANSparkJNI_c_1Spark_1CreateAnalogSensorSim
  (JNIEnv*, jclass, jlong handle)
{
    c_SIM_Spark_CreateSimAnalogSensor((c_Spark_handle)handle);
}

/*
 * Class:     com_revrobotics_jni_CANSparkJNI_c_1Spark
 * Method:    1CreateForwardLimitSwitchSim
 * Signature: (J)V
 */
JNIEXPORT void JNICALL
Java_com_revrobotics_jni_CANSparkJNI_c_1Spark_1CreateForwardLimitSwitchSim
  (JNIEnv*, jclass, jlong handle)
{
    c_SIM_Spark_CreateSimForwardLimitSwitch((c_Spark_handle)handle);
}

/*
 * Class:     com_revrobotics_jni_CANSparkJNI_c_1Spark
 * Method:    1CreateReverseLimitSwitchSim
 * Signature: (J)V
 */
JNIEXPORT void JNICALL
Java_com_revrobotics_jni_CANSparkJNI_c_1Spark_1CreateReverseLimitSwitchSim
  (JNIEnv*, jclass, jlong handle)
{
    c_SIM_Spark_CreateSimReverseLimitSwitch((c_Spark_handle)handle);
}

/*
 * Class:     com_revrobotics_jni_CANSparkJNI_c_1Spark
 * Method:    1CreateSimFaultManager
 * Signature: (J)V
 */
JNIEXPORT void JNICALL
Java_com_revrobotics_jni_CANSparkJNI_c_1Spark_1CreateSimFaultManager
  (JNIEnv*, jclass, jlong handle)
{
    c_SIM_Spark_CreateSimFaultManager((c_Spark_handle)handle);
}

/*
 * Class:     com_revrobotics_jni_CANSparkJNI_c_1Spark
 * Method:    1CreateRelativeEncoderSim
 * Signature: (J)V
 */
JNIEXPORT void JNICALL
Java_com_revrobotics_jni_CANSparkJNI_c_1Spark_1CreateRelativeEncoderSim
  (JNIEnv*, jclass, jlong handle)
{
    c_SIM_Spark_CreateSimRelativeEncoder((c_Spark_handle)handle);
}

/*
 * Class:     com_revrobotics_jni_CANSparkJNI_c_1Spark
 * Method:    1ConfigureAsync
 * Signature: (JLjava/lang/String;ZZ)I
 */
JNIEXPORT jint JNICALL
Java_com_revrobotics_jni_CANSparkJNI_c_1Spark_1ConfigureAsync
  (JNIEnv* env, jclass, jlong handle, jstring flattenedString,
   jboolean resetSafeParameters, jboolean persistParameters)
{
    const char* str = env->GetStringUTFChars(flattenedString, nullptr);
    char* cstr = nullptr;
    if (str) {
        cstr = strdup(str);
    }

    env->ReleaseStringUTFChars(flattenedString, str);

    return static_cast<jint>(c_Spark_ConfigureAsync(
        (c_Spark_handle)handle, cstr, static_cast<uint8_t>(resetSafeParameters),
        static_cast<uint8_t>(persistParameters)));
}

/*
 * Class:     com_revrobotics_jni_CANSparkJNI_c_1Spark
 * Method:    1StartFollowerModeAsync
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_revrobotics_jni_CANSparkJNI_c_1Spark_1StartFollowerModeAsync
  (JNIEnv*, jclass, jlong handle)
{
    return (jint)c_Spark_StartFollowerModeAsync((c_Spark_handle)handle);
}

/*
 * Class:     com_revrobotics_jni_CANSparkJNI_c_1Spark
 * Method:    1StopFollowerModeAsync
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_revrobotics_jni_CANSparkJNI_c_1Spark_1StopFollowerModeAsync
  (JNIEnv*, jclass, jlong handle)
{
    return (jint)c_Spark_StopFollowerModeAsync((c_Spark_handle)handle);
}

/*
 * Class:     com_revrobotics_jni_CANSparkJNI_c_1Spark
 * Method:    1GetPeriodicStatus0
 * Signature: (J)Ljava/lang/Object;
 */
JNIEXPORT jobject JNICALL
Java_com_revrobotics_jni_CANSparkJNI_c_1Spark_1GetPeriodicStatus0
  (JNIEnv* env, jclass, jlong handle)
{
    c_Spark_PeriodicStatus0 frame;
    c_REVLib_ErrorCode result =
        c_Spark_GetPeriodicStatus0((c_Spark_handle)handle, &frame);

    const auto constructor =
        env->GetMethodID(periodicStatus0Clazz, "<init>", "(DDDIZZZZZZIJ)V");
    jobject o = env->NewObject(
        periodicStatus0Clazz, constructor, frame.appliedOutput, frame.voltage,
        frame.current, frame.motorTemperature, frame.hardForwardLimitReached,
        frame.hardReverseLimitReached, frame.softForwardLimitReached,
        frame.softReverseLimitReached, frame.inverted,
        frame.primaryHeartbeatLock, result, frame.timestamp);

    return o;
}

/*
 * Class:     com_revrobotics_jni_CANSparkJNI_c_1Spark
 * Method:    1GetPeriodicStatus1
 * Signature: (J)Ljava/lang/Object;
 */
JNIEXPORT jobject JNICALL
Java_com_revrobotics_jni_CANSparkJNI_c_1Spark_1GetPeriodicStatus1
  (JNIEnv* env, jclass, jlong handle)
{
    c_Spark_PeriodicStatus1 frame;
    c_REVLib_ErrorCode result =
        c_Spark_GetPeriodicStatus1((c_Spark_handle)handle, &frame);

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
        frame.hasResetStickyWarning, frame.otherStickyWarning, frame.isFollower,
        result, frame.timestamp);
    return o;
}

/*
 * Class:     com_revrobotics_jni_CANSparkJNI_c_1Spark
 * Method:    1GetPeriodicStatus2
 * Signature: (J)Ljava/lang/Object;
 */
JNIEXPORT jobject JNICALL
Java_com_revrobotics_jni_CANSparkJNI_c_1Spark_1GetPeriodicStatus2
  (JNIEnv* env, jclass, jlong handle)
{
    c_Spark_PeriodicStatus2 frame;
    c_REVLib_ErrorCode result =
        c_Spark_GetPeriodicStatus2((c_Spark_handle)handle, &frame);

    const auto constructor =
        env->GetMethodID(periodicStatus2Clazz, "<init>", "(DDIJ)V");

    jobject o = env->NewObject(
        periodicStatus2Clazz, constructor, frame.primaryEncoderVelocity,
        frame.primaryEncoderPosition, result, frame.timestamp);
    return o;
}

/*
 * Class:     com_revrobotics_jni_CANSparkJNI_c_1Spark
 * Method:    1GetPeriodicStatus3
 * Signature: (J)Ljava/lang/Object;
 */
JNIEXPORT jobject JNICALL
Java_com_revrobotics_jni_CANSparkJNI_c_1Spark_1GetPeriodicStatus3
  (JNIEnv* env, jclass, jlong handle)
{
    c_Spark_PeriodicStatus3 frame;
    c_REVLib_ErrorCode result =
        c_Spark_GetPeriodicStatus3((c_Spark_handle)handle, &frame);

    const auto constructor =
        env->GetMethodID(periodicStatus3Clazz, "<init>", "(DDDIJ)V");
    jobject o = env->NewObject(periodicStatus3Clazz, constructor,
                               frame.analogVoltage, frame.analogVoltage,
                               frame.analogPosition, result, frame.timestamp);
    return o;
}

/*
 * Class:     com_revrobotics_jni_CANSparkJNI_c_1Spark
 * Method:    1GetPeriodicStatus4
 * Signature: (J)Ljava/lang/Object;
 */
JNIEXPORT jobject JNICALL
Java_com_revrobotics_jni_CANSparkJNI_c_1Spark_1GetPeriodicStatus4
  (JNIEnv* env, jclass, jlong handle)
{
    c_Spark_PeriodicStatus4 frame;
    c_REVLib_ErrorCode result =
        c_Spark_GetPeriodicStatus4((c_Spark_handle)handle, &frame);

    const auto constructor =
        env->GetMethodID(periodicStatus4Clazz, "<init>", "(DDIJ)V");
    jobject o = env->NewObject(
        periodicStatus4Clazz, constructor, frame.externalOrAltEncoderVelocity,
        frame.externalOrAltEncoderPosition, result, frame.timestamp);
    return o;
}

/*
 * Class:     com_revrobotics_jni_CANSparkJNI_c_1Spark
 * Method:    1GetPeriodicStatus5
 * Signature: (J)Ljava/lang/Object;
 */
JNIEXPORT jobject JNICALL
Java_com_revrobotics_jni_CANSparkJNI_c_1Spark_1GetPeriodicStatus5
  (JNIEnv* env, jclass, jlong handle)
{
    c_Spark_PeriodicStatus5 frame;
    c_REVLib_ErrorCode result =
        c_Spark_GetPeriodicStatus5((c_Spark_handle)handle, &frame);

    const auto constructor =
        env->GetMethodID(periodicStatus5Clazz, "<init>", "(DDIJ)V");
    jobject o = env->NewObject(
        periodicStatus5Clazz, constructor, frame.dutyCycleEncoderVelocity,
        frame.dutyCycleEncoderPosition, result, frame.timestamp);
    return o;
}

/*
 * Class:     com_revrobotics_jni_CANSparkJNI_c_1Spark
 * Method:    1GetPeriodicStatus6
 * Signature: (J)Ljava/lang/Object;
 */
JNIEXPORT jobject JNICALL
Java_com_revrobotics_jni_CANSparkJNI_c_1Spark_1GetPeriodicStatus6
  (JNIEnv* env, jclass, jlong handle)
{
    c_Spark_PeriodicStatus6 frame;
    c_REVLib_ErrorCode result =
        c_Spark_GetPeriodicStatus6((c_Spark_handle)handle, &frame);

    const auto constructor =
        env->GetMethodID(periodicStatus6Clazz, "<init>", "(DDZIJ)V");
    jobject o =
        env->NewObject(periodicStatus6Clazz, constructor,
                       frame.unadjustedDutyCycle, frame.dutyCyclePeriod,
                       frame.dutyCycleNoSignal, result, frame.timestamp);
    return o;
}

/*
 * Class:     com_revrobotics_jni_CANSparkJNI_c_1Spark
 * Method:    1GetPeriodicStatus7
 * Signature: (J)Ljava/lang/Object;
 */
JNIEXPORT jobject JNICALL
Java_com_revrobotics_jni_CANSparkJNI_c_1Spark_1GetPeriodicStatus7
  (JNIEnv* env, jclass, jlong handle)
{
    c_Spark_PeriodicStatus7 frame;
    c_REVLib_ErrorCode result =
        c_Spark_GetPeriodicStatus7((c_Spark_handle)handle, &frame);

    const auto constructor =
        env->GetMethodID(periodicStatus7Clazz, "<init>", "(DIJ)V");
    jobject o = env->NewObject(periodicStatus7Clazz, constructor,
                               frame.iAccumulation, result, frame.timestamp);

    return o;
}

/*
 * Class:     com_revrobotics_jni_CANSparkJNI_c_1Spark
 * Method:    1GetPeriodicStatus8
 * Signature: (J)Ljava/lang/Object;
 */
JNIEXPORT jobject JNICALL
Java_com_revrobotics_jni_CANSparkJNI_c_1Spark_1GetPeriodicStatus8
  (JNIEnv* env, jclass, jlong handle)
{
    c_Spark_PeriodicStatus8 frame;
    c_REVLib_ErrorCode result =
        c_Spark_GetPeriodicStatus8((c_Spark_handle)handle, &frame);

    const auto constructor =
        env->GetMethodID(periodicStatus8Clazz, "<init>", "(DZIIJ)V");
    jobject o = env->NewObject(periodicStatus8Clazz, constructor,
                               frame.setpoint, frame.isAtSetpoint != 0,
                               frame.selectedPidSlot, result, frame.timestamp);
    return o;
}

/*
 * Class:     com_revrobotics_jni_CANSparkJNI_c_1Spark
 * Method:    1GetPeriodicStatus9
 * Signature: (J)Ljava/lang/Object;
 */
JNIEXPORT jobject JNICALL
Java_com_revrobotics_jni_CANSparkJNI_c_1Spark_1GetPeriodicStatus9
  (JNIEnv* env, jclass, jlong handle)
{
    c_Spark_PeriodicStatus9 frame;
    c_REVLib_ErrorCode result =
        c_Spark_GetPeriodicStatus9((c_Spark_handle)handle, &frame);

    const auto constructor =
        env->GetMethodID(periodicStatus9Clazz, "<init>", "(DDIJ)V");
    jobject o = env->NewObject(
        periodicStatus9Clazz, constructor, frame.maxmotion_setpoint_position,
        frame.maxmotion_setpoint_velocity, result, frame.timestamp);
    return o;
}

}  // extern "C"
