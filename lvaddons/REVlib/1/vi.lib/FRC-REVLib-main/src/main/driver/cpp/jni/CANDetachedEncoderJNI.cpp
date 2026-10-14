/*
 * Copyright (c) 2025-2026 REV Robotics
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

#include <rev/CANDetachedEncoderDriver.h>

#include <cstring>
#include <string>

#include "com_revrobotics_jni_DetachedEncoderJNI.h"

/*
 * Class:     com_revrobotics_jni_DetachedEncoderJNI
 * Method:    create
 * Signature: (III)J
 */
JNIEXPORT jlong JNICALL
Java_com_revrobotics_jni_DetachedEncoderJNI_create
  (JNIEnv* env, jclass, jint bus, jint id, jint model)
{
    c_REVLib_ErrorCode status = c_REVLibError_None;
    auto handle = c_Detached_Create(
        bus, id, static_cast<c_Detached_EncoderModel>(model), &status);

    if (status != c_REVLibError_None) {
        jclass exceptionClass =
            env->FindClass("java/lang/IllegalStateException");

        std::string errorMessage;
        if (exceptionClass != nullptr) {
            switch (status) {
                case c_REVLibError_InvalidCANId: {
                    errorMessage = "Invalid CAN ID set for Encoder";
                    break;
                }
                default: {
                    errorMessage = "Unknown error creating Detached Encoder";
                }
            }
            env->ThrowNew(exceptionClass, errorMessage.c_str());
        }
    }
    return reinterpret_cast<jlong>(handle);
}

/*
 * Class:     com_revrobotics_jni_DetachedEncoderJNI
 * Method:    registerId
 * Signature: (II)I
 */
JNIEXPORT jint JNICALL
Java_com_revrobotics_jni_DetachedEncoderJNI_registerId
  (JNIEnv*, jclass, jint bus, jint id)
{
    return c_Detached_RegisterId(bus, id);
}

/*
 * Class:     com_revrobotics_jni_DetachedEncoderJNI
 * Method:    clearFaults
 * Signature: (J)V
 */
JNIEXPORT void JNICALL
Java_com_revrobotics_jni_DetachedEncoderJNI_clearFaults
  (JNIEnv*, jclass, jlong handle)
{
    auto* encoder = reinterpret_cast<c_Detached_handle>(handle);
    c_Detached_ClearFaults(encoder);
}

/*
 * Class:     com_revrobotics_jni_DetachedEncoderJNI
 * Method:    setEncoderPosition
 * Signature: (JD)I
 */
JNIEXPORT jint JNICALL
Java_com_revrobotics_jni_DetachedEncoderJNI_setEncoderPosition
  (JNIEnv*, jclass, jlong handle, jdouble newPosition)
{
    auto* encoder = reinterpret_cast<c_Detached_handle>(handle);

    return c_Detached_SetEncoderPosition(encoder,
                                         static_cast<float>(newPosition));
}

/*
 * Class:     com_revrobotics_jni_DetachedEncoderJNI
 * Method:    getEncoderModel
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_revrobotics_jni_DetachedEncoderJNI_getEncoderModel
  (JNIEnv*, jclass, jlong handle)
{
    auto* encoder = reinterpret_cast<c_Detached_handle>(handle);

    c_Detached_EncoderModel model;
    c_REVLib_ErrorCode status = c_Detached_GetEncoderModel(encoder, &model);

    if (status == c_REVLibError_None) {
        return model;
    }
    return c_Detached_kUnknown;
}

/*
 * Class:     com_revrobotics_jni_DetachedEncoderJNI
 * Method:    getFirmwareVersion
 * Signature: (J)Ljava/lang/Object;
 */
JNIEXPORT jobject JNICALL
Java_com_revrobotics_jni_DetachedEncoderJNI_getFirmwareVersion
  (JNIEnv* env, jclass, jlong handle)
{
    auto* encoder = reinterpret_cast<c_Detached_handle>(handle);

    c_Detached_FirmwareVersion firmwareVersion;
    c_Detached_GetFirmwareVersion(encoder, &firmwareVersion);

    const auto clazz = env->FindClass(
        "com/revrobotics/encoder/DetachedEncoder$FirmwareVersion");
    const auto constructor = env->GetMethodID(clazz, "<init>", "(IIIIII)V");

    return env->NewObject(clazz, constructor, firmwareVersion.fwYear,
                          firmwareVersion.fwMinor, firmwareVersion.fwFix,
                          firmwareVersion.fwPrerelease, firmwareVersion.hwMajor,
                          firmwareVersion.hwMinor);
}

/*
 * Class:     com_revrobotics_jni_DetachedEncoderJNI
 * Method:    getParameterType
 * Signature: (I)I
 */
JNIEXPORT jint JNICALL
Java_com_revrobotics_jni_DetachedEncoderJNI_getParameterType
  (JNIEnv*, jclass, jint index)
{
    return c_Detached_GetParameterType(
        static_cast<c_Detached_ConfigParameter>(index));
}

/*
 * Class:     com_revrobotics_jni_DetachedEncoderJNI
 * Method:    configure
 * Signature: (JLjava/lang/String;Z)I
 */
JNIEXPORT jint JNICALL
Java_com_revrobotics_jni_DetachedEncoderJNI_configure
  (JNIEnv* env, jclass, jlong handle, jstring flattened, jboolean reset)
{
    auto* encoder = reinterpret_cast<c_Detached_handle>(handle);
    const char* str = env->GetStringUTFChars(flattened, nullptr);
    char* cstr = nullptr;
    if (str) {
        cstr = strdup(str);
    }
    env->ReleaseStringUTFChars(flattened, str);

    return c_Detached_Configure(encoder, cstr, reset);
}

/*
 * Class:     com_revrobotics_jni_DetachedEncoderJNI
 * Method:    close
 * Signature: (J)V
 */
JNIEXPORT void JNICALL
Java_com_revrobotics_jni_DetachedEncoderJNI_close
  (JNIEnv*, jclass, jlong handle)
{
    auto* encoder = reinterpret_cast<c_Detached_handle>(handle);

    c_Detached_Close(encoder);
}

/*
 * Class:     com_revrobotics_jni_DetachedEncoderJNI
 * Method:    destroy
 * Signature: (J)V
 */
JNIEXPORT void JNICALL
Java_com_revrobotics_jni_DetachedEncoderJNI_destroy
  (JNIEnv*, jclass, jlong handle)
{
    c_Detached_Destroy(reinterpret_cast<c_Detached_handle>(handle));
}

/*
 * Class:     com_revrobotics_jni_DetachedEncoderJNI
 * Method:    isInverted
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL
Java_com_revrobotics_jni_DetachedEncoderJNI_isInverted
  (JNIEnv*, jclass, jlong handle)
{
    auto* encoder = reinterpret_cast<c_Detached_handle>(handle);
    bool inverted = false;
    auto status = c_Detached_GetInverted(encoder, &inverted);

    if (status == c_REVLibError_None) {
        return inverted;
    }
    // TODO(Landry): Is this a good default behavior?
    return false;
}

/*
 * Class:     com_revrobotics_jni_DetachedEncoderJNI
 * Method:    getAverageDepth
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_revrobotics_jni_DetachedEncoderJNI_getAverageDepth
  (JNIEnv*, jclass, jlong handle)
{
    auto* encoder = reinterpret_cast<c_Detached_handle>(handle);

    uint32_t averageDepth = 0;
    c_Detached_GetQuadratureAverageDepth(encoder, &averageDepth);
    return static_cast<jint>(averageDepth);
}

/*
 * Class:     com_revrobotics_jni_DetachedEncoderJNI
 * Method:    getPositionConversionFactor
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL
Java_com_revrobotics_jni_DetachedEncoderJNI_getPositionConversionFactor
  (JNIEnv*, jclass, jlong handle)
{
    auto* encoder = reinterpret_cast<c_Detached_handle>(handle);

    float conversion = 0;
    c_Detached_GetPositionConversionFactor(encoder, &conversion);
    return conversion;
}

/*
 * Class:     com_revrobotics_jni_DetachedEncoderJNI
 * Method:    getVelocityConversionFactor
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL
Java_com_revrobotics_jni_DetachedEncoderJNI_getVelocityConversionFactor
  (JNIEnv*, jclass, jlong handle)
{
    auto* encoder = reinterpret_cast<c_Detached_handle>(handle);

    float conversion = 0;
    c_Detached_GetVelocityConversionFactor(encoder, &conversion);
    return conversion;
}

/*
 * Class:     com_revrobotics_jni_DetachedEncoderJNI
 * Method:    getAngleConversionFactor
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL
Java_com_revrobotics_jni_DetachedEncoderJNI_getAngleConversionFactor
  (JNIEnv*, jclass, jlong handle)
{
    auto* encoder = reinterpret_cast<c_Detached_handle>(handle);

    float conversion = 0;
    c_Detached_GetAngleConversionFactor(encoder, &conversion);
    return conversion;
}

/*
 * Class:     com_revrobotics_jni_DetachedEncoderJNI
 * Method:    isDutyCycleZeroCentered
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL
Java_com_revrobotics_jni_DetachedEncoderJNI_isDutyCycleZeroCentered
  (JNIEnv*, jclass, jlong handle)
{
    auto* encoder = reinterpret_cast<c_Detached_handle>(handle);

    bool isDutyCycleZeroed = false;
    c_Detached_GetZeroCentered(encoder, &isDutyCycleZeroed);
    return isDutyCycleZeroed;
}

/*
 * Class:     com_revrobotics_jni_DetachedEncoderJNI
 * Method:    getDutyCycleAverageDepth
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_revrobotics_jni_DetachedEncoderJNI_getDutyCycleAverageDepth
  (JNIEnv*, jclass, jlong handle)
{
    auto* encoder = reinterpret_cast<c_Detached_handle>(handle);

    uint32_t averageDepth = 0;
    c_Detached_GetAbsoluteAverageDepth(encoder, &averageDepth);
    return static_cast<jint>(averageDepth);
}

/*
 * Class:     com_revrobotics_jni_DetachedEncoderJNI
 * Method:    getDutyCycleOffset
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL
Java_com_revrobotics_jni_DetachedEncoderJNI_getDutyCycleOffset
  (JNIEnv*, jclass, jlong handle)
{
    auto* encoder = reinterpret_cast<c_Detached_handle>(handle);

    float offset = 0;
    c_Detached_GetZeroOffset(encoder, &offset);
    return offset;
}

/*
 * Class:     com_revrobotics_jni_DetachedEncoderJNI
 * Method:    getDutyCycleStartPulseUs
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL
Java_com_revrobotics_jni_DetachedEncoderJNI_getDutyCycleStartPulseUs
  (JNIEnv*, jclass, jlong handle)
{
    auto* encoder = reinterpret_cast<c_Detached_handle>(handle);

    float startPulse = 0;
    c_Detached_GetStartPulseUs(encoder, &startPulse);
    return startPulse;
}

/*
 * Class:     com_revrobotics_jni_DetachedEncoderJNI
 * Method:    getDutyCycleEndPulseUs
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL
Java_com_revrobotics_jni_DetachedEncoderJNI_getDutyCycleEndPulseUs
  (JNIEnv*, jclass, jlong handle)
{
    auto* encoder = reinterpret_cast<c_Detached_handle>(handle);

    float endPulse = 0;
    c_Detached_GetEndPulseUs(encoder, &endPulse);
    return endPulse;
}

/*
 * Class:     com_revrobotics_jni_DetachedEncoderJNI
 * Method:    getDutyCyclePeriodUs
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL
Java_com_revrobotics_jni_DetachedEncoderJNI_getDutyCyclePeriodUs
  (JNIEnv*, jclass, jlong handle)
{
    auto* encoder = reinterpret_cast<c_Detached_handle>(handle);

    float period = 0;
    c_Detached_GetAbsolutePeriodUs(encoder, &period);
    return period;
}

/*
 * Class:     com_revrobotics_jni_DetachedEncoderJNI
 * Method:    getStatus0
 * Signature: (J)Ljava/lang/Object;
 */
JNIEXPORT jobject JNICALL
Java_com_revrobotics_jni_DetachedEncoderJNI_getStatus0
  (JNIEnv* env, jclass, jlong handle)
{
    auto* encoder = reinterpret_cast<c_Detached_handle>(handle);

    c_Detached_PeriodicStatus0 frame;
    auto status = c_Detached_GetPeriodicStatus0(encoder, &frame);

    jclass clazz = env->FindClass(
        "com/revrobotics/encoder/DetachedEncoder$PeriodicStatus0");
    jmethodID constructor = env->GetMethodID(clazz, "<init>", "(IIJ)V");

    return env->NewObject(clazz, constructor, frame.encoderModel, status,
                          frame.timestamp);
}

/*
 * Class:     com_revrobotics_jni_DetachedEncoderJNI
 * Method:    getStatus1
 * Signature: (J)Ljava/lang/Object;
 */
JNIEXPORT jobject JNICALL
Java_com_revrobotics_jni_DetachedEncoderJNI_getStatus1
  (JNIEnv* env, jclass, jlong handle)
{
    auto* encoder = reinterpret_cast<c_Detached_handle>(handle);

    c_Detached_PeriodicStatus1 frame;
    auto status = c_Detached_GetPeriodicStatus1(encoder, &frame);

    jclass clazz = env->FindClass(
        "com/revrobotics/encoder/DetachedEncoder$PeriodicStatus1");
    jmethodID constructor =
        env->GetMethodID(clazz, "<init>", "(ZZZZZZZZZZIJ)V");

    return env->NewObject(
        clazz, constructor, frame.unexpectedFault, frame.hasResetFault,
        frame.canTxFault, frame.canRxFault, frame.eepromFault,
        frame.unexpectedStickyFault, frame.hasResetStickyFault,
        frame.canTxStickyFault, frame.canRxStickyFault, frame.eepromStickyFault,
        status, frame.timestamp);
}

/*
 * Class:     com_revrobotics_jni_DetachedEncoderJNI
 * Method:    getStatus2
 * Signature: (J)Ljava/lang/Object;
 */
JNIEXPORT jobject JNICALL
Java_com_revrobotics_jni_DetachedEncoderJNI_getStatus2
  (JNIEnv* env, jclass, jlong handle)
{
    auto* encoder = reinterpret_cast<c_Detached_handle>(handle);

    c_Detached_PeriodicStatus2 frame;
    auto status = c_Detached_GetPeriodicStatus2(encoder, &frame);

    jclass clazz = env->FindClass(
        "com/revrobotics/encoder/DetachedEncoder$PeriodicStatus2");
    jmethodID constructor = env->GetMethodID(clazz, "<init>", "(FFIJ)V");

    return env->NewObject(clazz, constructor, frame.rawAbsoluteAngle,
                          frame.absoluteAngle, status, frame.timestamp);
}

/*
 * Class:     com_revrobotics_jni_DetachedEncoderJNI
 * Method:    getStatus3
 * Signature: (J)Ljava/lang/Object;
 */
JNIEXPORT jobject JNICALL
Java_com_revrobotics_jni_DetachedEncoderJNI_getStatus3
  (JNIEnv* env, jclass, jlong handle)
{
    auto* encoder = reinterpret_cast<c_Detached_handle>(handle);

    c_Detached_PeriodicStatus3 frame;
    auto status = c_Detached_GetPeriodicStatus3(encoder, &frame);

    jclass clazz = env->FindClass(
        "com/revrobotics/encoder/DetachedEncoder$PeriodicStatus3");
    jmethodID constructor = env->GetMethodID(clazz, "<init>", "(FIJ)V");

    return env->NewObject(clazz, constructor, frame.relativePosition, status,
                          frame.timestamp);
}

/*
 * Class:     com_revrobotics_jni_DetachedEncoderJNI
 * Method:    getStatus4
 * Signature: (J)Ljava/lang/Object;
 */
JNIEXPORT jobject JNICALL
Java_com_revrobotics_jni_DetachedEncoderJNI_getStatus4
  (JNIEnv* env, jclass, jlong handle)
{
    auto* encoder = reinterpret_cast<c_Detached_handle>(handle);

    c_Detached_PeriodicStatus4 frame;
    auto status = c_Detached_GetPeriodicStatus4(encoder, &frame);

    jclass clazz = env->FindClass(
        "com/revrobotics/encoder/DetachedEncoder$PeriodicStatus4");
    jmethodID constructor = env->GetMethodID(clazz, "<init>", "(FIJ)V");

    return env->NewObject(clazz, constructor, frame.encoderVelocity, status,
                          frame.timestamp);
}

/*
 * Class:     com_revrobotics_jni_DetachedEncoderJNI
 * Method:    getEncoderVelocityPeriodMs
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_revrobotics_jni_DetachedEncoderJNI_getEncoderVelocityPeriodMs
  (JNIEnv*, jclass, jlong handle)
{
    auto* encoder = reinterpret_cast<c_Detached_handle>(handle);

    uint32_t period = 0;
    c_Detached_GetStatusPeriod(encoder, c_Detached_kStatus4Period, &period);
    return static_cast<jint>(period);
}

/*
 * Class:     com_revrobotics_jni_DetachedEncoderJNI
 * Method:    getEncoderPositionPeriodMs
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_revrobotics_jni_DetachedEncoderJNI_getEncoderPositionPeriodMs
  (JNIEnv*, jclass, jlong handle)
{
    auto* encoder = reinterpret_cast<c_Detached_handle>(handle);

    uint32_t period = 0;
    c_Detached_GetStatusPeriod(encoder, c_Detached_kStatus3Period, &period);
    return static_cast<jint>(period);
}

/*
 * Class:     com_revrobotics_jni_DetachedEncoderJNI
 * Method:    getEncoderAnglePeriodMs
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_revrobotics_jni_DetachedEncoderJNI_getEncoderAnglePeriodMs
  (JNIEnv*, jclass, jlong handle)
{
    auto* encoder = reinterpret_cast<c_Detached_handle>(handle);

    uint32_t period = 0;
    c_Detached_GetStatusPeriod(encoder, c_Detached_kStatus2Period, &period);
    return static_cast<jint>(period);
}
