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

#include "com_revrobotics_jni_CANServoHubJNI.h"
#include "rev/CANServoHubDriver.h"
#include "rev/CANServoHubParameters.h"

extern "C" {

static jclass periodicStatus0Clazz;
static jclass periodicStatus1Clazz;
static jclass periodicStatus2Clazz;
static jclass periodicStatus3Clazz;
static jclass periodicStatus4Clazz;

static jclass loadClassWithGlobalRef(JNIEnv* env, std::string name) {
    jclass localClazz = env->FindClass(name.c_str());

    auto result = reinterpret_cast<jclass>(env->NewGlobalRef(localClazz));
    env->DeleteLocalRef(localClazz);

    return result;
}

void CANServoHubJNI_OnLoad(JNIEnv* env) {
    periodicStatus0Clazz = loadClassWithGlobalRef(
        env, "com/revrobotics/servohub/ServoHubLowLevel$PeriodicStatus0");
    periodicStatus1Clazz = loadClassWithGlobalRef(
        env, "com/revrobotics/servohub/ServoHubLowLevel$PeriodicStatus1");
    periodicStatus2Clazz = loadClassWithGlobalRef(
        env, "com/revrobotics/servohub/ServoHubLowLevel$PeriodicStatus2");
    periodicStatus3Clazz = loadClassWithGlobalRef(
        env, "com/revrobotics/servohub/ServoHubLowLevel$PeriodicStatus3");
    periodicStatus4Clazz = loadClassWithGlobalRef(
        env, "com/revrobotics/servohub/ServoHubLowLevel$PeriodicStatus4");

    // TODO(Landry): Cache fields globally? That's too much work for right now
    // :/
}

void CANServoHubJNI_OnUnLoad(JNIEnv* env) {
    env->DeleteGlobalRef(periodicStatus0Clazz);
    env->DeleteGlobalRef(periodicStatus1Clazz);
    env->DeleteGlobalRef(periodicStatus2Clazz);
    env->DeleteGlobalRef(periodicStatus3Clazz);
    env->DeleteGlobalRef(periodicStatus4Clazz);
}

/*
 * Class:     com_revrobotics_jni_CANServoHubJNI_c_1ServoHub
 * Method:    1RegisterId
 * Signature: (II)I
 */
JNIEXPORT jint JNICALL
Java_com_revrobotics_jni_CANServoHubJNI_c_1ServoHub_1RegisterId
  (JNIEnv*, jclass, jint busId, jint deviceId)
{
    return (jint)c_ServoHub_RegisterId(busId, deviceId);
}

/*
 * Class:     com_revrobotics_jni_CANServoHubJNI_c_1ServoHub
 * Method:    1Create
 * Signature: (II)J
 */
JNIEXPORT jlong JNICALL
Java_com_revrobotics_jni_CANServoHubJNI_c_1ServoHub_1Create
  (JNIEnv* env, jclass, jint busId, jint deviceId)
{
    c_REVLib_ErrorCode status;
    jlong handle =
        reinterpret_cast<jlong>(c_ServoHub_Create(busId, deviceId, &status));

    if (status != c_REVLibError_None) {
        jclass exceptionClass =
            env->FindClass("java/lang/IllegalStateException");
        if (exceptionClass != nullptr) {
            if (status == c_REVLibError_CantFindFirmware) {
                // Don't throw exception when no firmware is found. It's
                // possible the device is disconnected and we don't want to stop
                // the program if that is the case.
            } else if (status == c_REVLibError_FirmwareTooOld) {
                env->ThrowNew(exceptionClass,
                              fmt::format("The firmware version of Bus {} "
                                          "ServoHub #{} is too old and "
                                          "needs to be updated.",
                                          busId, deviceId)
                                  .c_str());
            } else if (status == c_REVLibError_FirmwareTooNew) {
                env->ThrowNew(
                    exceptionClass,
                    fmt::format("The firmware version of Bus {} ServoHub "
                                "#{} is too new for this "
                                "version of REVLib",
                                busId, deviceId)
                        .c_str());
            }
        }
    }

    return handle;
}

/*
 * Class:     com_revrobotics_jni_CANServoHubJNI_c_1ServoHub
 * Method:    1Close
 * Signature: (J)V
 */
JNIEXPORT void JNICALL
Java_com_revrobotics_jni_CANServoHubJNI_c_1ServoHub_1Close
  (JNIEnv*, jclass, jlong handle)
{
    c_ServoHub_Close(reinterpret_cast<c_ServoHub_handle>(handle));
}

/*
 * Class:     com_revrobotics_jni_CANServoHubJNI_c_1ServoHub
 * Method:    1Destroy
 * Signature: (J)V
 */
JNIEXPORT void JNICALL
Java_com_revrobotics_jni_CANServoHubJNI_c_1ServoHub_1Destroy
  (JNIEnv*, jclass, jlong handle)
{
    c_ServoHub_Destroy(reinterpret_cast<c_ServoHub_handle>(handle));
}

/*
 * Class:     com_revrobotics_jni_CANServoHubJNI_c_1ServoHub
 * Method:    1Configure
 * Signature: (JLjava/lang/String;Z)I
 */
JNIEXPORT jint JNICALL
Java_com_revrobotics_jni_CANServoHubJNI_c_1ServoHub_1Configure
  (JNIEnv* env, jclass, jlong handle, jstring flattenedString,
   jboolean resetSafeParameters)
{
    const char* str = env->GetStringUTFChars(flattenedString, nullptr);
    char* cstr = nullptr;
    if (str) {
        cstr = strdup(str);
    }

    env->ReleaseStringUTFChars(flattenedString, str);

    return static_cast<jint>(
        c_ServoHub_Configure(reinterpret_cast<c_ServoHub_handle>(handle), cstr,
                             static_cast<uint8_t>(resetSafeParameters)));
}

/*
 * Class:     com_revrobotics_jni_CANServoHubJNI_c_1ServoHub
 * Method:    1SetPeriodicFrameTimeout
 * Signature: (JI)V
 */
JNIEXPORT void JNICALL
Java_com_revrobotics_jni_CANServoHubJNI_c_1ServoHub_1SetPeriodicFrameTimeout
  (JNIEnv*, jclass, jlong handle, jint timeout_ms)
{
    c_ServoHub_SetPeriodicFrameTimeout(
        reinterpret_cast<c_ServoHub_handle>(handle), timeout_ms);
}

/*
 * Class:     com_revrobotics_jni_CANServoHubJNI_c_1ServoHub
 * Method:    1SetCANTimeout
 * Signature: (JI)I
 */
JNIEXPORT jint JNICALL
Java_com_revrobotics_jni_CANServoHubJNI_c_1ServoHub_1SetCANTimeout
  (JNIEnv*, jclass, jlong handle, jint timeout_ms)
{
    return static_cast<jint>(c_ServoHub_SetCANTimeout(
        reinterpret_cast<c_ServoHub_handle>(handle), timeout_ms));
}

/*
 * Class:     com_revrobotics_jni_CANServoHubJNI_c_1ServoHub
 * Method:    1SetCANMaxRetries
 * Signature: (JI)V
 */
JNIEXPORT void JNICALL
Java_com_revrobotics_jni_CANServoHubJNI_c_1ServoHub_1SetCANMaxRetries
  (JNIEnv*, jclass, jlong handle, jint numRetries)
{
    c_ServoHub_SetCANMaxRetries(reinterpret_cast<c_ServoHub_handle>(handle),
                                numRetries);
}

/*
 * Class:     com_revrobotics_jni_CANServoHubJNI_c_1ServoHub
 * Method:    1SetControlFramePeriod
 * Signature: (JI)V
 */
JNIEXPORT void JNICALL
Java_com_revrobotics_jni_CANServoHubJNI_c_1ServoHub_1SetControlFramePeriod
  (JNIEnv*, jclass, jlong handle, jint periodMs)
{
    c_ServoHub_SetControlFramePeriod(
        reinterpret_cast<c_ServoHub_handle>(handle), periodMs);
}

/*
 * Class:     com_revrobotics_jni_CANServoHubJNI_c_1ServoHub
 * Method:    1GetControlFramePeriod
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_revrobotics_jni_CANServoHubJNI_c_1ServoHub_1GetControlFramePeriod
  (JNIEnv*, jclass, jlong handle)
{
    return static_cast<jint>(c_ServoHub_GetControlFramePeriod(
        reinterpret_cast<c_ServoHub_handle>(handle)));
}

/*
 * Class:     com_revrobotics_jni_CANServoHubJNI_c_1ServoHub
 * Method:    1GetFirmwareVersion
 * Signature: (JLjava/lang/Object;)V
 */
JNIEXPORT void JNICALL
Java_com_revrobotics_jni_CANServoHubJNI_c_1ServoHub_1GetFirmwareVersion
  (JNIEnv* env, jclass, jlong handle, jobject version)
{
    c_ServoHub_FirmwareVersion driverFwVersion;
    c_ServoHub_GetFirmwareVersion(reinterpret_cast<c_ServoHub_handle>(handle),
                                  &driverFwVersion);

    jfieldID fwFix =
        env->GetFieldID(env->GetObjectClass(version), "firmwareFix", "I");
    env->SetIntField(version, fwFix, driverFwVersion.fwFix);

    jfieldID fwMinor =
        env->GetFieldID(env->GetObjectClass(version), "firmwareMinor", "I");
    env->SetIntField(version, fwMinor, driverFwVersion.fwMinor);

    jfieldID fwYear =
        env->GetFieldID(env->GetObjectClass(version), "firmwareYear", "I");
    env->SetIntField(version, fwYear, driverFwVersion.fwYear);

    jfieldID hwMinor =
        env->GetFieldID(env->GetObjectClass(version), "hardwareMinor", "I");
    env->SetIntField(version, hwMinor, driverFwVersion.hwMinor);

    jfieldID hwMajor =
        env->GetFieldID(env->GetObjectClass(version), "hardwareMajor", "I");
    env->SetIntField(version, hwMajor, driverFwVersion.hwMajor);
}

/*
 * Class:     com_revrobotics_jni_CANServoHubJNI_c_1ServoHub
 * Method:    1ClearFaults
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_revrobotics_jni_CANServoHubJNI_c_1ServoHub_1ClearFaults
  (JNIEnv*, jclass, jlong handle)
{
    return static_cast<jint>(
        c_ServoHub_ClearFaults(reinterpret_cast<c_ServoHub_handle>(handle)));
}

/*
 * Class:     com_revrobotics_jni_CANServoHubJNI_c_1ServoHub
 * Method:    1GetPeriodStatus0
 * Signature: (J)Ljava/lang/Object;
 */
JNIEXPORT jobject JNICALL
Java_com_revrobotics_jni_CANServoHubJNI_c_1ServoHub_1GetPeriodStatus0
  (JNIEnv* env, jclass, jlong handle)
{
    c_ServoHub_PeriodicStatus0 cStatus0;
    c_REVLib_ErrorCode code = c_ServoHub_GetPeriodicStatus0(
        static_cast<c_ServoHub_handle>(
            reinterpret_cast<c_ServoHub_handle>(handle)),
        &cStatus0);

    const auto constructor =
        env->GetMethodID(periodicStatus0Clazz, "<init>", "(DDDZZIZZIJ)V");
    jobject o =
        env->NewObject(periodicStatus0Clazz, constructor, cStatus0.voltage,
                       cStatus0.servoVoltage, cStatus0.deviceCurrent,
                       cStatus0.primaryHeartbeatLock, cStatus0.systemEnabled,
                       cStatus0.communicationMode, cStatus0.programmingEnabled,
                       cStatus0.activelyProgramming, code, cStatus0.timestamp);
    return o;
}

/*
 * Class:     com_revrobotics_jni_CANServoHubJNI_c_1ServoHub
 * Method:    1GetPeriodStatus1
 * Signature: (J)Ljava/lang/Object;
 */
JNIEXPORT jobject JNICALL
Java_com_revrobotics_jni_CANServoHubJNI_c_1ServoHub_1GetPeriodStatus1
  (JNIEnv* env, jclass, jlong handle)
{
    c_ServoHub_PeriodicStatus1 cStatus1;
    c_REVLib_ErrorCode code = c_ServoHub_GetPeriodicStatus1(
        static_cast<c_ServoHub_handle>(
            reinterpret_cast<c_ServoHub_handle>(handle)),
        &cStatus1);

    const auto constructor = env->GetMethodID(
        periodicStatus1Clazz, "<init>", "(ZZZZZZZZZZZZZZZZZZZZZZZZZZZZIJ)V");
    jobject o = env->NewObject(
        periodicStatus1Clazz, constructor, cStatus1.regulatorPowerGoodFault,
        cStatus1.brownout, cStatus1.canWarning, cStatus1.canBusOff,
        cStatus1.hardwareFault, cStatus1.firmwareFault, cStatus1.hasReset,
        cStatus1.lowBatteryFault, cStatus1.channel0Overcurrent,
        cStatus1.channel1Overcurrent, cStatus1.channel2Overcurrent,
        cStatus1.channel3Overcurrent, cStatus1.channel4Overcurrent,
        cStatus1.channel5Overcurrent, cStatus1.stickyRegulatorPowerGoodFault,
        cStatus1.stickyBrownout, cStatus1.stickyCanWarning,
        cStatus1.stickyCanBusOff, cStatus1.stickyHardwareFault,
        cStatus1.stickyFirmwareFault, cStatus1.stickyHasReset,
        cStatus1.stickyLowBatteryFault, cStatus1.stickyChannel0Overcurrent,
        cStatus1.stickyChannel1Overcurrent, cStatus1.stickyChannel2Overcurrent,
        cStatus1.stickyChannel3Overcurrent, cStatus1.stickyChannel4Overcurrent,
        cStatus1.stickyChannel5Overcurrent, code, cStatus1.timestamp);

    return o;
}

/*
 * Class:     com_revrobotics_jni_CANServoHubJNI_c_1ServoHub
 * Method:    1GetPeriodStatus2
 * Signature: (J)Ljava/lang/Object;
 */
JNIEXPORT jobject JNICALL
Java_com_revrobotics_jni_CANServoHubJNI_c_1ServoHub_1GetPeriodStatus2
  (JNIEnv* env, jclass, jlong handle)
{
    c_ServoHub_PeriodicStatus2 cStatus2;
    c_REVLib_ErrorCode code = c_ServoHub_GetPeriodicStatus2(
        static_cast<c_ServoHub_handle>(
            reinterpret_cast<c_ServoHub_handle>(handle)),
        &cStatus2);

    const auto constructor =
        env->GetMethodID(periodicStatus2Clazz, "<init>", "(SSSZZZZZZIJ)V");
    jobject o =
        env->NewObject(periodicStatus2Clazz, constructor,
                       cStatus2.channel0PulseWidth, cStatus2.channel1PulseWidth,
                       cStatus2.channel2PulseWidth, cStatus2.channel0Enabled,
                       cStatus2.channel1Enabled, cStatus2.channel2Enabled,
                       cStatus2.channel0OutOfRange, cStatus2.channel1OutOfRange,
                       cStatus2.channel2OutOfRange, code, cStatus2.timestamp);

    return o;
}

/*
 * Class:     com_revrobotics_jni_CANServoHubJNI_c_1ServoHub
 * Method:    1GetPeriodStatus3
 * Signature: (J)Ljava/lang/Object;
 */
JNIEXPORT jobject JNICALL
Java_com_revrobotics_jni_CANServoHubJNI_c_1ServoHub_1GetPeriodStatus3
  (JNIEnv* env, jclass, jlong handle)
{
    c_ServoHub_PeriodicStatus3 cStatus3;
    c_REVLib_ErrorCode code = c_ServoHub_GetPeriodicStatus3(
        static_cast<c_ServoHub_handle>(
            reinterpret_cast<c_ServoHub_handle>(handle)),
        &cStatus3);

    const auto constructor =
        env->GetMethodID(periodicStatus3Clazz, "<init>", "(SSSZZZZZZIJ)V");
    jobject o =
        env->NewObject(periodicStatus3Clazz, constructor,
                       cStatus3.channel3PulseWidth, cStatus3.channel4PulseWidth,
                       cStatus3.channel5PulseWidth, cStatus3.channel3Enabled,
                       cStatus3.channel4Enabled, cStatus3.channel5Enabled,
                       cStatus3.channel3OutOfRange, cStatus3.channel4OutOfRange,
                       cStatus3.channel5OutOfRange, code, cStatus3.timestamp);
    return o;
}

/*
 * Class:     com_revrobotics_jni_CANServoHubJNI_c_1ServoHub
 * Method:    1GetPeriodStatus4
 * Signature: (J)Ljava/lang/Object;
 */
JNIEXPORT jobject JNICALL
Java_com_revrobotics_jni_CANServoHubJNI_c_1ServoHub_1GetPeriodStatus4
  (JNIEnv* env, jclass, jlong handle)
{
    c_ServoHub_PeriodicStatus4 cStatus4;
    c_REVLib_ErrorCode code = c_ServoHub_GetPeriodicStatus4(
        static_cast<c_ServoHub_handle>(
            reinterpret_cast<c_ServoHub_handle>(handle)),
        &cStatus4);

    const auto constructor =
        env->GetMethodID(periodicStatus4Clazz, "<init>", "(DDDDDDIJ)V");
    jobject o = env->NewObject(
        periodicStatus4Clazz, constructor, cStatus4.channel0Current,
        cStatus4.channel1Current, cStatus4.channel2Current,
        cStatus4.channel3Current, cStatus4.channel4Current,
        cStatus4.channel5Current, code, cStatus4.timestamp);

    return o;
}

/*
 * Class:     com_revrobotics_jni_CANServoHubJNI_c_1ServoHub
 * Method:    1SetBankPulsePeriod
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL
Java_com_revrobotics_jni_CANServoHubJNI_c_1ServoHub_1SetBankPulsePeriod
  (JNIEnv*, jclass, jlong handle, jint bank, jint pulsePeriod_us)
{
    return (jint)c_ServoHub_SetBankPulsePeriod(
        reinterpret_cast<c_ServoHub_handle>(handle),
        static_cast<c_ServoHub_Bank>(bank),
        static_cast<int32_t>(pulsePeriod_us));
}

/*
 * Class:     com_revrobotics_jni_CANServoHubJNI_c_1ServoHub
 * Method:    1GetChannelPulseRange
 * Signature: (JILjava/lang/Object;)V
 */
JNIEXPORT void JNICALL
Java_com_revrobotics_jni_CANServoHubJNI_c_1ServoHub_1GetChannelPulseRange
  (JNIEnv* env, jclass, jlong handle, jint channelId, jobject pulseRange_us)
{
    c_ServoHub_ChannelPulseRange cPulseRange;
    c_ServoHub_GetChannelPulseRange(reinterpret_cast<c_ServoHub_handle>(handle),
                                    static_cast<c_ServoHub_Channel>(channelId),
                                    &cPulseRange);

    jfieldID minPulse =
        env->GetFieldID(env->GetObjectClass(pulseRange_us), "minPulse_us", "I");
    env->SetIntField(pulseRange_us, minPulse, cPulseRange.minPulse_us);
    jfieldID centerPulse = env->GetFieldID(env->GetObjectClass(pulseRange_us),
                                           "centerPulse_us", "I");
    env->SetIntField(pulseRange_us, centerPulse, cPulseRange.centerPulse_us);
    jfieldID maxPulse =
        env->GetFieldID(env->GetObjectClass(pulseRange_us), "maxPulse_us", "I");
    env->SetIntField(pulseRange_us, maxPulse, cPulseRange.maxPulse_us);
}

/*
 * Class:     com_revrobotics_jni_CANServoHubJNI_c_1ServoHub
 * Method:    1GetChannelDisableBehavior
 * Signature: (JI)Z
 */
JNIEXPORT jboolean JNICALL
Java_com_revrobotics_jni_CANServoHubJNI_c_1ServoHub_1GetChannelDisableBehavior
  (JNIEnv*, jclass, jlong handle, jint channelId)
{
    bool disableBehavior;
    c_ServoHub_GetChannelDisableBehavior(
        reinterpret_cast<c_ServoHub_handle>(handle),
        static_cast<c_ServoHub_Channel>(channelId), &disableBehavior);
    return static_cast<jboolean>(disableBehavior);
}

/*
 * Class:     com_revrobotics_jni_CANServoHubJNI_c_1ServoHub
 * Method:    1SetChannelPulseWidth
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL
Java_com_revrobotics_jni_CANServoHubJNI_c_1ServoHub_1SetChannelPulseWidth
  (JNIEnv*, jclass, jlong handle, jint channelId, jint pulseWidth_us)
{
    return (jint)c_ServoHub_SetChannelPulseWidth(
        reinterpret_cast<c_ServoHub_handle>(handle),
        static_cast<c_ServoHub_Channel>(channelId),
        static_cast<int32_t>(pulseWidth_us));
}

/*
 * Class:     com_revrobotics_jni_CANServoHubJNI_c_1ServoHub
 * Method:    1SetChannelEnabled
 * Signature: (JIZ)I
 */
JNIEXPORT jint JNICALL
Java_com_revrobotics_jni_CANServoHubJNI_c_1ServoHub_1SetChannelEnabled
  (JNIEnv*, jclass, jlong handle, jint channelId, jboolean enabled)
{
    return (jint)c_ServoHub_SetChannelEnabled(
        reinterpret_cast<c_ServoHub_handle>(handle),
        static_cast<c_ServoHub_Channel>(channelId), static_cast<bool>(enabled));
}

/*
 * Class:     com_revrobotics_jni_CANServoHubJNI_c_1ServoHub
 * Method:    1SetChannelPowered
 * Signature: (JIZ)I
 */
JNIEXPORT jint JNICALL
Java_com_revrobotics_jni_CANServoHubJNI_c_1ServoHub_1SetChannelPowered
  (JNIEnv*, jclass, jlong handle, jint channelId, jboolean powered)
{
    return (jint)c_ServoHub_SetChannelPowered(
        reinterpret_cast<c_ServoHub_handle>(handle),
        static_cast<c_ServoHub_Channel>(channelId), static_cast<bool>(powered));
}

/*
 * Class:     com_revrobotics_jni_CANServoHubJNI_c_1ServoHub
 * Method:    1ConfigureAsync
 * Signature: (JLjava/lang/String;Z)I
 */
JNIEXPORT jint JNICALL
Java_com_revrobotics_jni_CANServoHubJNI_c_1ServoHub_1ConfigureAsync
  (JNIEnv* env, jclass, jlong handle, jstring flattenedString,
   jboolean resetSafeParameters)
{
    const char* str = env->GetStringUTFChars(flattenedString, nullptr);
    char* cstr = nullptr;
    if (str) {
        cstr = strdup(str);
    }

    env->ReleaseStringUTFChars(flattenedString, str);

    return static_cast<jint>(c_ServoHub_ConfigureAsync(
        reinterpret_cast<c_ServoHub_handle>(handle), cstr,
        static_cast<uint8_t>(resetSafeParameters)));
}

/*
 * Class:     com_revrobotics_jni_CANServoHubJNI_c_1ServoHub
 * Method:    1GetParameterType
 * Signature: (I)I
 */
JNIEXPORT jint JNICALL
Java_com_revrobotics_jni_CANServoHubJNI_c_1ServoHub_1GetParameterType
  (JNIEnv*, jclass, jint paramId)
{
    return static_cast<jint>(c_ServoHub_GetParameterType(
        static_cast<c_ServoHub_ConfigParameter>(paramId)));
}

/*
 * Class:     com_revrobotics_jni_CANServoHubJNI_c_1ServoHub
 * Method:    1CreateSimFaultManager
 * Signature: (J)V
 */
JNIEXPORT void JNICALL
Java_com_revrobotics_jni_CANServoHubJNI_c_1ServoHub_1CreateSimFaultManager
  (JNIEnv*, jclass, jlong handle)
{
    c_SIM_ServoHub_CreateSimFaultManager(
        reinterpret_cast<c_ServoHub_handle>(handle));
}

}  // extern "C"
