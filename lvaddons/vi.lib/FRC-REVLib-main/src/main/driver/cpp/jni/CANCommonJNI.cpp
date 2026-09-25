/*
 * Copyright (c) 2020-2025 REV Robotics
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

#include <fmt/format.h>

#include "com_revrobotics_jni_CANCommonJNI.h"
#include "rev/CANCommonParameters.h"

/*
 * Class:     com_revrobotics_jni_CANCommonJNI_c_1REVLib
 * Method:    1FlattenParameterInt32
 * Signature: (II)Ljava/lang/String;
 */
JNIEXPORT jstring JNICALL
Java_com_revrobotics_jni_CANCommonJNI_c_1REVLib_1FlattenParameterInt32
  (JNIEnv* env, jclass, jint parameterId, jint value)
{
    const uint32_t kMaxStringLength = 16;
    char flattenedString[kMaxStringLength];
    c_REVLib_FlattenParameterInt32(parameterId, value, flattenedString,
                                   kMaxStringLength);

    return env->NewStringUTF(flattenedString);
}

/*
 * Class:     com_revrobotics_jni_CANCommonJNI_c_1REVLib
 * Method:    1FlattenParameterUint32
 * Signature: (II)Ljava/lang/String;
 */
JNIEXPORT jstring JNICALL
Java_com_revrobotics_jni_CANCommonJNI_c_1REVLib_1FlattenParameterUint32
  (JNIEnv* env, jclass, jint parameterId, jint value)
{
    const uint32_t kMaxStringLength = 16;
    char flattenedString[kMaxStringLength];
    c_REVLib_FlattenParameterUint32(parameterId, value, flattenedString,
                                    kMaxStringLength);

    return env->NewStringUTF(flattenedString);
}

/*
 * Class:     com_revrobotics_jni_CANCommonJNI_c_1REVLib
 * Method:    1FlattenParameterFloat
 * Signature: (IF)Ljava/lang/String;
 */
JNIEXPORT jstring JNICALL
Java_com_revrobotics_jni_CANCommonJNI_c_1REVLib_1FlattenParameterFloat
  (JNIEnv* env, jclass, jint parameterId, jfloat value)
{
    const uint32_t kMaxStringLength = 16;
    char flattenedString[kMaxStringLength];
    c_REVLib_FlattenParameterFloat(parameterId, value, flattenedString,
                                   kMaxStringLength);

    return env->NewStringUTF(flattenedString);
}

/*
 * Class:     com_revrobotics_jni_CANCommonJNI_c_1REVLib
 * Method:    1FlattenParameterBool
 * Signature: (IZ)Ljava/lang/String;
 */
JNIEXPORT jstring JNICALL
Java_com_revrobotics_jni_CANCommonJNI_c_1REVLib_1FlattenParameterBool
  (JNIEnv* env, jclass, jint parameterId, jboolean value)
{
    const uint32_t kMaxStringLength = 16;
    char flattenedString[kMaxStringLength];
    c_REVLib_FlattenParameterBool(parameterId, value, flattenedString,
                                  kMaxStringLength);

    return env->NewStringUTF(flattenedString);
}
