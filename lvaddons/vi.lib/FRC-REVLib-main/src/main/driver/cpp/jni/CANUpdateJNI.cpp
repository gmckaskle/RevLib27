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

#include <cstdlib>

#include "com_revrobotics_jni_CANUpdateJNI.h"
#include "rev/canupdate/CANSparkSWDL.h"

extern "C" {
/*
 * Class:     com_revrobotics_jni_CANUpdateJNI
 * Method:    SetSWDLDevices
 * Signature: (I[I)Z
 */
JNIEXPORT jboolean JNICALL
Java_com_revrobotics_jni_CANUpdateJNI_SetSWDLDevices
  (JNIEnv* env, jclass, jint busId, jintArray arr)
{
    int numDevices = env->GetArrayLength(arr);
    jint* deviceIDs = env->GetIntArrayElements(arr, 0);
    int* intDeviceIDs = static_cast<int*>(std::calloc(numDevices, sizeof(int)));
    if (intDeviceIDs == nullptr) {
        env->ReleaseIntArrayElements(arr, deviceIDs, 0);
        return JNI_FALSE;
    }

    for (int i = 0; i < numDevices; i++) {
        intDeviceIDs[i] = static_cast<int>(deviceIDs[i]);
    }

    env->ReleaseIntArrayElements(arr, deviceIDs, 0);
    jboolean result = c_Spark_SetSWDLDevices(busId, numDevices, intDeviceIDs)
                          ? JNI_TRUE
                          : JNI_FALSE;
    std::free(intDeviceIDs);
    return result;
}

/*
 * Class:     com_revrobotics_jni_CANUpdateJNI
 * Method:    ResetSWDL
 * Signature: (I)V
 */
JNIEXPORT void JNICALL
Java_com_revrobotics_jni_CANUpdateJNI_ResetSWDL
  (JNIEnv*, jclass, jint busId)
{
    c_Spark_ResetSWDL(busId);
}

/*
 * Class:     com_revrobotics_jni_CANUpdateJNI
 * Method:    IterateSWDL
 * Signature: (ILjava/lang/String;Ljava/lang/String;)I
 */
JNIEXPORT jint JNICALL
Java_com_revrobotics_jni_CANUpdateJNI_IterateSWDL
  (JNIEnv* env, jclass, jint busId, jstring dfu, jstring bin)
{
    const char* dfuFilePath = env->GetStringUTFChars(dfu, 0);
    const char* binFilePath = env->GetStringUTFChars(bin, 0);

    int retVal = c_Spark_IterateSWDL(busId, dfuFilePath, binFilePath);

    env->ReleaseStringUTFChars(dfu, dfuFilePath);
    env->ReleaseStringUTFChars(bin, binFilePath);
    return retVal;
}

/*
 * Class:     com_revrobotics_jni_CANUpdateJNI
 * Method:    SWDLChecksumFailFramePresent
 * Signature: (I)Z
 */
JNIEXPORT jboolean JNICALL
Java_com_revrobotics_jni_CANUpdateJNI_SWDLChecksumFailFramePresent
  (JNIEnv*, jclass, jint busId)
{
    bool result = c_Spark_SWDLChecksumFailFramePresent(busId);
    return result ? JNI_TRUE : JNI_FALSE;
}

}  // extern "C"
