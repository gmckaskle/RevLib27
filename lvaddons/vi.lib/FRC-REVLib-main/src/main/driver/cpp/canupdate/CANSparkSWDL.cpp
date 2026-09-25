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

#include "rev/canupdate/CANSparkSWDL.h"

#include <rev/driver/REVLibDriver.h>

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>

#include "rev/CANSparkFrames.h"
#include "rev/REVUtils.h"
#include "rev/canupdate/DfuSeFile.h"

extern "C" {

struct SWDLContext {
    std::vector<int32_t> deviceIds;
    uint8_t* buffer{};
    size_t fileLength{};

    int iterateStep{};
    unsigned int fileReadPosition{};
    uint64_t checksum{};

    ~SWDLContext() { delete[] buffer; }
};

typedef std::unordered_map<int, SWDLContext> ContextsByBus;

static ContextsByBus contextsByBus;

void c_Spark_ResetSWDL(int busId) { contextsByBus.erase(busId); }

bool c_Spark_SetSWDLDevices(int busId, const int numDevicesToAdd,
                            const int* deviceIDlist) {
    for (int i = 0; i < numDevicesToAdd; i++) {
        contextsByBus[busId].deviceIds.push_back(deviceIDlist[i]);
    }
    return true;
}

bool c_Spark_PrepSWDLBinFile(const char* binFileName, uint8_t*& buffer,
                             size_t* fileLen) {
    std::FILE* binFile = std::fopen(binFileName, "rb");
    if (!binFile) {
        std::fprintf(stderr, "Unable to open file %s\n", binFileName);
        return false;
    }

    // Get file length
    std::fseek(binFile, 0, SEEK_END);
    *fileLen = static_cast<size_t>(std::ftell(binFile));
    std::rewind(binFile);

    if (*fileLen == 0) {
        std::fprintf(stderr, "File is empty or could not determine length\n");
        std::fclose(binFile);
        return false;
    }

    // Allocate memory
    buffer = new uint8_t[*fileLen + 1];

    // Read file into buffer
    if (!fread(buffer, *fileLen, 1, binFile)) {
        std::fprintf(stderr, "Error reading file!\n");
        std::fclose(binFile);
        return false;
    }

    std::fclose(binFile);
    return true;
}

void c_Spark_EnableSWDLBootloader(int busId) {
    const uint8_t emptyArr[8]{};
    int32_t status = 0;

    for (const auto& deviceIDs = contextsByBus[busId].deviceIds;
         const int deviceID : deviceIDs) {
        getREVLibDriver()->writeCanFrame(
            busId, SPARK_ENTER_SWDL_CAN_BOOTLOADER_FRAME_ID + deviceID,
            emptyArr, 8, SEND_PERIOD_NO_REPEAT, &status);
    }
}

int c_Spark_IterateSWDL(int busId, const char* dfuFilePath,
                        const char* binFilePath) {
    auto& context = contextsByBus[busId];
    if (context.deviceIds.empty()) {
        std::fprintf(stderr, "No devices set for SWDL\n");
        c_Spark_ResetSWDL(busId);
        return -1;
    }

    switch (context.iterateStep) {
        case 0: {
            if (c_Spark_DFUtoBin(dfuFilePath, binFilePath)) {
                context.iterateStep++;
                return 0;
            }
            std::fprintf(stderr, "Failed to convert DFU to binary file\n");
            c_Spark_ResetSWDL(busId);
            return -1;  // Indicate an error
        }
        case 1: {
            auto& iterateBuffer = context.buffer;
            if (iterateBuffer != nullptr) {
                delete[] iterateBuffer;
            }
            context.fileLength = 0;
            if (c_Spark_PrepSWDLBinFile(binFilePath, iterateBuffer,
                                        &context.fileLength)) {
                context.iterateStep++;
                return 0;
            }
            std::fprintf(stderr, "Failed to set SWDL binary file\n");
            c_Spark_ResetSWDL(busId);
            return -1;  // Indicate an error
        }
        case 2: {
            c_Spark_EnableSWDLBootloader(busId);
            context.iterateStep++;
            return 0;
        }
        case 3: {
            if (const auto iterateBuffer = context.buffer;
                iterateBuffer == nullptr || context.fileLength == 0) {
                std::fprintf(stderr, "Invalid buffer or file length\n");
                c_Spark_ResetSWDL(busId);
                return -1;  // Indicate an error
            }
            context.fileReadPosition = 0u;
            context.checksum = 0;

            context.iterateStep++;
            return 0;
        }
        case 4: {
            const auto iterateBuffer = context.buffer;
            uint8_t tmpData[8] = {};
            int32_t status = 0;

            // This code should handle the case where the data is not a multiple
            // of 8 bytes, since tmpData is zeroed before this.
            std::memcpy(
                tmpData, &iterateBuffer[context.fileReadPosition],
                std::min(8U, static_cast<uint32_t>(context.fileLength -
                                                   context.fileReadPosition)));
            context.checksum += ArrToUint64(tmpData);

            getREVLibDriver()->writeCanFrame(busId, SPARK_SWDL_DATA_FRAME_ID,
                                             tmpData, 8, SEND_PERIOD_NO_REPEAT,
                                             &status);

            auto downloaded = static_cast<int8_t>(
                static_cast<float>(context.fileReadPosition) *
                (1.0f / static_cast<float>(context.fileLength)) * 100);

            if (downloaded >= 100) {
                downloaded = 99;
            }

            context.fileReadPosition += 8u;

            if (context.fileReadPosition >= context.fileLength) {
                context.iterateStep++;
                return 100;
            }
            return downloaded;
        }
        case 5: {
            uint8_t tmpData[8] = {};
            int32_t status = 0;

            Uint64ToArr(context.checksum, tmpData);

            getREVLibDriver()->writeCanFrame(
                busId, SPARK_SWDL_CHECKSUM_FRAME_ID, tmpData, 8,
                SEND_PERIOD_NO_REPEAT, &status);
            return 100;
        }
        default: {
            std::fprintf(stderr, "Unknown switch iteration step\n");
            c_Spark_ResetSWDL(busId);
            return -1;  // Indicate an error
        }
    }
}

bool c_Spark_SWDLChecksumFailFramePresent(int busId) {
    uint32_t id = SPARK_SWDL_RETRANSMIT_FRAME_ID;
    uint8_t data[8] = {};
    uint8_t dataSize = 0;
    uint64_t timeStamp = 0;
    const int32_t timeoutMs = 5000;  // 5 seconds timeout
    int32_t status = 0;

    getREVLibDriver()->receiveCanMessageTimeout(busId, &id, 0x1FFFFFFF, data,
                                                &dataSize, &timeStamp,
                                                timeoutMs, &status);

    if (status == 0) {
        // if fail frame received, iterateStep will be set to 0
        // which resets the SWDL process and prepares to retransmit
        contextsByBus[busId].iterateStep = 0;
        return true;  // Checksum fail frame received
    }
    return false;  // No checksum fail frame received
}

}  // extern "C"
