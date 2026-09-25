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

#include "MessageQueueDriver.h"

#include <rev/driver/REVLibDriver.h>

#include <chrono>
#include <cstring>
#include <iostream>
#include <locale>
#include <map>
#include <mutex>
#include <queue>
#include <thread>
#include <utility>

#include "MessageQueueDriverErrors.h"

#define MESSAGE_QUEUE_SIZE 50

typedef struct {
    CanMessage message;
    int32_t periodMs;
} CanMessageWithPeriod;

std::map<uint32_t, CanMessage> messagesById;
std::map<uint32_t, bool> isNew;

std::mutex sendMutex;
std::queue<CanMessageWithPeriod> messagesToSend;

typedef struct {
    int32_t busId;
    uint32_t messageId;
    uint32_t messageMask;
    uint32_t maxMessages;
} CanMessageStreamMetadata;

struct CanMessageStream {
    CanMessageStream() = default;
    explicit CanMessageStream(const CanMessageStreamMetadata& meta)
        : metadata(meta) {}
    CanMessageStreamMetadata metadata;
    std::mutex mutex;
    std::queue<CanMessage> messages;
};

uint32_t nextStreamHandle = 0;
std::map<uint32_t, CanMessageStream> messageStreams;

// handle, metadata to create
std::queue<std::pair<uint32_t, CanMessageStreamMetadata>>
    createMessageStreamRequests;
std::queue<uint32_t> closeMessageStreamRequests;
// associated message stream handle, message
std::queue<std::pair<uint32_t, CanMessage>> messageStreamEntries;

MessageQueueDriver messageQueueDriver;

// TODO - deal with busId
CanMessage* getMessage(int32_t busId, uint32_t id, uint32_t mask) {
    for (auto it = messagesById.begin(); it != messagesById.end(); ++it) {
        CanMessage* message = &it->second;
        if ((message->messageID & mask) == (id & mask)) {
            isNew[message->messageID] = false;
            return message;
        }
    }
    return nullptr;
}

// TODO - deal with busId
CanMessage* getMessageTimeout(uint32_t now, int32_t busId, uint32_t id,
                              uint32_t mask, uint32_t timeoutMs) {
    for (auto it = messagesById.begin(); it != messagesById.end(); ++it) {
        CanMessage* message = &it->second;
        const bool maskMatches = (message->messageID & mask) == (id & mask);
        const bool withinTimeout = now - message->timestamp <= timeoutMs;
        if (maskMatches && withinTimeout) {
            isNew[message->messageID] = false;
            return message;
        }
    }
    return nullptr;
}

void MessageQueueDriver::initializeDriver() {}

void MessageQueueDriver::receiveCanMessageLatest(
    int32_t busId, uint32_t* messageID, uint32_t messageIDMask, uint8_t* data,
    uint8_t* dataSize, uint64_t* timeStamp, int32_t* status) {
    const CanMessage* message = getMessage(busId, *messageID, messageIDMask);

    if (!message) {
        *status = NO_MESSAGE;
        return;
    }

    *messageID = message->messageID;
    *dataSize = message->dataSize;
    *timeStamp = message->timestamp;
    std::memcpy(data, message->data, *dataSize);

    *status = SUCCESS;
}
void MessageQueueDriver::receiveCanMessageTimeout(
    int32_t busId, uint32_t* messageID, const uint32_t messageIDMask,
    uint8_t* data, uint8_t* dataSize, uint64_t* timeStamp,
    const uint32_t timeout, int32_t* status) {
    const auto now = static_cast<uint32_t>(getPreciseTime() / 1000);
    const CanMessage* message =
        getMessageTimeout(now, busId, *messageID, messageIDMask, timeout);

    if (!message) {
        *status = NO_MESSAGE;
        return;
    }

    *messageID = message->messageID;
    *dataSize = message->dataSize;
    *timeStamp = message->timestamp;
    std::memcpy(data, message->data, *dataSize);

    *status = SUCCESS;
}

// TODO - deal with busId
void MessageQueueDriver::receiveCanMessageNew(int32_t busId, uint32_t id,
                                              uint8_t* data, uint8_t* dataSize,
                                              uint64_t* timestamp,
                                              int32_t* status) {
    if (!isNew[id]) {
        *status = NO_MESSAGE;
        return;
    }

    const CanMessage* message = &messagesById[id];
    *dataSize = message->dataSize;
    *timestamp = message->timestamp;
    std::memcpy(data, message->data, *dataSize);

    *status = SUCCESS;
}

// deal with busId
void MessageQueueDriver::writeCanFrame(int32_t busId, const uint32_t messageId,
                                       const uint8_t* data,
                                       const uint8_t dataSize, int32_t periodMs,
                                       int32_t* status) {
    CanMessageWithPeriod message = {
        .message =
            {
                .messageID = messageId,
                .dataSize = dataSize,
                .timestamp = static_cast<uint32_t>(getPreciseTime()),
                .isRtr = false,
            },
        .periodMs = periodMs,
    };
    std::memcpy(message.message.data, data, dataSize);
    std::unique_lock<std::mutex> lock(sendMutex);
    messagesToSend.push(message);
}
// deal with busId
void MessageQueueDriver::writeCanRtrFrame(int32_t busId, uint32_t messageId,
                                          uint8_t dataSize, int32_t periodMs,
                                          int32_t* status) {
    CanMessageWithPeriod message = {
        .message =
            {
                .messageID = messageId,
                .dataSize = dataSize,
                .timestamp = static_cast<uint32_t>(getPreciseTime()),
                .isRtr = true,
            },
        .periodMs = periodMs,
    };
    std::memset(message.message.data, 0, dataSize);
    std::unique_lock<std::mutex> lock(sendMutex);
    messagesToSend.push(message);
}

// TODO - deal with busId
int32_t MessageQueueDriver::createCanStream(int32_t busId, uint32_t messageId,
                                            uint32_t messageMask,
                                            uint32_t maxMessages,
                                            int32_t* status) {
    uint32_t handle = nextStreamHandle++;
    CanMessageStreamMetadata metadata = {
        .messageId = messageId,
        .messageMask = messageMask,
        .maxMessages = maxMessages,
    };
    createMessageStreamRequests.push(std::make_pair(handle, metadata));
    messageStreams.emplace(std::piecewise_construct,
                           std::forward_as_tuple(handle),
                           std::forward_as_tuple(metadata));
    *status = SUCCESS;
    return handle;
}
void MessageQueueDriver::receiveCanMessages(int32_t streamHandle,
                                            CanMessage* messages,
                                            uint32_t length, uint32_t* numRead,
                                            int32_t* status) {
    if (messageStreams.find(streamHandle) == messageStreams.end()) {
        *status = -1;
        *numRead = 0;
        return;
    }
    CanMessageStream& stream = messageStreams[streamHandle];
    std::lock_guard<std::mutex> lock(stream.mutex);
    *numRead = 0;
    while (!stream.messages.empty() && *numRead < length) {
        messages[*numRead] = stream.messages.front();
        stream.messages.pop();
        (*numRead)++;
    }
    if (*numRead == 0) {
        *status = NO_MESSAGE;
    } else {
        *status = SUCCESS;
    }
}
void MessageQueueDriver::closeStream(int32_t streamHandle) {
    if (messageStreams.find(streamHandle) == messageStreams.end()) {
        return;
    }
    closeMessageStreamRequests.push(streamHandle);
    messageStreams.erase(streamHandle);
}

// TODO(Landry): Will sim be useful for hardware client?
int32_t MessageQueueDriver::createSimDevice(const char* name) {
    std::cerr << "createSimDevice is a no-op in MessageQueueDriver"
              << std::endl;
    return 0;
}
int32_t MessageQueueDriver::createSimValue(int32_t device, const char* name,
                                           int direction,
                                           const REVLibValue defaultValue) {
    std::cerr << "createSimValue is a no-op in MessageQueueDriver" << std::endl;
    return 0;
}
void MessageQueueDriver::setSimValue(int32_t handle, REVLibValue* value) {
    std::cerr << "setSimValue is a no-op in MessageQueueDriver" << std::endl;
}
void MessageQueueDriver::getSimValue(int32_t handle, REVLibValue* value) {
    std::cerr << "getSimValue is a no-op in MessageQueueDriver" << std::endl;
}
void MessageQueueDriver::freeSimDevice(int32_t handle) {
    std::cerr << "freeSimDevice is a no-op in MessageQueueDriver" << std::endl;
}
void MessageQueueDriver::freeSimValue(int32_t handle) {
    std::cerr << "freeSimValue is a no-op in MessageQueueDriver" << std::endl;
}

bool MessageQueueDriver::areOutputsEnabled() { return true; }

uint32_t MessageQueueDriver::allocatePreciseTimer() { return 1; }
void MessageQueueDriver::preciseDelayMicroseconds(
    uint32_t handle, const uint64_t timeMicroseconds) {
    std::this_thread::sleep_for(std::chrono::microseconds(timeMicroseconds));
}
uint64_t MessageQueueDriver::getPreciseTime() {
    const auto now = std::chrono::system_clock::now().time_since_epoch();

    return std::chrono::duration_cast<std::chrono::microseconds>(now).count();
}
uint64_t MessageQueueDriver::getCANPacketBaseTime() { return getPreciseTime(); }
void MessageQueueDriver::closePreciseTimer(uint32_t handle) {}

const char* MessageQueueDriver::getErrorMessage(int status) {
    switch (status) {
        case SUCCESS:
            return "SUCCESS";
        case NO_MESSAGE:
            return "No Message";
        default:
            return "Unknown Error";
    }
}

void MessageQueueDriver::sendError(const int code, const char* message,
                                   bool printAlso) {
    std::cerr << "Got error (" << code << ") Message: " << message << std::endl;
}

void MessageQueueDriver::sendWarning(const int code, const char* message,
                                     bool printAlso) {
    std::cout << "Got warning (" << code << ") Message: " << message
              << std::endl;
}

void MessageQueueDriver::sendConsoleLine(const char* message) {
    std::cout << "Got console line Message: " << message << std::endl;
}

c_REVLib_ErrorCode MessageQueueDriver::driverErrorToRevlibError(
    const int status) {
    switch (status) {
        case SUCCESS:
            return c_REVLibError_None;
        case NO_MESSAGE:
            return c_REVLibError_CANTimeout;
        default:
            return c_REVLibError_HAL;
    }
}
void MessageQueueDriver::reportDeviceUsage(REVDevice device, int busId,
                                           int deviceId) {
    std::cerr << "reportDeviceUsage is a no-op in MessageQueueDriver"
              << std::endl;
}

REVLibDriver* getMessageQueueDriver() { return &messageQueueDriver; }

void submitFrame(const CanMessage* frame) {
    CanMessage message = {
        .messageID = frame->messageID,
        .dataSize = frame->dataSize,
        .timestamp = frame->timestamp,
    };
    std::memcpy(message.data, frame->data, frame->dataSize);
    messagesById.insert_or_assign(frame->messageID, message);

    isNew[frame->messageID] = true;
}

int getFrameToSend(CanMessage* message, uint32_t* periodMs) {
    std::unique_lock<std::mutex> lock(sendMutex);
    if (messagesToSend.empty()) {
        return 0;
    }
    const auto [messageToSend, messagePeriod] = messagesToSend.front();
    messagesToSend.pop();

    *periodMs = messagePeriod;
    *message = messageToSend;
    return 1;
}

void resetMessageQueue() {
    std::unique_lock lock(sendMutex);

    // clear send queue
    std::queue<CanMessageWithPeriod> empty;
    std::swap(messagesToSend, empty);

    isNew.clear();
    messagesById.clear();
}

void submitStreamFrame(const CanMessage* frame, uint32_t streamHandle) {
    if (messageStreams.find(streamHandle) == messageStreams.end()) {
        std::cerr << "Stream handle not found: " << streamHandle << std::endl;
        return;
    }

    CanMessageStream& stream = messageStreams[streamHandle];
    std::lock_guard<std::mutex> lock(stream.mutex);
    CanMessage message = {
        .messageID = frame->messageID,
        .dataSize = frame->dataSize,
        .timestamp = frame->timestamp,
    };
    std::memcpy(message.data, frame->data, frame->dataSize);

    if (stream.messages.size() < stream.metadata.maxMessages) {
        stream.messages.push(message);
    } else {
        std::cerr << "Stream full, dropping message for handle: "
                  << streamHandle << std::endl;
    }
}

int getStreamToCreate(uint32_t* handle, uint32_t* messageId,
                      uint32_t* messageMask, uint32_t* maxMessages) {
    if (createMessageStreamRequests.empty()) {
        return false;
    }
    auto request = createMessageStreamRequests.front();
    createMessageStreamRequests.pop();

    *handle = request.first;
    *messageId = request.second.messageId;
    *messageMask = request.second.messageMask;
    *maxMessages = request.second.maxMessages;
    return 1;
}

int getStreamToClose(uint32_t* handle) {
    if (closeMessageStreamRequests.empty()) {
        return 0;
    }
    *handle = closeMessageStreamRequests.front();
    closeMessageStreamRequests.pop();
    return 1;
}

void resetMessageStreams() {
    // clear message streams
    messageStreams.clear();

    // clear stream requests
    std::queue<std::pair<uint32_t, CanMessageStreamMetadata>>
        emptyCreateRequests;
    std::swap(createMessageStreamRequests, emptyCreateRequests);

    std::queue<uint32_t> emptyCloseRequests;
    std::swap(closeMessageStreamRequests, emptyCloseRequests);
}
