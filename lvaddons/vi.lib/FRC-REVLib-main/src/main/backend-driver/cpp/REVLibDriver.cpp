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

#include "rev/driver/REVLibDriver.h"

#include <string>

#ifdef _WIN32
#define WPI_BINARY_NAME "REVLibWpi.dll"
#elif __APPLE__
#define WPI_BINARY_NAME "libREVLibWpi.dylib"
#else
#define WPI_BINARY_NAME "libREVLibWpi.so"
#endif

#ifdef _WIN32
#include <windows.h>
#define LIB_HANDLE HMODULE
#define LOAD_LIBRARY(name) LoadLibraryA(name)
#define GET_SYMBOL(handle, name) GetProcAddress(handle, name)
#define CLOSE_LIBRARY(handle) FreeLibrary(handle)
#else
#include <dlfcn.h>
#define LIB_HANDLE void*
#define LOAD_LIBRARY(name) dlopen(name, RTLD_LAZY)
#define GET_SYMBOL(handle, name) dlsym(handle, name)
#define CLOSE_LIBRARY(handle) dlclose(handle)
#endif

#include <iostream>

REVLibDriver* currentDriver = nullptr;

void tryLoadDriver() {
    const auto name = WPI_BINARY_NAME;
    LIB_HANDLE wpiHandle = LOAD_LIBRARY(name);

    if (wpiHandle != nullptr) {
        auto* getWpiDriver = reinterpret_cast<void* (*)()>(
            GET_SYMBOL(wpiHandle, "getRevLibWpiDriver"));

        if (getWpiDriver != nullptr) {
            currentDriver = static_cast<REVLibDriver*>(getWpiDriver());
        } else {
            std::cerr << "Unable to find wpi driver getter" << std::endl;
        }
    } else {
        std::cerr << "Unable to find wpi driver binary" << std::endl;
    }
    std::flush(std::cerr);
}

REVLibDriver* getREVLibDriver() {
    // TODO(Landry): Handle case where driver hasn't been setup?
    if (currentDriver == nullptr) {
        tryLoadDriver();
    }
    if (currentDriver == nullptr) {
        const auto errorText =
            std::string("Unable to load a REVLib driver. Either ensure that ") +
            WPI_BINARY_NAME +
            " is on the search path, or provide your own REVLibDriver subclass "
            "and call setREVLibDriver";
        // TODO        throw std::runtime_error(errorText);
    }
    return currentDriver;
}

void setREVLibDriver(REVLibDriver* newDriver) {
    if (newDriver == nullptr) {
        std::cout << "setREVLibDriver told to make driver NULL!" << std::endl;
    }

    if (currentDriver != nullptr) {
        std::cout << "setREVLibDriver already set! Overriding it" << std::endl;
    }
    currentDriver = newDriver;
}
