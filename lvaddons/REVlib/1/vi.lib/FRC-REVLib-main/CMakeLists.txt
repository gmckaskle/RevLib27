# IMPORTANT!!! This is only for desktop-only drivers. It will not build a full REVLib

cmake_minimum_required(VERSION 3.22)

set(CMAKE_CXX_STANDARD 20)

set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS TRUE)

project(REVLibDesktopDrivers C CXX)

if(
    ${CMAKE_SYSTEM_NAME} STREQUAL "Windows"
    AND NOT ${CMAKE_GENERATOR} STREQUAL "Visual Studio 18 2026"
)
    message(FATAL_ERROR "On windows, Visual Studio 18 2026 must be used")
endif()

set(BACKEND_DRIVER_INCLUDE
    ${CMAKE_CURRENT_LIST_DIR}/src/main/backend-driver/include
)

add_subdirectory(src/main/message-queue-driver)
add_subdirectory(src/main/callback-driver)
