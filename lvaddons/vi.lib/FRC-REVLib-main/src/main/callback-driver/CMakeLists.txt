project(CallbackDriver)

file(GLOB CallbackSrcs ${CMAKE_CURRENT_LIST_DIR}/cpp/*.cpp)

add_library(CallbackDriver SHARED ${CallbackSrcs})

target_include_directories(
    CallbackDriver
    PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include
)
target_include_directories(CallbackDriver PUBLIC ${BACKEND_DRIVER_INCLUDE})
