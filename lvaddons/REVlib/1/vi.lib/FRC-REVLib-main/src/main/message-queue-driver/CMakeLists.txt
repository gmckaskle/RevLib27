project(MessageQueueDriver)

file(GLOB MessageQueueSrcs ${CMAKE_CURRENT_LIST_DIR}/cpp/*.cpp)

add_library(MessageQueueDriver SHARED ${MessageQueueSrcs})

target_include_directories(
    MessageQueueDriver
    PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include
)
target_include_directories(MessageQueueDriver PUBLIC ${BACKEND_DRIVER_INCLUDE})
