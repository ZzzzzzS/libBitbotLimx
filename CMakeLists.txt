cmake_minimum_required(VERSION 3.14 FATAL_ERROR)
project(bitbot_limx)


set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

aux_source_directory(src/bus SRC_BUS)
aux_source_directory(src/bus/cifx SRC_BUS)
aux_source_directory(src/device SRC_DEVICE)
aux_source_directory(src/kernel SRC_KERNEL)


if (${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
    set(LIB_PATH ${PROJECT_SOURCE_DIR}/lib/aarch64)
    set(LIMX_LIBS 
        ${LIB_PATH}/libpointfoot_sdk_lowlevel.so
    )
  elseif (${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86_64")
    set(LIB_PATH ${PROJECT_SOURCE_DIR}/lib/amd64)
    set(LIMX_LIBS 
        ${LIB_PATH}/libpointfoot_sdk_lowlevel.so
    )
  elseif(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "arm")
    set(LIB_PATH ${PROJECT_SOURCE_DIR}/lib/arm32)
    set(LIMX_LIBS 
        ${LIB_PATH}/libpointfoot_sdk_lowlevel.so
    )
  elseif (WIN32)
    set(LIB_PATH ${PROJECT_SOURCE_DIR}/lib/win)
    set(LIMX_LIBS 
        ${LIB_PATH}/libpointfoot_sdk_lowlevel.lib
    )
    #install(DIRECTORY ${LIB_PATH}/. DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
  else()
    message(FATAL_ERROR "Unsupported architecture!")
  endif()

message("

BUILDING BITBOT LIMX")
message("MAKE BITBOT EVERY WHERE!")

message("+----------------------------------+
| BBBB   III TTTTT BBBB   OOO  TTTTT|
| B   B   I    T   B   B O   O   T  |
| BBBB    I    T   BBBB  O   O   T  |
| B   B   I    T   B   B O   O   T  |
| BBBB   III   T   BBBB   OOO    T  |
|                                   |
|    L      III   M   M   X   X     |
|    L       I    MM MM    X X      |
|    L       I    M M M     X       |
|    L       I    M   M    X X      |
|    LLLLL  III   M   M   X   X     |
+----------------------------------+
")



add_library(BitbotLimx STATIC
            ./include/kernel/limx_kernel.hpp
            ${SRC_BUS}
            ${SRC_KERNEL}
            ${SRC_DEVICE}
)

target_include_directories(BitbotLimx PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(BitbotLimx 
    BitbotKernel
    ${LIMX_LIBS}
)

link_libraries(${LIMX_LIBS})