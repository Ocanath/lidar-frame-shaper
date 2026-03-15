cmake_minimum_required(VERSION 3.22)
project(lidar_frame_parser C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ============================================================================
# External Dependencies
# ============================================================================

# byte-stuffing (provides: cobs, PPP)
add_subdirectory(external/byte-stuffing)

# serial-cross-platform (provides: serial)
add_subdirectory(external/serial-cross-platform)

# dartt-protocol (provides: dartt_protocol, dartt_checksum)
add_subdirectory(external/dartt-protocol)

# nlohmann/json (provides: nlohmann_json::nlohmann_json)
# add_subdirectory(external/json)

# Eigen (header-only linear algebra)
add_subdirectory(external/eigen)

# ============================================================================
# Main Application
# ============================================================================

set(APP_SOURCES
    src/main.cpp
    src/dartt_init.cpp
    src/motor.cpp
	src/trig_fixed.c
	src/lidar.cpp
)

# ============================================================================
# Platform-specific SDL2 + target setup
# ============================================================================
if(WIN32)
    set(APP_TARGET ${PROJECT_NAME})
    add_executable(${PROJECT_NAME} ${APP_SOURCES})

    # tinycsocket socket libraries
    target_link_libraries(${PROJECT_NAME} wsock32 ws2_32 iphlpapi)

else()
    # Linux
    set(APP_TARGET ${PROJECT_NAME})
    add_executable(${PROJECT_NAME} ${APP_SOURCES})
endif()

# Common libraries for all platforms
target_link_libraries(${APP_TARGET}
    cobs
    dartt_protocol
    dartt_checksum
    Eigen3::Eigen
	serial
)
