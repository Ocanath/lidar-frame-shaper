cmake_minimum_required(VERSION 3.22)
project(spooler_controller C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ============================================================================
# External Dependencies
# ============================================================================

# OpenGL (not needed on Android — uses GLESv3)
if(NOT ANDROID)
    find_package(OpenGL REQUIRED)
endif()

# ImGui (no CMakeLists.txt, add manually)
set(IMGUI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/imgui)
add_library(imgui STATIC
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/backends/imgui_impl_sdl2.cpp
    ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
)
target_include_directories(imgui PUBLIC
    ${IMGUI_DIR}
    ${IMGUI_DIR}/backends
)
if(ANDROID)
    target_compile_definitions(imgui PUBLIC IMGUI_IMPL_OPENGL_ES3)
    target_link_libraries(imgui PUBLIC SDL2 GLESv3 EGL)
else()
    target_include_directories(imgui PUBLIC ${SDL2_INCLUDE_DIRS})
    target_link_libraries(imgui PUBLIC ${SDL2_LIBRARIES} OpenGL::GL)
endif()

# byte-stuffing (provides: cobs, PPP)
add_subdirectory(external/byte-stuffing)

# serial-cross-platform (provides: serial)
# add_subdirectory(external/serial-cross-platform)

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
    src/plotting.cpp
    src/colors.cpp
    src/ui.cpp
    src/motor.cpp
    src/spooler_robot.cpp
	src/trig_fixed.c
)

# ============================================================================
# Platform-specific SDL2 + target setup
# ============================================================================
if(ANDROID)
    # Build SDL2 from source (required for Android)
    add_subdirectory(external/SDL2-src)

    # SDLActivity always loads "libmain.so" — library must be named "main"
    add_library(main SHARED ${APP_SOURCES})
    set(APP_TARGET main)

    target_compile_definitions(main PRIVATE IMGUI_IMPL_OPENGL_ES3)

    # Android NDK lacks linux/if_packet.h
    target_compile_definitions(main PRIVATE TCS_MISSING_AF_PACKET)

    target_link_libraries(main
        SDL2::SDL2
        SDL2::SDL2main
        GLESv3
        EGL
        android
        log
    )

    # Ensure SDL_main is linked
    target_link_options(main PRIVATE "-u" "SDL_main")

    # Android 15+ requires 16KB-aligned ELF load segments
    target_link_options(main PRIVATE "-Wl,-z,max-page-size=16384")

elseif(WIN32)
    set(APP_TARGET ${PROJECT_NAME})
    add_executable(${PROJECT_NAME} ${APP_SOURCES})

    # Path to your local SDL2 copy
    set(SDL2_DIR "${CMAKE_SOURCE_DIR}/external/SDL")

    # Determine the target architecture
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        set(SDL2_LIB_DIR "${SDL2_DIR}/lib/x64")
    else()
        set(SDL2_LIB_DIR "${SDL2_DIR}/lib/x86")
    endif()

    include_directories(${SDL2_DIR}/include)

    target_link_libraries(${PROJECT_NAME}
        ${SDL2_LIB_DIR}/SDL2main.lib
        ${SDL2_LIB_DIR}/SDL2.lib
    )

    # tinycsocket socket libraries
    target_link_libraries(${PROJECT_NAME} wsock32 ws2_32 iphlpapi)

else()
    # Linux
    set(APP_TARGET ${PROJECT_NAME})
    add_executable(${PROJECT_NAME} ${APP_SOURCES})

    find_package(SDL2 REQUIRED)
    include_directories(${SDL2_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} ${SDL2_LIBRARIES})

endif()

# Common libraries for all platforms
target_link_libraries(${APP_TARGET}
    imgui
    cobs
    PPP
    dartt_protocol
    dartt_checksum
    Eigen3::Eigen
)
