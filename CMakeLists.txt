cmake_minimum_required(VERSION 3.20)
project(scan_to_drape LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (EMSCRIPTEN)
    # Web build: GLFW/GL come from Emscripten's own compatibility layer (-s USE_GLFW=3,
    # WebGL2 via FULL_ES3), not vcpkg -- only glm/stb are still needed as header-only libs,
    # so their vcpkg include dir is added directly rather than going through find_package's
    # binary-target resolution (which assumes a vcpkg triplet this toolchain doesn't have).
    add_executable(viewer src/main.cpp)
    target_include_directories(viewer PRIVATE include "C:/coding/vcpkg/installed/x64-windows/include")
    target_compile_definitions(viewer PRIVATE SHADER_DIR="shaders")

    set(EMSCRIPTEN_LINK_FLAGS
        "-s USE_GLFW=3 -s FULL_ES3=1 -s MIN_WEBGL_VERSION=2 -s MAX_WEBGL_VERSION=2 \
         -s ALLOW_MEMORY_GROWTH=0 -s INITIAL_MEMORY=268435456 -s EXIT_RUNTIME=0 \
         -s EXPORTED_RUNTIME_METHODS=['ccall'] \
         --preload-file ${CMAKE_SOURCE_DIR}/models/mechander_web.ply@mechander_web.ply \
         --preload-file ${CMAKE_SOURCE_DIR}/shaders@shaders \
         --shell-file ${CMAKE_SOURCE_DIR}/web/shell.html")
    set_target_properties(viewer PROPERTIES
        SUFFIX ".html"
        LINK_FLAGS "${EMSCRIPTEN_LINK_FLAGS}"
    )
else()
    find_package(glfw3 CONFIG REQUIRED)
    find_package(glad CONFIG REQUIRED)
    find_package(glm CONFIG REQUIRED)
    find_package(Stb REQUIRED)

    add_executable(viewer
        src/main.cpp
    )

    target_include_directories(viewer PRIVATE include ${Stb_INCLUDE_DIR})
    target_link_libraries(viewer PRIVATE glfw glad::glad glm::glm)

    target_compile_definitions(viewer PRIVATE SHADER_DIR="${CMAKE_SOURCE_DIR}/shaders")

    if (MSVC)
        # Source files are UTF-8 (em-dashes etc. in comments); without this MSVC reads them
        # using the system codepage (949/Korean on this machine) and warns C4819.
        target_compile_options(viewer PRIVATE /W4 /utf-8)
    endif()
endif()
