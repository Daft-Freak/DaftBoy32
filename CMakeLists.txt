cmake_minimum_required(VERSION 3.13.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)

include(${32BLIT_DIR}/32blit-pico/sdk_import.cmake OPTIONAL)

project(gameblit)

#set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fsanitize=address")

if(MSVC)
  add_compile_options("/W4" "/wd4244" "/wd4324" "/wd4458" "/wd4100")
else()
  add_compile_options("-Wall" "-Wextra" "-Wdouble-promotion" "-Wno-unused-parameter")
endif()

option(BUILD_32BLIT "Build 32blit UI" ON)
option(BUILD_SDL "Build minimal SDL UI" OFF)
option(BUILD_TESTS "Build test runner" OFF)

#add_definitions("-DPROFILER")

add_subdirectory(core)

if(BUILD_32BLIT)
    find_package (32BLIT CONFIG REQUIRED PATHS ../32blit-sdk)
    add_subdirectory(32blit DaftBoy32)
endif()

if(BUILD_SDL)
    add_subdirectory(minsdl)
endif()

if(BUILD_TESTS)
    add_subdirectory(tests)
endif()

# setup release packages
set(PROJECT_DISTRIBS LICENSE README.md)
install (FILES ${PROJECT_DISTRIBS} DESTINATION .)
set (CPACK_INCLUDE_TOPLEVEL_DIRECTORY OFF)
set (CPACK_GENERATOR "ZIP" "TGZ")
include (CPack)
