cmake_minimum_required(VERSION 3.13.0)
project(gameblit)

#set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fsanitize=address")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")

option(BUILD_32BLIT ON "Build 32blit UI")
option(BUILD_TESTS OFF "Build test runner")

#add_definitions("-DPROFILER")

add_subdirectory(core)

if(BUILD_32BLIT)
    find_package (32BLIT CONFIG REQUIRED PATHS ../32blit-sdk)
    add_subdirectory(32blit DaftBoy32)
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
