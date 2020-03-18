cmake_minimum_required(VERSION 3.1.0)
set(32BLIT_PATH "../" CACHE PATH "Path to 32blit.cmake")
project(gameblit)

#set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fsanitize=address")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")

include(${32BLIT_PATH}/32blit.cmake)

#add_definitions("-DPROFILER")

blit_executable(gameblit
    DMGAPU.cpp
    DMGCPU.cpp
    gameblit.cpp
)
blit_assets_yaml(gameblit assets.yml)
