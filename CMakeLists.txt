cmake_minimum_required(VERSION 3.9.0)
project(gameblit)
set(32BLIT_PATH "../" CACHE PATH "Path to 32blit.cmake")

#set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fsanitize=address")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")

include(${32BLIT_PATH}/32blit.cmake)

#add_definitions("-DPROFILER")

blit_executable(gameblit
    control-icons.cpp
    file-browser.cpp
    menu.cpp

    gameblit.cpp

    DMGAPU.cpp
    DMGCPU.cpp
    DMGDisplay.cpp
    DMGMemory.cpp
)
blit_assets_yaml(gameblit assets.yml)
blit_metadata(gameblit metadata.yml)

set_target_properties(gameblit PROPERTIES INTERPROCEDURAL_OPTIMIZATION TRUE)

# setup release packages
install (FILES ${PROJECT_DISTRIBS} DESTINATION .)
set (CPACK_INCLUDE_TOPLEVEL_DIRECTORY OFF)
set (CPACK_GENERATOR "ZIP" "TGZ")
include (CPack)