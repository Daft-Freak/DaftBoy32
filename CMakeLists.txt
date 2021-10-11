cmake_minimum_required(VERSION 3.13.0)
project(gameblit)

#set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fsanitize=address")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")

find_package (32BLIT CONFIG REQUIRED PATHS ../32blit-sdk)

#add_definitions("-DPROFILER")

add_subdirectory(core)

blit_executable(gameblit
    control-icons.cpp
    file-browser.cpp
    menu.cpp

    gameblit.cpp
)
blit_assets_yaml(gameblit assets.yml)
blit_metadata(gameblit metadata.yml)

target_link_libraries(gameblit DaftBoyCore)
set_target_properties(gameblit PROPERTIES INTERPROCEDURAL_OPTIMIZATION TRUE)

# setup release packages
set(PROJECT_DISTRIBS LICENSE README.md)
install (FILES ${PROJECT_DISTRIBS} DESTINATION .)
set (CPACK_INCLUDE_TOPLEVEL_DIRECTORY OFF)
set (CPACK_GENERATOR "ZIP" "TGZ")
include (CPack)
