blit_executable(DaftBoy32
    menu.cpp

    gameblit.cpp
)
blit_assets_yaml(DaftBoy32 assets.yml)
blit_metadata(DaftBoy32 metadata.yml)

add_subdirectory(DUH)

target_link_libraries(DaftBoy32 DaftBoyCore DUH)

if(32BLIT_PICO)
    if(${PICO_BOARD} STREQUAL "pimoroni_picosystem")
        target_compile_definitions(DaftBoy32 PRIVATE -DDISPLAY_RGB565)
    elseif(${PICO_ADDON} STREQUAL "pimoroni_picovision")
        target_compile_definitions(DaftBoy32 PRIVATE -DDISPLAY_RB_SWAP)
    endif()
else()
    set_target_properties(DaftBoy32 PROPERTIES INTERPROCEDURAL_OPTIMIZATION TRUE)
endif()