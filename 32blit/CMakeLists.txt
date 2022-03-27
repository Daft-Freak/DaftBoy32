blit_executable(DaftBoy32
    menu.cpp

    gameblit.cpp
)
blit_assets_yaml(DaftBoy32 assets.yml)
blit_metadata(DaftBoy32 metadata.yml)

add_subdirectory(DUH)

target_link_libraries(DaftBoy32 DaftBoyCore DUH)

if(32BLIT_PICO)
    target_compile_definitions(DaftBoy32 PRIVATE -DDISPLAY_RGB565)

    if(${PICO_BOARD} STREQUAL "vgaboard")
        # this requires some patches to the SDK
        target_compile_definitions(DaftBoy32 PRIVATE ALLOW_HIRES=1 DISPLAY_WIDTH=212 DISPLAY_HEIGHT=160)
    endif()

    # LTO, but just the emu code
    get_target_property(DB32_SOURCES DaftBoy32 SOURCES)
    get_target_property(CORE_SOURCES DaftBoyCore INTERFACE_SOURCES)
    set_source_files_properties(${CORE_SOURCES} ${DB32_SOURCES} COMPILE_FLAGS -flto)
else()
    set_target_properties(DaftBoy32 PROPERTIES INTERPROCEDURAL_OPTIMIZATION TRUE)
endif()
