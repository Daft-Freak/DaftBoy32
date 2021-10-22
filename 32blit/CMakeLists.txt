blit_executable(DaftBoy32
    menu.cpp

    gameblit.cpp
)
blit_assets_yaml(DaftBoy32 assets.yml)
blit_metadata(DaftBoy32 metadata.yml)

add_subdirectory(DUH)

target_link_libraries(DaftBoy32 DaftBoyCore DUH)

if(32BLIT_PICO)
    target_compile_definitions(DaftBoy32 PRIVATE -DNO_GBC -DNO_AUDIO -DDISPLAY_RGB565)
else()
    set_target_properties(DaftBoy32 PROPERTIES INTERPROCEDURAL_OPTIMIZATION TRUE)
endif()