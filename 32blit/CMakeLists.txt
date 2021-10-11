blit_executable(DaftBoy32
    control-icons.cpp
    file-browser.cpp
    menu.cpp

    gameblit.cpp
)
blit_assets_yaml(DaftBoy32 assets.yml)
blit_metadata(DaftBoy32 metadata.yml)

target_link_libraries(DaftBoy32 DaftBoyCore)
set_target_properties(DaftBoy32 PROPERTIES INTERPROCEDURAL_OPTIMIZATION TRUE)