# minimal SDL shell

find_package(SDL2 REQUIRED)

add_executable(DaftBoySDL Main.cpp)

target_link_libraries(DaftBoySDL DaftBoyCore DaftBoyAdvanceCore SDL2::SDL2)