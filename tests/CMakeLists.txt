add_executable(test-runner
    runner.cpp
)
find_package(PNG REQUIRED)
target_link_libraries(test-runner PNG::PNG DaftBoyCore)