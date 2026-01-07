add_executable(unit_tests
    test_pid.cpp
    test_filter.cpp
    test_fsm.cpp
)

target_link_libraries(unit_tests
    gtest_main
    drone_core
)

include(GoogleTest)
gtest_discover_tests(unit_tests)
