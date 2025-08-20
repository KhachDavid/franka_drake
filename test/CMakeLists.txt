find_package(GTest REQUIRED)

add_executable(test_pick_and_place_helpers
    test_pick_and_place_helpers.cpp
)

target_link_libraries(test_pick_and_place_helpers
    franka_drake_core
    GTest::GTest
    GTest::Main
)

add_test(NAME test_pick_and_place_helpers COMMAND test_pick_and_place_helpers)


