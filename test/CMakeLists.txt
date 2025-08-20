find_package(GTest REQUIRED)

add_executable(test_warehouse_helpers
    test_warehouse_helpers.cpp
)

target_link_libraries(test_warehouse_helpers
    franka_drake_core
    GTest::GTest
    GTest::Main
)

add_test(NAME test_warehouse_helpers COMMAND test_warehouse_helpers)


