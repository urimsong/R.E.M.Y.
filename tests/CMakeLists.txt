add_executable(unit_tests
    test_main.cpp
)

enable_testing()
add_test(NAME unit_tests COMMAND unit_tests)
