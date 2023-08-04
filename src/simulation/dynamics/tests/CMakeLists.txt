add_executable(test_categorizer test_categorizer.cpp)
target_link_libraries(test_categorizer GTest::gtest_main)
target_link_libraries(test_categorizer dynamicsLib)

if(CMAKE_HOST_SYSTEM_PROCESSOR STREQUAL "arm64" AND CMAKE_GENERATOR STREQUAL "Xcode")
    set(CMAKE_GTEST_DISCOVER_TESTS_DISCOVERY_MODE PRE_TEST)
endif()

gtest_discover_tests(test_categorizer)