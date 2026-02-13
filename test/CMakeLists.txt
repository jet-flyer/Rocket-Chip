# RocketChip Host Tests
# Registered via add_subdirectory() from top-level CMakeLists.txt
# Test files are added here as each IVP step is implemented.

include(GoogleTest)

# IVP-39: Vec3 math library
add_executable(test_vec3 test_vec3.cpp)
target_link_libraries(test_vec3 PRIVATE rc_math GTest::gtest_main)
gtest_discover_tests(test_vec3)

# IVP-39: Quaternion math library
add_executable(test_quat test_quat.cpp)
target_link_libraries(test_quat PRIVATE rc_math GTest::gtest_main)
gtest_discover_tests(test_quat)

# IVP-40: Matrix operations + named state indices
add_executable(test_mat test_mat.cpp)
target_link_libraries(test_mat PRIVATE rc_math GTest::gtest_main)
gtest_discover_tests(test_mat)

# IVP-41: 1D barometric altitude Kalman filter
add_executable(test_baro_kf test_baro_kf.cpp)
target_link_libraries(test_baro_kf PRIVATE rc_fusion GTest::gtest_main)
gtest_discover_tests(test_baro_kf)

# IVP-42a: ESKF core propagation + IVP-42b: CSV trajectory tests
add_executable(test_eskf_propagation test_eskf_propagation.cpp)
target_link_libraries(test_eskf_propagation PRIVATE rc_fusion GTest::gtest_main)
gtest_discover_tests(test_eskf_propagation
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

# IVP-42c: Replay harness (standalone tool, not a test)
add_executable(replay_harness replay/replay_harness.cpp)
target_link_libraries(replay_harness PRIVATE rc_fusion)
target_include_directories(replay_harness PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_definitions(replay_harness PRIVATE _CRT_SECURE_NO_WARNINGS)

# IVP-42c: Replay regression test
add_executable(test_replay_regression test_replay_regression.cpp)
target_link_libraries(test_replay_regression PRIVATE rc_fusion GTest::gtest_main)
target_include_directories(test_replay_regression PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
gtest_discover_tests(test_replay_regression
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

# IVP-43: ESKF barometric altitude measurement update
add_executable(test_eskf_update test_eskf_update.cpp)
target_link_libraries(test_eskf_update PRIVATE rc_fusion GTest::gtest_main)
gtest_discover_tests(test_eskf_update)
