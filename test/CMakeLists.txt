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

# IVP-44: ESKF magnetometer heading measurement update
add_executable(test_eskf_mag_update test_eskf_mag_update.cpp)
target_link_libraries(test_eskf_mag_update PRIVATE rc_fusion GTest::gtest_main)
gtest_discover_tests(test_eskf_mag_update)

# IVP-44b: ESKF zero-velocity pseudo-measurement (ZUPT)
add_executable(test_eskf_zupt test_eskf_zupt.cpp)
target_link_libraries(test_eskf_zupt PRIVATE rc_fusion GTest::gtest_main)
gtest_discover_tests(test_eskf_zupt)

# IVP-44: WMM magnetic declination lookup table
add_executable(test_wmm_declination test_wmm_declination.cpp)
target_link_libraries(test_wmm_declination PRIVATE rc_fusion GTest::gtest_main)
gtest_discover_tests(test_wmm_declination)

# IVP-46: ESKF GPS position & velocity measurement update
add_executable(test_eskf_gps_update test_eskf_gps_update.cpp)
target_link_libraries(test_eskf_gps_update PRIVATE rc_fusion GTest::gtest_main)
gtest_discover_tests(test_eskf_gps_update)

# IVP-45: Mahony AHRS cross-check
add_executable(test_mahony test_mahony.cpp)
target_link_libraries(test_mahony PRIVATE rc_fusion GTest::gtest_main)
gtest_discover_tests(test_mahony)

# Bierman measurement update tests (ESKF_USE_BIERMAN=1)
# Separate fusion library with Bierman flag — existing rc_fusion stays unchanged.
add_library(rc_fusion_bierman STATIC
    ${CMAKE_SOURCE_DIR}/src/fusion/baro_kf.cpp
    ${CMAKE_SOURCE_DIR}/src/fusion/eskf.cpp
    ${CMAKE_SOURCE_DIR}/src/fusion/eskf_codegen.cpp
    ${CMAKE_SOURCE_DIR}/src/fusion/ud_factor.cpp
    ${CMAKE_SOURCE_DIR}/src/fusion/wmm_declination.cpp
    ${CMAKE_SOURCE_DIR}/src/fusion/mahony_ahrs.cpp
)
target_link_libraries(rc_fusion_bierman PUBLIC rc_math)
target_compile_definitions(rc_fusion_bierman PUBLIC ESKF_USE_BIERMAN=1)

add_executable(test_eskf_bierman test_eskf_bierman.cpp)
target_link_libraries(test_eskf_bierman PRIVATE rc_fusion_bierman GTest::gtest_main)
gtest_discover_tests(test_eskf_bierman)
