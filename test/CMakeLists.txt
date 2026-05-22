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

# WMM geomagnetic field lookup tables (declination + inclination + intensity)
add_executable(test_wmm_declination test_wmm_declination.cpp)
target_link_libraries(test_wmm_declination PRIVATE rc_fusion GTest::gtest_main)
gtest_discover_tests(test_wmm_declination)

# IVP-101: 3-axis magnetometer fusion
add_executable(test_eskf_mag_3axis test_eskf_mag_3axis.cpp)
target_link_libraries(test_eskf_mag_3axis PRIVATE rc_fusion GTest::gtest_main)
gtest_discover_tests(test_eskf_mag_3axis)

# ESKF GPS position & velocity measurement update
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
    ${CMAKE_SOURCE_DIR}/src/fusion/eskf.cpp
    ${CMAKE_SOURCE_DIR}/src/fusion/eskf_codegen.cpp
    ${CMAKE_SOURCE_DIR}/src/fusion/ud_factor.cpp
    ${CMAKE_SOURCE_DIR}/src/fusion/wmm_tables.cpp
    ${CMAKE_SOURCE_DIR}/src/fusion/mahony_ahrs.cpp
    ${CMAKE_SOURCE_DIR}/src/fusion/innovation_monitor.cpp
    ${CMAKE_SOURCE_DIR}/src/fusion/confidence_gate.cpp
)
target_link_libraries(rc_fusion_bierman PUBLIC rc_math)
target_compile_definitions(rc_fusion_bierman PUBLIC ESKF_USE_BIERMAN=1)

add_executable(test_eskf_bierman test_eskf_bierman.cpp)
target_link_libraries(test_eskf_bierman PRIVATE rc_fusion_bierman GTest::gtest_main)
gtest_discover_tests(test_eskf_bierman)

# IVP-49/50: Data model and timestamp tests
add_executable(test_data_model test_data_model.cpp)
target_link_libraries(test_data_model PRIVATE rc_logging GTest::gtest_main)
gtest_discover_tests(test_data_model)

# IVP-51: PCM frame format tests
add_executable(test_pcm_frame test_pcm_frame.cpp)
target_link_libraries(test_pcm_frame PRIVATE rc_logging GTest::gtest_main)
gtest_discover_tests(test_pcm_frame)

# IVP-52b: Ring buffer + crash recovery tests
add_executable(test_ring_buffer test_ring_buffer.cpp)
target_link_libraries(test_ring_buffer PRIVATE rc_logging GTest::gtest_main)
gtest_discover_tests(test_ring_buffer)

# IVP-52c: Log decimator tests
add_executable(test_log_decimator test_log_decimator.cpp)
target_link_libraries(test_log_decimator PRIVATE rc_logging GTest::gtest_main)
gtest_discover_tests(test_log_decimator)

# IVP-53a: CRC-32 + Flight log table tests
add_executable(test_flight_table test_flight_table.cpp)
target_link_libraries(test_flight_table PRIVATE rc_logging GTest::gtest_main)
gtest_discover_tests(test_flight_table)

# Pre-Stage 8: ESKF reset API tests (Flight Director)
add_executable(test_eskf_reset test_eskf_reset.cpp)
target_link_libraries(test_eskf_reset PRIVATE rc_fusion GTest::gtest_main)
gtest_discover_tests(test_eskf_reset)

# IVP-58: Telemetry encoder (CCSDS + MAVLink)
add_executable(test_telemetry_encoder test_telemetry_encoder.cpp)
target_link_libraries(test_telemetry_encoder PRIVATE rc_telemetry GTest::gtest_main)
gtest_discover_tests(test_telemetry_encoder)

# IVP-62b: MAVLink RX parser
add_executable(test_mavlink_rx test_mavlink_rx.cpp)
target_link_libraries(test_mavlink_rx PRIVATE rc_telemetry GTest::gtest_main)
gtest_discover_tests(test_mavlink_rx)

# Launch abort — flight_director safety posture flag (2026-04-22)
add_executable(test_flight_director_launch_abort test_flight_director_launch_abort.cpp)
target_link_libraries(test_flight_director_launch_abort PRIVATE rc_flight_director GTest::gtest_main)
gtest_discover_tests(test_flight_director_launch_abort)

# ESKF runaway-restart brake — see src/fusion/eskf_brake.cpp
add_executable(test_eskf_disable_brake test_eskf_disable_brake.cpp)
target_link_libraries(test_eskf_disable_brake PRIVATE rc_fusion GTest::gtest_main)
gtest_discover_tests(test_eskf_disable_brake)

# IVP-68: Flight Director HSM tests
add_executable(test_flight_director test_flight_director.cpp)
target_link_libraries(test_flight_director PRIVATE rc_flight_director GTest::gtest_main)
gtest_discover_tests(test_flight_director)

# IVP-131: Flight Director Replay — phase-window test against OpenRocket
# trajectory (council 2026-05-21, pivot from per-row state tolerance).
add_executable(test_flight_director_replay test_flight_director_replay.cpp)
target_link_libraries(test_flight_director_replay PRIVATE rc_flight_director GTest::gtest_main)
gtest_discover_tests(test_flight_director_replay
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

# IVP-69: Go/No-Go checks + command handler tests
add_executable(test_go_nogo test_go_nogo.cpp)
target_link_libraries(test_go_nogo PRIVATE rc_flight_director GTest::gtest_main)
gtest_discover_tests(test_go_nogo)

# Bench sim retirement (2026-04-11): command handler accept/reject matrix
add_executable(test_command_handler test_command_handler.cpp)
target_link_libraries(test_command_handler PRIVATE rc_flight_director GTest::gtest_main)
gtest_discover_tests(test_command_handler)

# R-25-exec gate scaffold (2026-05-13): test_mode three-condition gate +
# clearing semantics. Verifies the council-required runtime gate that
# replaces the parallel bench tier (Approach A).
add_executable(test_test_mode test_test_mode.cpp)
target_link_libraries(test_test_mode PRIVATE rc_flight_director GTest::gtest_main)
gtest_discover_tests(test_test_mode)

# IVP-123: haversine distance + bearing math
add_executable(test_haversine test_haversine.cpp)
target_link_libraries(test_haversine PRIVATE GTest::gtest_main)
gtest_discover_tests(test_haversine)

# IVP-70: Guard functions + evaluator tests
add_executable(test_guards test_guards.cpp)
target_link_libraries(test_guards PRIVATE rc_flight_director GTest::gtest_main)
gtest_discover_tests(test_guards)

# IVP-71: Guard combinator + safety lockout tests
add_executable(test_guard_combinator test_guard_combinator.cpp)
target_link_libraries(test_guard_combinator PRIVATE rc_flight_director GTest::gtest_main)
gtest_discover_tests(test_guard_combinator)

# IVP-72: Action executor + flight actions tests
add_executable(test_action_executor test_action_executor.cpp)
target_link_libraries(test_action_executor PRIVATE rc_flight_director GTest::gtest_main)
gtest_discover_tests(test_action_executor)

# IVP-83: Phase-scheduled Q/R tests
add_executable(test_phase_qr test_phase_qr.cpp)
target_link_libraries(test_phase_qr PRIVATE rc_fusion GTest::gtest_main)
gtest_discover_tests(test_phase_qr)

# IVP-84: Confidence gate tests
add_executable(test_confidence_gate test_confidence_gate.cpp)
target_link_libraries(test_confidence_gate PRIVATE rc_fusion GTest::gtest_main)
gtest_discover_tests(test_confidence_gate)

# IVP-83: Innovation monitor tests
add_executable(test_innovation_monitor test_innovation_monitor.cpp)
target_link_libraries(test_innovation_monitor PRIVATE rc_fusion GTest::gtest_main)
gtest_discover_tests(test_innovation_monitor)

# IVP-74: Mission profile configuration tests
add_executable(test_mission_profile test_mission_profile.cpp)
target_link_libraries(test_mission_profile PRIVATE rc_flight_director GTest::gtest_main)
target_include_directories(test_mission_profile PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
gtest_discover_tests(test_mission_profile)

# IVP-111: Health monitor encoding + contract tests
add_executable(test_health_monitor test_health_monitor.cpp)
target_link_libraries(test_health_monitor PRIVATE GTest::gtest_main)
target_include_directories(test_health_monitor PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/include)
gtest_discover_tests(test_health_monitor)

# Stage T Batch B IVP-T14: RF link-health state machine (pure helpers)
add_executable(test_rf_link_health test_rf_link_health.cpp)
target_link_libraries(test_rf_link_health PRIVATE GTest::gtest_main)
target_include_directories(test_rf_link_health PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/include)
gtest_discover_tests(test_rf_link_health)

# IVP-115: Notification engine resolver tests
add_executable(test_notify
    test_notify.cpp
    ${CMAKE_SOURCE_DIR}/src/notify/notify_backend_led.cpp
)
target_link_libraries(test_notify PRIVATE GTest::gtest_main)
target_include_directories(test_notify PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/src/notify
    ${CMAKE_SOURCE_DIR}/include)
target_compile_definitions(test_notify PRIVATE ROCKETCHIP_HOST_TEST=1)
gtest_discover_tests(test_notify)

# Stage L IVP-L3: pre-arm fail auto-clear helper (pure, header-only)
add_executable(test_prearm_fail_ticks test_prearm_fail_ticks.cpp)
target_link_libraries(test_prearm_fail_ticks PRIVATE GTest::gtest_main)
target_include_directories(test_prearm_fail_ticks PRIVATE
    ${CMAKE_SOURCE_DIR}/include)
gtest_discover_tests(test_prearm_fail_ticks)

# R-6c (2026-05-07 audit): Levenberg-Marquardt solver host coverage.
# Pure-function module extracted from calibration_manager.cpp; tests pin
# bytes-on-wire identity of the LM math under template dispatch.
add_executable(test_calibration_lm
    test_calibration_lm.cpp
    ${CMAKE_SOURCE_DIR}/src/calibration/lm_solver.cpp)
target_link_libraries(test_calibration_lm PRIVATE GTest::gtest_main)
target_include_directories(test_calibration_lm PRIVATE
    ${CMAKE_SOURCE_DIR})
gtest_discover_tests(test_calibration_lm)

# R-5 Unit B (2026-05-16): rc_log host tests — printf-subset formatter byte-on-wire
# parity against libc snprintf for the 13 format-spec patterns from
# docs/audits/STDIO_FORMAT_SPEC_INVENTORY_2026-05-15.md plus truncation
# and bench_sim-monitored log-line tests. Council 3-persona acceptance
# criterion #1 (float-rounding parity).
add_executable(test_rc_log test_rc_log.cpp)
target_link_libraries(test_rc_log PRIVATE rc_log GTest::gtest_main)
gtest_discover_tests(test_rc_log)

# rc_snprintf — buffer-bound sibling of rc_log (R-5 Unit G.1).
add_executable(test_rc_snprintf test_rc_snprintf.cpp)
target_link_libraries(test_rc_snprintf PRIVATE rc_log GTest::gtest_main)
gtest_discover_tests(test_rc_snprintf)

# -----------------------------------------------------------------------------
# Host Python: Tier 4 `_rc_test_common` harness + scripts/*.py syntax check.
#
# Requires Python 3 with `pip install pyserial` (imports `_rc_test_common`).
#
# Run with normal host build (`cmake .. -DBUILD_TESTS=ON`):
#   ctest -R scripts_
# or:
#   ctest -L python
# -----------------------------------------------------------------------------
find_package(Python3 COMPONENTS Interpreter)
if(Python3_Interpreter_FOUND)
    add_test(
        NAME scripts_rc_test_common
        COMMAND ${Python3_EXECUTABLE} scripts/test__rc_test_common.py
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    )
    add_test(
        NAME scripts_python_compileall
        COMMAND ${Python3_EXECUTABLE} -m compileall -q scripts
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    )
    # R-25-exec audit invariant — every fault_force_* entry checks
    # rc::test_mode_active() at line 1 of body. Mechanizes the previously-
    # manual grep + walk-through per
    # standards/CODING_STANDARDS.md "R-25-exec test-mode audit invariant".
    add_test(
        NAME scripts_fault_force_gate_audit
        COMMAND ${Python3_EXECUTABLE} scripts/audit/check_fault_force_gates.py
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    )
    # L2-P3 citation inventory — informational. Produces a deduplicated
    # population of citations across standards/*.md for audit cycles per
    # standards/AUDIT_GUIDANCE.md Tier 3.8 (L2-P2 exhaustive + L2-P4 scope).
    # Wired as a ctest so the script's own self-consistency is gated
    # (parses every standards/*.md without exception); the inventory output
    # itself is not checked against a baseline (it WILL change as standards
    # evolve — by design).
    add_test(
        NAME scripts_list_citations
        COMMAND ${Python3_EXECUTABLE} scripts/audit/list_citations.py
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    )
    # L2-P3 citation verification — walks the inventory and checks each
    # citation against its source. Phase 1 URL liveness via curl,
    # Phase 2 file-path liveness, Phase 3 LL/IVP/R-finding cross-refs,
    # Phase 4 emits a manual PDF checklist.
    #
    # NOT wired into the pre-commit gate: Phase 1 does ~18 HTTPS
    # requests (~30-60s wall time, transient 5xx common). That belongs
    # in audit-cycle runs, not per-commit. Invoke manually:
    #   python scripts/audit/verify_citations.py
    # OR via the audit-cycle label:
    #   ctest -L audit-cycle
    add_test(
        NAME scripts_verify_citations
        COMMAND ${Python3_EXECUTABLE} scripts/audit/verify_citations.py
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    )
    set_tests_properties(scripts_verify_citations PROPERTIES
        TIMEOUT 300
        LABELS "audit-cycle"
    )
    # C6.a Mission Profile threshold cross-check vs OpenRocket predictions
    # (council-approved 2026-05-22, see docs/decisions/
    # OPENROCKET_INTEGRATION_EVALUATION_2026-05-22.md). Advisory only —
    # surfaces threshold-vs-prediction conflicts; humans disposition.
    # Audit-cycle label (not per-commit) because changing motor or .rkt
    # could produce CONFLICT exit-1 without flight-critical implication.
    add_test(
        NAME scripts_check_mission_profile_vs_or
        COMMAND ${Python3_EXECUTABLE} scripts/audit/check_mission_profile_vs_or.py
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    )
    set_tests_properties(scripts_check_mission_profile_vs_or PROPERTIES
        LABELS "audit-cycle"
    )
    set_tests_properties(
        scripts_rc_test_common
        scripts_python_compileall
        scripts_fault_force_gate_audit
        scripts_list_citations
        PROPERTIES LABELS "python"
    )
else()
    message(WARNING "Python 3 interpreter not found — skipping scripts/ CMake tests")
endif()
