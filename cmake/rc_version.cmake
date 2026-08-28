# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (c) 2025-2026 Rocket Chip Project
#
# Generate ${CMAKE_BINARY_DIR}/generated/rocketchip/version.h
#
# Product numbers come from the repo-root RC_VERSION file (humans edit that
# only at a release sitting). Git identity is captured every configure.
# Reconfigure is forced when RC_VERSION, .git/HEAD, or .git/index move.
# See standards/VERSIONING.md.
#
# Include this module after project() from both the host (BUILD_TESTS)
# and target branches of the root CMakeLists.txt.

if(NOT DEFINED RC_BUILD_CONFIG)
    set(RC_BUILD_CONFIG "flight")
endif()

set(RC_VERSION_FILE "${CMAKE_SOURCE_DIR}/RC_VERSION")
if(NOT EXISTS "${RC_VERSION_FILE}")
    message(FATAL_ERROR "RocketChip RC_VERSION file missing: ${RC_VERSION_FILE}")
endif()

set(RC_VERSION_MAJOR "")
set(RC_VERSION_MINOR "")
set(RC_VERSION_PATCH "")
set(RC_VERSION_TWEAK "0")
set(RC_EXTRAVERSION "")

file(STRINGS "${RC_VERSION_FILE}" _rc_version_lines)
foreach(_line IN LISTS _rc_version_lines)
    if(_line MATCHES "^[ \t]*#")
        continue()
    endif()
    if(_line MATCHES "^VERSION_MAJOR[ \t]*=[ \t]*([0-9]+)")
        set(RC_VERSION_MAJOR "${CMAKE_MATCH_1}")
    elseif(_line MATCHES "^VERSION_MINOR[ \t]*=[ \t]*([0-9]+)")
        set(RC_VERSION_MINOR "${CMAKE_MATCH_1}")
    elseif(_line MATCHES "^PATCHLEVEL[ \t]*=[ \t]*([0-9]+)")
        set(RC_VERSION_PATCH "${CMAKE_MATCH_1}")
    elseif(_line MATCHES "^VERSION_TWEAK[ \t]*=[ \t]*([0-9]+)")
        set(RC_VERSION_TWEAK "${CMAKE_MATCH_1}")
    elseif(_line MATCHES "^EXTRAVERSION[ \t]*=[ \t]*(.*)$")
        string(STRIP "${CMAKE_MATCH_1}" RC_EXTRAVERSION)
        string(REGEX REPLACE "^\"(.*)\"$" "\\1" RC_EXTRAVERSION "${RC_EXTRAVERSION}")
        string(REGEX REPLACE "^'(.*)'$" "\\1" RC_EXTRAVERSION "${RC_EXTRAVERSION}")
    endif()
endforeach()

if(RC_VERSION_MAJOR STREQUAL "" OR RC_VERSION_MINOR STREQUAL "" OR RC_VERSION_PATCH STREQUAL "")
    message(FATAL_ERROR
        "RC_VERSION must set VERSION_MAJOR, VERSION_MINOR, and PATCHLEVEL (${RC_VERSION_FILE})")
endif()

set(RC_GIT_HASH "unknown")
set(RC_BUILD_IDENTITY "unknown")
set(RC_BUILD_NUMBER 0)

find_package(Git QUIET)
if(Git_FOUND OR GIT_FOUND)
    if(NOT GIT_EXECUTABLE)
        set(GIT_EXECUTABLE git)
    endif()

    execute_process(
        COMMAND ${GIT_EXECUTABLE} rev-parse --short HEAD
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        OUTPUT_VARIABLE RC_GIT_HASH
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
        RESULT_VARIABLE _rc_git_hash_rv
    )
    if(NOT _rc_git_hash_rv EQUAL 0 OR RC_GIT_HASH STREQUAL "")
        set(RC_GIT_HASH "unknown")
    endif()

    execute_process(
        COMMAND ${GIT_EXECUTABLE} describe --abbrev=12 --always --dirty
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        OUTPUT_VARIABLE RC_BUILD_IDENTITY
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
        RESULT_VARIABLE _rc_desc_rv
    )
    if(NOT _rc_desc_rv EQUAL 0 OR RC_BUILD_IDENTITY STREQUAL "")
        set(RC_BUILD_IDENTITY "${RC_GIT_HASH}")
    endif()

    execute_process(
        COMMAND ${GIT_EXECUTABLE} describe --tags --abbrev=0 --match "v*"
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        OUTPUT_VARIABLE RC_LAST_V_TAG
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
        RESULT_VARIABLE _rc_tag_rv
    )
    if(_rc_tag_rv EQUAL 0 AND NOT RC_LAST_V_TAG STREQUAL "")
        execute_process(
            COMMAND ${GIT_EXECUTABLE} rev-list --count ${RC_LAST_V_TAG}..HEAD
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
            OUTPUT_VARIABLE RC_BUILD_NUMBER
            OUTPUT_STRIP_TRAILING_WHITESPACE
            ERROR_QUIET
            RESULT_VARIABLE _rc_count_rv
        )
    else()
        execute_process(
            COMMAND ${GIT_EXECUTABLE} rev-list --count HEAD
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
            OUTPUT_VARIABLE RC_BUILD_NUMBER
            OUTPUT_STRIP_TRAILING_WHITESPACE
            ERROR_QUIET
            RESULT_VARIABLE _rc_count_rv
        )
    endif()
    if(NOT _rc_count_rv EQUAL 0 OR RC_BUILD_NUMBER STREQUAL "")
        set(RC_BUILD_NUMBER 0)
    endif()

    # Rebuild (reconfigure) when git moves. Worktrees have .git as a file,
    # so resolve HEAD/index via git rather than assuming .git/HEAD exists.
    execute_process(
        COMMAND ${GIT_EXECUTABLE} rev-parse --git-path HEAD
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        OUTPUT_VARIABLE _rc_git_head
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )
    execute_process(
        COMMAND ${GIT_EXECUTABLE} rev-parse --git-path index
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        OUTPUT_VARIABLE _rc_git_index
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )
    if(_rc_git_head AND NOT IS_ABSOLUTE "${_rc_git_head}")
        get_filename_component(_rc_git_head "${CMAKE_SOURCE_DIR}/${_rc_git_head}" ABSOLUTE)
    endif()
    if(_rc_git_index AND NOT IS_ABSOLUTE "${_rc_git_index}")
        get_filename_component(_rc_git_index "${CMAKE_SOURCE_DIR}/${_rc_git_index}" ABSOLUTE)
    endif()
    if(EXISTS "${_rc_git_head}")
        set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${_rc_git_head}")
    endif()
    if(EXISTS "${_rc_git_index}")
        set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${_rc_git_index}")
    endif()

    execute_process(
        COMMAND ${GIT_EXECUTABLE} symbolic-ref -q HEAD
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        OUTPUT_VARIABLE _rc_symref
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )
    if(_rc_symref)
        execute_process(
            COMMAND ${GIT_EXECUTABLE} rev-parse --git-path ${_rc_symref}
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
            OUTPUT_VARIABLE _rc_ref_path
            OUTPUT_STRIP_TRAILING_WHITESPACE
            ERROR_QUIET
        )
        if(_rc_ref_path AND NOT IS_ABSOLUTE "${_rc_ref_path}")
            get_filename_component(_rc_ref_path "${CMAKE_SOURCE_DIR}/${_rc_ref_path}" ABSOLUTE)
        endif()
        if(EXISTS "${_rc_ref_path}")
            set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${_rc_ref_path}")
        endif()
    endif()
endif()

set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${RC_VERSION_FILE}")

set(RC_FIRMWARE_VERSION "${RC_VERSION_MAJOR}.${RC_VERSION_MINOR}.${RC_VERSION_PATCH}")
if(RC_EXTRAVERSION STREQUAL "")
    set(RC_VERSION_STRING "${RC_FIRMWARE_VERSION}")
else()
    set(RC_VERSION_STRING "${RC_FIRMWARE_VERSION}-${RC_EXTRAVERSION}")
endif()

set(RC_VERSION_GENERATED_DIR "${CMAKE_BINARY_DIR}/generated")
file(MAKE_DIRECTORY "${RC_VERSION_GENERATED_DIR}/rocketchip")
configure_file(
    "${CMAKE_SOURCE_DIR}/include/rocketchip/version.h.in"
    "${RC_VERSION_GENERATED_DIR}/rocketchip/version.h"
    @ONLY
)

message(STATUS "RocketChip ${RC_VERSION_STRING}  git ${RC_GIT_HASH}  identity ${RC_BUILD_IDENTITY}  build ${RC_BUILD_NUMBER}")
