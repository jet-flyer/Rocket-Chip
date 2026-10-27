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
#
# `cmake -DRC_FW_TREE_SOURCE=<repo> -P cmake/rc_version.cmake` prints the
# firmware-tree id (same value as scripts/firmware_tree.py elf_tree_id,
# plus -dirty when a non-excluded [elf] path is dirty).

# Git pathspec glob -> full-match regex. * does not match /. ** crosses /.
# Same shapes as scripts/firmware_tree.py glob_to_regex.
function(rc_glob_to_regex pattern out_var)
    string(LENGTH "${pattern}" _n)
    set(_i 0)
    set(_re "^")
    while(_i LESS _n)
        math(EXPR _left "${_n} - ${_i}")
        set(_four "")
        set(_three "")
        if(_left GREATER_EQUAL 4)
            string(SUBSTRING "${pattern}" ${_i} 4 _four)
        endif()
        if(_left GREATER_EQUAL 3)
            string(SUBSTRING "${pattern}" ${_i} 3 _three)
        endif()
        if(_four STREQUAL "/**/")
            string(APPEND _re "/(.*/)?")
            math(EXPR _i "${_i} + 4")
        elseif(_three STREQUAL "**/")
            string(APPEND _re "(.*/)?")
            math(EXPR _i "${_i} + 3")
        elseif(_three STREQUAL "/**" AND _left EQUAL 3)
            string(APPEND _re "/.*")
            math(EXPR _i "${_i} + 3")
        elseif(_left GREATER_EQUAL 2)
            string(SUBSTRING "${pattern}" ${_i} 2 _two)
            if(_two STREQUAL "**")
                string(APPEND _re ".*")
                math(EXPR _i "${_i} + 2")
            else()
                rc_glob_one_char("${pattern}" ${_i} _piece _step)
                string(APPEND _re "${_piece}")
                math(EXPR _i "${_i} + ${_step}")
            endif()
        else()
            rc_glob_one_char("${pattern}" ${_i} _piece _step)
            string(APPEND _re "${_piece}")
            math(EXPR _i "${_i} + ${_step}")
        endif()
    endwhile()
    string(APPEND _re "$")
    set(${out_var} "${_re}" PARENT_SCOPE)
endfunction()

function(rc_glob_one_char pattern index out_piece out_step)
    string(SUBSTRING "${pattern}" ${index} 1 _ch)
    if(_ch STREQUAL "*")
        set(${out_piece} "[^/]*" PARENT_SCOPE)
    elseif(_ch STREQUAL "?")
        set(${out_piece} "[^/]" PARENT_SCOPE)
    else()
        set(_specials ".^$+*?()[]{}|\\")
        string(FIND "${_specials}" "${_ch}" _pos)
        if(NOT _pos EQUAL -1)
            set(${out_piece} "\\${_ch}" PARENT_SCOPE)
        else()
            set(${out_piece} "${_ch}" PARENT_SCOPE)
        endif()
    endif()
    set(${out_step} 1 PARENT_SCOPE)
endfunction()

function(rc_glob_matches path pattern out_var)
    rc_glob_to_regex("${pattern}" _re)
    if("${path}" MATCHES "${_re}")
        set(${out_var} TRUE PARENT_SCOPE)
    else()
        set(${out_var} FALSE PARENT_SCOPE)
    endif()
endfunction()

# Strip one pair of git C-quotes and undo \\ \" \n \t. Octal bytes use
# string(ASCII), which matches the Python filter for bytes 0-127.
function(rc_git_unquote in out_var)
    string(LENGTH "${in}" _n)
    if(_n LESS 2)
        set(${out_var} "${in}" PARENT_SCOPE)
        return()
    endif()
    string(SUBSTRING "${in}" 0 1 _a)
    math(EXPR _last "${_n} - 1")
    string(SUBSTRING "${in}" ${_last} 1 _b)
    if(NOT _a STREQUAL "\"" OR NOT _b STREQUAL "\"")
        set(${out_var} "${in}" PARENT_SCOPE)
        return()
    endif()
    math(EXPR _inner "${_n} - 2")
    string(SUBSTRING "${in}" 1 ${_inner} _body)
    string(LENGTH "${_body}" _bn)
    set(_res "")
    set(_i 0)
    while(_i LESS _bn)
        string(SUBSTRING "${_body}" ${_i} 1 _c)
        if(NOT _c STREQUAL "\\")
            string(APPEND _res "${_c}")
            math(EXPR _i "${_i} + 1")
        else()
            math(EXPR _i "${_i} + 1")
            if(NOT _i LESS _bn)
                break()
            endif()
            string(SUBSTRING "${_body}" ${_i} 1 _e)
            math(EXPR _i "${_i} + 1")
            if(_e STREQUAL "n")
                string(APPEND _res "\n")
            elseif(_e STREQUAL "t")
                string(APPEND _res "\t")
            elseif(_e STREQUAL "\\" OR _e STREQUAL "\"")
                string(APPEND _res "${_e}")
            elseif(_e MATCHES "[0-7]")
                set(_oct "${_e}")
                foreach(_k RANGE 1 2)
                    if(NOT _i LESS _bn)
                        break()
                    endif()
                    string(SUBSTRING "${_body}" ${_i} 1 _d)
                    if(NOT _d MATCHES "[0-7]")
                        break()
                    endif()
                    string(APPEND _oct "${_d}")
                    math(EXPR _i "${_i} + 1")
                endforeach()
                set(_val 0)
                string(LENGTH "${_oct}" _ol)
                set(_j 0)
                while(_j LESS _ol)
                    string(SUBSTRING "${_oct}" ${_j} 1 _d)
                    math(EXPR _val "${_val} * 8 + ${_d}")
                    math(EXPR _j "${_j} + 1")
                endwhile()
                string(ASCII ${_val} _ch)
                string(APPEND _res "${_ch}")
            else()
                string(APPEND _res "${_e}")
            endif()
        endif()
    endwhile()
    set(${out_var} "${_res}" PARENT_SCOPE)
endfunction()

function(rc_path_excluded path excludes_str out_var)
    set(_xs "${excludes_str}")
    set(_hit FALSE)
    foreach(_g IN LISTS _xs)
        if(_g MATCHES "^!")
            string(SUBSTRING "${_g}" 1 -1 _g)
            string(STRIP "${_g}" _g)
        endif()
        if(NOT _g STREQUAL "")
            rc_glob_matches("${path}" "${_g}" _m)
            if(_m)
                set(_hit TRUE)
                break()
            endif()
        endif()
    endforeach()
    set(${out_var} ${_hit} PARENT_SCOPE)
endfunction()

# file(WRITE) on Windows stores LF as CRLF. The tree id hashes LF bytes
# (git ls-tree, and scripts/firmware_tree.py). Write those bytes back.
function(rc_write_lf path content)
    if(CMAKE_HOST_WIN32)  # host, not target: WIN32 is false in the ARM cross build
        set(_crlf "${path}.crlf")
        set(_ps1 "${path}.lf.ps1")
        file(WRITE "${_crlf}" "${content}")
        file(WRITE "${_ps1}" [=[
param([string]$Src, [string]$Dst)
$bytes = [System.IO.File]::ReadAllBytes($Src)
$count = 0
foreach ($one in $bytes) { if ($one -ne 13) { $count++ } }
$out = New-Object byte[] $count
$index = 0
foreach ($one in $bytes) {
    if ($one -ne 13) { $out[$index] = $one; $index++ }
}
[System.IO.File]::WriteAllBytes($Dst, $out)
]=])
        execute_process(
            COMMAND powershell -NoProfile -File "${_ps1}" -Src "${_crlf}" -Dst "${path}"
            RESULT_VARIABLE _rv
            ERROR_VARIABLE _err
        )
        file(REMOVE "${_crlf}" "${_ps1}")
        if(NOT _rv EQUAL 0)
            message(FATAL_ERROR "firmware-tree listing write failed: ${_err}")
        endif()
    else()
        file(WRITE "${path}" "${content}")
    endif()
endfunction()

# Drop ls-tree lines whose path matches a global exclude. Kept lines stay
# LF-terminated so the blob id matches scripts/firmware_tree.py.
function(rc_filter_firmware_listing listing excludes_str)
    file(READ "${listing}" _raw)
    if(_raw STREQUAL "")
        return()
    endif()
    string(REGEX REPLACE "\n$" "" _body "${_raw}")
    string(REPLACE ";" "\\;" _body "${_body}")
    string(REPLACE "\n" ";" _lines "${_body}")
    set(_out "")
    foreach(_line IN LISTS _lines)
        string(REPLACE "\r" "" _line "${_line}")
        set(_drop FALSE)
        if(_line MATCHES "\t(.*)$")
            set(_path "${CMAKE_MATCH_1}")
            rc_git_unquote("${_path}" _path)
            rc_path_excluded("${_path}" "${excludes_str}" _drop)
        endif()
        if(NOT _drop)
            string(APPEND _out "${_line}\n")
        endif()
    endforeach()
    rc_write_lf("${listing}" "${_out}")
endfunction()

function(rc_status_is_dirty text excludes_str out_var)
    set(_dirty FALSE)
    if(NOT text STREQUAL "")
        string(REPLACE "\r\n" "\n" text "${text}")
        string(REPLACE "\r" "\n" text "${text}")
        string(REPLACE ";" "\\;" text "${text}")
        string(REPLACE "\n" ";" _lines "${text}")
        foreach(_line IN LISTS _lines)
            string(LENGTH "${_line}" _n)
            if(_n LESS 4)
                continue()
            endif()
            string(SUBSTRING "${_line}" 3 -1 _rest)
            string(FIND "${_rest}" " -> " _arrow)
            if(_arrow EQUAL -1)
                set(_paths "${_rest}")
            else()
                string(SUBSTRING "${_rest}" 0 ${_arrow} _old)
                math(EXPR _npos "${_arrow} + 4")
                string(SUBSTRING "${_rest}" ${_npos} -1 _new)
                set(_paths "${_old};${_new}")
            endif()
            foreach(_p IN LISTS _paths)
                rc_git_unquote("${_p}" _p)
                rc_path_excluded("${_p}" "${excludes_str}" _drop)
                if(NOT _drop)
                    set(_dirty TRUE)
                endif()
            endforeach()
        endforeach()
    endif()
    set(${out_var} ${_dirty} PARENT_SCOPE)
endfunction()

# Blob id of the [elf] listing with global ! excludes removed.
# out_var is "<hash>" or "<hash>-dirty" or "unknown".
function(rc_firmware_tree_id source_dir git_exe out_var)
    set(_id "unknown")
    set(_list "${source_dir}/scripts/ci/firmware_paths.txt")
    if(NOT EXISTS "${_list}")
        set(${out_var} "${_id}" PARENT_SCOPE)
        return()
    endif()
    file(STRINGS "${_list}" _lines)
    set(_section "")
    set(_paths "")
    set(_excludes "")
    foreach(_line IN LISTS _lines)
        string(STRIP "${_line}" _line)
        if(_line STREQUAL "" OR _line MATCHES "^#")
            continue()
        endif()
        if(_line MATCHES "^\\[(.+)\\]$")
            set(_section "${CMAKE_MATCH_1}")
            continue()
        endif()
        if(_line MATCHES "^!")
            string(SUBSTRING "${_line}" 1 -1 _glob)
            string(STRIP "${_glob}" _glob)
            if(_glob STREQUAL "")
                message(FATAL_ERROR "${_list}: empty exclude")
            endif()
            if(_glob MATCHES "\\\\")
                message(FATAL_ERROR "${_list}: use / not \\")
            endif()
            list(APPEND _excludes "${_glob}")
            continue()
        endif()
        if(_section STREQUAL "elf")
            list(APPEND _paths "${_line}")
        endif()
    endforeach()
    if(NOT _paths)
        set(${out_var} "unknown" PARENT_SCOPE)
        return()
    endif()
    if(NOT CMAKE_BINARY_DIR)
        set(_listing "${source_dir}/rc_firmware_tree.txt")
    else()
        set(_listing "${CMAKE_BINARY_DIR}/rc_firmware_tree.txt")
    endif()
    execute_process(
        COMMAND "${git_exe}" -c core.quotePath=true ls-tree -r HEAD -- ${_paths}
        WORKING_DIRECTORY "${source_dir}"
        OUTPUT_FILE "${_listing}"
        ERROR_QUIET
        RESULT_VARIABLE _ls_rv
    )
    if(NOT _ls_rv EQUAL 0)
        set(${out_var} "unknown" PARENT_SCOPE)
        return()
    endif()
    if(_excludes)
        rc_filter_firmware_listing("${_listing}" "${_excludes}")
    endif()
    execute_process(
        COMMAND "${git_exe}" hash-object --no-filters "${_listing}"
        WORKING_DIRECTORY "${source_dir}"
        OUTPUT_VARIABLE _hash
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
        RESULT_VARIABLE _hash_rv
    )
    if(NOT _hash_rv EQUAL 0 OR _hash STREQUAL "")
        set(${out_var} "unknown" PARENT_SCOPE)
        return()
    endif()
    set(_id "${_hash}")
    execute_process(
        COMMAND "${git_exe}" -c core.quotePath=true status --porcelain --untracked-files=all -- ${_paths}
        WORKING_DIRECTORY "${source_dir}"
        OUTPUT_VARIABLE _dirty
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )
    rc_status_is_dirty("${_dirty}" "${_excludes}" _is_dirty)
    if(_is_dirty)
        string(APPEND _id "-dirty")
    endif()
    set(${out_var} "${_id}" PARENT_SCOPE)
endfunction()

if(CMAKE_SCRIPT_MODE_FILE)
    if(NOT RC_FW_TREE_SOURCE)
        message(FATAL_ERROR "RC_FW_TREE_SOURCE is not set")
    endif()
    find_package(Git QUIET)
    if(NOT GIT_EXECUTABLE)
        set(GIT_EXECUTABLE git)
    endif()
    rc_firmware_tree_id("${RC_FW_TREE_SOURCE}" "${GIT_EXECUTABLE}" _rc_probe_id)
    execute_process(COMMAND "${CMAKE_COMMAND}" -E echo "${_rc_probe_id}")
    return()
endif()

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
set(RC_FIRMWARE_TREE_ID "unknown")
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

    # Firmware-tree hash (build ID for the bench gate). Paths come from the
    # ONE tracked list, scripts/ci/firmware_paths.txt [elf]. Same recipe as
    # scripts/firmware_tree.py elf_tree_id(): blob id of
    # `git ls-tree -r HEAD -- <[elf] paths>` with global ! excludes removed.
    # A docs-only commit keeps it. "-dirty" when a non-excluded [elf] path
    # differs from HEAD at configure time.
    set(_rc_fw_list "${CMAKE_SOURCE_DIR}/scripts/ci/firmware_paths.txt")
    if(EXISTS "${_rc_fw_list}")
        set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${_rc_fw_list}")
        rc_firmware_tree_id("${CMAKE_SOURCE_DIR}" "${GIT_EXECUTABLE}" RC_FIRMWARE_TREE_ID)
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

message(STATUS "RocketChip ${RC_VERSION_STRING}  git ${RC_GIT_HASH}  identity ${RC_BUILD_IDENTITY}  fwtree ${RC_FIRMWARE_TREE_ID}  build ${RC_BUILD_NUMBER}")
