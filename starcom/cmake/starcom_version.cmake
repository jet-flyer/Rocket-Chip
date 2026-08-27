# Generate ${CMAKE_CURRENT_BINARY_DIR}/generated/starcom/version.hpp
#
# Product numbers: starcom/STARCOM_VERSION (humans edit at increment close
# or a tagged cut). Git identity is captured every configure.
# Reconfigure when STARCOM_VERSION, git HEAD, or git index move.
# See VERSIONING.md.
#
# Include after project() from starcom/CMakeLists.txt.

get_filename_component(STARCOM_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)

set(STARCOM_VERSION_FILE "${STARCOM_ROOT}/STARCOM_VERSION")
if(NOT EXISTS "${STARCOM_VERSION_FILE}")
  message(FATAL_ERROR "STARCOM_VERSION file missing: ${STARCOM_VERSION_FILE}")
endif()

set(STARCOM_VERSION_MAJOR "")
set(STARCOM_VERSION_MINOR "")
set(STARCOM_VERSION_PATCH "")
set(STARCOM_VERSION_TWEAK "0")
set(STARCOM_EXTRAVERSION "")

file(STRINGS "${STARCOM_VERSION_FILE}" _sc_version_lines)
foreach(_line IN LISTS _sc_version_lines)
  if(_line MATCHES "^[ \t]*#")
    continue()
  endif()
  if(_line MATCHES "^VERSION_MAJOR[ \t]*=[ \t]*([0-9]+)")
    set(STARCOM_VERSION_MAJOR "${CMAKE_MATCH_1}")
  elseif(_line MATCHES "^VERSION_MINOR[ \t]*=[ \t]*([0-9]+)")
    set(STARCOM_VERSION_MINOR "${CMAKE_MATCH_1}")
  elseif(_line MATCHES "^PATCHLEVEL[ \t]*=[ \t]*([0-9]+)")
    set(STARCOM_VERSION_PATCH "${CMAKE_MATCH_1}")
  elseif(_line MATCHES "^VERSION_TWEAK[ \t]*=[ \t]*([0-9]+)")
    set(STARCOM_VERSION_TWEAK "${CMAKE_MATCH_1}")
  elseif(_line MATCHES "^EXTRAVERSION[ \t]*=[ \t]*(.*)$")
    string(STRIP "${CMAKE_MATCH_1}" STARCOM_EXTRAVERSION)
    string(REGEX REPLACE "^\"(.*)\"$" "\\1" STARCOM_EXTRAVERSION "${STARCOM_EXTRAVERSION}")
    string(REGEX REPLACE "^'(.*)'$" "\\1" STARCOM_EXTRAVERSION "${STARCOM_EXTRAVERSION}")
  endif()
endforeach()

if(STARCOM_VERSION_MAJOR STREQUAL "" OR STARCOM_VERSION_MINOR STREQUAL ""
   OR STARCOM_VERSION_PATCH STREQUAL "")
  message(FATAL_ERROR
    "STARCOM_VERSION must set VERSION_MAJOR, VERSION_MINOR, and PATCHLEVEL (${STARCOM_VERSION_FILE})")
endif()

set(STARCOM_GIT_HASH "unknown")
set(STARCOM_BUILD_IDENTITY "unknown")
set(STARCOM_BUILD_NUMBER 0)

find_package(Git QUIET)
if(Git_FOUND OR GIT_FOUND)
  if(NOT GIT_EXECUTABLE)
    set(GIT_EXECUTABLE git)
  endif()

  execute_process(
    COMMAND ${GIT_EXECUTABLE} rev-parse --short HEAD
    WORKING_DIRECTORY ${STARCOM_ROOT}
    OUTPUT_VARIABLE STARCOM_GIT_HASH
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
    RESULT_VARIABLE _sc_git_hash_rv)
  if(NOT _sc_git_hash_rv EQUAL 0 OR STARCOM_GIT_HASH STREQUAL "")
    set(STARCOM_GIT_HASH "unknown")
  endif()

  execute_process(
    COMMAND ${GIT_EXECUTABLE} describe --abbrev=12 --always --dirty
    WORKING_DIRECTORY ${STARCOM_ROOT}
    OUTPUT_VARIABLE STARCOM_BUILD_IDENTITY
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
    RESULT_VARIABLE _sc_desc_rv)
  if(NOT _sc_desc_rv EQUAL 0 OR STARCOM_BUILD_IDENTITY STREQUAL "")
    set(STARCOM_BUILD_IDENTITY "${STARCOM_GIT_HASH}")
  endif()

  execute_process(
    COMMAND ${GIT_EXECUTABLE} describe --tags --abbrev=0 --match "starcom-v*"
    WORKING_DIRECTORY ${STARCOM_ROOT}
    OUTPUT_VARIABLE STARCOM_LAST_V_TAG
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
    RESULT_VARIABLE _sc_tag_rv)
  if(_sc_tag_rv EQUAL 0 AND NOT STARCOM_LAST_V_TAG STREQUAL "")
    execute_process(
      COMMAND ${GIT_EXECUTABLE} rev-list --count ${STARCOM_LAST_V_TAG}..HEAD -- .
      WORKING_DIRECTORY ${STARCOM_ROOT}
      OUTPUT_VARIABLE STARCOM_BUILD_NUMBER
      OUTPUT_STRIP_TRAILING_WHITESPACE
      ERROR_QUIET
      RESULT_VARIABLE _sc_count_rv)
  else()
    execute_process(
      COMMAND ${GIT_EXECUTABLE} rev-list --count HEAD -- .
      WORKING_DIRECTORY ${STARCOM_ROOT}
      OUTPUT_VARIABLE STARCOM_BUILD_NUMBER
      OUTPUT_STRIP_TRAILING_WHITESPACE
      ERROR_QUIET
      RESULT_VARIABLE _sc_count_rv)
  endif()
  if(NOT _sc_count_rv EQUAL 0 OR STARCOM_BUILD_NUMBER STREQUAL "")
    set(STARCOM_BUILD_NUMBER 0)
  endif()

  execute_process(
    COMMAND ${GIT_EXECUTABLE} rev-parse --git-path HEAD
    WORKING_DIRECTORY ${STARCOM_ROOT}
    OUTPUT_VARIABLE _sc_git_head
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET)
  execute_process(
    COMMAND ${GIT_EXECUTABLE} rev-parse --git-path index
    WORKING_DIRECTORY ${STARCOM_ROOT}
    OUTPUT_VARIABLE _sc_git_index
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET)
  if(_sc_git_head AND NOT IS_ABSOLUTE "${_sc_git_head}")
    get_filename_component(_sc_git_head "${STARCOM_ROOT}/${_sc_git_head}" ABSOLUTE)
  endif()
  if(_sc_git_index AND NOT IS_ABSOLUTE "${_sc_git_index}")
    get_filename_component(_sc_git_index "${STARCOM_ROOT}/${_sc_git_index}" ABSOLUTE)
  endif()
  if(EXISTS "${_sc_git_head}")
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${_sc_git_head}")
  endif()
  if(EXISTS "${_sc_git_index}")
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${_sc_git_index}")
  endif()
endif()

set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${STARCOM_VERSION_FILE}")

set(STARCOM_LIBRARY_VERSION
    "${STARCOM_VERSION_MAJOR}.${STARCOM_VERSION_MINOR}.${STARCOM_VERSION_PATCH}")
if(STARCOM_EXTRAVERSION STREQUAL "")
  set(STARCOM_VERSION_STRING "${STARCOM_LIBRARY_VERSION}")
else()
  set(STARCOM_VERSION_STRING "${STARCOM_LIBRARY_VERSION}-${STARCOM_EXTRAVERSION}")
endif()

set(STARCOM_VERSION_GENERATED_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
file(MAKE_DIRECTORY "${STARCOM_VERSION_GENERATED_DIR}/starcom")
configure_file(
  "${STARCOM_ROOT}/include/starcom/version.hpp.in"
  "${STARCOM_VERSION_GENERATED_DIR}/starcom/version.hpp"
  @ONLY)

message(STATUS "Starcom ${STARCOM_VERSION_STRING}  git ${STARCOM_GIT_HASH}  identity ${STARCOM_BUILD_IDENTITY}  build ${STARCOM_BUILD_NUMBER}")
