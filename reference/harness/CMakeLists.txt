cmake_minimum_required(VERSION 3.16)
project(nav_oracle CXX)

# The upstream commit the oracle is pinned to (reference/UPSTREAM.md). Records
# carry the commit they were built from; the comparator refuses to mix commits.
set(NAV_UPSTREAM_PIN "9f4ce64458dfae86e1239c525ddc219c4e9e06f1")

# Path to a recastnavigation checkout at the pinned commit. Defaults to the
# NAV_UPSTREAM_DIR environment variable.
set(RECAST_SOURCE_DIR "$ENV{NAV_UPSTREAM_DIR}" CACHE PATH "recastnavigation checkout at the pinned commit")
if(NOT EXISTS "${RECAST_SOURCE_DIR}/Recast/Include/Recast.h")
  message(FATAL_ERROR "Set -DRECAST_SOURCE_DIR (or NAV_UPSTREAM_DIR) to a recastnavigation checkout at ${NAV_UPSTREAM_PIN}")
endif()

# The checkout must be at the pin. A source tree without git metadata can be
# accepted by passing -DNAV_UPSTREAM_COMMIT=<sha> after checking it by hand.
option(NAV_ALLOW_UNPINNED "Build against a checkout that is not at the pinned commit" OFF)
if(NOT NAV_UPSTREAM_COMMIT)
  find_package(Git QUIET)
  if(GIT_FOUND)
    execute_process(COMMAND "${GIT_EXECUTABLE}" -C "${RECAST_SOURCE_DIR}" rev-parse HEAD
      OUTPUT_VARIABLE NAV_UPSTREAM_COMMIT OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET RESULT_VARIABLE git_result)
    if(NOT git_result EQUAL 0)
      set(NAV_UPSTREAM_COMMIT "")
    endif()
    execute_process(COMMAND "${GIT_EXECUTABLE}" -C "${RECAST_SOURCE_DIR}" status --porcelain --untracked-files=no
      OUTPUT_VARIABLE git_dirty OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
    if(git_dirty)
      message(FATAL_ERROR "The upstream checkout has local modifications:\n${git_dirty}")
    endif()
  endif()
endif()
if(NOT NAV_UPSTREAM_COMMIT STREQUAL NAV_UPSTREAM_PIN)
  if(NAV_ALLOW_UNPINNED)
    message(WARNING "Upstream checkout is at '${NAV_UPSTREAM_COMMIT}', not the pin ${NAV_UPSTREAM_PIN}")
  else()
    message(FATAL_ERROR "Upstream checkout is at '${NAV_UPSTREAM_COMMIT}', expected ${NAV_UPSTREAM_PIN} "
      "(set NAV_ALLOW_UNPINNED=ON to build anyway; such records are a different oracle)")
  endif()
endif()

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

file(GLOB UPSTREAM_SOURCES
  "${RECAST_SOURCE_DIR}/Recast/Source/*.cpp"
  "${RECAST_SOURCE_DIR}/Detour/Source/*.cpp"
  "${RECAST_SOURCE_DIR}/DetourCrowd/Source/*.cpp"
  "${RECAST_SOURCE_DIR}/DetourTileCache/Source/*.cpp")

# Two oracles from the same sources: `nav_oracle` with upstream's default 32-bit
# poly refs, and `nav_oracle64` with DT_POLYREF64 (64-bit refs, the port's
# default). Records carry the width as `polyref_bits` in their metadata.
# The oracle's float behaviour is part of the contract (reference/RECORDS.md):
# no fused multiply-add contraction, no fast-math, baseline x86-64 SSE2 code.
set(NAV_ORACLE_FLAGS -O2 -ffp-contract=off -fno-fast-math -fno-finite-math-only)
string(REPLACE ";" " " NAV_ORACLE_FLAGS_TEXT "${NAV_ORACLE_FLAGS}")
find_package(Threads REQUIRED)

function(nav_add_oracle name)
  add_executable(${name} nav_oracle.cpp ${UPSTREAM_SOURCES})
  target_include_directories(${name} PRIVATE
    "${RECAST_SOURCE_DIR}/Recast/Include"
    "${RECAST_SOURCE_DIR}/Detour/Include"
    "${RECAST_SOURCE_DIR}/DetourCrowd/Include"
    "${RECAST_SOURCE_DIR}/DetourTileCache/Include")
  target_compile_options(${name} PRIVATE ${NAV_ORACLE_FLAGS})
  target_compile_definitions(${name} PRIVATE
    NAV_UPSTREAM_COMMIT="${NAV_UPSTREAM_COMMIT}"
    NAV_ORACLE_FLAGS="${CMAKE_BUILD_TYPE}: ${NAV_ORACLE_FLAGS_TEXT}"
    ${ARGN})
  target_link_libraries(${name} PRIVATE Threads::Threads)
endfunction()

nav_add_oracle(nav_oracle)
nav_add_oracle(nav_oracle64 DT_POLYREF64)
