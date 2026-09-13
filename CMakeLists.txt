cmake_minimum_required(VERSION 3.24)
project(PhysicsEngine VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

function(reject_nonreproducible_fp_flags)
    set(DISALLOWED_FP_FLAGS
      "-ffast-math"
      "-Ofast"
      "-funsafe-math-optimizations"
      "-fassociative-math"
      "-freciprocal-math"
      "-ffinite-math-only"
      "/fp:fast"
  )
    set(FLAG_VARIABLES
      CMAKE_CXX_FLAGS
      CMAKE_CXX_FLAGS_DEBUG
      CMAKE_CXX_FLAGS_RELEASE
      CMAKE_CXX_FLAGS_RELWITHDEBINFO
      CMAKE_CXX_FLAGS_MINSIZEREL
  )

    foreach(flag_var IN LISTS FLAG_VARIABLES)
        foreach(disallowed_flag IN LISTS DISALLOWED_FP_FLAGS)
            string(FIND "${${flag_var}}" "${disallowed_flag}" flag_index)
            if(NOT flag_index EQUAL -1)
                message(FATAL_ERROR
            "Flag ${disallowed_flag} in ${flag_var} is not allowed because it "
            "can break deterministic floating-point replay.")
            endif()
        endforeach()
    endforeach()
endfunction()

reject_nonreproducible_fp_flags()

file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS
    "math/*.cpp"
    "physics/*.cpp"
    "collision/*.cpp"
    "core/*.cpp"
)

add_library(physics_core ${SOURCES})
target_include_directories(physics_core PUBLIC .)
target_compile_features(physics_core PUBLIC cxx_std_20)
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(physics_core PUBLIC -ffp-contract=off)
endif()

find_package(Threads REQUIRED)
target_link_libraries(physics_core PUBLIC Threads::Threads)

# Warnings are applied per target rather than globally so the fetched GoogleTest sources
# are not held to them.
function(physics_enable_warnings target)
    if(MSVC)
        target_compile_options(${target} PRIVATE /W4)
    else()
        target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
    endif()
    if(PHYSICS_WARNINGS_AS_ERRORS)
        if(MSVC)
            target_compile_options(${target} PRIVATE /WX)
        else()
            target_compile_options(${target} PRIVATE -Werror)
        endif()
    endif()
endfunction()

option(PHYSICS_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)

physics_enable_warnings(physics_core)

option(BUILD_DEMO "Build the SFML debug/demo executable" OFF)

if(BUILD_DEMO)
    if(APPLE)
        foreach(SFML_PREFIX "/opt/homebrew/opt/sfml@2" "/usr/local/opt/sfml@2")
            if(EXISTS "${SFML_PREFIX}")
                list(PREPEND CMAKE_PREFIX_PATH "${SFML_PREFIX}")
            endif()
        endforeach()
    endif()

    find_package(SFML 2.5 COMPONENTS graphics window system REQUIRED)

    add_executable(physics main.cpp)
    target_link_libraries(physics PRIVATE physics_core sfml-graphics sfml-window sfml-system)
    physics_enable_warnings(physics)
endif()

option(BUILD_HEADLESS_DEMOS "Build headless demo executables" ON)

if(BUILD_HEADLESS_DEMOS)
    add_executable(headless_replay demo/headless_replay.cpp)
    target_link_libraries(headless_replay PRIVATE physics_core)
    physics_enable_warnings(headless_replay)
endif()

option(BUILD_BENCHMARKS "Build benchmark executables" OFF)

if(BUILD_BENCHMARKS)
    set(PHYSICS_BENCHMARKS
        memory_budget_bench
        world_step_bench
        vec3_bench
        sphere_pair_bench
        aabb_pair_bench
        bvh_pair_bench
        collision_pipeline_bench
        contact_solver_bench
        body_layout_bench
        sweep_mode_bench
        job_system_bench
    )

    foreach(bench IN LISTS PHYSICS_BENCHMARKS)
        add_executable(${bench} bench/${bench}.cpp)
        target_link_libraries(${bench} PRIVATE physics_core)
        physics_enable_warnings(${bench})
    endforeach()
endif()

include(CTest)

if(BUILD_TESTING)
    include(FetchContent)

    FetchContent_Declare(
      googletest
      URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
      DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )

    FetchContent_MakeAvailable(googletest)

    file(GLOB_RECURSE TEST_SOURCES CONFIGURE_DEPENDS
      "tests/*.cpp"
    )

    add_executable(physics_tests ${TEST_SOURCES})

    target_link_libraries(physics_tests PRIVATE physics_core GTest::gtest_main)
    physics_enable_warnings(physics_tests)

    include(GoogleTest)
    gtest_discover_tests(physics_tests PROPERTIES TIMEOUT 120)
    if(TARGET headless_replay)
        add_test(NAME HeadlessReplay.CliVerification COMMAND headless_replay --steps 60 --threads 4)
        set_tests_properties(HeadlessReplay.CliVerification PROPERTIES TIMEOUT 120)
    endif()
endif()
