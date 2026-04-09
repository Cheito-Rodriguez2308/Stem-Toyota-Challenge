# -----------------------------------------------------------------------------
# Purpose:
#   Top-level CMake build configuration for the differential-drive robot
#   simulation repository.
#
# Role In The Architecture:
#   Defines the reusable library target, the example executable, and the test
#   subtree. Build logic is intentionally lightweight so the engineering focus
#   stays on the simulation code rather than on framework overhead.
#
# Key Targets:
#   - robot_sim: core library
#   - robot_sim_examples: minimal executable example
#
# Dependencies:
#   - Eigen
#   - nlohmann/json
# -----------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.21)

project(robot_competition_sim
    VERSION 0.1.0
    DESCRIPTION "Differential-drive robot simulation and structural pre-analysis framework"
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(ROBOT_SIM_BUILD_TESTS "Build unit tests" ON)
option(ROBOT_SIM_WARNINGS_AS_ERRORS "Treat warnings as errors" OFF)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(nlohmann_json 3.2 REQUIRED)

add_library(robot_sim
    src/control_primitives.cpp
    src/config_loader.cpp
    src/csv_logger.cpp
    src/drivetrain_model.cpp
    src/math_utils.cpp
    src/random_utils.cpp
    src/robot_controllers.cpp
    src/scenario_framework.cpp
    src/sensor_simulator.cpp
    src/simulation_modules.cpp
    src/simulator.cpp
)

target_compile_features(robot_sim PUBLIC cxx_std_20)
set_target_properties(robot_sim PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(robot_sim
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(robot_sim
    PUBLIC
        Eigen3::Eigen
        nlohmann_json::nlohmann_json
)

add_executable(robot_sim_examples
    examples/run_scenarios.cpp
)

target_compile_features(robot_sim_examples PRIVATE cxx_std_20)

target_link_libraries(robot_sim_examples
    PRIVATE
        robot_sim
)

if(MSVC)
    target_compile_options(robot_sim PRIVATE /W4 /permissive-)
    if(ROBOT_SIM_WARNINGS_AS_ERRORS)
        target_compile_options(robot_sim PRIVATE /WX)
    endif()
else()
    target_compile_options(robot_sim PRIVATE -Wall -Wextra -Wpedantic)
    if(ROBOT_SIM_WARNINGS_AS_ERRORS)
        target_compile_options(robot_sim PRIVATE -Werror)
    endif()
endif()

enable_testing()

if(ROBOT_SIM_BUILD_TESTS)
    add_subdirectory(tests EXCLUDE_FROM_ALL)
endif()
