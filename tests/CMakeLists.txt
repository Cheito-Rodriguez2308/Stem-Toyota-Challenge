# -----------------------------------------------------------------------------
# Purpose:
#   Test-target definitions for the repository.
#
# Role In The Architecture:
#   Keeps test wiring separate from the library and executable build logic while
#   remaining intentionally lightweight.
#
# Key Target:
#   - robot_sim_test_drivetrain
# -----------------------------------------------------------------------------

add_executable(robot_sim_test_drivetrain
    test_drivetrain.cpp
)

target_compile_features(robot_sim_test_drivetrain PRIVATE cxx_std_20)

target_link_libraries(robot_sim_test_drivetrain
    PRIVATE
        robot_sim
)

add_test(
    NAME robot_sim_test_drivetrain
    COMMAND robot_sim_test_drivetrain
)
