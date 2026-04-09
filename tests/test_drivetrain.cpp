/*
 * Purpose:
 *   Minimal regression test for the differential-drive dynamics update.
 *
 * Role In The Architecture:
 *   Provides a lightweight sanity check that equal positive left/right voltage
 *   commands produce equal wheel speeds and zero turning response in a symmetric
 *   configuration.
 *
 * Dependencies:
 *   - drivetrain_model.hpp
 */

#include "drivetrain_model.hpp"

#include <cmath>
#include <iostream>

int main() {
    using namespace robot_sim;

    DriveParams drive{};
    drive.wheel_radius_m = 0.05;
    drive.track_width_m = 0.30;
    drive.nominal_battery_voltage_v = 12.0;
    drive.enable_battery_sag = false;
    drive.enable_acceleration_saturation = true;
    drive.max_acceleration_mps2 = 10.0;
    drive.drivetrain_drag_n_per_mps = 0.0;
    drive.rolling_resistance_n = 0.0;
    drive.wheel_velocity_response_time_s = 0.2;
    drive.motor.free_speed_radps = 40.0;

    ScenarioParams scenario{};
    scenario.name = "test_equal_voltage_forward";
    scenario.time_step_s = 0.02;
    scenario.integrator = IntegratorType::Euler;
    scenario.battery_voltage_v = 12.0;

    RobotState initial_state{};
    initial_state.battery_voltage_v = 12.0;

    DifferentialDriveDynamics dynamics(drive);
    dynamics.reset(scenario, initial_state);

    RobotInput input{};
    input.left_voltage_command_v = 6.0;
    input.right_voltage_command_v = 6.0;

    const RobotState next_state =
        dynamics.step(0.0, scenario.time_step_s, initial_state, input);

    if (!(next_state.left_wheel_velocity_mps > 0.0 &&
          next_state.right_wheel_velocity_mps > 0.0)) {
        std::cerr << "Expected both wheel velocities to increase under equal positive voltage.\n";
        return 1;
    }

    if (std::abs(next_state.left_wheel_velocity_mps - next_state.right_wheel_velocity_mps) > 1e-9) {
        std::cerr << "Expected equal left/right wheel velocities for equal voltages.\n";
        return 1;
    }

    if (std::abs(next_state.angular_velocity_radps) > 1e-9) {
        std::cerr << "Expected zero angular velocity for equal wheel commands.\n";
        return 1;
    }

    if (std::abs(next_state.pose.theta_rad) > 1e-9) {
        std::cerr << "Expected zero heading change for equal wheel commands.\n";
        return 1;
    }

    return 0;
}
