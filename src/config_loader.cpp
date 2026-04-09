/*
 * Purpose:
 *   Implements JSON parsing for the repository configuration schema.
 *
 * Role In The Architecture:
 *   This file is the translation boundary between external configuration files
 *   and the strongly typed internal parameter structs used by the simulator.
 *
 * Key Functions:
 *   - ConfigLoader::load_bundle
 *   - ConfigLoader::load_drive_params
 *   - ConfigLoader::load_scenario_params
 *
 * Dependencies:
 *   - config_loader.hpp
 *   - standard file I/O and exception support
 */

#include "config_loader.hpp"

#include <fstream>
#include <stdexcept>
#include <string>

namespace robot_sim {
namespace {

/**
 * Returns a JSON field when present, otherwise preserves the provided default.
 *
 * Why:
 *   The configuration structs define the repository defaults. This helper keeps
 *   the parsing code concise while honoring those struct-level defaults.
 */
template <typename T>
T value_or(const nlohmann::json& json_object, const char* key, const T& default_value) {
    return json_object.contains(key) ? json_object.at(key).get<T>() : default_value;
}

/// Converts a user-facing integrator string into the internal enum.
IntegratorType parse_integrator_type(const std::string& value) {
    if (value == "euler") {
        return IntegratorType::Euler;
    }
    if (value == "rk4") {
        return IntegratorType::RK4;
    }

    throw std::invalid_argument("Unknown integrator type: " + value);
}

/// Converts a scenario label string into the internal scenario enum.
ScenarioType parse_scenario_type(const std::string& value) {
    if (value == "straight_line") {
        return ScenarioType::StraightLine;
    }
    if (value == "ninety_degree_turn") {
        return ScenarioType::NinetyDegreeTurn;
    }
    if (value == "in_place_turn") {
        return ScenarioType::InPlaceTurn;
    }
    if (value == "point_to_point") {
        return ScenarioType::PointToPoint;
    }
    if (value == "curved_path_follow") {
        return ScenarioType::CurvedPathFollow;
    }
    if (value == "push_load") {
        return ScenarioType::PushLoad;
    }
    if (value == "low_friction") {
        return ScenarioType::LowFriction;
    }
    if (value == "sensor_drift") {
        return ScenarioType::SensorDrift;
    }
    if (value == "asymmetric_drivetrain") {
        return ScenarioType::AsymmetricDrivetrain;
    }
    if (value == "battery_sag") {
        return ScenarioType::BatterySag;
    }
    if (value == "custom") {
        return ScenarioType::Custom;
    }

    throw std::invalid_argument("Unknown scenario type: " + value);
}

/// Converts a structural support-condition string into the internal enum.
SupportCondition parse_support_condition(const std::string& value) {
    if (value == "fixed_fixed") {
        return SupportCondition::FixedFixed;
    }
    if (value == "simply_supported") {
        return SupportCondition::SimplySupported;
    }
    if (value == "cantilever") {
        return SupportCondition::Cantilever;
    }
    if (value == "fixed_pinned") {
        return SupportCondition::FixedPinned;
    }

    throw std::invalid_argument("Unknown support_condition: " + value);
}

/// Converts a structural member type string into the internal enum.
StructuralMemberType parse_member_type(const std::string& value) {
    if (value == "beam") {
        return StructuralMemberType::Beam;
    }
    if (value == "plate") {
        return StructuralMemberType::Plate;
    }

    throw std::invalid_argument("Unknown structural member type: " + value);
}

/// Parses one sensor channel error model in native sensor units.
SensorChannelErrorModel load_sensor_channel(const nlohmann::json& json_object) {
    SensorChannelErrorModel channel{};
    channel.gaussian_noise_stddev =
        value_or(json_object, "gaussian_noise_stddev", channel.gaussian_noise_stddev);
    channel.static_bias = value_or(json_object, "static_bias", channel.static_bias);
    channel.drift_per_second = value_or(json_object, "drift_per_second", channel.drift_per_second);
    channel.quantization_step = value_or(json_object, "quantization_step", channel.quantization_step);
    channel.delay_s = value_or(json_object, "delay_s", channel.delay_s);
    channel.dropout_probability = value_or(json_object, "dropout_probability", channel.dropout_probability);
    channel.corruption_probability =
        value_or(json_object, "corruption_probability", channel.corruption_probability);
    channel.corruption_scale = value_or(json_object, "corruption_scale", channel.corruption_scale);
    return channel;
}

/// Parses one PID gain block.
PIDGains load_pid(const nlohmann::json& json_object) {
    PIDGains gains{};
    gains.kp = value_or(json_object, "kp", gains.kp);
    gains.ki = value_or(json_object, "ki", gains.ki);
    gains.kd = value_or(json_object, "kd", gains.kd);
    gains.integral_min = value_or(json_object, "integral_min", gains.integral_min);
    gains.integral_max = value_or(json_object, "integral_max", gains.integral_max);
    gains.output_min = value_or(json_object, "output_min", gains.output_min);
    gains.output_max = value_or(json_object, "output_max", gains.output_max);
    return gains;
}

/// Parses material properties used by the simplified structural screening layer.
MaterialProperties load_material(const nlohmann::json& json_object) {
    MaterialProperties material{};
    material.name = value_or(json_object, "name", material.name);
    material.youngs_modulus_pa = value_or(json_object, "youngs_modulus_pa", material.youngs_modulus_pa);
    material.shear_modulus_pa = value_or(json_object, "shear_modulus_pa", material.shear_modulus_pa);
    material.yield_strength_pa = value_or(json_object, "yield_strength_pa", material.yield_strength_pa);
    material.density_kg_per_m3 = value_or(json_object, "density_kg_per_m3", material.density_kg_per_m3);
    return material;
}

}  // namespace

nlohmann::json ConfigLoader::load_json_file(const std::filesystem::path& path) {
    std::ifstream input_stream(path);
    if (!input_stream) {
        throw std::runtime_error("Failed to open JSON config file: " + path.string());
    }

    nlohmann::json parsed_json;
    input_stream >> parsed_json;
    return parsed_json;
}

DriveParams ConfigLoader::load_drive_params(const nlohmann::json& json_object) {
    DriveParams params{};
    params.mass_kg = value_or(json_object, "mass_kg", params.mass_kg);
    params.wheel_radius_m = value_or(json_object, "wheel_radius_m", params.wheel_radius_m);
    params.track_width_m = value_or(json_object, "track_width_m", params.track_width_m);
    params.wheel_base_m = value_or(json_object, "wheel_base_m", params.wheel_base_m);
    params.wheel_velocity_response_time_s =
        value_or(json_object, "wheel_velocity_response_time_s", params.wheel_velocity_response_time_s);
    params.drivetrain_drag_n_per_mps =
        value_or(json_object, "drivetrain_drag_n_per_mps", params.drivetrain_drag_n_per_mps);
    params.rolling_resistance_n = value_or(json_object, "rolling_resistance_n", params.rolling_resistance_n);
    params.wheel_inertia_kgm2 = value_or(json_object, "wheel_inertia_kgm2", params.wheel_inertia_kgm2);
    params.battery_internal_resistance_ohm =
        value_or(json_object, "battery_internal_resistance_ohm", params.battery_internal_resistance_ohm);
    params.nominal_battery_voltage_v =
        value_or(json_object, "nominal_battery_voltage_v", params.nominal_battery_voltage_v);
    params.min_battery_voltage_v =
        value_or(json_object, "min_battery_voltage_v", params.min_battery_voltage_v);
    params.max_acceleration_mps2 =
        value_or(json_object, "max_acceleration_mps2", params.max_acceleration_mps2);
    params.max_angular_acceleration_radps2 =
        value_or(json_object, "max_angular_acceleration_radps2", params.max_angular_acceleration_radps2);
    params.left_wheel_radius_scale =
        value_or(json_object, "left_wheel_radius_scale", params.left_wheel_radius_scale);
    params.right_wheel_radius_scale =
        value_or(json_object, "right_wheel_radius_scale", params.right_wheel_radius_scale);
    params.left_drive_gain_scale =
        value_or(json_object, "left_drive_gain_scale", params.left_drive_gain_scale);
    params.right_drive_gain_scale =
        value_or(json_object, "right_drive_gain_scale", params.right_drive_gain_scale);
    params.traction_coefficient = value_or(json_object, "traction_coefficient", params.traction_coefficient);
    params.enable_battery_sag = value_or(json_object, "enable_battery_sag", params.enable_battery_sag);
    params.enable_acceleration_saturation =
        value_or(json_object, "enable_acceleration_saturation", params.enable_acceleration_saturation);

    if (json_object.contains("motor")) {
        const auto& motor_json = json_object.at("motor");
        params.motor.nominal_voltage_v =
            value_or(motor_json, "nominal_voltage_v", params.motor.nominal_voltage_v);
        params.motor.stall_torque_nm =
            value_or(motor_json, "stall_torque_nm", params.motor.stall_torque_nm);
        params.motor.free_speed_radps =
            value_or(motor_json, "free_speed_radps", params.motor.free_speed_radps);
        params.motor.terminal_resistance_ohm =
            value_or(motor_json, "terminal_resistance_ohm", params.motor.terminal_resistance_ohm);
        params.motor.torque_constant_nm_per_a =
            value_or(motor_json, "torque_constant_nm_per_a", params.motor.torque_constant_nm_per_a);
        params.motor.back_emf_constant_v_per_radps =
            value_or(motor_json, "back_emf_constant_v_per_radps", params.motor.back_emf_constant_v_per_radps);
        params.motor.rotor_inertia_kgm2 =
            value_or(motor_json, "rotor_inertia_kgm2", params.motor.rotor_inertia_kgm2);
        params.motor.motor_count_per_side =
            value_or(motor_json, "motor_count_per_side", params.motor.motor_count_per_side);
    }

    return params;
}

SensorParams ConfigLoader::load_sensor_params(const nlohmann::json& json_object) {
    SensorParams params{};
    if (json_object.contains("left_encoder")) {
        params.left_encoder = load_sensor_channel(json_object.at("left_encoder"));
    }
    if (json_object.contains("right_encoder")) {
        params.right_encoder = load_sensor_channel(json_object.at("right_encoder"));
    }
    if (json_object.contains("imu_heading")) {
        params.imu_heading = load_sensor_channel(json_object.at("imu_heading"));
    }
    if (json_object.contains("gyro")) {
        params.gyro = load_sensor_channel(json_object.at("gyro"));
    }

    params.enable_dropout = value_or(json_object, "enable_dropout", params.enable_dropout);
    params.enable_corruption = value_or(json_object, "enable_corruption", params.enable_corruption);
    params.enable_delay = value_or(json_object, "enable_delay", params.enable_delay);
    params.enable_quantization = value_or(json_object, "enable_quantization", params.enable_quantization);
    params.enable_odometry_estimator =
        value_or(json_object, "enable_odometry_estimator", params.enable_odometry_estimator);
    return params;
}

ControllerParams ConfigLoader::load_controller_params(const nlohmann::json& json_object) {
    ControllerParams params{};
    if (json_object.contains("heading_pid")) {
        params.heading_pid = load_pid(json_object.at("heading_pid"));
    }
    if (json_object.contains("distance_pid")) {
        params.distance_pid = load_pid(json_object.at("distance_pid"));
    }
    if (json_object.contains("path_pid")) {
        params.path_pid = load_pid(json_object.at("path_pid"));
    }

    params.feedforward_kv = value_or(json_object, "feedforward_kv", params.feedforward_kv);
    params.feedforward_ka = value_or(json_object, "feedforward_ka", params.feedforward_ka);
    params.max_voltage_step_per_s =
        value_or(json_object, "max_voltage_step_per_s", params.max_voltage_step_per_s);
    params.max_linear_speed_mps =
        value_or(json_object, "max_linear_speed_mps", params.max_linear_speed_mps);
    params.max_angular_speed_radps =
        value_or(json_object, "max_angular_speed_radps", params.max_angular_speed_radps);
    params.pure_pursuit_lookahead_m =
        value_or(json_object, "pure_pursuit_lookahead_m", params.pure_pursuit_lookahead_m);
    params.pure_pursuit_cruise_speed_mps =
        value_or(json_object, "pure_pursuit_cruise_speed_mps", params.pure_pursuit_cruise_speed_mps);
    params.heading_alignment_threshold_rad =
        value_or(json_object, "heading_alignment_threshold_rad", params.heading_alignment_threshold_rad);
    params.goal_tolerance_m = value_or(json_object, "goal_tolerance_m", params.goal_tolerance_m);
    params.heading_tolerance_rad =
        value_or(json_object, "heading_tolerance_rad", params.heading_tolerance_rad);
    return params;
}

ScenarioParams ConfigLoader::load_scenario_params(const nlohmann::json& json_object) {
    ScenarioParams params{};
    params.name = value_or(json_object, "name", params.name);
    params.description = value_or(json_object, "description", params.description);
    params.type = parse_scenario_type(value_or(json_object, "type", std::string("point_to_point")));
    params.initial_pose.x_m = value_or(json_object, "initial_x_m", params.initial_pose.x_m);
    params.initial_pose.y_m = value_or(json_object, "initial_y_m", params.initial_pose.y_m);
    params.initial_pose.theta_rad =
        value_or(json_object, "initial_theta_rad", params.initial_pose.theta_rad);
    params.target_position_m.x() = value_or(json_object, "target_x_m", params.target_position_m.x());
    params.target_position_m.y() = value_or(json_object, "target_y_m", params.target_position_m.y());
    params.target_heading_rad = value_or(json_object, "target_heading_rad", params.target_heading_rad);
    params.duration_s = value_or(json_object, "duration_s", params.duration_s);
    params.time_step_s = value_or(json_object, "time_step_s", params.time_step_s);
    params.integrator = parse_integrator_type(value_or(json_object, "integrator", std::string("euler")));
    params.stop_position_tolerance_m =
        value_or(json_object, "stop_position_tolerance_m", params.stop_position_tolerance_m);
    params.stop_heading_tolerance_rad =
        value_or(json_object, "stop_heading_tolerance_rad", params.stop_heading_tolerance_rad);
    params.stop_on_goal = value_or(json_object, "stop_on_goal", params.stop_on_goal);
    params.enable_push_load = value_or(json_object, "enable_push_load", params.enable_push_load);
    params.enable_low_friction = value_or(json_object, "enable_low_friction", params.enable_low_friction);
    params.enable_sensor_drift = value_or(json_object, "enable_sensor_drift", params.enable_sensor_drift);
    params.enable_asymmetric_drive =
        value_or(json_object, "enable_asymmetric_drive", params.enable_asymmetric_drive);
    params.enable_battery_sag = value_or(json_object, "enable_battery_sag", params.enable_battery_sag);
    params.battery_voltage_v = value_or(json_object, "battery_voltage_v", params.battery_voltage_v);
    params.friction_scale = value_or(json_object, "friction_scale", params.friction_scale);
    params.push_force_n = value_or(json_object, "push_force_n", params.push_force_n);
    params.external_disturbance_force_n =
        value_or(json_object, "external_disturbance_force_n", params.external_disturbance_force_n);
    params.imu_bias_override_rad =
        value_or(json_object, "imu_bias_override_rad", params.imu_bias_override_rad);
    params.imu_drift_override_radps =
        value_or(json_object, "imu_drift_override_radps", params.imu_drift_override_radps);
    params.left_drive_scale = value_or(json_object, "left_drive_scale", params.left_drive_scale);
    params.right_drive_scale = value_or(json_object, "right_drive_scale", params.right_drive_scale);
    params.left_wheel_radius_scale =
        value_or(json_object, "left_wheel_radius_scale", params.left_wheel_radius_scale);
    params.right_wheel_radius_scale =
        value_or(json_object, "right_wheel_radius_scale", params.right_wheel_radius_scale);
    params.controller_name = value_or(json_object, "controller_name", params.controller_name);

    if (json_object.contains("path_waypoints_m")) {
        params.path_waypoints_m.clear();
        for (const auto& point_json : json_object.at("path_waypoints_m")) {
            if (!point_json.is_array() || point_json.size() != 2) {
                throw std::invalid_argument("Each path waypoint must be a [x, y] array.");
            }

            params.path_waypoints_m.emplace_back(
                point_json.at(0).get<double>(),
                point_json.at(1).get<double>());
        }
    }

    return params;
}

MonteCarloConfig ConfigLoader::load_monte_carlo_config(const nlohmann::json& json_object) {
    auto read_range = [](const nlohmann::json& range_json, const MonteCarloRange& defaults) {
        MonteCarloRange range = defaults;
        range.min_value = value_or(range_json, "min", range.min_value);
        range.max_value = value_or(range_json, "max", range.max_value);
        return range;
    };

    MonteCarloConfig config{};
    config.seed = value_or(json_object, "seed", config.seed);
    config.run_count = value_or(json_object, "run_count", config.run_count);

    if (json_object.contains("wheel_radius_scale")) {
        config.wheel_radius_scale = read_range(json_object.at("wheel_radius_scale"), config.wheel_radius_scale);
    }
    if (json_object.contains("mass_scale")) {
        config.mass_scale = read_range(json_object.at("mass_scale"), config.mass_scale);
    }
    if (json_object.contains("friction_scale")) {
        config.friction_scale = read_range(json_object.at("friction_scale"), config.friction_scale);
    }
    if (json_object.contains("battery_voltage_v")) {
        config.battery_voltage_v = read_range(json_object.at("battery_voltage_v"), config.battery_voltage_v);
    }
    if (json_object.contains("imu_bias_rad")) {
        config.imu_bias_rad = read_range(json_object.at("imu_bias_rad"), config.imu_bias_rad);
    }
    if (json_object.contains("imu_drift_radps")) {
        config.imu_drift_radps = read_range(json_object.at("imu_drift_radps"), config.imu_drift_radps);
    }
    if (json_object.contains("encoder_noise_stddev_m")) {
        config.encoder_noise_stddev_m =
            read_range(json_object.at("encoder_noise_stddev_m"), config.encoder_noise_stddev_m);
    }
    if (json_object.contains("motor_imbalance_scale")) {
        config.motor_imbalance_scale =
            read_range(json_object.at("motor_imbalance_scale"), config.motor_imbalance_scale);
    }
    if (json_object.contains("external_disturbance_n")) {
        config.external_disturbance_n =
            read_range(json_object.at("external_disturbance_n"), config.external_disturbance_n);
    }

    return config;
}

StructuralMember ConfigLoader::load_structural_member(const nlohmann::json& json_object) {
    StructuralMember member{};
    member.name = value_or(json_object, "name", member.name);
    member.type = parse_member_type(value_or(json_object, "type", std::string("beam")));
    member.support_condition =
        parse_support_condition(value_or(json_object, "support_condition", std::string("simply_supported")));
    member.length_m = value_or(json_object, "length_m", member.length_m);
    member.width_m = value_or(json_object, "width_m", member.width_m);
    member.height_m = value_or(json_object, "height_m", member.height_m);
    member.thickness_m = value_or(json_object, "thickness_m", member.thickness_m);
    member.area_m2 = value_or(json_object, "area_m2", member.area_m2);
    member.second_moment_area_m4 =
        value_or(json_object, "second_moment_area_m4", member.second_moment_area_m4);
    member.section_modulus_m3 =
        value_or(json_object, "section_modulus_m3", member.section_modulus_m3);
    member.centroid_to_outer_fiber_m =
        value_or(json_object, "centroid_to_outer_fiber_m", member.centroid_to_outer_fiber_m);
    member.notes = value_or(json_object, "notes", member.notes);

    if (json_object.contains("material")) {
        member.material = load_material(json_object.at("material"));
    }

    return member;
}

StructuralLoadCase ConfigLoader::load_structural_load_case(const nlohmann::json& json_object) {
    StructuralLoadCase load_case{};
    load_case.name = value_or(json_object, "name", load_case.name);
    load_case.point_load_n = value_or(json_object, "point_load_n", load_case.point_load_n);
    load_case.point_load_location_m =
        value_or(json_object, "point_load_location_m", load_case.point_load_location_m);
    load_case.distributed_load_n_per_m =
        value_or(json_object, "distributed_load_n_per_m", load_case.distributed_load_n_per_m);
    load_case.axial_force_n = value_or(json_object, "axial_force_n", load_case.axial_force_n);
    load_case.torsional_moment_nm =
        value_or(json_object, "torsional_moment_nm", load_case.torsional_moment_nm);
    load_case.scenario_name = value_or(json_object, "scenario_name", load_case.scenario_name);
    return load_case;
}

SimulationConfigBundle ConfigLoader::load_bundle(const nlohmann::json& json_object) {
    SimulationConfigBundle bundle{};

    if (json_object.contains("drive")) {
        bundle.drive = load_drive_params(json_object.at("drive"));
    }
    if (json_object.contains("sensors")) {
        bundle.sensors = load_sensor_params(json_object.at("sensors"));
    }
    if (json_object.contains("controllers")) {
        bundle.controllers = load_controller_params(json_object.at("controllers"));
    }
    if (json_object.contains("monte_carlo")) {
        bundle.monte_carlo = load_monte_carlo_config(json_object.at("monte_carlo"));
    }
    if (json_object.contains("scenarios")) {
        bundle.scenarios.clear();
        for (const auto& scenario_json : json_object.at("scenarios")) {
            bundle.scenarios.push_back(load_scenario_params(scenario_json));
        }
    }
    if (json_object.contains("structural_members")) {
        bundle.structural_members.clear();
        for (const auto& member_json : json_object.at("structural_members")) {
            bundle.structural_members.push_back(load_structural_member(member_json));
        }
    }
    if (json_object.contains("structural_load_cases")) {
        bundle.structural_load_cases.clear();
        for (const auto& load_case_json : json_object.at("structural_load_cases")) {
            bundle.structural_load_cases.push_back(load_structural_load_case(load_case_json));
        }
    }

    return bundle;
}

SimulationConfigBundle ConfigLoader::load_bundle_from_file(const std::filesystem::path& path) {
    return load_bundle(load_json_file(path));
}

}  // namespace robot_sim
