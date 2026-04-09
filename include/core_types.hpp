#pragma once

/*
 * Purpose:
 *   Defines the shared data contracts used by simulation, sensing, control,
 *   logging, Monte Carlo analysis, and the simplified structural screening
 *   workflow.
 *
 * Role In The Architecture:
 *   This header is the common vocabulary of the repository. Most modules depend
 *   on these types, but no control law or physics integration lives here.
 *
 * Key Types:
 *   - RobotState / RobotInput
 *   - SensorReadings / EstimatedState
 *   - DriveParams / SensorParams / ControllerParams / ScenarioParams
 *   - StructuralMember / StructuralLoadCase / StructuralAnalysisResult
 *   - SimulationResult / MonteCarloSummary
 *
 * Dependencies:
 *   - Eigen for compact 2D vectors used in goals and waypoints.
 *
 * Conventions:
 *   - SI units are used throughout the repository.
 *   - Variable suffixes such as _m, _s, _rad, _v, and _n encode units.
 *   - Structural types support preliminary screening only and do not replace
 *     full FEM in an external solver.
 */

#include <Eigen/Dense>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace robot_sim {

/// Fixed-step integration method used by the reduced-order drivetrain model.
enum class IntegratorType {
    Euler,  ///< First-order explicit Euler integration.
    RK4     ///< Classical fourth-order Runge-Kutta integration.
};

/// Scenario category used to select controller defaults and reporting labels.
enum class ScenarioType {
    StraightLine,
    NinetyDegreeTurn,
    InPlaceTurn,
    PointToPoint,
    CurvedPathFollow,
    PushLoad,
    LowFriction,
    SensorDrift,
    AsymmetricDrivetrain,
    BatterySag,
    Custom
};

/**
 * Idealized support condition for the structural screening layer.
 *
 * Limitation:
 *   Real joints, bolt preload, slip, weld flexibility, and contact compliance
 *   are not represented by these labels.
 */
enum class SupportCondition {
    FixedFixed,
    SimplySupported,
    Cantilever,
    FixedPinned
};

/// Simplified member shape family used by the structural screening layer.
enum class StructuralMemberType {
    Beam,
    Plate
};

/**
 * Planar pose in the world frame.
 *
 * Units:
 *   - x_m, y_m: meters [m]
 *   - theta_rad: radians [rad]
 */
struct Pose2D {
    double x_m{0.0};       ///< World-frame x position [m].
    double y_m{0.0};       ///< World-frame y position [m].
    double theta_rad{0.0}; ///< World-frame heading [rad].
};

/**
 * Ground-truth plant state produced by the simulator.
 *
 * Layer:
 *   Simulation / plant state.
 *
 * Assumptions:
 *   - The robot is a rigid planar differential-drive body.
 *   - Wheel states are reduced to left/right longitudinal motion only.
 *
 * Not Modeled:
 *   Wheel slip state, chassis flex, contact impulses, and multibody dynamics.
 */
struct RobotState {
    Pose2D pose{};                        ///< Ground-truth pose [m, m, rad].
    double left_wheel_velocity_mps{0.0};  ///< Left wheel linear velocity [m/s].
    double right_wheel_velocity_mps{0.0}; ///< Right wheel linear velocity [m/s].
    double linear_velocity_mps{0.0};      ///< Chassis forward velocity [m/s].
    double angular_velocity_radps{0.0};   ///< Chassis yaw rate [rad/s].
    double left_wheel_position_m{0.0};    ///< Integrated left wheel travel [m].
    double right_wheel_position_m{0.0};   ///< Integrated right wheel travel [m].
    double battery_voltage_v{12.0};       ///< Available battery voltage [V].
    double sim_time_s{0.0};               ///< Simulation time stamp [s].
};

/**
 * Controller output and lumped external disturbance inputs applied to the plant.
 *
 * Layer:
 *   Control output / plant input.
 *
 * Assumptions:
 *   External loads are simplified lumped forces or moments, not resolved
 *   contact mechanics.
 */
struct RobotInput {
    double left_voltage_command_v{0.0}; ///< Requested left drive voltage [V].
    double right_voltage_command_v{0.0};///< Requested right drive voltage [V].
    double external_force_x_n{0.0};     ///< Lumped disturbance force along x [N].
    double external_force_y_n{0.0};     ///< Reserved lateral disturbance force [N].
    double external_yaw_moment_nm{0.0}; ///< Reserved yaw disturbance moment [N*m].
};

/**
 * Error model for one sensor channel.
 *
 * Units:
 *   Values use the native units of the sensor quantity being modeled.
 */
struct SensorChannelErrorModel {
    double gaussian_noise_stddev{0.0};  ///< Additive white noise std. dev. [sensor units].
    double static_bias{0.0};            ///< Constant sensor bias [sensor units].
    double drift_per_second{0.0};       ///< Linear drift rate [sensor units / s].
    double quantization_step{0.0};      ///< Quantization step [sensor units].
    double delay_s{0.0};                ///< Sample delay [s].
    double dropout_probability{0.0};    ///< Probability of invalidating a sample [-].
    double corruption_probability{0.0}; ///< Probability of large corruption [-].
    double corruption_scale{0.0};       ///< Relative corruption magnitude scale [-].
};

/**
 * Simulated sensor outputs visible to the estimator and controller layers.
 *
 * Layer:
 *   Sensor outputs.
 */
struct SensorReadings {
    double timestamp_s{0.0};       ///< Measurement time stamp [s].
    double left_encoder_m{0.0};    ///< Left encoder travel [m].
    double right_encoder_m{0.0};   ///< Right encoder travel [m].
    double imu_heading_rad{0.0};   ///< IMU heading [rad].
    double gyro_z_radps{0.0};      ///< Gyroscope yaw rate [rad/s].
    bool left_encoder_valid{true}; ///< True when the left encoder sample is usable.
    bool right_encoder_valid{true};///< True when the right encoder sample is usable.
    bool imu_valid{true};          ///< True when the IMU heading sample is usable.
    bool gyro_valid{true};         ///< True when the gyro sample is usable.
};

/**
 * Estimated robot state delivered to controllers.
 *
 * Layer:
 *   Estimation / controller input.
 *
 * Usage:
 *   Controllers should act on this estimate, not on RobotState, so sensor and
 *   estimator imperfections remain in the closed loop.
 */
struct EstimatedState {
    Pose2D pose{};                        ///< Estimated pose [m, m, rad].
    double linear_velocity_mps{0.0};      ///< Estimated forward speed [m/s].
    double angular_velocity_radps{0.0};   ///< Estimated yaw rate [rad/s].
    double covariance_trace{0.0};         ///< Lightweight uncertainty indicator [-].
};

/**
 * Aggregate motor constants used by the simple drivetrain model.
 *
 * Assumptions:
 *   These constants are treated as effective motor-side parameters. The current
 *   simulator does not yet solve full electromechanical winding dynamics.
 */
struct MotorParams {
    double nominal_voltage_v{12.0};            ///< Nominal supply voltage [V].
    double stall_torque_nm{1.0};               ///< Stall torque [N*m].
    double free_speed_radps{1.0};              ///< No-load shaft speed [rad/s].
    double terminal_resistance_ohm{1.0};       ///< Terminal resistance [ohm].
    double torque_constant_nm_per_a{1.0};      ///< Torque constant [N*m/A].
    double back_emf_constant_v_per_radps{1.0}; ///< Back-EMF constant [V/(rad/s)].
    double rotor_inertia_kgm2{0.0};            ///< Rotor inertia [kg*m^2].
    std::size_t motor_count_per_side{1U};      ///< Number of motors per drivetrain side [-].
};

/**
 * Drivetrain and chassis parameters for the reduced-order plant model.
 *
 * Assumptions:
 *   - The robot is a differential-drive chassis.
 *   - Wheel speed follows a first-order response to commanded voltage.
 *   - Drag and rolling resistance are lumped scalar terms.
 *
 * Not Modeled:
 *   Detailed gearbox dynamics, lateral tire slip, compliance, or impacts.
 */
struct DriveParams {
    double mass_kg{15.0};                          ///< Robot mass [kg].
    double wheel_radius_m{0.045};                 ///< Nominal wheel radius [m].
    double track_width_m{0.30};                   ///< Left-to-right wheel track width [m].
    double wheel_base_m{0.30};                    ///< Reserved longitudinal wheelbase [m].
    double wheel_velocity_response_time_s{0.25};  ///< First-order wheel-speed time constant [s].
    double drivetrain_drag_n_per_mps{1.0};        ///< Lumped drag coefficient [N/(m/s)].
    double rolling_resistance_n{2.0};             ///< Lumped rolling resistance magnitude [N].
    double wheel_inertia_kgm2{0.0};               ///< Reserved wheel inertia parameter [kg*m^2].
    double battery_internal_resistance_ohm{0.015};///< Battery internal resistance [ohm].
    double nominal_battery_voltage_v{12.0};       ///< Nominal battery voltage [V].
    double min_battery_voltage_v{8.0};            ///< Minimum allowed battery voltage [V].
    double max_acceleration_mps2{4.0};            ///< Linear acceleration saturation [m/s^2].
    double max_angular_acceleration_radps2{20.0}; ///< Reserved angular acceleration saturation [rad/s^2].
    double left_wheel_radius_scale{1.0};          ///< Calibration scale on left wheel radius [-].
    double right_wheel_radius_scale{1.0};         ///< Calibration scale on right wheel radius [-].
    double left_drive_gain_scale{1.0};            ///< Calibration scale on left drive authority [-].
    double right_drive_gain_scale{1.0};           ///< Calibration scale on right drive authority [-].
    double traction_coefficient{1.0};             ///< Reserved traction coefficient [-].
    bool enable_battery_sag{true};                ///< Enables simple battery sag approximation.
    bool enable_acceleration_saturation{true};    ///< Enables acceleration clamping.
    MotorParams motor{};                          ///< Effective motor parameters.
};

/**
 * Global sensor subsystem configuration.
 */
struct SensorParams {
    SensorChannelErrorModel left_encoder{}; ///< Left encoder error model [m-domain].
    SensorChannelErrorModel right_encoder{};///< Right encoder error model [m-domain].
    SensorChannelErrorModel imu_heading{};  ///< IMU heading error model [rad-domain].
    SensorChannelErrorModel gyro{};         ///< Gyroscope error model [rad/s-domain].
    bool enable_dropout{true};              ///< Enables dropout handling.
    bool enable_corruption{true};           ///< Enables corrupted-sample generation.
    bool enable_delay{false};               ///< Enables sample delay using buffered truth.
    bool enable_quantization{true};         ///< Enables quantization.
    bool enable_odometry_estimator{true};   ///< Selects wheel odometry over perfect estimation.
};

/**
 * Scalar PID gain set with integral and output limits.
 */
struct PIDGains {
    double kp{0.0};            ///< Proportional gain [output / error].
    double ki{0.0};            ///< Integral gain [output / (error*s)].
    double kd{0.0};            ///< Derivative gain [(output*s) / error].
    double integral_min{-1e9}; ///< Lower clamp on integral state [error*s].
    double integral_max{1e9};  ///< Upper clamp on integral state [error*s].
    double output_min{-12.0};  ///< Minimum controller output [typically V].
    double output_max{12.0};   ///< Maximum controller output [typically V].
};

/**
 * Aggregate controller tuning parameters.
 *
 * Note:
 *   Some fields support future path-following extensions and remain in the
 *   schema even if not all are used by the current minimal controllers.
 */
struct ControllerParams {
    PIDGains heading_pid{};                   ///< Heading-loop gains [typically V/rad].
    PIDGains distance_pid{};                  ///< Distance-loop gains [typically V/m].
    PIDGains path_pid{};                      ///< Reserved path-tracking gains.
    double feedforward_kv{0.0};               ///< Velocity feedforward gain [V/(m/s)].
    double feedforward_ka{0.0};               ///< Acceleration feedforward gain [V/(m/s^2)].
    double max_voltage_step_per_s{60.0};      ///< Voltage slew-rate limit [V/s].
    double max_linear_speed_mps{1.5};         ///< Desired forward speed limit [m/s].
    double max_angular_speed_radps{6.0};      ///< Desired yaw-rate limit [rad/s].
    double pure_pursuit_lookahead_m{0.35};    ///< Reserved Pure Pursuit lookahead [m].
    double pure_pursuit_cruise_speed_mps{1.0};///< Reserved Pure Pursuit speed [m/s].
    double heading_alignment_threshold_rad{0.75}; ///< Reserved heading gating threshold [rad].
    double goal_tolerance_m{0.05};            ///< Goal position tolerance [m].
    double heading_tolerance_rad{0.03};       ///< Goal heading tolerance [rad].
};

/**
 * Scenario definition used to configure one simulation run.
 */
struct ScenarioParams {
    std::string name{"unnamed_scenario"};   ///< Scenario identifier used in logs.
    std::string description{};              ///< Human-readable scenario description.
    ScenarioType type{ScenarioType::PointToPoint}; ///< Scenario category.
    Pose2D initial_pose{};                  ///< Initial pose [m, m, rad].
    Eigen::Vector2d target_position_m{0.0, 0.0}; ///< Goal position [m].
    double target_heading_rad{0.0};         ///< Goal heading [rad].
    double duration_s{5.0};                 ///< Maximum simulation duration [s].
    double time_step_s{0.01};               ///< Fixed integration step size [s].
    IntegratorType integrator{IntegratorType::Euler}; ///< Numerical integrator.
    double stop_position_tolerance_m{0.05}; ///< Position tolerance [m].
    double stop_heading_tolerance_rad{0.03};///< Heading tolerance [rad].
    bool stop_on_goal{true};                ///< Stops run when goal tolerances are met.
    bool enable_push_load{false};           ///< Adds opposing push/load term.
    bool enable_low_friction{false};        ///< Reserved low-friction modifier.
    bool enable_sensor_drift{false};        ///< Enables scenario-specific IMU drift overrides.
    bool enable_asymmetric_drive{false};    ///< Reserved asymmetric-drive modifier flag.
    bool enable_battery_sag{false};         ///< Forces battery sag on for the scenario.
    double battery_voltage_v{12.0};         ///< Initial battery voltage [V].
    double friction_scale{1.0};             ///< Reserved friction multiplier [-].
    double push_force_n{0.0};               ///< Additional opposing load [N].
    double external_disturbance_force_n{0.0}; ///< Lumped disturbance force [N].
    double imu_bias_override_rad{0.0};      ///< Scenario IMU bias override [rad].
    double imu_drift_override_radps{0.0};   ///< Scenario IMU drift override [rad/s].
    double left_drive_scale{1.0};           ///< Scenario left drive gain scale [-].
    double right_drive_scale{1.0};          ///< Scenario right drive gain scale [-].
    double left_wheel_radius_scale{1.0};    ///< Scenario left wheel radius scale [-].
    double right_wheel_radius_scale{1.0};   ///< Scenario right wheel radius scale [-].
    std::string controller_name{"point_to_point"}; ///< Preferred controller key.
    std::vector<Eigen::Vector2d> path_waypoints_m{}; ///< Optional path waypoints [m].
};

/// Inclusive scalar range used by Monte Carlo parameter randomization.
struct MonteCarloRange {
    double min_value{0.0}; ///< Minimum sampled value [native units].
    double max_value{0.0}; ///< Maximum sampled value [native units].
};

/**
 * Monte Carlo configuration for later robustness studies.
 */
struct MonteCarloConfig {
    std::uint32_t seed{1U};                        ///< Reproducible random seed.
    std::size_t run_count{100U};                   ///< Number of Monte Carlo runs.
    MonteCarloRange wheel_radius_scale{0.98, 1.02}; ///< Wheel-radius scale perturbation [-].
    MonteCarloRange mass_scale{0.9, 1.1};         ///< Mass scale perturbation [-].
    MonteCarloRange friction_scale{0.8, 1.2};     ///< Friction scale perturbation [-].
    MonteCarloRange battery_voltage_v{11.0, 12.6};///< Initial battery voltage perturbation [V].
    MonteCarloRange imu_bias_rad{0.0, 0.02};      ///< IMU bias perturbation [rad].
    MonteCarloRange imu_drift_radps{0.0, 0.002};  ///< IMU drift perturbation [rad/s].
    MonteCarloRange encoder_noise_stddev_m{0.0, 0.002}; ///< Encoder noise perturbation [m].
    MonteCarloRange motor_imbalance_scale{0.95, 1.05};  ///< Drivetrain mismatch perturbation [-].
    MonteCarloRange external_disturbance_n{0.0, 10.0};  ///< Disturbance-force perturbation [N].
};

/**
 * Simplified isotropic material properties for structural screening.
 *
 * Limitation:
 *   These values are adequate for beam/plate screening approximations, not for
 *   certifiable stress prediction.
 */
struct MaterialProperties {
    std::string name{"generic"};      ///< Material label.
    double youngs_modulus_pa{69.0e9}; ///< Young's modulus [Pa].
    double shear_modulus_pa{26.0e9};  ///< Shear modulus [Pa].
    double yield_strength_pa{250.0e6};///< Approximate yield strength [Pa].
    double density_kg_per_m3{2700.0}; ///< Density [kg/m^3].
};

/**
 * Simplified structural member definition for preliminary screening.
 *
 * Reliability Boundary:
 *   This supports ranked screening only. Trustworthy local stress maps still
 *   require external FEM.
 */
struct StructuralMember {
    std::string name{};                         ///< Member identifier.
    StructuralMemberType type{StructuralMemberType::Beam}; ///< Beam or plate abstraction.
    SupportCondition support_condition{SupportCondition::SimplySupported}; ///< Idealized support model.
    MaterialProperties material{};             ///< Material properties.
    double length_m{0.0};                      ///< Span or characteristic length [m].
    double width_m{0.0};                       ///< Width [m].
    double height_m{0.0};                      ///< Height [m].
    double thickness_m{0.0};                   ///< Wall or plate thickness [m].
    double area_m2{0.0};                       ///< Cross-sectional area [m^2].
    double second_moment_area_m4{0.0};         ///< Second moment of area [m^4].
    double section_modulus_m3{0.0};            ///< Section modulus [m^3].
    double centroid_to_outer_fiber_m{0.0};     ///< Centroid-to-extreme-fiber distance [m].
    std::string notes{};                       ///< Free-form modeling notes.
};

/**
 * Simplified structural load case for one screening evaluation.
 */
struct StructuralLoadCase {
    std::string name{};                   ///< Load case identifier.
    double point_load_n{0.0};             ///< Transverse point load [N].
    double point_load_location_m{0.0};    ///< Point-load application location [m].
    double distributed_load_n_per_m{0.0}; ///< Uniform distributed load [N/m].
    double axial_force_n{0.0};            ///< Axial force [N].
    double torsional_moment_nm{0.0};      ///< Torsional moment [N*m].
    std::string scenario_name{};          ///< Optional originating scenario name.
};

/**
 * Output of one simplified structural screening calculation.
 */
struct StructuralAnalysisResult {
    std::string member_name{};            ///< Structural member identifier.
    std::string load_case_name{};         ///< Load case identifier.
    double max_reaction_n{0.0};           ///< Maximum support reaction [N].
    double max_shear_n{0.0};              ///< Maximum internal shear force [N].
    double max_bending_moment_nm{0.0};    ///< Maximum bending moment [N*m].
    double max_bending_stress_pa{0.0};    ///< Peak bending stress estimate [Pa].
    double average_shear_stress_pa{0.0};  ///< Average shear stress estimate [Pa].
    double safety_factor_indicator{0.0};  ///< Yield-style screening ratio [-].
    std::string likely_critical_region{}; ///< Likely hotspot description.
    std::string warning{};                ///< Warning about invalid assumptions or inputs.
};

/**
 * One denormalized row of simulation output prepared for CSV logging.
 */
struct SimulationTraceRow {
    std::string scenario_name{};     ///< Scenario identifier.
    std::string run_id{};            ///< Run identifier.
    double time_s{0.0};              ///< Simulation time [s].
    double x_m{0.0};                 ///< True x position [m].
    double y_m{0.0};                 ///< True y position [m].
    double theta_rad{0.0};           ///< True heading [rad].
    double left_velocity_mps{0.0};   ///< True left wheel velocity [m/s].
    double right_velocity_mps{0.0};  ///< True right wheel velocity [m/s].
    double omega_radps{0.0};         ///< True yaw rate [rad/s].
    double left_voltage_v{0.0};      ///< Left voltage command [V].
    double right_voltage_v{0.0};     ///< Right voltage command [V].
    double estimated_x_m{0.0};       ///< Estimated x position [m].
    double estimated_y_m{0.0};       ///< Estimated y position [m].
    double estimated_theta_rad{0.0}; ///< Estimated heading [rad].
    double left_encoder_m{0.0};      ///< Left encoder reading [m].
    double right_encoder_m{0.0};     ///< Right encoder reading [m].
    double imu_heading_rad{0.0};     ///< IMU heading reading [rad].
    double gyro_z_radps{0.0};        ///< Gyroscope reading [rad/s].
    double position_error_m{0.0};    ///< Distance to target [m].
    double heading_error_rad{0.0};   ///< Heading error to target heading [rad].
};

/**
 * Aggregate summary metrics for one scenario run.
 */
struct ScenarioSummary {
    std::string scenario_name{};    ///< Scenario identifier.
    std::string run_id{};           ///< Run identifier.
    bool success{false};            ///< True when final tolerances are met.
    std::string termination_reason{"not_run"}; ///< Why the run ended.
    std::size_t sample_count{0U};   ///< Number of logged samples.
    double final_position_error_m{0.0}; ///< Final position error [m].
    double final_heading_error_rad{0.0};///< Final heading error [rad].
    double rms_path_error_m{0.0};       ///< RMS path error [m].
    double max_path_error_m{0.0};       ///< Maximum path error [m].
    double overshoot_m{0.0};            ///< Position overshoot metric [m].
    double settling_time_s{std::numeric_limits<double>::quiet_NaN()}; ///< First settled time [s].
    double duration_s{0.0};             ///< Simulated duration [s].
};

/// Full result of one simulation run.
struct SimulationResult {
    ScenarioSummary summary{};               ///< Run-level metrics.
    std::vector<SimulationTraceRow> trace{}; ///< Time-history samples.
};

/// Aggregate metrics for a Monte Carlo batch.
struct MonteCarloSummary {
    std::string scenario_name{};                ///< Scenario identifier.
    std::size_t run_count{0U};                  ///< Number of executed runs.
    std::size_t success_count{0U};              ///< Number of successful runs.
    double success_rate{0.0};                   ///< success_count / run_count [-].
    double mean_final_position_error_m{0.0};    ///< Mean final position error [m].
    double stddev_final_position_error_m{0.0};  ///< Std. dev. of final position error [m].
    double mean_final_heading_error_rad{0.0};   ///< Mean final heading error [rad].
    double stddev_final_heading_error_rad{0.0}; ///< Std. dev. of final heading error [rad].
    double mean_rms_path_error_m{0.0};          ///< Mean RMS path error [m].
    double stddev_rms_path_error_m{0.0};        ///< Std. dev. of RMS path error [m].
    double worst_case_position_error_m{0.0};    ///< Worst final position error [m].
    double worst_case_heading_error_rad{0.0};   ///< Worst final heading error [rad].
    std::string best_run_id{};                  ///< Identifier of best-performing run.
    std::string worst_run_id{};                 ///< Identifier of worst-performing run.
};

/**
 * Convenience bundle returned by the configuration loader.
 */
struct SimulationConfigBundle {
    DriveParams drive{};                                ///< Drivetrain parameters.
    SensorParams sensors{};                             ///< Sensor parameters.
    ControllerParams controllers{};                     ///< Controller parameters.
    std::vector<ScenarioParams> scenarios{};            ///< Scenario definitions.
    MonteCarloConfig monte_carlo{};                     ///< Monte Carlo configuration.
    std::vector<StructuralMember> structural_members{}; ///< Structural member definitions.
    std::vector<StructuralLoadCase> structural_load_cases{}; ///< Structural load cases.
};

}  // namespace robot_sim
