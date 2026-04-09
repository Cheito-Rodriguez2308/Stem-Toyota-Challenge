# Robot Competition Simulation Framework

Production-style C++ repository for a differential-drive competition robot simulator and pre-structural analysis workflow.

The project is organized as a modular engineering codebase for reviewer-friendly robotics development. It emphasizes clear separation between the true plant state, simulated sensors, estimated state, controller outputs, scenario orchestration, and future structural screening work.

## Purpose

This repository provides a practical foundation for:

- differential-drive robot motion simulation in 2D
- configurable encoder and IMU/gyro sensor simulation
- closed-loop autonomous controller development
- repeatable scenario execution with CSV logging
- later extension into Monte Carlo robustness analysis
- later extension into simplified structural pre-analysis workflows

## Key Features

- Reduced-order differential-drive dynamics with Euler and RK4 integration options.
- Sensor modeling with configurable noise, bias, drift, delay, dropout, corruption, and quantization.
- Clear separation between `RobotState`, `SensorReadings`, `EstimatedState`, and `RobotInput`.
- PID-based point-to-point and turn-in-place controllers.
- Scenario factory and scenario runner for reproducible autonomous studies.
- CSV trace and summary logging for downstream review and plotting.
- JSON-based configuration using `nlohmann/json`.
- Linear algebra support through Eigen.

## Repository Layout

```text
Toyota Simulations/
|-- CMakeLists.txt
|-- README.md
|-- data/
|   `-- sample_competition_robot.json
|-- docs/
|   `-- architecture_overview.md
|-- examples/
|   `-- run_scenarios.cpp
|-- include/
|   |-- config_loader.hpp
|   |-- control_primitives.hpp
|   |-- core_types.hpp
|   |-- csv_logger.hpp
|   |-- drivetrain_model.hpp
|   |-- math_utils.hpp
|   |-- random_utils.hpp
|   |-- robot_controllers.hpp
|   |-- robot_sim.hpp
|   |-- scenario_framework.hpp
|   |-- sensor_simulator.hpp
|   |-- simulation_modules.hpp
|   `-- simulator.hpp
|-- output/
|-- src/
|   |-- config_loader.cpp
|   |-- control_primitives.cpp
|   |-- csv_logger.cpp
|   |-- drivetrain_model.cpp
|   |-- math_utils.cpp
|   |-- random_utils.cpp
|   |-- robot_controllers.cpp
|   |-- scenario_framework.cpp
|   |-- sensor_simulator.cpp
|   |-- simulation_modules.cpp
|   `-- simulator.cpp
`-- tests/
    |-- CMakeLists.txt
    `-- test_drivetrain.cpp
```

## Architecture Overview

The repository is split into distinct engineering layers:

- Core data and configuration
- True-state simulation
- Sensor modeling
- State estimation
- Control
- Scenario orchestration
- Reporting and logging
- Future structural screening

That separation is deliberate. Motion simulation, sensor modeling, and structural mechanics are related workflow stages, but they are not the same layer and should not be mixed.

## Module Boundaries

### Core and Utilities

- `include/core_types.hpp` defines the shared data contracts and unit conventions.
- `include/config_loader.hpp` and `src/config_loader.cpp` translate JSON config into typed parameters.
- `include/math_utils.hpp` and `include/random_utils.hpp` provide deterministic helper functions used across the repository.

### Simulation Layer

- `include/simulation_modules.hpp` defines the interfaces between initialization, sensing, estimation, control, dynamics, and stopping.
- `include/simulator.hpp` and `src/simulator.cpp` implement the fixed-step simulation loop.
- `include/drivetrain_model.hpp` and `src/drivetrain_model.cpp` implement the true robot plant model.

### Sensor and Estimation Layer

- `include/sensor_simulator.hpp` and `src/sensor_simulator.cpp` implement the encoder and IMU/gyro models plus the wheel-odometry estimator.

### Control Layer

- `include/control_primitives.hpp` and `src/control_primitives.cpp` implement PID, slew-rate limiting, and simple feedforward.
- `include/robot_controllers.hpp` and `src/robot_controllers.cpp` implement point-to-point and turn-in-place control.

### Orchestration and Reporting Layer

- `include/scenario_framework.hpp` and `src/scenario_framework.cpp` create scenarios and wire modules into a runnable workflow.
- `include/csv_logger.hpp` and `src/csv_logger.cpp` write CSV traces and summaries.
- `examples/run_scenarios.cpp` is the minimal end-to-end executable.

### Structural Mechanics Boundary

Structural mechanics is intentionally separate from the motion-control path.

- The repository design allows a later structural screening layer based on simplified beam/plate mechanics.
- That screening layer is not full FEA or FEM.
- FDTD does not belong in the mechanical stress path.
- Trustworthy full stress maps, local concentrations, contact behavior, bolt slip, buckling, nonlinear material response, and joint compliance still require external FEM.

## Governing Equations In The Current Motion Stack

### Differential-Drive Kinematics

- `v = (v_r + v_l) / 2`
- `omega = (v_r - v_l) / track_width`
- `x_dot = v * cos(theta)`
- `y_dot = v * sin(theta)`
- `theta_dot = omega`

### Simplified Wheel-Speed Dynamics

Each side uses a reduced-order first-order response toward a voltage-scaled free-speed target:

- `v_target ~= gain_scale * free_speed * (V_cmd / V_nominal)`
- `dv/dt ~= (v_target - v_current) / tau`

The model then subtracts simple drag and rolling-resistance terms and optionally clamps acceleration.

### Battery Sag Approximation

Battery voltage is reduced algebraically using an internal-resistance estimate:

- `V_available ~= V_nominal - R_internal * I_total`

This is a practical approximation, not a chemistry or thermal model.

## Modeling Assumptions And Limitations

### Drivetrain

Assumptions:
- planar rigid-body differential drive
- longitudinal wheel-speed model only
- lumped drag and rolling resistance
- simple voltage-to-speed response

Not modeled:
- wheel scrub and lateral slip
- impacts and contact-rich interactions
- drivetrain backlash
- chassis flex
- terrain variation
- detailed motor winding dynamics

Results become unreliable when:
- aggressive turning produces significant scrub or slip
- contact, collisions, or terrain changes dominate behavior
- structural compliance materially affects motion

### Sensors

Assumptions:
- independent channel-wise error models
- linear drift accumulation
- sample-history delay
- additive white Gaussian noise

Not modeled:
- correlated timing faults
- thermal drift
- vibration-sensitive IMU behavior
- hardware bus scheduling

Results become unreliable when:
- sensor timing is asynchronous and important
- noise spectra must match real hardware
- wheel slip dominates encoder interpretation

### Control

Assumptions:
- fixed or near-fixed control period
- estimated planar state is the control input
- left/right voltage mixing is sufficient for the current abstraction

Not modeled:
- embedded firmware timing jitter
- actuator deadband compensation
- current-loop dynamics
- advanced trajectory tracking or optimal control

Results become unreliable when:
- latency is large
- actuator nonlinearities dominate
- aggressive maneuvers require more detailed low-level control modeling

## Build Instructions

### Dependencies

- CMake 3.21+
- C++20 compiler
- Eigen
- nlohmann/json

### Configure And Build

```bash
cmake -S . -B build
cmake --build build
```

### Run The Example

```bash
./build/robot_sim_examples data/sample_competition_robot.json
```

The example writes CSV files under `output/`.

### Run The Test

```bash
ctest --test-dir build --output-on-failure
```

## Architecture Notes

If you are reviewing the code for engineering rigor, the best reading order is:

1. `README.md` for scope, assumptions, and subsystem boundaries.
2. `include/core_types.hpp` to understand the data contracts and units.
3. `include/simulation_modules.hpp` and `include/simulator.hpp` to understand how modules interact.
4. `src/drivetrain_model.cpp`, `src/sensor_simulator.cpp`, and `src/robot_controllers.cpp` for the main plant, measurement, and control logic.
5. `src/scenario_framework.cpp` and `examples/run_scenarios.cpp` to see the full execution path.

At runtime, modules interact in this order:

`ScenarioRunner -> Simulator -> SensorModel -> StateEstimator -> Controller -> DynamicsModel -> CSVLogger`

That flow keeps the true state, sensor readings, estimated state, and controller outputs separate so each engineering layer can be reviewed independently.

## Limitations

This repository is intentionally a reduced-order robotics simulator, not a full high-fidelity multibody or structural solver.

- The drivetrain model does not capture wheel scrub, tire slip dynamics, collisions, terrain interaction, or chassis compliance in a high-fidelity way.
- The sensor models are configurable approximations and do not reproduce full hardware timing, vibration, or thermal behavior.
- The current control layer is practical and readable, but it does not represent embedded-rate firmware timing, current loops, or advanced optimal control.
- Structural analysis is intentionally separate from the motion stack. Simplified structural screening can help prioritize later work, but trustworthy full stress maps still require external FEM tools.
- FDTD is not a structural stress-analysis method and does not belong in the mechanical analysis path.
