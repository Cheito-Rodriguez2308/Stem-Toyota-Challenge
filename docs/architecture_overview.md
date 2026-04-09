# Architecture Overview

## Purpose

This document explains how the current repository is organized, what the main modules do, and where the present fidelity boundaries are.

The repository is intentionally split into distinct engineering layers:

- true-state simulation
- sensor modeling
- state estimation
- control
- orchestration and logging
- future structural screening

That separation is deliberate. Motion simulation and structural stress screening are not the same problem and should not be conflated.

## Execution Flow

The current end-to-end path for a scenario run is:

1. `ScenarioRunner` selects or receives a `ScenarioParams` object.
2. `DefaultStateInitializer` creates the initial true `RobotState`.
3. `Simulator` executes the fixed-step loop.
4. `DifferentialDriveSensorSimulator` converts the true state into `SensorReadings`.
5. `WheelOdometryEstimator` converts measurements into `EstimatedState`.
6. A controller converts the estimate into `RobotInput`.
7. `DifferentialDriveDynamics` advances the true state.
8. `Simulator` records `SimulationTraceRow` values and computes a `ScenarioSummary`.
9. `CSVLogger` writes output files.

## Main Modules

### `core_types`

Purpose:
- define shared data contracts and unit conventions

Important separation:
- `RobotState` is the true plant state
- `SensorReadings` are simulated measurements
- `EstimatedState` is what the controller should trust
- `RobotInput` is the controller output sent to the plant

### `simulation_modules`

Purpose:
- define the abstract interfaces between layers

Why it matters:
- the simulator loop does not need to know which concrete sensor model,
  controller, or estimator is used

### `simulator`

Purpose:
- run one fixed-step simulation

Assumptions:
- fixed time step
- synchronous module update order
- no event-driven scheduler

Misses:
- asynchronous buses
- multi-rate embedded loops
- hybrid contact events

### `drivetrain_model`

Purpose:
- simulate the ground-truth planar motion of the robot

Current model:
- differential-drive kinematics
- simple first-order wheel-speed dynamics
- optional drag
- optional acceleration saturation
- optional battery-voltage scaling

Assumptions:
- rigid planar robot
- longitudinal motion only
- lumped resistance effects

Misses:
- wheel slip and scrub models
- chassis compliance
- terrain changes
- collisions and impacts

Higher-fidelity needs:
- multibody contact simulation
- identified tire/traction models
- richer actuator modeling

### `sensor_simulator`

Purpose:
- simulate encoder and IMU/gyro measurements

Current effects:
- Gaussian noise
- static bias
- drift
- quantization
- delay
- dropout
- corrupted samples

Assumptions:
- per-channel independent error models
- delay represented with buffered truth
- drift represented as a simple accumulated term

Misses:
- correlated channel faults
- thermal behavior
- bus timing artifacts
- hardware-specific vibration sensitivity

### `robot_controllers`

Purpose:
- convert estimated state into left/right voltage commands

Current controllers:
- point-to-point controller
- turn-in-place controller

Assumptions:
- planar state feedback
- fixed control period
- simple left/right voltage mixing

Misses:
- current control
- deadband compensation
- advanced trajectory tracking
- real embedded timing jitter

### `scenario_framework`

Purpose:
- make scenario execution simple and repeatable

Current responsibilities:
- provide example scenarios
- build module objects
- run batches
- write CSV outputs

### `csv_logger`

Purpose:
- export trace and summary data for plotting and review

Why it matters:
- engineering review is easier when the simulator emits transparent data in a
  simple external format

## Structural Mechanics Boundary

The structural path is intentionally separate from the robotics motion path.

Important boundary conditions:

- the repository may include a simplified structural screening layer later
- that screening layer is not full FEM
- FDTD is not a structural mechanics method and does not belong in the stress-analysis path
- reliable full stress maps still require external FEM tools such as CalculiX, FreeCAD FEM, COMSOL, ANSYS, or Abaqus

Structural screening becomes unreliable when any of the following dominate:

- local stress concentrations
- contact loads
- bolt slip or joint flexibility
- chassis flex
- impact loading
- buckling
- nonlinear material behavior
- uncertain real material properties

## Suggested Reading Order

For a reviewer:

1. `README.md`
2. `include/core_types.hpp`
3. `include/simulation_modules.hpp`
4. `include/simulator.hpp`
5. `src/drivetrain_model.cpp`
6. `src/sensor_simulator.cpp`
7. `src/robot_controllers.cpp`
8. `src/scenario_framework.cpp`
9. `examples/run_scenarios.cpp`
