#pragma once

/*
 * Purpose:
 *   Convenience umbrella header that pulls in the primary public headers of
 *   the repository.
 *
 * Role In The Architecture:
 *   This file is purely ergonomic. It is useful for examples or quick tools
 *   that want broad access to the library without managing many include lines.
 *
 * Key Contents:
 *   - Core types and configuration
 *   - Math and random utilities
 *   - Simulation, sensing, control, and scenario orchestration headers
 *
 * Dependencies:
 *   - All public repository headers listed below
 */

#include "control_primitives.hpp"
#include "config_loader.hpp"
#include "core_types.hpp"
#include "csv_logger.hpp"
#include "drivetrain_model.hpp"
#include "math_utils.hpp"
#include "random_utils.hpp"
#include "robot_controllers.hpp"
#include "scenario_framework.hpp"
#include "sensor_simulator.hpp"
#include "simulation_modules.hpp"
#include "simulator.hpp"
