
#pragma once

#include "AutonSelector/auton_selector.hpp"
#include "api.h"


namespace ez {
namespace as {
extern AutonSelector auton_selector;

/**
 * @param menu_gif_path The path to the gif to display on the selection menu
 * @param gif_path The path to the gif to display during a match
 */
void initialize(std::string menu_gif_path = "", std::string gif_path = "");

/**
 * Wrapper for pros::lcd::shutdown.
 */
void shutdown();

}  // namespace as
}  // namespace ez