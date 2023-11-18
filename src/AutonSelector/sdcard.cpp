
#include <filesystem>

#include "main.h"

namespace as {
AutonSelector auton_selector{};

// Initialize auton selector
void initialize(std::string menu_gif_path, std::string gif_path) {
  
  // Set the menu gif path and the gif path
  if (menu_gif_path != "") {
    auton_selector.menu_gif_path = menu_gif_path;
  }
  if (gif_path != "") {
    auton_selector.gif_path = gif_path;
  }

  // Create the auton selector
  auton_selector.create();
}

void shutdown() {
  pros::lcd::shutdown();
}
}  // namespace ez::as
