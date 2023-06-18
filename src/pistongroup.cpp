#include "main.h"

// Constructor for multiple pistons
PistonGroup::PistonGroup(std::vector<int> input_ports, bool default_state) {
  for (auto i : input_ports) {
    pros::ADIDigitalOut temp(i, default_state);
    pistons.push_back(temp);
  }
  reversed = default_state;
}

// Constructor for one piston
PistonGroup::PistonGroup(int input_port, bool default_state) {
  pros::ADIDigitalOut temp(input_port, default_state);
  pistons.push_back(temp);
  reversed = default_state;
}

void PistonGroup::set(bool input) {
  for (auto i : pistons) {
    i.set_value(reversed ? !input : input);
  }
  current = input;
}

// Get the current state
bool PistonGroup::get() { return current; }

// Toggle for user control
void PistonGroup::button(int toggle) {
  if (toggle && !last_press) {
    set(!get());
  }
  last_press = toggle;
}

// Two button control for piston
void PistonGroup::button(int active, int deactive) {
  if (active && !get())
    set(true);
  else if (deactive && get())
    set(false);
}