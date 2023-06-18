#pragma once

#include "api.h"

class PistonGroup {
 public:
  std::vector<pros::ADIDigitalOut> pistons;
  PistonGroup(std::vector<int> input_ports, bool default_state = false);
  PistonGroup(int input_port, bool default_state = false);
  void set(bool input);
  bool get();
  void button(int toggle);
  void button(int active, int deactive);


 private:
  bool reversed = false;
  bool current = false;
  int last_press = 0;
};