#pragma once
#include <tuple>
#include "AutonSelector/auton.hpp"

using namespace std;
class AutonSelector {
 public:
  std::vector<Auton> Autons;
  int selected_auton;
  int auton_count;
  std::string menu_gif_path;
  std::string gif_path;
  AutonSelector();
  AutonSelector(std::vector<Auton> autons);
  
  char* generateAutonList();
  void call_selected_auton();
  void call_auton(int auton_number);
  void print_selected_auton();
  void add_autons(std::vector<Auton> autons);
  void create();
  void end();
  void ImuInitializeGif(int gif_length, std::string gif_path);

};
