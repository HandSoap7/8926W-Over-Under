#pragma once
#include <functional>
#include <iostream>

class Auton {
 public:
  Auton();
  Auton(std::string, std::function<void()>);
  std::string Name;
  std::function<void()> auton_call;

 private:
};
