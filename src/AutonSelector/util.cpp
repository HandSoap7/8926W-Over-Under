/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "main.h"

pros::Controller master(pros::E_CONTROLLER_MASTER);

namespace ez {
int mode = DISABLE;

/*
 __        __              _      _           _       
 \ \      / (_) __ _  __ _| | ___| |__   ___ | |_ ___ 
  \ \ /\ / /| |/ _` |/ _` | |/ _ \ '_ \ / _ \| __/ __|
   \ V  V / | | (_| | (_| | |  __/ |_) | (_) | |_\__ \
    \_/\_/  |_|\__, |\__, |_|\___|_.__/ \___/ \__|___/
               |___/ |___/                          
*/


void print_WIGGLE_template() {
  std::cout << R"(
      


                                                                          .-'''-.                      
                                     .---.                               '   _    \                    
              .--.                   |   |      __.....__     /|       /   /` '.   \                   
       _     _|__|  .--./)   .--./)  |   |  .-''         '.   ||      .   |     \  '                   
 /\    \\   //.--. /.''\\   /.''\\   |   | /     .-''"'-.  `. ||      |   '      |  '  .|              
 `\\  //\\ // |  || |  | | | |  | |  |   |/     /________\   \||  __  \    \     / / .' |_             
   \`//  \'/  |  | \`-' /   \`-' /   |   ||                  |||/'__ '.`.   ` ..' /.'     |       _    
    \|   |/   |  | /("'`    /("'`    |   |\    .-------------'|:/`  '. '  '-...-'`'--.  .-'     .' |   
     '        |  | \ '---.  \ '---.  |   | \    '-.____...---.||     | |             |  |      .   | / 
              |__|  /'""'.\  /'""'.\ |   |  `.             .' ||\    / '             |  |    .'.'| |// 
                   ||     ||||     ||'---'    `''-...... -'   |/\'..' /              |  '.'.'.'.-'  /  
                   \'. __// \'. __//                          '  `'-'`               |   / .'   \_.'   
                    `'---'   `'---'                                                  `'-'              



)" << '\n';

  printf("WIGGLY BOY");
}



std::string get_last_word(std::string text) {
  std::string word = "";
  for (int i = text.length() - 1; i >= 0; i--) {
    if (text[i] != ' ') {
      word += text[i];
    } else {
      std::reverse(word.begin(), word.end());
      return word;
    }
  }
  std::reverse(word.begin(), word.end());
  return word;
}
std::string get_rest_of_the_word(std::string text, int position) {
  std::string word = "";
  for (int i = position; i < text.length(); i++) {
    if (text[i] != ' ' && text[i] != '\n') {
      word += text[i];
    } else {
      return word;
    }
  }
  return word;
}
//All iance\n\nWE WIN THESE!!!!! 
void print_to_screen(std::string text, int line) {
  int CurrAutoLine = line;
  std::vector<string> texts = {};
  std::string temp = "";

  for (int i = 0; i < text.length(); i++) {
    if (text[i] != '\n' && temp.length() + 1 > 32) {
      auto last_word = get_last_word(temp);
      if (last_word == temp) {
        texts.push_back(temp);
        temp = text[i];
      } else {
        int size = last_word.length(); 

        auto rest_of_word = get_rest_of_the_word(text, i); 
        temp.erase(temp.length() - size, size);
        texts.push_back(temp);
        last_word += rest_of_word;
        i += rest_of_word.length();
        temp = last_word;
        if (i >= text.length() - 1) {
          texts.push_back(temp);
          break;
        }
        
      }
    }
    if (i >= text.length() - 1) {
      temp += text[i];
      texts.push_back(temp);
      temp = "";
      break;
    } else if (text[i] == '\n') {
      texts.push_back(temp);
      temp = "";
    } else {
      temp += text[i];
    }
  }
  for (auto i : texts) {
    if (CurrAutoLine > 7) {
      pros::lcd::clear();
      pros::lcd::set_text(line, "Out of Bounds. Print Line is too far down");
      return;
    }
    pros::lcd::clear_line(CurrAutoLine);
    pros::lcd::set_text(CurrAutoLine, i);
    CurrAutoLine++;
  }
}

std::string exit_to_string(exit_output input) {
  switch ((int)input) {
    case RUNNING:
      return "Running";
    case SMALL_EXIT:
      return "Small";
    case BIG_EXIT:
      return "Big";
    case VELOCITY_EXIT:
      return "Velocity";
    case mA_EXIT:
      return "mA";
    case ERROR_NO_CONSTANTS:
      return "Error: Exit condition constants not set!";
    default:
      return "Error: Out of bounds!";
  }

  return "Error: Out of bounds!";
}
namespace util {
bool AUTON_RAN = true;

bool is_reversed(double input) {
  if (input < 0) return true;
  return false;
}

int sgn(double input) {
  if (input > 0)
    return 1;
  else if (input < 0)
    return -1;
  return 0;
}

double clip_num(double input, double max, double min) {
  if (input > max)
    return max;
  else if (input < min)
    return min;
  return input;
}

}  // namespace util
}  // namespace ez
