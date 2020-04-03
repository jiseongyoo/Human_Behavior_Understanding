#include "human_behavior_understanding/histogram.hpp"

// private function
bool histogram::isNum(char ch){
  return ('0' <= ch && ch <= '9') ? true : false;
}

// protected functions
float histogram::get_magnitude(const float x, const float y){
  return std::sqrt(x*x + y*y);
}

float histogram::get_magnitude(const float x, const float y, const float z){
  return std::sqrt(x*x + y*y + z*z);
}

float histogram::get_angle(const float x, const float y){
  float ret = atan2(y, x);
  return (ret < 0 ? ret + 2 * M_PI : ret); // in radian [0, 2*M_PI]
}

float histogram::get_angle(const float x1, const float y1, const float x2, const float y2){
  return acos((x1 * x2 + y1 * y2) / (get_magnitude(x1, y1) * get_magnitude(x2, y2))); // in radian [0, M_PI]
}

float histogram::get_angle(const float x1, const float y1, const float z1, const float x2, const float y2, const float z2){
  return acos((x1*x2 + y1*y2 + z1*z2) / (get_magnitude(x1, y1, z1) * get_magnitude(x2, y2, z2))); // in radian [0, M_PI]
}

float histogram::rad2deg(const float theta){
  return theta * (180.0 / M_PI);
}

string histogram::num2string(const int num){
  int digit = 1, n = num / 10;
  while (n){
    digit++;
    n /= 10;
  }
  std::string str;
  digit = digit >= 2 ? digit : 2;
  str.resize(digit);
  n = num;
  for (int i = digit-1; i >= 0; --i){
    str[i] = n % 10 + '0';
    n /= 10;
  }
  return str;
}

string histogram::get_outfile_name(string histogram_name, int number_of_bins, bool isTest){
  stringstream ss;
  ss << histogram_name << "_b" << num2string(number_of_bins);
  if (isTest) ss << ".t";
  return ss.str();
}

int histogram::get_activity(string &file_name){
  char substr[13] = {"a??_s??_e??_"};
  int a = file_name.length() - 12;

  for (;a >= 0; a--){
    if (substr[0] == file_name[a] && substr[3] == file_name[a+3] &&
        substr[4] == file_name[a+4] && substr[7] == file_name[a+7] &&
        substr[8] == file_name[a+8] && substr[11] == file_name[a+11]){
      if (isNum(file_name[a+1]) && isNum(file_name[a+2]) &&
          isNum(file_name[a+5]) && isNum(file_name[a+6]) &&
          isNum(file_name[a+9]) && isNum(file_name[a+10]))
        break;
    }
  }
  return (file_name[a+1] - '0') * 10 + file_name[a+2] - '0';
}

// public functions
void histogram::read_file_path(const vector<string> &file_path){
  this->file_path.clear();

  for (auto & fp : file_path){
    this->file_path.push_back(fp);
  }
}
