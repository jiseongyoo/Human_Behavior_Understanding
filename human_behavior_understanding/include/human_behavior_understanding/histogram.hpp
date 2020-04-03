#ifndef HISTOGRAM_HPP_
#define HISTOGRAM_HPP_

#include <math.h>
#include <vector>
#include <fstream>
#include <sstream>

#include "rclcpp/rclcpp.hpp"

#define NUMBER_OF_JOINTS  20  // number of joints

using namespace std;

// base class for histogram
class histogram{
private:
  bool isNum(char ch);
protected:
  // compute magnitude and angle of a vector
  float get_magnitude(const float x, const float y);
  float get_magnitude(const float x, const float y, const float z);
  float get_angle(const float x, const float y);
  float get_angle(const float x1, const float y1, const float x2, const float y2);
  float get_angle(const float x1, const float y1, const float z1, const float x2, const float y2, const float z2);
  float rad2deg(const float theta);
  string num2string(int num);
  string get_outfile_name(string histogram_name, int number_of_bins, bool isTest);
  int get_activity(string &file_name);

  struct JointInfo{
    unsigned int frameID;
    float x, y, z;
  }joint_info[20];

  vector<string> file_path;
  int number_of_bins;
  string outfile_path;
  ofstream outfile;
public:
  // read input file
  void read_file_path(const vector<string> &file_path);

  // interface pattern for histogram
  virtual void buildHistogram() = 0;
  virtual void createHistogram() = 0;
  virtual void computeHistogram() = 0;
  virtual void concatenateHistogram() = 0;
  virtual void normalizeHistogram() = 0;
};

#endif
