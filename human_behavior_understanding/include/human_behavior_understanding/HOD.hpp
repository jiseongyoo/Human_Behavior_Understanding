#ifndef HOD_HPP_
#define HOD_HPP_

#include <cstring>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "human_behavior_understanding/histogram.hpp"
#include "hbu_interface/srv/histogram.hpp"

#define LEVEL_OF_HISTOGRAM  3
#define LENGTH_OF_HISTOGRAM ((1 << LEVEL_OF_HISTOGRAM) - 1)

using namespace std;
using namespace std::placeholders;
using namespace rclcpp;

class HOD : public histogram, public rclcpp::Node{
private:
  struct FrameInfo{
    int frameID;
    float xy_yz_xz[3][NUMBER_OF_JOINTS][2]; // [0] xy, [1] yz, [2] xz
  };
  using HistogramInterface = hbu_interface::srv::Histogram;
  Service<HistogramInterface>::SharedPtr histogram_server;
  vector<FrameInfo> frame_info;
  // magnitude and angle of differences of xy, yz and xz
  vector<float> mag_xy_yz_xz[3][NUMBER_OF_JOINTS];        // [0] xy, [1] yz, [2] xz
  vector<float> ang_xy_yz_xz[3][NUMBER_OF_JOINTS];        // [0] xy, [1] yz, [2] xz
  // histogram of xy, yz and xz with level
  vector<float> histogram_xy_yz_xz[LENGTH_OF_HISTOGRAM][3][NUMBER_OF_JOINTS];
  vector<float> histogram_final;

  void computeHistogram();
  void calculateMagAng();
  void buildHistogram();
  void normalizeHistogram();
  void concatenateHistogram();
  void createHistogram();

  void calc_xy_yz_xz(JointInfo *joint_info);
  void service_callback(
    const std::shared_ptr<HistogramInterface::Request> request,
    std::shared_ptr<HistogramInterface::Response> response
  );
public:
  HOD(const rclcpp::NodeOptions & options);
};

#endif
