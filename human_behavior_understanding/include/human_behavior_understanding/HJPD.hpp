#ifndef HJPD_HPP_
#define HJPD_HPP_

#include <cstring>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "human_behavior_understanding/histogram.hpp"
#include "hbu_interface/srv/histogram.hpp"

#define REF_JOINT   0 // joint 1

using namespace std;
using namespace std::placeholders;
using namespace rclcpp;

class HJPD : public histogram, public rclcpp::Node{
private:
  struct FrameInfo{
  	int frameID;
  	float dx_dy_dz[3][NUMBER_OF_JOINTS];
  };
  using HistogramInterface = hbu_interface::srv::Histogram;
  Service<HistogramInterface>::SharedPtr histogram_server;
  vector<FrameInfo> frame_info;
  // displacement of x, y and z
  vector<float> histogram_dx_dy_dz[3][NUMBER_OF_JOINTS];
  vector<float> histogram_final;

  void computeHistogram();
  void buildHistogram();
  void normalizeHistogram();
  void concatenateHistogram();
  void createHistogram();

  void calc_dx_dy_dz(JointInfo *joint_info);
  void service_callback(const std::shared_ptr<HistogramInterface::Request> request,
                              std::shared_ptr<HistogramInterface::Response> response);
public:
  HJPD(const rclcpp::NodeOptions & options);
};

#endif
