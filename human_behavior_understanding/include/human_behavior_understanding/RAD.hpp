#ifndef RAD_HPP_
#define RAD_HPP_

#include <cstring>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "human_behavior_understanding/histogram.hpp"
#include "hbu_interface/srv/histogram.hpp"

#define CENTER_OF_HIP     0	  // joint 1
#define HEAD              3		// joint 4
#define RIGHT_HAND        7 	// joint 8
#define LEFT_HAND         11	// joint 12
#define RIGHT_FOOT        15	// joint 16
#define LEFT_FOOT         19	// joint 20

using namespace std;
using namespace std::placeholders;
using namespace rclcpp;

class RAD : public histogram, public rclcpp::Node{
private:
  struct FrameInfo{
    int frameID;
    float d[5], th[5];
  };
  using HistogramInterface = hbu_interface::srv::Histogram;
  Service<HistogramInterface>::SharedPtr histogram_server;
  vector<FrameInfo> frame_info;
  vector<float> histogram_d[5];
  vector<float> histogram_th[5];
  vector<float> histogram_final;

  void computeHistogram();
  void buildHistogram();
  void normalizeHistogram();
  void concatenateHistogram();
  void createHistogram();

  void calc_d_th(JointInfo *joint_info);
  void service_callback(const std::shared_ptr<HistogramInterface::Request> request,
                              std::shared_ptr<HistogramInterface::Response> response);
public:
  RAD(const NodeOptions & options);
};

#endif
