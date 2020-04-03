#ifndef HISTOGRAM_CLIENT_HPP_
#define HISTOGRAM_CLIENT_HPP_

#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "hbu_interface/srv/histogram.hpp"

using namespace std;
using namespace std::placeholders;
using namespace rclcpp;
using HistogramInterface = hbu_interface::srv::Histogram;

class HistogramClient : public rclcpp::Node{
private:
  string histogram_name;
  rclcpp::Client<HistogramInterface>::SharedPtr histogram_client;
  int id;
public:
  HistogramClient(const string &node_name, const string &histogram_name, const NodeOptions &options);
  void send_request(vector<string> &file_path, bool isTest = false);
  void set_NumberOfBins(int value);
  string get_histogram_name();
};

#endif
