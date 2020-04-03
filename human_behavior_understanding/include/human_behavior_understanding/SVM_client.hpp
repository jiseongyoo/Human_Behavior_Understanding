#ifndef SVM_CLIENT_HPP_
#define SVM_CLIENT_HPP_

#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "hbu_interface/srv/svm.hpp"

using namespace std;
using namespace std::placeholders;
using namespace rclcpp;

class SVMClient : public rclcpp::Node{
private:
  string svm_name;
  using SVMInterface = hbu_interface::srv::SVM;
  rclcpp::Client<SVMInterface>::SharedPtr svm_client;
  int id;

public:
  SVMClient(const string &node_name, const NodeOptions &options);
  void send_request(string &file_path, bool isScale = false);
  void send_request(string &test_file_path, string &train_model_file_path);
};


#endif
