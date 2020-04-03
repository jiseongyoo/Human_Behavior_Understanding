#ifndef SVM_SERVER_HPP_
#define SVM_SERVER_HPP_

#include <string>
#include <sstream>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "hbu_interface/srv/svm.hpp"
#include "human_behavior_understanding/svm_scale.hpp"
#include "human_behavior_understanding/svm_train.hpp"
#include "human_behavior_understanding/svm_predict.hpp"

using namespace std;
using namespace std::placeholders;
using namespace rclcpp;

class SVM : public rclcpp::Node{
private:
  using SVMInterface = hbu_interface::srv::SVM;
  Service<SVMInterface>::SharedPtr svm_server;
  void service_callback(const std::shared_ptr<SVMInterface::Request> request,
                              std::shared_ptr<SVMInterface::Response> response);
  bool check_file_name(const string &file_name, const string &format);
  int id;
public:
  SVM(const NodeOptions & options);
};

#endif
