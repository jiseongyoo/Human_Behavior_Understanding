#include "rclcpp/rclcpp.hpp"
#include "human_behavior_understanding/RAD.hpp"
#include "human_behavior_understanding/HJPD.hpp"
#include "human_behavior_understanding/HOD.hpp"
#include "human_behavior_understanding/SVM_server.hpp"

int main(int argc, char** argv){
  // ROS2 communication init
  rclcpp::init(argc, argv);

  // create SingleThreadedExecutor
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;

  // create histogram component nodes
  auto rad = std::make_shared<RAD>(options);
  auto hjpd = std::make_shared<HJPD>(options);
  auto hod = std::make_shared<HOD>(options);
  auto svm = std::make_shared<SVM>(options);

  // add components to executor
  exec.add_node(rad);   RCLCPP_INFO(rad->get_logger(), "%s is added to executor", rad->get_name());
  exec.add_node(hjpd);  RCLCPP_INFO(hjpd->get_logger(), "%s is added to executor", hjpd->get_name());
  exec.add_node(hod);   RCLCPP_INFO(hod->get_logger(), "%s is added to executor", hod->get_name());
  exec.add_node(svm);   RCLCPP_INFO(svm->get_logger(), "%s is added to executor", svm->get_name());

  exec.spin();
  rclcpp::shutdown();

  return 0;
}
