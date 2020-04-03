#include "human_behavior_understanding/SVM_client.hpp"

SVMClient::SVMClient(const string &node_name, const NodeOptions &options = rclcpp::NodeOptions())
  : Node(node_name, options){
  svm_name = "SVM";
  svm_client = create_client<SVMInterface>(svm_name.c_str());
  id = 0;
}

void SVMClient::send_request(string &file_path, bool isScale){
  auto request = std::make_shared<SVMInterface::Request>();
  request->file_path[0] = file_path;
  request->test = false;
  request->scale = isScale;

  while (!svm_client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for '%s'", svm_name.c_str());
      return;
    }
    RCLCPP_INFO(this->get_logger(), "'%s' not available, waiting again...", svm_name.c_str());
  }
  RCLCPP_INFO(this->get_logger(), "Request[%d] '%s'", id++, svm_name.c_str());

  // request and spin until response
  auto response = svm_client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), response) == rclcpp::executor::FutureReturnCode::SUCCESS)
    RCLCPP_INFO(this->get_logger(), "'%s' responsed %s", svm_name.c_str(), response.get()->result.c_str());
  else
    RCLCPP_ERROR(this->get_logger(), "'%s' didn't response", svm_name.c_str());
}

void SVMClient::send_request(string &test_file_path, string &train_file_path){
  auto request = std::make_shared<SVMInterface::Request>();
  request->file_path[0] = test_file_path;
  request->file_path[1] = train_file_path;
  request->test = true;
  request->scale = false;

  while (!svm_client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for '%s'", svm_name.c_str());
      return;
    }
    RCLCPP_INFO(this->get_logger(), "'%s' not available, waiting again...", svm_name.c_str());
  }
  RCLCPP_INFO(this->get_logger(), "Request[%d] '%s'", id, svm_name.c_str());

  // request and spin until response
  auto response = svm_client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), response) == rclcpp::executor::FutureReturnCode::SUCCESS)
    RCLCPP_INFO(this->get_logger(), "'%s' responsed %s", svm_name.c_str(), response.get()->result.c_str());
  else
    RCLCPP_ERROR(this->get_logger(), "'%s' didn't response", svm_name.c_str());
}
