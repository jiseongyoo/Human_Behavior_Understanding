#include "human_behavior_understanding/histogram_client.hpp"

// public functions
HistogramClient::HistogramClient(const string &node_name, const string &histogram_name,
  const NodeOptions &options = NodeOptions()) : Node(node_name, options), histogram_name(histogram_name){
  histogram_client = create_client<HistogramInterface>(histogram_name);
  id = 0;
}

void HistogramClient::send_request(std::vector<string> &file_path, bool isTest){
  auto request = std::make_shared<HistogramInterface::Request>();
  request->file_path = file_path;
  request->test = isTest;

  while (!histogram_client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for '%s'", histogram_name.c_str());
      return;
    }
    RCLCPP_INFO(this->get_logger(), "'%s' not available, waiting again...", histogram_name.c_str());
  }
  RCLCPP_INFO(this->get_logger(), "Request[%d] '%s'", id++, histogram_name.c_str());

  // request and spin until response
  auto response = histogram_client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), response) == rclcpp::executor::FutureReturnCode::SUCCESS)
    RCLCPP_INFO(this->get_logger(), "'%s' responsed success", histogram_name.c_str());
  else
    RCLCPP_ERROR(this->get_logger(), "'%s' responsed failure", histogram_name.c_str());
}

void HistogramClient::set_NumberOfBins(int value){
  auto parameter_client = std::make_shared<rclcpp::SyncParametersClient>(this, histogram_name.c_str());

  while (!parameter_client->wait_for_service(std::chrono::seconds(1))){
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for '%s'", histogram_name.c_str());
      return;
    }
    RCLCPP_INFO(this->get_logger(), "'%s' not available, waiting again...", histogram_name.c_str());
  }

  auto result = parameter_client->set_parameters({Parameter("NumberOfBins", value)});
  if (!result[0].successful)
    RCLCPP_ERROR(this->get_logger(), "Failed to set 'NumberOfBins': %s",
      result[0].reason.c_str());
  else
    RCLCPP_INFO(this->get_logger(), "Successed to set 'NumberOfBins' : %s",
      parameter_client->get_parameters({"NumberOfBins"})[0].value_to_string().c_str());
}

string HistogramClient::get_histogram_name(){
  return this->histogram_name;
}
