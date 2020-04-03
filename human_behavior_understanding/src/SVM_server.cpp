#include "human_behavior_understanding/SVM_server.hpp"

SVM::SVM(const NodeOptions & options): Node("SVM", options){
  svm_server = create_service<SVMInterface>("SVM",
     std::bind(&SVM::service_callback, this, _1, _2));
  id = 0;
  RCLCPP_INFO(this->get_logger(), "%s server is created.", this->get_name());
}

void SVM::service_callback(const std::shared_ptr<SVMInterface::Request> request,
                            std::shared_ptr<SVMInterface::Response> response){
  if (request->scale){
    // SVM scale
    RCLCPP_INFO(this->get_logger(), "SVM scale mode [%d]", id++);

    stringstream input_file_path;
    stringstream output_file_path;

    input_file_path << request->file_path[0];
    output_file_path << request->file_path[0] << ".scale";

    std::vector<string> scale_argv;
    scale_argv.push_back("-l");  scale_argv.push_back("-1");
    scale_argv.push_back("-u");  scale_argv.push_back("1");
    scale_argv.push_back("-s");  scale_argv.push_back(output_file_path.str());
    scale_argv.push_back(input_file_path.str());

    ns_svm_scale::execute(scale_argv);

    RCLCPP_INFO(this->get_logger(), "SVM scale is finished...");
    response->result = "scale success";
    return;
  }

  if (request->test){
    // SVM test
    RCLCPP_INFO(this->get_logger(), "SVM test mode [%d]", id++);

    const string test_file_path = request->file_path[0];
    const string train_file_path = request->file_path[1];
    const string predict_file_path = test_file_path + ".predict";
    const string accuracy_file_path = predict_file_path + ".accuray";

    if (!check_file_name(test_file_path, "t") || check_file_name(test_file_path, "model")){
      RCLCPP_ERROR(this->get_logger(), "Wrong format of test file -> %s", test_file_path.c_str());
      response->result = "failed";
      return;
    }
    if (!check_file_name(train_file_path, "model") || check_file_name(train_file_path, "t")){
      RCLCPP_ERROR(this->get_logger(), "Wrong format of model file -> %s", train_file_path.c_str());
      response->result = "failed";
      return;
    }
    if (check_file_name(test_file_path, "scale") != check_file_name(train_file_path, "scale")){
      RCLCPP_ERROR(this->get_logger(), "Scale mismatch between test and model files");
      response->result = "failed";
      return;
    }

    char test_argv[5][1024];
    test_argv[0][0] = '\n';
    for (unsigned int i = 0; i <= test_file_path.length(); i++)
      test_argv[1][i] = test_file_path[i];
    for (unsigned int i = 0; i <= train_file_path.length(); i++)
      test_argv[2][i] = train_file_path[i];
    for (unsigned int i = 0; i <= predict_file_path.length(); i++)
      test_argv[3][i] = predict_file_path[i];
    for (unsigned int i = 0; i <= accuracy_file_path.length(); i++)
      test_argv[4][i] = accuracy_file_path[i];

    svm_predict_function(5, test_argv);

    RCLCPP_INFO(this->get_logger(), "SVM test is finished...");
    response->result = "test success";
    return;
  }
  else{
    // SVM train
    RCLCPP_INFO(this->get_logger(), "SVM train mode [%d]", id++);

    const string histogram_path = request->file_path[0];

    char train_argv[2][1024];
    train_argv[0][0] = '\n';
    for (unsigned int i = 0; i <= histogram_path.length(); i++)
      train_argv[1][i] = histogram_path[i];

    svm_train_function(2, train_argv);

    RCLCPP_INFO(this->get_logger(), "SVM train is finished...");
    response->result = "train success";
    return;
  }
}

bool SVM::check_file_name(const string &file_name, const string &format){
  bool ret = false;
  char *name_str = new char[file_name.length()];
  strcpy(name_str, file_name.c_str());
  char *p = strrchr(name_str, '/');
  char *d = &name_str[file_name.length()];

  while(p < d){
    d = strrchr(name_str, '.');
    ret = true;
    for (unsigned int i = 0; i <= format.length(); ++i){
      if (*(d + i + 1) != format[i]){
        *d = 0;
        ret = false;
        break;
      }
    }
    if (ret)  break;
  }

  delete[] name_str;
  return ret;
}

// create component registraction macro
RCLCPP_COMPONENTS_REGISTER_NODE(SVM)
