#include <string>
#include <sstream>
#include <fstream>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "hbu_interface/srv/histogram.hpp"
#include "hbu_interface/srv/svm.hpp"
#include "human_behavior_understanding/histogram_client.hpp"
#include "human_behavior_understanding/SVM_client.hpp"

std::string num2string(int num);

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;

  int bin_lb = 5, bin_ub = 20;

  // find all of input files for train and test
  std::vector<string> train_file_path, test_file_path, single_file_path;
  for (int act = 1; act <= 16; ++act){
    for (int sub = 1; sub <= 10; ++sub){
      for (int epi = 1; epi <= 2; ++epi){
        std::stringstream ss_train, ss_test;

        ss_train << "./src/human_behavior_understanding/dataset/train/"
                 << "a" << num2string(act) << "_s" << num2string(sub) << "_e" << num2string(epi)
                 << "_skeleton_proj.txt";
        ss_test << "./src/human_behavior_understanding/dataset/test/"
                << "a" << num2string(act) << "_s" << num2string(sub) << "_e" << num2string(epi)
                << "_skeleton_proj.txt";

        if (std::fstream(ss_train.str()).good())
          train_file_path.push_back(ss_train.str());

        if (std::fstream(ss_test.str()).good())
          test_file_path.push_back(ss_test.str());
      }
    }
  }

  auto RAD_client = std::make_shared<HistogramClient>("Histogram_client_RAD", "RAD", options);
  auto HJPD_client = std::make_shared<HistogramClient>("Histogram_client_HJPD", "HJPD", options);
  auto HOD_client = std::make_shared<HistogramClient>("Histogram_client_HOD", "HOD", options);

  for (int i = bin_lb; i <= bin_ub; ++i){
    RAD_client->set_NumberOfBins(i);
    HJPD_client->set_NumberOfBins(i);
    HOD_client->set_NumberOfBins(i);

    RAD_client->send_request(train_file_path);
    HJPD_client->send_request(train_file_path);
    HOD_client->send_request(train_file_path);

    RAD_client->send_request(test_file_path, true);
    HJPD_client->send_request(test_file_path, true);
    HOD_client->send_request(test_file_path, true);
  }

  vector<string> histogram;
  histogram.push_back("RAD");
  histogram.push_back("HJPD");
  histogram.push_back("HOD");

  auto SVM_client = std::make_shared<SVMClient>("SVM_client", options);

  // find all of histogram files to scale
  std::vector<string> histogram_files;
  for (int bin = bin_lb; bin <= bin_ub; ++bin){
    for (unsigned int i = 0; i < histogram.size(); ++i){
      std::stringstream ss_histogram_train, ss_histogram_test;
      ss_histogram_train << "./src/human_behavior_understanding/dataout/"<< histogram[i] << "_b" << num2string(bin);
      ss_histogram_test  << "./src/human_behavior_understanding/dataout/"<< histogram[i] << "_b" << num2string(bin) << ".t";

      if (std::fstream(ss_histogram_train.str()).good())
        histogram_files.push_back(ss_histogram_train.str());
      if (std::fstream(ss_histogram_test.str()).good())
        histogram_files.push_back(ss_histogram_test.str());
    }
  }

  // scale train and test histogram files
  for (auto &path : histogram_files){
    SVM_client->send_request(path, true);
  }

  // find all train histogram_files
  std::vector<string> train_files;
  for (int bin = bin_lb; bin <= bin_ub; ++bin){
    for (unsigned int i = 0; i < histogram.size(); ++i){
      std::stringstream ss_train, ss_train_s;
      ss_train   << "./src/human_behavior_understanding/dataout/"<< histogram[i] << "_b" << num2string(bin);
      ss_train_s << "./src/human_behavior_understanding/dataout/"<< histogram[i] << "_b" << num2string(bin) << ".scale";

      if (std::fstream(ss_train.str()).good())
        train_files.push_back(ss_train.str());
      if (std::fstream(ss_train_s.str()).good())
        train_files.push_back(ss_train_s.str());
    }
  }

  // train all train histogram files
  for (auto &path : train_files){
    SVM_client->send_request(path, false);
  }

  std::vector<string> test_files, model_files;
  for (int bin = bin_lb; bin <= bin_ub; ++bin){
    for (unsigned int i = 0; i < histogram.size(); ++i){
      std::stringstream ss_test, ss_model;
      std::stringstream ss_test_s, ss_model_s;

      ss_test  << "./src/human_behavior_understanding/dataout/"<< histogram[i] << "_b" << num2string(bin) << ".t";
      ss_model << "./src/human_behavior_understanding/dataout/"<< histogram[i] << "_b" << num2string(bin) << ".model";

      ss_test_s  << "./src/human_behavior_understanding/dataout/"<< histogram[i] << "_b" << num2string(bin) << ".t.scale";
      ss_model_s << "./src/human_behavior_understanding/dataout/"<< histogram[i] << "_b" << num2string(bin) << ".scale.model";

      if (std::fstream(ss_test.str()).good() && std::fstream(ss_model.str()).good()){
        test_files.push_back(ss_test.str());
        model_files.push_back(ss_model.str());
      }

      if (std::fstream(ss_test_s.str()).good() && std::fstream(ss_model_s.str()).good()){
        test_files.push_back(ss_test_s.str());
        model_files.push_back(ss_model_s.str());
      }
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("SVM_controller"), "%d files will be tested", test_files.size());

  if (test_files.size() != model_files.size()){
    RCLCPP_ERROR(rclcpp::get_logger("SVM_controller"), "Wrong number of test and model files");
  }
  else{
    for (unsigned int i = 0; i < test_files.size(); ++i)
      SVM_client->send_request(test_files[i], model_files[i]);
  }

  RCLCPP_INFO(rclcpp::get_logger("SVM_controller"), "Work finished");
  rclcpp::shutdown();

  return 0;
}

std::string num2string(const int num){
  int digit = 1, n = num / 10;
  while (n){
    digit++;
    n /= 10;
  }
  std::string str;
  digit = digit >= 2 ? digit : 2;
  str.resize(digit);
  n = num;
  for (int i = digit-1; i >= 0; --i){
    str[i] = n % 10 + '0';
    n /= 10;
  }
  return str;
}
