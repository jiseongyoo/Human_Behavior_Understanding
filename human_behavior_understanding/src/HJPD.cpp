#include "human_behavior_understanding/HJPD.hpp"

HJPD::HJPD(const rclcpp::NodeOptions & options) : Node("HJPD", options){
  this->declare_parameter("NumberOfBins", 5);

  histogram_server = create_service<HistogramInterface>("HJPD",
     bind(&HJPD::service_callback, this, _1, _2));

  outfile_path = "./src/human_behavior_understanding/dataout/";

  RCLCPP_INFO(this->get_logger(), "%s histogram server is created.", this->get_name());
}


void HJPD::computeHistogram(){
  RCLCPP_INFO(this->get_logger(), "Reading %d files...", file_path.size());

  for (auto & fp : file_path){
    // clear frame_info vector
    frame_info.clear();

    // open file and check existance
    ifstream open_file(fp);
    if (!open_file.is_open()){
      RCLCPP_WARN(this->get_logger(), "Cannot open input file '%s'.", fp.c_str());
      continue;
    }

    // find activitiy
    int act = get_activity(fp);
    outfile << num2string(act) << " ";

    // read 20 lines
    bool notFinish = true;
    int numLine = 0;
    string line, token[5];
    for (int frameNum = 1; notFinish; ++frameNum){
      for (int jointNum = 1; jointNum <= NUMBER_OF_JOINTS; ++jointNum){
        notFinish = (bool)getline(open_file, line);  // read a line
        ++numLine;
        if (!notFinish)  break;          // check file is finished

        // tokenize line
        char *ch = new char[line.length() + 1];
        strcpy(ch, line.c_str());
        string token[5];
        token[0] = std::strtok(ch, " ");
        for (int i = 1; i < 5; ++i){
          token[i] = std::strtok(NULL, " ");
        } delete[] ch;

        // store joint information
        joint_info[jointNum-1].frameID = stoi(token[0]);
        joint_info[jointNum-1].x = stof(token[2]);
        joint_info[jointNum-1].y = stof(token[3]);
        joint_info[jointNum-1].z = stof(token[4]);
      }

      // calculate d1~d5, th1~th5 and put them to frame_info
      if (!notFinish) break;
      calc_dx_dy_dz(joint_info);
    }

    // check not a number
    for (unsigned int t = 1; t <= frame_info.size(); ++t){
      for (int i = 0; i < NUMBER_OF_JOINTS; ++i){
        for (int j = 0; j < 3; ++j)
        if (isnan(frame_info[t].dx_dy_dz[j][i]))
          RCLCPP_ERROR(this->get_logger(), "NaN -> %s : frame %d.", fp.c_str(), i);
      }
    }

    buildHistogram();
    normalizeHistogram();
    concatenateHistogram();
    createHistogram();
  }
}

void HJPD::buildHistogram(){
//  RCLCPP_INFO(this->get_logger(), "frame size = %d", frame_info.size());

  // initialize histogram
  for (int i = 0; i < NUMBER_OF_JOINTS; ++i){
    for (int j = 0; j < 3; ++j){
      histogram_dx_dy_dz[j][i].clear();
      histogram_dx_dy_dz[j][i].resize(number_of_bins);
    }
  }

  if (number_of_bins <= 0){
    RCLCPP_ERROR(this->get_logger(), "Number of bins invalid.");
    return;
  }

  // set lowerbound and upperbound for dx, dy and dz
  const float lb[3] = {-1.0, -1.0, -1.0}, ub[3] = {1.0, 1.0, 1.0};
  const float delta[3] = {(ub[0] - lb[0]) / number_of_bins,
                          (ub[1] - lb[1]) / number_of_bins,
                          (ub[2] - lb[2]) / number_of_bins,};
  for (int j = 0; j < 3; ++j){
    for (int i = 0; i < NUMBER_OF_JOINTS; ++i){
      for (unsigned int t = 0; t < frame_info.size(); ++t){
        int index;

        if (frame_info[t].dx_dy_dz[j][i] <= lb[j])
          index = 0;
        else if (frame_info[t].dx_dy_dz[j][i] >= ub[j])
          index = number_of_bins-1;
        else
          index = int((frame_info[t].dx_dy_dz[j][i] - lb[j]) / delta[j]);

        if (index >= number_of_bins || index < 0){
          RCLCPP_ERROR(this->get_logger(), "Histgoram index invalid.");
          return;
        }
        histogram_dx_dy_dz[j][i][index] += 1.0;
      }
    }
  }

  // check validity
  for (int j = 0; j < 3; ++j){
    for (int i = 0; i < NUMBER_OF_JOINTS; ++i){
      unsigned int sum = 0;
      for (int b = 0; b < number_of_bins; ++b){
        sum += histogram_dx_dy_dz[j][i][b];
      }
      if (frame_info.size() != sum)
        RCLCPP_ERROR(this->get_logger(), "Histogram invalid.");
    }
  }
}

void HJPD::normalizeHistogram(){
  for (int i = 0; i < NUMBER_OF_JOINTS; ++i){
    for (int j = 0; j < 3; ++j){
      for (int b = 0; b < number_of_bins; ++b){
        histogram_dx_dy_dz[j][i][b] /= frame_info.size();
      }
    }
  }
}

void HJPD::concatenateHistogram(){
  histogram_final.clear();

  if (number_of_bins <= 0){
    RCLCPP_ERROR(this->get_logger(), "Number of bins invalid.");
    return;
  }

  for (int i = 0; i < NUMBER_OF_JOINTS; ++i){
    for (int j = 0; j < 3; ++j){
      for (int b = 0; b < number_of_bins; ++b){
        histogram_final.push_back(histogram_dx_dy_dz[j][i][b]);
      }
    }
  }
}

// store result in a line, 1 file becomes 1 line.
void HJPD::createHistogram(){
  // create outfile
  for (unsigned int i = 1; i <= histogram_final.size(); ++i){
    outfile << i << ":" << histogram_final[i - 1] << " ";
  }
  outfile << "\n";
}

void HJPD::calc_dx_dy_dz(JointInfo *joint_info){
  FrameInfo temp;
  temp.frameID = joint_info->frameID;

  for (int i = 0; i < NUMBER_OF_JOINTS; ++i){
    temp.dx_dy_dz[0][i] = joint_info[i].x - joint_info[REF_JOINT].x;
    temp.dx_dy_dz[1][i] = joint_info[i].y - joint_info[REF_JOINT].y;
    temp.dx_dy_dz[2][i] = joint_info[i].z - joint_info[REF_JOINT].z;
  }

  frame_info.push_back(temp);
  // minDx = -1.050000, maxDx = 1.119000
  // minDy = -1.049000, maxDy = 0.912000
  // minDz = -0.906000, maxDz = 1.990000
}

void HJPD::service_callback(const std::shared_ptr<HistogramInterface::Request> request,
                            std::shared_ptr<HistogramInterface::Response> response){
  // get input file path from client
  file_path = request->file_path;

  // get number of bins and create output file name
  number_of_bins = get_parameter("NumberOfBins").as_int();
  string outfile_name = get_outfile_name(this->get_name(), number_of_bins, request->test);

  // create outfile
  outfile = ofstream(outfile_path + outfile_name.c_str());
  RCLCPP_INFO(this->get_logger(), "Creating '%s' is started...", outfile_name.c_str());

  // compute and create histogram
  computeHistogram();

  // close outfile
  outfile.close();

  RCLCPP_INFO(this->get_logger(), "Creating '%s' is finished...", outfile_name.c_str());

  // send result to client
  response->result = "HJPD histogram is created.";
}

// create component registraction macro
RCLCPP_COMPONENTS_REGISTER_NODE(HJPD)
