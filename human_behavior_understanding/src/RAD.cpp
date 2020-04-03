#include "human_behavior_understanding/RAD.hpp"

RAD::RAD(const NodeOptions & options) : Node("RAD", options){
  declare_parameter("NumberOfBins", 5);

  histogram_server = create_service<HistogramInterface>("RAD",
     std::bind(&RAD::service_callback, this, _1, _2));

  outfile_path = "./src/human_behavior_understanding/dataout/";

  RCLCPP_INFO(this->get_logger(), "%s histogram server is created.", this->get_name());
}

void RAD::computeHistogram(){
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
      calc_d_th(joint_info);
    }

    // check not a number
    for (unsigned int i = 1; i <= frame_info.size(); ++i){
      for (int j = 0; j < 5; ++j){
        if (isnan(frame_info[i].d[j]))
          RCLCPP_ERROR(this->get_logger(), "NaN -> %s : frame %d, d%d.", fp.c_str(), i, j);
        if (isnan(frame_info[i].th[j]))
          RCLCPP_ERROR(this->get_logger(), "NaN -> %s : frame %d, th%d.", fp.c_str(), i, j);
      }
    }

    buildHistogram();
    normalizeHistogram();
    concatenateHistogram();
    createHistogram();
  }
}

void RAD::buildHistogram(){
  // initialize histogram
  for (int i = 0; i < 5; i++){
    histogram_d[i].clear();   histogram_d[i].resize(number_of_bins);
    histogram_th[i].clear();  histogram_th[i].resize(number_of_bins);
  }

  if (number_of_bins <= 0){
    RCLCPP_ERROR(this->get_logger(), "Number of bins invalid.");
    return;
  }

  // set lowerbound and upperbound for d
  const float d_lb = 0.1, d_ub = 1.0, delta_d = (d_ub - d_lb) / number_of_bins;
  for (int i = 0; i < 5; ++i){
    for (unsigned int j = 0; j < frame_info.size(); ++j){
      int index;

      if (frame_info[j].d[i] <= d_lb)
        index = 0;
      else if (frame_info[j].d[i] >= d_ub)
        index = number_of_bins-1;
      else
        index = int((frame_info[j].d[i] - d_lb) / delta_d);

      if (index >= number_of_bins || index < 0){
        RCLCPP_ERROR(this->get_logger(), "Histgoram index invalid.");
        return;
      }
      histogram_d[i][index] += 1.0;
    }
  }

  // set lowerbound and upperbound for th
  const float th_lb = 0.0, th_ub = 3.0, delta_th = (th_ub - th_lb) / number_of_bins;
  for (int i = 0; i < 5; ++i){
    for (unsigned int j = 0; j < frame_info.size(); ++j){
      int index;
      if (frame_info[j].th[i] < th_lb)
        index = 0;
      else if (frame_info[j].th[i] > th_ub)
        index = number_of_bins-1;
      else
        index = int((frame_info[j].th[i] - th_lb) / delta_th);

      if (index >= number_of_bins || index < 0){
        RCLCPP_ERROR(this->get_logger(), "Histgoram index invalid.");
        return;
      }

      histogram_th[i][index] += 1.0;
    }
  }
}

void RAD::normalizeHistogram(){
  if (frame_info.size() == 0){
    RCLCPP_ERROR(this->get_logger(), "No frame info.");
    return;
  }

  for (int i = 0; i < 5; ++i){
    for (int j = 0; j < number_of_bins; ++j){
      histogram_d[i][j] /= frame_info.size();
      histogram_th[i][j] /= frame_info.size();
    }
  }
}

void RAD::concatenateHistogram(){
  histogram_final.clear();
  histogram_final.resize(10 * number_of_bins);

  if (number_of_bins <= 0){
    RCLCPP_ERROR(this->get_logger(), "Number of bins invalid.");
    return;
  }

  for (int i = 0; i < number_of_bins; ++i){
    for (int j = 0; j < 5; ++j){
      histogram_final[j + 10*i] = histogram_d[j][i];
      histogram_final[5 + j + 10 * i] = histogram_th[j][i];
    }
  }
}

// store result in a line, 1 file becomes 1 line.
void RAD::createHistogram(){
  for (unsigned int i = 1; i <= histogram_final.size(); ++i){
    outfile << i << ":" << histogram_final[i - 1] << " ";
  }
  outfile << "\n";
}

void RAD::calc_d_th(JointInfo *joint_info){
  FrameInfo temp;
  temp.frameID = joint_info->frameID;
  int target[6] = {CENTER_OF_HIP, HEAD, RIGHT_HAND, RIGHT_FOOT, LEFT_FOOT, LEFT_HAND};

  for (int i = 0; i < 5; ++i){
    temp.d[i] = get_magnitude(joint_info[target[i + 1]].x - joint_info[target[0]].x,
                              joint_info[target[i + 1]].y - joint_info[target[0]].y,
                              joint_info[target[i + 1]].z - joint_info[target[0]].z);
    temp.th[i] = get_angle(joint_info[target[i + 1]].x - joint_info[target[0]].x,
                           joint_info[target[i + 1]].y - joint_info[target[0]].y,
                           joint_info[target[i + 1]].z - joint_info[target[0]].z,
                           joint_info[target[(i + 1) % 5 + 1]].x - joint_info[target[0]].x,
                           joint_info[target[(i + 1) % 5 + 1]].y - joint_info[target[0]].y,
                           joint_info[target[(i + 1) % 5 + 1]].z - joint_info[target[0]].z);
  }

  frame_info.push_back(temp);
  // min.d = 0.047896
  // max.d = 1.018372
  // min.th = 0.006505
  // max.th = 2.934911
}

void RAD::service_callback(const std::shared_ptr<HistogramInterface::Request> request,
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
  response->result = "RAD histogram is created.";
}

// create component registraction macro
RCLCPP_COMPONENTS_REGISTER_NODE(RAD)
