#include "human_behavior_understanding/HOD.hpp"

HOD::HOD(const rclcpp::NodeOptions & options) : Node("HOD", options){
  declare_parameter("NumberOfBins", 5);

  histogram_server = create_service<HistogramInterface>("HOD",
     std::bind(&HOD::service_callback, this, _1, _2));

  outfile_path = "./src/human_behavior_understanding/dataout/";

  RCLCPP_INFO(this->get_logger(), "%s histogram server is created.", this->get_name());
}

void HOD::computeHistogram(){
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
      calc_xy_yz_xz(joint_info);
    }

    // check not a number
    for (unsigned int t = 1; t <= frame_info.size(); ++t){
      for (int i = 0; i < NUMBER_OF_JOINTS; ++i){
        for (int j = 0; j < 3; ++j){
          if (isnan(frame_info[t].xy_yz_xz[j][i][0]) || isnan(frame_info[t].xy_yz_xz[j][i][1]))
            RCLCPP_ERROR(this->get_logger(), "NaN -> %s : frame %d, joint %d.", fp.c_str(), t, i);
        }
      }
    }

    buildHistogram();
    normalizeHistogram();
    concatenateHistogram();
    createHistogram();
  }
}

void HOD::calculateMagAng(){
  // initialize magitude and angle vectors
  for (int i = 0; i < NUMBER_OF_JOINTS; ++i){
    for (int j = 0; j < 3; ++j){
      mag_xy_yz_xz[j][i].clear();
      ang_xy_yz_xz[j][i].clear();
    }
  }

  // calculate magnitude and angle of between frameIDs
  for (unsigned int t = 1; t < frame_info.size(); ++t){
    for (int i = 0; i < NUMBER_OF_JOINTS; ++i){
      for (int j = 0; j < 3; ++j){
        // get magnitude of displacements of xy, yz and xz between consecutive frames
        mag_xy_yz_xz[j][i].push_back(
          get_magnitude(
            frame_info[t].xy_yz_xz[j][i][0] - frame_info[t-1].xy_yz_xz[j][i][0],
            frame_info[t].xy_yz_xz[j][i][1] - frame_info[t-1].xy_yz_xz[j][i][1]
          )
        );

        // get angle of displacements of xy, yz and xz between consecutive frames in degree [0, 360]
        ang_xy_yz_xz[j][i].push_back(
          rad2deg(get_angle(
            frame_info[t].xy_yz_xz[j][i][0] - frame_info[t-1].xy_yz_xz[j][i][0],
            frame_info[t].xy_yz_xz[j][i][1] - frame_info[t-1].xy_yz_xz[j][i][1]
          )
        ));
      }
    }
  }

//  RCLCPP_INFO(this->get_logger(), "Checking calculated magnitudes and angles...");
  for (int i = 0; i < NUMBER_OF_JOINTS; ++i){
    for (int j = 0; j < 3; ++j)
    if (frame_info.size() - 1 != mag_xy_yz_xz[j][i].size() ||
        frame_info.size() - 1 != ang_xy_yz_xz[j][i].size())
      RCLCPP_ERROR(this->get_logger(), "Size of magnitudes and angles invalid.");
  }
}

void HOD::buildHistogram(){
  calculateMagAng();

  // initialize histogram xy, yz and xz
  for (int l = 0; l < LENGTH_OF_HISTOGRAM; ++l){
    for (int i = 0; i < NUMBER_OF_JOINTS; ++i){
      for (int j = 0; j < 3; ++j){
        histogram_xy_yz_xz[l][j][i].clear();
        histogram_xy_yz_xz[l][j][i].resize(number_of_bins);
      }
    }
  }

  if (number_of_bins < 0){
    RCLCPP_ERROR(this->get_logger(), "Number of bins invalid.");
    return;
  }

  // set lowerbound and upperbound for angle
  const float ang_lb = 0.0, ang_ub = 360.0, delta_ang = (ang_ub - ang_lb) / number_of_bins;
  for (int level = 1; level <= LEVEL_OF_HISTOGRAM; ++level){
    unsigned int div = 1 << (level - 1), subset_size = (frame_info.size() + 1) / div;
    for (int order = 1; order <= (1 << (level - 1)); ++order){
      int l = ((1 << (level - 1)) - 1) + (order - 1);

      for (int i = 0; i < NUMBER_OF_JOINTS; ++i){
        for (int j = 0; j < 3; ++j){
          for (unsigned int d = 0; d < div; ++d){  // time section d
            for (unsigned int t = d * subset_size; (t < (d + 1) * subset_size) && (t < frame_info.size() - 1); ++t){
              if (ang_xy_yz_xz[j][i][t] > 360.0 || ang_xy_yz_xz[j][i][t] < 0.0){
                RCLCPP_ERROR(this->get_logger(), "Wrong angle data");
                return;
              }
              int index = int(ang_xy_yz_xz[j][i][t] / delta_ang);
              index = index <= number_of_bins-1 ? index : number_of_bins - 1;

              histogram_xy_yz_xz[l][j][i][index] += mag_xy_yz_xz[j][i][t];
            }
          }
        }
      }
    }
  }
}

void HOD::normalizeHistogram(){
  // normalize histogram
  for (int l = 0; l < LENGTH_OF_HISTOGRAM; ++l){
    for (int j = 0; j < 3; ++j){
      for (int i = 0; i < NUMBER_OF_JOINTS; ++i){
        float sum = 0.0;
        for (int b = 0; b < number_of_bins; ++b)
          sum += histogram_xy_yz_xz[l][j][i][b];
        for (int b = 0; b < number_of_bins; ++b)
          histogram_xy_yz_xz[l][j][i][b] /= sum;
      }
    }
  }
}

void HOD::concatenateHistogram(){
  // initialize histogram_final
  histogram_final.clear();

  // concatenate all histograms
  for (int l = 0; l < LENGTH_OF_HISTOGRAM; ++l){
    for (int i = 0; i < NUMBER_OF_JOINTS; ++i){
      for (int j = 0; j < 3; ++j){
        for (int b = 0; b < number_of_bins; ++b){
          histogram_final.push_back(histogram_xy_yz_xz[l][j][i][b]);
        }
      }
    }
  }
}

// store result in a line, 1 file becomes 1 line.
void HOD::createHistogram(){
  // create outfile
  for (unsigned int i = 1; i <= histogram_final.size(); ++i){
    outfile << i << ":" << histogram_final[i - 1] << " ";
  }
  outfile << "\n";
}

void HOD::calc_xy_yz_xz(JointInfo *joint_info){
  FrameInfo temp;
  temp.frameID = joint_info->frameID;

  for (int i = 0; i < NUMBER_OF_JOINTS; ++i){
    temp.xy_yz_xz[0][i][0] = joint_info[i].x;
    temp.xy_yz_xz[0][i][1] = joint_info[i].y;
    temp.xy_yz_xz[1][i][0] = joint_info[i].y;
    temp.xy_yz_xz[1][i][1] = joint_info[i].z;
    temp.xy_yz_xz[2][i][0] = joint_info[i].x;
    temp.xy_yz_xz[2][i][1] = joint_info[i].z;
  }

  frame_info.push_back(temp);
}

void HOD::service_callback(
  const std::shared_ptr<HistogramInterface::Request> request,
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
  response->result = "HOD histogram is created.";
}

// create component registraction macro
RCLCPP_COMPONENTS_REGISTER_NODE(HOD)
