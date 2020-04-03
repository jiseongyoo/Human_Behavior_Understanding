#ifndef SVM_SCALE_HPP_
#define SVM_SCALE_HPP_

#include <vector>
#include <cstring>
#include <string>
#include <fstream>
#include <iostream>

using namespace std;

namespace ns_svm_scale{
	void exit_with_help();
	int execute(vector<string> argv);
	void parse_line(string &line, vector<string> &parsed_out);
	void get_min_max(vector<string> &line_parse, float &min_value, float &max_value);
	void scaled_out(vector<string> &line_parse, ofstream &out_file, float &min_value, float &max_value);
}

#endif
