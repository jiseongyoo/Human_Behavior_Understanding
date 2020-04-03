#include "human_behavior_understanding/svm_scale.hpp"

void ns_svm_scale::exit_with_help(){
	printf(
	"Usage: svm-scale [options] data_filename\n"
	"options:\n"
	"-l lower : x scaling lower limit (default -1)\n"
	"-u upper : x scaling upper limit (default +1)\n"
	"-s save_filename : save scaling parameters to save_filename\n"
	);
	exit(1);
}

float min_value = -1, max_value = 1;
float lower, upper;
ofstream outfile;

#define max(x,y) (((x)>(y))?(x):(y))
#define min(x,y) (((x)<(y))?(x):(y))

int ns_svm_scale::execute(vector<string> argv){
	unsigned int i;
	string outfile_name = "default";

	for (i = 0; i < argv.size(); ++i){
		if (argv[i][0] != '-')	break;
		i++;
		switch (argv[i-1][1]){
		case 'l':
			lower = stof(argv[i]);
			break;
		case 'u':
			upper = stof(argv[i]);
			break;
		case 's':
			outfile_name = argv[i];
			break;
		default:
			ns_svm_scale::exit_with_help();
		}
	}

	if (argv.size() != i+1)
		ns_svm_scale::exit_with_help();

	string infile_name = argv[i];

	// open file and check existance
	ifstream open_file(infile_name);
	if (!open_file.is_open()){
		printf("Cannot open input file '%s'.", argv[0].c_str());
		return -1;
	}

	ofstream out_file(outfile_name);

	string line;
	while (getline(open_file, line)){
		vector<string> line_parse;
		ns_svm_scale::parse_line(line, line_parse);
		ns_svm_scale::get_min_max(line_parse, min_value, max_value);
		ns_svm_scale::scaled_out(line_parse, out_file, min_value, max_value);
	}

	out_file.close();

	return 0;
}

void ns_svm_scale::parse_line(string &line, vector<string> &parsed_out){
	parsed_out.clear();

	char *ch = new char[line.length() + 1];
	strcpy(ch, line.c_str());
	char *token = strtok(ch, " ");
	parsed_out.push_back(token);
	token = strtok(NULL, " ");

	while(token != NULL){
		string str(token);

		int i = 0;
		for (char c : str){
			if (c == ':')	break;
			++i;
		}

		string index = str.substr(0,i);
		string value = str.substr(i+1, str.length() - i - 1);

		parsed_out.push_back(index);
		parsed_out.push_back(value);

		token = strtok(NULL, " ");
  }

	delete[] ch;
}

void ns_svm_scale::get_min_max(vector<string> &line_parse, float &min_value, float &max_value){
	min_value = 1;
	max_value = -1;

	for (unsigned int i = 2; i < line_parse.size(); i += 2){
		min_value = min(min_value, stof(line_parse[i]));
		max_value = max(max_value, stof(line_parse[i]));
	}
}

void ns_svm_scale::scaled_out(vector<string> &line_parse, ofstream &out_file, float &min_value, float &max_value){
	unsigned int i = 0;
	out_file << line_parse[i++] << " ";
	for (; i < line_parse.size();){
		out_file << line_parse[i++] << ":";
		out_file << (lower + (upper - lower) * (stof(line_parse[i++]) - min_value) / (max_value - min_value)) << " ";
	}

	out_file << "\n";
}
