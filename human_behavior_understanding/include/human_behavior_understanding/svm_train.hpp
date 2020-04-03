#ifndef SVM_TRAIN_HPP
#define SVM_TRAIN_HPP

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>
#include "human_behavior_understanding/svm.hpp"

extern struct svm_parameter param;		// set by parse_command_line
extern struct svm_problem prob;		// set by read_problem
extern struct svm_node *x_space;
extern int cross_validation;
extern int nr_fold;

namespace train
{
	extern char *line;
	extern int max_line_len;
	char* readline(FILE *input);
	extern struct svm_model *model;
}

#define Malloc(type,n) (type *)malloc((n)*sizeof(type))

void print_null(const char *s);
void train_exit_with_help();
void train_exit_input_error(int line_num);
void parse_command_line(int argc, char **argv, char *input_file_name, char *model_file_name);
void read_problem(const char *filename);
void do_cross_validation();
int svm_train_function(int argc, char argv[][1024]);
void do_cross_validation();
void parse_command_line(int argc, char argv[][1024], char *input_file_name, char *model_file_name);
void read_problem(const char *filename);

#endif
