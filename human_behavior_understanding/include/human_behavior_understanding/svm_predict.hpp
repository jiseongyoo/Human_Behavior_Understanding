#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <vector>
#include "human_behavior_understanding/svm.hpp"

namespace pred
{
	extern char *line;
	extern int max_line_len;
	char* readline(FILE *input);
	extern struct svm_model* model;
}

// static int (*info)(const char *fmt,...) = &printf;
extern struct svm_node *x;
extern int max_nr_attr;
extern int predict_probability;

int print_null(const char *s,...);
void predict_exit_with_help();
void predict_exit_input_error(int line_num);
void predict(FILE *input, FILE *output);
void svm_predict_function(int argc, char argv[][1024]);
