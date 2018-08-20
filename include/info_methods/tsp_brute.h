#ifndef TSP_BRUTE_H
#define	TSP_BRUTE_H

#include <string>
#include <vector>

std::vector<int> tsp_path(const std::string &distance_filename);
char ch_cap(char ch);
bool ch_eqi(char ch1, char ch2);
int ch_to_digit(char ch);
int file_column_count(std::string input_filename);
int file_row_count(std::string input_filename);
void perm_next3(int n, int p[], bool &more);
double r8_huge();
double *r8mat_data_read(std::string input_filename, int m, int n);
void r8mat_header_read(std::string input_filename, int &m, int &n);
void r8mat_print(int m, int n, double a[], std::string title);
void r8mat_print_some(int m, int n, double a[], int ilo, int jlo, int ihi, int jhi, std::string title);
int s_len_trim(std::string s);
double s_to_r8(std::string s, int *lchar, bool *error);
bool s_to_r8vec(std::string s, int n, double rvec[]);
int s_word_count(std::string s);
void timestamp();

#endif	/* TSP_BRUTE_H */

