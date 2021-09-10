int i4_min(int i1, int i2);

void pgma_check_data(int xsize, int ysize, int maxg, int *garray);
void pgma_example(int xsize, int ysize, int *garray);

void pgma_read(std::string file_in_name, int &xsize, int &ysize, int &maxg, int **garrary);
void pgma_read_data(std::ifstream &file_in, int xsize, int ysize, int *garray);
void pgma_read_header(std::ifstream &file_in, int &xsize, int &ysize, int &maxg);
void pgma_read_test(std::string file_in_name);

void pgma_write(std::string file_out_name, int xsize, int ysize, int *garray);
void pgma_write_data(std::ofstream &file_out, int xsize, int ysize, int *garray);
void pgma_write_header(std::ofstream &file_out, std::string file_out_name, int xsize, int ysize, int maxg);
void pgma_write_test(std::string file_out_name);

int s_len_trim(std::string s);
void s_word_extract_first(std::string s, std::string &s1, std::string &s2);
