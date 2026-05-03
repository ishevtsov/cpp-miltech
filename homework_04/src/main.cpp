#include <iostream>
#include <fstream>

int main(int argc, char** argv)
{
  if (argc != 2) {
    std::cerr << "usage: ugv_odometry <input_path>\n";
    return 1;
  }

  const char* filename = argv[1];

  // The input file read as an argument is expected to be a text file with 5 whitespace-separated numbers per line:
  //   timestamp_ms fl_ticks fr_ticks bl_ticks br_ticks
  std::ifstream infile(filename);
  if (!infile.is_open()) {
    std::cerr << "error: could not open input file\n";
    return 1;
  }

  const int MAX_LINE_LENGTH = 512;

  long timestamp_ms[MAX_LINE_LENGTH]{};
  long fl_ticks[MAX_LINE_LENGTH]{};
  long fr_ticks[MAX_LINE_LENGTH]{};
  long bl_ticks[MAX_LINE_LENGTH]{};
  long br_ticks[MAX_LINE_LENGTH]{};

  int line_num = 0;

  while (infile >> timestamp_ms[line_num] >> fl_ticks[line_num] >> fr_ticks[line_num] >> bl_ticks[line_num] >> br_ticks[line_num]) {
    line_num++;
  }
  infile.close();

  // TODO: implement wheel odometry for a 4-wheel differential-drive UGV.
  //
  // Parameters:
  //   ticks_per_revolution = 1024
  //   wheel_radius_m       = 0.3
  //   wheelbase_m          = 1.0
  //
  // Input:  text file with 5 whitespace-separated numbers per line:
  //         timestamp_ms fl_ticks fr_ticks bl_ticks br_ticks
  // Output: same tabular format on stdout, starting from the second sample:
  //         timestamp_ms x y theta

  return 0;
}
