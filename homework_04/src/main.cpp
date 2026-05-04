#include <iostream>
#include <fstream>
#include <cmath>

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
  float x = 0.0;
  float y = 0.0;
  float theta = 0.0;
  float ticks_per_revolution = 1024.0f;
  float wheel_radius_m = 0.3f;
  float wheelbase_m = 1.0f;

  for (int i = 1; i < line_num; i++) {
    float dt = (timestamp_ms[i] - timestamp_ms[i - 1]) / 1000.0;  // convert ms to seconds

    float fl_distance = (fl_ticks[i] - fl_ticks[i - 1]) * (2 * M_PI * wheel_radius_m) / ticks_per_revolution;
    float fr_distance = (fr_ticks[i] - fr_ticks[i - 1]) * (2 * M_PI * wheel_radius_m) / ticks_per_revolution;
    float bl_distance = (bl_ticks[i] - bl_ticks[i - 1]) * (2 * M_PI * wheel_radius_m) / ticks_per_revolution;
    float br_distance = (br_ticks[i] - br_ticks[i - 1]) * (2 * M_PI * wheel_radius_m) / ticks_per_revolution;

    float left_distance = (fl_distance + bl_distance) / 2.0;
    float right_distance = (fr_distance + br_distance) / 2.0;

    float linear_velocity = (left_distance + right_distance) / 2.0 / dt;
    float angular_velocity = (right_distance - left_distance) / wheelbase_m / dt;

    theta += angular_velocity * dt;
    x += linear_velocity * cos(theta) * dt;
    y += linear_velocity * sin(theta) * dt;

    std::cout << timestamp_ms[i] << " " << x << " " << y << " " << theta << "\n";
  }

  return 0;
}
