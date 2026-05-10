#include <iostream>
#include <fstream>
#include <cmath>

// Constants for the robot's specifications
constexpr int MAX_LINE_LENGTH = 512;
constexpr float ticks_per_revolution = 1024.0f;
constexpr float wheel_radius_m = 0.3f;
constexpr float wheelbase_m = 1.0f;

// Function to calculate the time difference in seconds between two timestamps
float DeltaTicks(const long timestamp_ms[], int i)
{
  // convert ms to seconds
  return (timestamp_ms[i] - timestamp_ms[i - 1]) / 1000.0;
}

// Function to calculate the distance traveled by a wheel
float CalculateWheelDistance(long current_ticks, long previous_ticks)
{
  float delta_ticks = current_ticks - previous_ticks;
  return delta_ticks * (2 * M_PI * wheel_radius_m) / ticks_per_revolution;
}

// Calculates the average of two distances.
float CalculateAverageDistance(float front_distance, float back_distance)
{
  return (front_distance + back_distance) / 2.0f;
}

// Calculate linea velocity
float CalculateLinearVelocity(float left_distance, float right_distance, float dt)
{
  return (left_distance + right_distance) / 2.0f / dt;
}

// Calculate angular velocity
float CalculateAngularVelocity(float left_distance, float right_distance, float dt)
{
  return (right_distance - left_distance) / wheelbase_m / dt;
}

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

  float x = 0.0;
  float y = 0.0;
  float theta = 0.0;

  for (int i = 1; i < line_num; i++) {
    float dt = DeltaTicks(timestamp_ms, i);

    float fl_distance = CalculateWheelDistance(fl_ticks[i], fl_ticks[i - 1]);
    float fr_distance = CalculateWheelDistance(fr_ticks[i], fr_ticks[i - 1]);
    float bl_distance = CalculateWheelDistance(bl_ticks[i], bl_ticks[i - 1]);
    float br_distance = CalculateWheelDistance(br_ticks[i], br_ticks[i - 1]);

    float left_distance = CalculateAverageDistance(fl_distance, bl_distance);
    float right_distance = CalculateAverageDistance(fr_distance, br_distance);

    float linear_velocity = CalculateLinearVelocity(left_distance, right_distance, dt);
    float angular_velocity = CalculateAngularVelocity(left_distance, right_distance, dt);

    theta += angular_velocity * dt;
    x += linear_velocity * cos(theta) * dt;
    y += linear_velocity * sin(theta) * dt;

    std::cout << timestamp_ms[i] << " " << x << " " << y << " " << theta << "\n";
  }

  return 0;
}
