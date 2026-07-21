#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <random>
#include <string>

#include "ur5e_kinematics/ur5e_kinematics.hpp"

namespace
{
std::size_t parseSampleCount(int argc, char ** argv)
{
  if (argc < 2) {
    return 1000;
  }
  try {
    return std::max<std::size_t>(1, std::stoul(argv[1]));
  } catch (const std::exception &) {
    std::cerr << "Invalid sample count: " << argv[1] << '\n';
    return 1000;
  }
}
}  // namespace

int main(int argc, char ** argv)
{
  using ur5e_kinematics::IkOptions;
  using ur5e_kinematics::JointVector;
  using ur5e_kinematics::Ur5eKinematics;

  const std::size_t sample_count = parseSampleCount(argc, argv);
  Ur5eKinematics model;
  std::mt19937 generator(20260714U);
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  std::normal_distribution<double> seed_noise(0.0, 0.18);

  IkOptions options;
  options.max_iterations = 250;

  std::size_t success_count = 0;
  double successful_position_error_sum = 0.0;
  double successful_orientation_error_sum = 0.0;
  double successful_worst_position_error = 0.0;
  double successful_worst_orientation_error = 0.0;
  double iteration_sum = 0.0;

  const auto start = std::chrono::steady_clock::now();
  for (std::size_t sample = 0; sample < sample_count; ++sample) {
    JointVector target_joints;
    for (std::size_t joint = 0; joint < ur5e_kinematics::kDof; ++joint) {
      // Stay away from exact mechanical limits to measure local numerical IK,
      // rather than failure caused by deliberately infeasible seeds.
      const double lower = model.lowerLimits()[joint] * 0.75;
      const double upper = model.upperLimits()[joint] * 0.75;
      target_joints[joint] = lower + unit(generator) * (upper - lower);
    }

    JointVector seed = target_joints;
    for (std::size_t joint = 0; joint < ur5e_kinematics::kDof; ++joint) {
      seed[joint] += seed_noise(generator);
    }

    const auto result = model.inverse(model.forward(target_joints), seed, options);
    if (result.converged) {
      ++success_count;
      successful_position_error_sum += result.position_error;
      successful_orientation_error_sum += result.orientation_error;
      successful_worst_position_error =
        std::max(successful_worst_position_error, result.position_error);
      successful_worst_orientation_error =
        std::max(successful_worst_orientation_error, result.orientation_error);
    }
    iteration_sum += static_cast<double>(result.iterations);
  }
  const auto end = std::chrono::steady_clock::now();
  const double elapsed_ms =
    std::chrono::duration<double, std::milli>(end - start).count();
  const double successful_denominator =
    static_cast<double>(std::max<std::size_t>(1, success_count));
  const double success_rate =
    100.0 * static_cast<double>(success_count) / static_cast<double>(sample_count);

  std::cout << std::fixed << std::setprecision(8)
            << "UR5e self-developed kinematics benchmark\n"
            << "samples: " << sample_count << '\n'
            << "success_count: " << success_count << '\n'
            << "failure_count: " << sample_count - success_count << '\n'
            << "success_rate: " << success_rate << "%\n"
            << "successful_mean_position_error_m: "
            << successful_position_error_sum / successful_denominator << '\n'
            << "successful_worst_position_error_m: "
            << successful_worst_position_error << '\n'
            << "successful_mean_orientation_error_rad: "
            << successful_orientation_error_sum / successful_denominator << '\n'
            << "successful_worst_orientation_error_rad: "
            << successful_worst_orientation_error << '\n'
            << "mean_iterations: " << iteration_sum / sample_count << '\n'
            << "mean_solve_time_us: " << 1000.0 * elapsed_ms / sample_count << '\n';

  // A numerical local IK method is not globally complete. Treat >=99% as a
  // passing regression threshold while still reporting every failed sample.
  return success_rate >= 99.0 ? 0 : 1;
}
