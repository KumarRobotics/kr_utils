#ifndef KR_COMMON_SAMPLE_HPP_
#define KR_COMMON_SAMPLE_HPP_

namespace kr {
namespace common {

#include <random>
#include <chrono>

class Sample {
 public:
  static int Uniform(int upper, int lower) {
    std::uniform_int_distribution<int> distribution(upper, lower);
    return distribution(generator_int);
  }

  /**
   * @brief Uniform distribution real
   */
  template <typename T>
  static T Uniform() {
    std::uniform_real_distribution<T> distribution(0.0, 1.0);
    return distribution(generator_real);
  }

  /**
   * @brief Gaussian distribution
   */
  template <typename T>
  static T Gaussian(T mean = 0.0, T stddev = 1.0) {
    std::normal_distribution<double> distribution(mean, stddev);
    return distribution(generator_real);
  }

 private:
};

}  // namespace common
}  // namespace kr

#endif  // KR_COMMON_SAMPLE_HPP_
