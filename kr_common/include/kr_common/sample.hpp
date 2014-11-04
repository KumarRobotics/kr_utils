#ifndef KR_COMMON_SAMPLE_HPP_
#define KR_COMMON_SAMPLE_HPP_

#include <random>
#include <chrono>
#include <type_traits>

namespace kr {

/**
 * @brief The Sample class, a thin wrapper over <random> with a few
 * novice-friendly functions. This will be probably be deprecated after C++14.
 * Since there's a plan for augmenting the <algorithm> header with <sample>.
 */
class Sample {
 public:
  /**
   * @brief Randomize the engine
   */
  static void Randomize() { engine_.seed(rd_()); }

  template <typename T>
  static T Uniform(T upper, T lower) {
    static_assert(std::is_arithmetic<T>::value,
                  "template argument not an arithmetic type.");
    return UniformImpl(upper, lower, std::is_integral<T>{});
  }

  template <typename T = double>
  static T Gaussian(T mean = 0, T stddev = 1.0) {
    std::normal_distribution<T> distribution(mean, stddev);
    return distribution(engine_);
  }

 private:
  template <typename T>
  static T UniformImpl(T upper, T lower, std::true_type) {
    std::uniform_int_distribution<T> distribution(upper, lower);
    return distribution(engine_);
  }

  template <typename T>
  static T UniformImpl(T upper, T lower, std::false_type) {
    std::uniform_real_distribution<T> distribution(upper, lower);
    return distribution(engine_);
  }

  static std::random_device rd_;
  static std::default_random_engine engine_;
};

std::random_device Sample::rd_;
std::default_random_engine Sample::engine_;

}  // namespace kr

#endif  // KR_COMMON_SAMPLE_HPP_
