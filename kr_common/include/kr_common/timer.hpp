#ifndef KR_COMMON_TIMER_HPP_
#define KR_COMMON_TIEMR_HPP_

#include <chrono>
#include <string>
#include <cassert>
#include <ostream>
#include <iomanip>
#include <iostream>
#include <thread>

namespace kr {

typedef std::chrono::seconds sec;
typedef std::chrono::milliseconds ms;
typedef std::chrono::microseconds us;
typedef std::chrono::nanoseconds ns;

/**
 * @brief The is_duration struct
 */
template <typename>
struct is_duration : std::false_type {};

template <typename T, typename U>
struct is_duration<std::chrono::duration<T, U>> : std::true_type {};

/**
 * @brief Unit
 * @return unit as a string
 */
template <typename T>
std::string Unit() {
  return "unknown unit";
}

template <>
std::string Unit<sec>() {
  return "sec";
}

template <>
std::string Unit<ms>() {
  return "ms";
}

template <>
std::string Unit<us>() {
  return "us";
}

template <>
std::string Unit<ns>() {
  return "ns";
}

/**
 * @brief Ratio
 * @return ratio of type T and U
 */
template <typename T, typename U>
double Ratio() {
  typedef typename T::period TP;
  typedef typename U::period UP;
  typedef typename std::ratio_divide<TP, UP>::type RP;
  return static_cast<double>(RP::num) / RP::den;
}

/**
 * @brief The Timer class
 */
template <typename D>
class Timer {
  static_assert(is_duration<D>::value, "Not a valid duration type");

 public:
  explicit Timer(const std::string& name, int report_every_n_iter = 0)
      : name_(name), report_every_n_iter_(report_every_n_iter) {}

  int iteration() const { return iteration_; }
  const std::string& name() const { return name_; }

  /**
   * @brief Start, start timing
   */
  void Start() {
    assert(!running_);
    running_ = true;
    start_ = std::chrono::high_resolution_clock::now();
  }

  /**
   * @brief Stop, stop timing
   */
  void Stop() {
    elapsed_ = std::chrono::duration_cast<D>(
        std::chrono::high_resolution_clock::now() - start_);
    assert(running_);
    total_ += elapsed_;
    ++iteration_;
    min_ = std::min(elapsed_, min_);
    max_ = std::max(elapsed_, max_);
    running_ = false;
    if (report_every_n_iter_ == 0) return;
    if (!(iteration_ % report_every_n_iter_)) Report();
  }

  /**
   * @brief Elapsed, last elapsed time duration
   */
  template <typename T = D>
  double Elapsed() const {
    return elapsed_.count() * Ratio<D, T>();
  }

  /**
   * @brief Min, shortest time duration recorded
   */
  template <typename T = D>
  double Min() const {
    return min_.count() * Ratio<D, T>();
  }

  /**
   * @brief Max, longest time duration recorded
   */
  template <typename T = D>
  double Max() const {
    return max_.count() * Ratio<D, T>();
  }

  /**
   * @brief Average, average time duration
   */
  template <typename T = D>
  double Average() const {
    return total_.count() * Ratio<D, T>() / iteration_;
  }

  /**
   * @brief Reset timer
   */
  void Reset() {
    iteration_ = 0;
    running_ = false;
  }

  template <typename T = D>
  void Sleep(int tick) {
    T duration(tick);
    std::this_thread::sleep_for(duration);
  }

  /**
   * @brief Report
   * @param unit_name A string representing the unit
   */
  template <typename T = D>
  void Report(std::ostream& os = std::cout) const {
    os << name_ << " - iterations: " << iteration_ << ", unit: " << Unit<T>()
       << ", average: " << Average<T>() << " "
       << ", min: " << Min<T>() << ", max: " << Max<T>() << std::endl;
  }

 private:
  std::string name_{"timer"};
  int iteration_{0};
  int report_every_n_iter_{0};
  bool running_{false};
  std::chrono::high_resolution_clock::time_point start_;
  D min_{D::max()};
  D max_{D::min()};
  D elapsed_{0};
  D total_{0};
};

typedef Timer<sec> TimerSec;
typedef Timer<ms> TimerMs;
typedef Timer<us> TimerUs;
typedef Timer<ns> TimerNs;

template <typename C>
void PrintClockData() {
  std::cout << "- precision: ";
  // if time unit is less or equal one millisecond
  typedef typename C::period P;  // type of time unit
  if (std::ratio_less_equal<P, std::milli>::value) {
    // convert to and print as milliseconds
    typedef typename std::ratio_multiply<P, std::kilo>::type TT;
    std::cout << std::fixed << static_cast<double>(TT::num) / TT::den << " ms"
              << std::endl;
  } else {
    // print as seconds
    std::cout << std::fixed << static_cast<double>(P::num) / P::den << " sec"
              << std::endl;
  }
  std::cout << "- is_steady: " << std::boolalpha << C::is_steady << std::endl;
}

}  // namespace kr

#endif  // KR_COMMON_TIMER_HPP_
