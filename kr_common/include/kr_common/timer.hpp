#ifndef KR_COMMON_TIMER_HPP_
#define KR_COMMON_TIEMR_HPP_

#include <chrono>
#include <string>
#include <cassert>
#include <ostream>
#include <iomanip>
#include <iostream>

namespace kr {

using std::chrono::microseconds;
using std::chrono::milliseconds;
using std::chrono::nanoseconds;

namespace common {

template <typename T, typename U>
double Ratio() {
  typedef typename T::period TP;
  typedef typename U::period UP;
  typedef typename std::ratio_divide<TP, UP>::type RP;
  return static_cast<double>(RP::num) / RP::den;
}

template <typename D>
class Timer {
 public:
  explicit Timer(const std::string& name, const std::string& unit,
                 int report_every_n_iter = 0)
      : name_(name), unit_(unit), report_every_n_iter_(report_every_n_iter) {}

  Timer(const Timer&) = delete;
  Timer& operator=(const Timer&) = delete;

  int iteration() const { return iteration_; }
  const std::string& name() const { return name_; }

  /**
   * @brief Start
   */
  void Start() {
    assert(!running_);
    running_ = true;
    start_ = std::chrono::high_resolution_clock::now();
  }

  /**
   * @brief Stop
   */
  void Stop() {
    assert(running_);
    elapsed_ = std::chrono::duration_cast<D>(
        std::chrono::high_resolution_clock::now() - start_);
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
   * @brief Min, shortest time duration
   */
  template <typename T = D>
  double Min() const {
    return min_.count() * Ratio<D, T>();
  }

  /**
   * @brief Max, longest time duration
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
   * @brief Reset
   */
  void Reset() {
    iteration_ = 0;
    running_ = false;
  }

  /**
   * @brief Report
   * @param unit_name A string representing the unit
   */
  template <typename T = D>
  void Report(const std::string& unit_name = "") const {
    std::cout << name_ << " - iterations: " << iteration_
              << ", unit: " << (unit_name.empty() ? unit_ : unit_name)
              << ", average: " << Average<T>() << " "
              << ", min: " << Min<T>() << ", max: " << Max<T>() << std::endl;
  }

 private:
  std::string name_{"timer"};
  std::string unit_{""};
  int iteration_{0};
  int report_every_n_iter_{0};
  bool running_{false};
  std::chrono::high_resolution_clock::time_point start_;
  D min_{D::max()};
  D max_{D::min()};
  D elapsed_{0};
  D total_{0};
};

template <typename C>
void PrintClockData() {
  std::cout << "- precision: ";
  // if time unit is less or equal one millisecond
  typedef typename C::period P;  // type of time unit
  if (std::ratio_less_equal<P, std::milli>::value) {
    // convert to and print as milliseconds
    typedef typename std::ratio_multiply<P, std::kilo>::type TT;
    std::cout << std::fixed << static_cast<double>(TT::num) / TT::den
              << " milliseconds" << std::endl;
  } else {
    // print as seconds
    std::cout << std::fixed << static_cast<double>(P::num) / P::den
              << " seconds" << std::endl;
  }
  std::cout << "- is_steady: " << std::boolalpha << C::is_steady << std::endl;
}

}  // namespace common
}  // namespace kr

#endif  // KR_COMMON_TIMER_HPP_
