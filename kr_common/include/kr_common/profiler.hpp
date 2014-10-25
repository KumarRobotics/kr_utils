#ifndef KR_COMMON_PROFILER_HPP_
#define KR_COMMON_PROFILER_HPP_

#include <initializer_list>
#include <unordered_map>
#include <algorithm>
#include <stdexcept>

#include "kr_common/timer.hpp"

namespace kr {
namespace common {

template <typename D>
class Profiler {
  static_assert(is_duration<D>::value, "Not a valid duration type");

 public:
  Profiler() = default;
  Profiler(std::initializer_list<std::string> il) {
    std::for_each(il.begin(), il.end(),
                  [this](const std::string& name) { AddTimer(name); });
  }

  void AddTimer(const std::string& name) {
    timers_.insert(std::make_pair(name, Timer<D>(name)));
  }

  void StartTimer(const std::string& name) {
    auto timer = timers_.find(name);
    if (timer == timers_.end()) {
      throw std::runtime_error("StartTimer: Timer not found.");
    }
    timer->second.Start();
  }

  void StopTimer(const std::string& name) {
    auto timer = timers_.find(name);
    if (timer == timers_.end()) {
      throw std::runtime_error("StopTimer: Timer not found");
    }
    timer->second.Stop();
  }

  template <typename T = D>
  void ReportAll() const {
    for (const std::pair<std::string, Timer<D>>& p : timers_) {
      p.second.template Report<T>();
    }
  }

 private:
  std::unordered_map<std::string, Timer<D>> timers_;
};

}  // namespace common
}  // namespace kr

#endif  // KR_COMMON_PROFILER_HPP_
