#include "kr_common/timer.hpp"
#include <unistd.h>

int main(int agrc, char** argv) {
  using namespace kr::common;

  std::cout << "system_clock: " << std::endl;
  PrintClockData<std::chrono::system_clock>();
  std::cout << "\nhigh_resolution_clock: " << std::endl;
  PrintClockData<std::chrono::high_resolution_clock>();
  std::cout << "\nsteady_clock: " << std::endl;
  PrintClockData<std::chrono::steady_clock>();

  Timer<kr::us> timer("test");
  timer.Start();
  usleep(8000);
  timer.Stop();
  timer.Report();
  timer.Start();
  timer.Sleep<kr::ms>(2);
  timer.Stop();
  timer.Report<kr::ms>();
}