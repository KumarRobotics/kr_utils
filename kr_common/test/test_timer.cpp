#include "kr_common/timer.hpp"
#include <unistd.h>

int main(int agrc, char** argv) {
  kr::common::Timer<kr::microseconds> timer("test", "micro");
  timer.Start();
  usleep(800);
  timer.Stop();
  timer.Report<kr::milliseconds>("ms");
}
