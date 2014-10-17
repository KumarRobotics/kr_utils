#include "kr_common/timer.hpp"
#include <unistd.h>

int main(int agrc, char** argv) {
  kr::common::TimerUs timer("test", "us");
  timer.Start();
  usleep(800);
  timer.Stop();
  timer.Report<kr::ms>("ms");
}
