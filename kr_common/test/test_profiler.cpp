#include "kr_common/profiler.hpp"
#include <unistd.h>

using namespace kr::common;

int main(int argc, char** argv) {
  Profiler<kr::ms> profiler = {"test1", "test2"};

  profiler.AddTimer("test3");
  profiler.StartTimer("test1");
  usleep(100000);
  profiler.StopTimer("test1");

  profiler.StartTimer("test2");
  usleep(100000);
  profiler.StopTimer("test2");

  profiler.StartTimer("test3");
  usleep(100000);
  profiler.StopTimer("test3");

  profiler.ReportAll();
}
