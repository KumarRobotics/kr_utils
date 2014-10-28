#include "kr_common/sample.hpp"
#include <iostream>

using namespace kr::common;

int main(int argc, char** argv) {
  std::cout << Sample::Uniform<int>(0, 10) << std::endl;
}
