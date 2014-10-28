#include "kr_common/sample.hpp"
#include <iostream>

using namespace kr;

int main(int argc, char** argv) {
  Sample::Randomize();
  for (int i = 0; i < 10; ++i) {
    std::cout << Sample::Uniform(0, 10) << " ";
  }
  std::cout << std::endl;

  for (int i = 0; i < 10; ++i) {
    std::cout << Sample::Uniform(0.0, 10.0) << " ";
  }
  std::cout << std::endl;

  for (int i = 0; i < 10; ++i) {
    std::cout << Sample::Gaussian(0.0, 1.0) << " ";
  }
  std::cout << std::endl;
}
