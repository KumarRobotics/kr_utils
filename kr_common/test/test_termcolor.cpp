#include "kr_common/termcolor.hpp"

int main() {
  std::cout << tc::blue << "blue" << std::endl;
  std::cout << tc::red << tc::on_blue << "red on blue" << tc::reset
            << std::endl;
  std::cout << tc::bold << tc::red << tc::on_blue << "bold red on blue" << tc::reset
            << std::endl;
}
