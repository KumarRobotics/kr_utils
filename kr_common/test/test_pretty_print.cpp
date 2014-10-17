#include <iostream>
#include <vector>
#include "kr_common/pretty_print.hpp"
#include <map>

int main() {
  std::vector<int> foo;
  foo.push_back(1);
  foo.push_back(2);
  foo.push_back(3);
  std::cout << "My vector: " << foo << std::endl;

  typedef std::map<int, std::set<std::string>> map_type;
  map_type bar = {
      {1, std::set<std::string>{std::string("cat"), std::string("dog"),
                                std::string("doe")}},
      {3, std::set<std::string>{std::string("goose"), std::string("moose"),
                                std::string("ruse")}}};
  std::cout << bar << std::endl;

  int arr[] = {1, 4, 9, 16};
  std::cout << arr << std::endl;
}
