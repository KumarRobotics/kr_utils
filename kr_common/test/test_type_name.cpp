#include "kr_common/type_name.hpp"
#include <gtest/gtest.h>

class Dummy {
};

TEST(TypeNameTest, TestClass) {
  EXPECT_EQ(kr::type_name<Dummy>(), "Dummy");
}

TEST(TypeNameTest, TestConstClass) {
  EXPECT_EQ(kr::type_name<const Dummy>(), "Dummy const");
}

TEST(TypeNameTest, TestClassRef) {
  EXPECT_EQ(kr::type_name<Dummy&>(), "Dummy&");
}

TEST(TypeNameTest, TestConstClassRef) {
  EXPECT_EQ(kr::type_name<const Dummy&>(), "Dummy const&");
}
