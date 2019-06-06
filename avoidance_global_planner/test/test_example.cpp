#include <gtest/gtest.h>

#include <string>
using std::string;

class StringTest : public ::testing::Test {
 public:
  string actualString;
  string wrongString;
  string expectString;

  char *actualValTrue;
  char *actualValFalse;
  char *expectVal;

  // Use this method to set up any state that you need for all of your tests
  void SetUp() override {
    actualString = "hello gtest";
    wrongString = "hello world";
    expectString = "hello gtest";

    actualValTrue = new char[actualString.size() + 1];
    strncpy(actualValTrue, actualString.c_str(), actualString.size() + 1);

    actualValFalse = new char[wrongString.size() + 1];
    strncpy(actualValFalse, wrongString.c_str(), wrongString.size() + 1);

    expectVal = new char[expectString.size() + 1];
    strncpy(expectVal, expectString.c_str(), expectString.size() + 1);
  }

  // Use this method to clean up any memory, network etc. after each test
  void TearDown() override {
    delete[] actualValTrue;
    delete[] actualValFalse;
    delete[] expectVal;
  }
};

// Example code we are testing:
namespace myNormalCode {

void reverseInPlace(string &toReverse) {
  // NB! this only works for ASCII
  for (int i = 0, j = toReverse.size() - 1; i < j; i++, j--) {
    char tmp = toReverse[i];
    toReverse[i] = toReverse[j];
    toReverse[j] = tmp;
  }
}
}

TEST_F(StringTest, StrEqual) {
  // GIVEN: two strings that are the same

  // THEN: we expect them to be equal
  EXPECT_STREQ(actualString.c_str(), expectString.c_str());
}

TEST_F(StringTest, CStrNotEqual) {
  // GIVEN: two char* that are NOT the same

  // THEN: we expect them to be not equal
  EXPECT_STRNE(expectVal, actualValFalse);
}

TEST_F(StringTest, testReverse) {
  // GIVEN: a string, and a manually reversed string as well
  string toReverse = actualString;
  const string expectedReversed = "tsetg olleh";

  // WHEN: we reverse the string
  myNormalCode::reverseInPlace(toReverse);

  // THEN: they should be the same
  EXPECT_STREQ(expectedReversed.c_str(), toReverse.c_str());
}
