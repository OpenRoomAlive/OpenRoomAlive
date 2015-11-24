// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include <mutex>

#include <gtest/gtest.h>

#include "test/Environment.h"

namespace dv { namespace test {

const Environment *kTestEnv = nullptr;


Environment::Environment(const std::vector<const char*> &args)
  : args_(args)
{
}

Environment::~Environment() {
}

void Environment::SetUp() {
  namespace fs = boost::filesystem;
  ASSERT_FALSE(args_.empty());

  for (const auto &dir : fs::absolute(fs::current_path(), args_[0])) {
    if (dir == "build") {
      dataDirectory_ /= "data";
      break;
    }
    dataDirectory_ /= dir;
  }

  ASSERT_TRUE(fs::is_directory(dataDirectory_));
}

const boost::filesystem::path Environment::getDataFile(
    const std::string &name) const
{
  return dataDirectory_ / boost::filesystem::path(name);
}

}}

/**
 * Entry point of the test suite.
 */
int main(int argc, char **argv) {
  const std::vector<const char*> args(argv, argv + argc);

  testing::InitGoogleTest(&argc, argv);

  dv::test::kTestEnv = static_cast<const dv::test::Environment*>(
      testing::AddGlobalTestEnvironment(new dv::test::Environment(args)));

  return RUN_ALL_TESTS();
}
