// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once

#include <boost/filesystem.hpp>

namespace dv { namespace test {

/**
 * Test environment which provides access to mock test data.
 */
class Environment : public ::testing::Environment {
 public:
  Environment(const std::vector<const char*> &args);
  ~Environment();

  void SetUp() override;

  /**
   * Returns a path to a data file.
   */
  const boost::filesystem::path getDataFile(const std::string &name) const;

 private:
  /// List of command line arguments.
  const std::vector<const char*> &args_;
  /// Path to the directory containing our data.
  boost::filesystem::path dataDirectory_;
};

/**
 * Unique environment instance.
 */
extern const Environment *kTestEnv;

}}
