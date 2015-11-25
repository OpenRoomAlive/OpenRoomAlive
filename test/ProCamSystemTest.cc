// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include <random>

#include <gtest/gtest.h>

#include "master/ProCamSystem.h"

using namespace dv;
using namespace dv::master;


float kColourCamMatrix[9] = {
  7, 0, 7,
  0, 7, 7,
  0, 0, 1
};
float kIrCamMatrix[9] = {
  9, 0, 9,
  0, 9, 9,
  0, 0, 1
};
float kDistCoeffs[5] = {
  1, 2, 3, 4, 5
};


/**
 * Tests loading the system from a json object.
 */
TEST(ProCamSystemTest, TestLoadFromJSON) {
  folly::dynamic data = folly::dynamic::object
      ( 0, folly::dynamic::object
        ( "actual-res", folly::dynamic::object
            ( "width", 1024 )
            ( "height", 1024 )
        )
        ( "effective-res", folly::dynamic::object
            ( "width", 512 )
            ( "height", 512 )
        )
      );

  // Create the system from json.
  auto system = std::make_shared<ProCamSystem>();
  system->fromJSON(data);


  // Assert on available properties of cameras.
  auto sys = std::static_pointer_cast<const ProCamSystem>(system);
  std::shared_ptr<const ProCam> cam;

  ASSERT_NO_THROW(cam = sys->getProCam(0));
  EXPECT_EQ(1024, cam->getActualResolution().width);
  EXPECT_EQ(1024, cam->getActualResolution().height);
}


/**
 * Tests writing the system to a json object.
 */
TEST(ProCamSystemTest, TestWriteToJSON) {
  auto sys = std::make_shared<ProCamSystem>();

  for (const auto &id : { 1, 2, 3 }) {
    sys->addProCam(
        id,
        { cv::Mat(3, 3, CV_32F, kColourCamMatrix), cv::Mat(0, 0, CV_32F) },
        { cv::Mat(3, 3, CV_32F, kIrCamMatrix), cv::Mat(1, 5, CV_32F) },
        cv::Size(2 * id, 3 * id),
        cv::Size(4 * id, 5 * id),
        std::chrono::milliseconds(6)
    );
  }

  auto json = sys->toJSON();
  ASSERT_EQ(3, json.size());

  auto cam1 = json[1];
  EXPECT_EQ( 2, cam1["actual-res"]["width"].asInt());
  EXPECT_EQ( 3, cam1["actual-res"]["height"].asInt());
  EXPECT_EQ( 4, cam1["effective-res"]["width"].asInt());
  EXPECT_EQ( 5, cam1["effective-res"]["height"].asInt());

  auto cam2 = json[2];
  EXPECT_EQ( 4, cam2["actual-res"]["width"].asInt());
  EXPECT_EQ( 6, cam2["actual-res"]["height"].asInt());
  EXPECT_EQ( 8, cam2["effective-res"]["width"].asInt());
  EXPECT_EQ(10, cam2["effective-res"]["height"].asInt());

  auto cam3 = json[3];
  EXPECT_EQ( 6, cam3["actual-res"]["width"].asInt());
  EXPECT_EQ( 9, cam3["actual-res"]["height"].asInt());
  EXPECT_EQ(12, cam3["effective-res"]["width"].asInt());
  EXPECT_EQ(15, cam3["effective-res"]["height"].asInt());
}
