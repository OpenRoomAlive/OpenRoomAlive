// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include <random>

#include <folly/json.h>

#include <gtest/gtest.h>

#include "master/ProCamSystem.h"

using namespace dv;
using namespace dv::master;

using namespace std::literals;


namespace {

enum class MatType {
  IDENTITY,
  PATTERN,
  ZERO
};

folly::dynamic genJsonMat(int height, int width, int depth) {
  folly::dynamic mat = {};
  for (int i = 0; i < height; ++i) {
    for (int j = 0; j < width; ++j) {
      for (int k = 0; k < depth; ++k) {
        mat.push_back((1 + i * width * depth + j * depth + k) & 0xFF);
      }
    }
  }
  return mat;
}

void checkJsonMat(
    int height,
    int width,
    int depth,
    const folly::dynamic &arr,
    MatType type = MatType::PATTERN)
{
  ASSERT_TRUE(arr.isArray());
  ASSERT_EQ(width * height * depth, static_cast<int>(arr.size()));

  for (int i = 0, idx = 0; i < height; ++i) {
    for (int j = 0; j < width; ++j) {
      for (int k = 0; k < depth; ++k, ++idx) {
        auto elem = arr[i * width * depth + j * depth + k].asDouble();
        switch (type) {
          case MatType::ZERO: {
            EXPECT_EQ(0, elem);
            break;
          }
          case MatType::PATTERN: {
            EXPECT_EQ((1 + i * width * depth + j * depth + k) & 0xFF, elem);
            break;
          }
          case MatType::IDENTITY: {
            EXPECT_EQ(i == j ? 1 : 0, elem);
            break;
          }
        }
      }
    }
  }
}


template<typename T, size_t D>
cv::Mat genCVMat(int height, int width) {
  cv::Mat mat = cv::Mat::zeros(height, width, CVType<T, D>::type);
  for (int i = 0, idx = 0; i < height; ++i) {
    auto ptr = mat.ptr<T>(i);
    for (int j = 0; j < width; ++j) {
      for (size_t k = 0; k < D; ++k, ++idx) {
        ptr[j * D + k] = (1 + i * width * D + j * D + k) & 0xFF;
      }
    }
  }
  return mat;
}

template<typename T>
void checkCVMat(int height, int width, int depth, const cv::Mat &mat) {
  ASSERT_EQ(height, mat.rows);
  ASSERT_EQ(width, mat.cols);
  ASSERT_EQ(depth, mat.channels());

  for (int i = 0; i < height; ++i) {
    auto ptr = mat.ptr<T>(i);
    for (int j = 0; j < width; ++j) {
      for (int k = 0; k < depth; ++k) {
        EXPECT_EQ(
            (1 + i * width * depth + j * depth + k) & 0xFF,
            ptr[j * depth + k]
        );
      }
    }
  }
}

}


/**
 * Tests loading the system from a json object.
 */
TEST(ProCamSystemTest, TestLoadFromJSON) {
  folly::dynamic data = folly::dynamic::object
      ( "0", folly::dynamic::object
        ( "color-cam", folly::dynamic::object
          ( "proj", genJsonMat(3, 3, 1) )
          ( "dist", genJsonMat(0, 0, 1) )
        )
        ( "ir-cam", folly::dynamic::object
          ( "proj", genJsonMat(3, 3, 1) )
          ( "dist", genJsonMat(1, 5, 1) )
        )
        ( "actual-res", folly::dynamic::object
            ( "width", 1024 )
            ( "height", 1024 )
        )
        ( "effective-res", folly::dynamic::object
            ( "width", 512 )
            ( "height", 512 )
        )
        ( "latency", 100 )
        ( "projector", folly::dynamic::object
          ( "proj", genJsonMat(3, 3, 1) )
          ( "dist", genJsonMat(1, 5, 1) )
        )
        ( "baseline", folly::dynamic::object
          ( "color", genJsonMat(424, 512, 4) )
          ( "depth", genJsonMat(424, 512, 1 ) )
          ( "var", genJsonMat(424, 512, 1) )
        )
        ( "poses", folly::dynamic::object
          ( "1", folly::dynamic::object
            ( "rvec", genJsonMat(3, 1, 1) )
            ( "tvec", genJsonMat(3, 1, 1) )
          )
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

  EXPECT_EQ(512, cam->getEffectiveResolution().width);
  EXPECT_EQ(512, cam->getEffectiveResolution().height);

  EXPECT_EQ(100ms, cam->getLatency());

  auto color = cam->getColorCameraModel();
  checkCVMat<float>(3, 3, 1, color.calib);
  checkCVMat<float>(0, 0, 1, color.dist);

  auto ir = cam->getIrCameraModel();
  checkCVMat<float>(3, 3, 1, ir.calib);
  checkCVMat<float>(1, 5, 1, ir.dist);

  checkCVMat<double>(3, 3, 1, cam->getProjMatrix());
  checkCVMat<double>(1, 5, 1, cam->getProjDistortion());
  checkCVMat<float>(424, 512, 1, cam->getBaselineDepth());
  checkCVMat<float>(424, 512, 1, cam->getBaselineVariance());
  checkCVMat<uint8_t>(424, 512, 4, cam->getBaselineColor());

  auto pose = cam->getPose(1);
  checkCVMat<double>(3, 1, 1, pose.rvec);
  checkCVMat<double>(3, 1, 1, pose.tvec);
}


/**
 * Tests writing the system to a json object.
 */
TEST(ProCamSystemTest, TestWriteToJSON) {
  auto ids = { 1, 2 };
  auto sys = std::make_shared<ProCamSystem>();

  for (const auto &id : ids) {
    sys->addProCam(
        id,
        { genCVMat<double, 1>(3, 3), genCVMat<double, 1>(0, 0) },
        { genCVMat<double, 1>(3, 3), genCVMat<double, 1>(1, 5) },
        cv::Size(2 * id, 3 * id),
        cv::Size(4 * id, 5 * id),
        std::chrono::milliseconds(6 * id)
    );
  }

  auto json = sys->toJSON();
  ASSERT_NO_THROW(folly::toPrettyJson(json));
  ASSERT_EQ(2, json.size());

  for (const auto &id : ids) {
    auto cam = json[std::to_string(id)];
    EXPECT_EQ(2 * id, cam["actual-res"]["width"].asInt());
    EXPECT_EQ(3 * id, cam["actual-res"]["height"].asInt());
    EXPECT_EQ(4 * id, cam["effective-res"]["width"].asInt());
    EXPECT_EQ(5 * id, cam["effective-res"]["height"].asInt());
    EXPECT_EQ(6 * id, cam["latency"].asInt());
    checkJsonMat(3, 3, 1, cam["color-cam"]["proj"]);
    checkJsonMat(0, 0, 1, cam["color-cam"]["dist"]);
    checkJsonMat(3, 3, 1, cam["ir-cam"]["proj"]);
    checkJsonMat(1, 5, 1, cam["ir-cam"]["dist"]);
    checkJsonMat(3, 3, 1, cam["projector"]["proj"], MatType::IDENTITY);
    checkJsonMat(1, 5, 1, cam["projector"]["dist"], MatType::ZERO);
    checkJsonMat(0, 0, 1, cam["baseline"]["depth"], MatType::ZERO);
    checkJsonMat(0, 0, 1, cam["baseline"]["var"], MatType::ZERO);
    checkJsonMat(0, 0, 3, cam["baseline"]["color"], MatType::ZERO);
  }
}
