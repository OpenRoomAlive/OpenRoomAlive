// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once

#include <opencv2/opencv.hpp>

#include "core/ProCam.h"


/**
 * Contains functions for converting values across types.
 */
namespace dv { namespace conv {


/**
 * Converts a cv::Mat to a thrift frame.
 */
void cvMatToThriftFrame(const cv::Mat& image, Frame& frame);

/**
 * Converts a thrift frame to a cv::Mat.
 */
void thriftFrameToCvMat(const Frame& frame, cv::Mat& image);

}}

