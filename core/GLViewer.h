// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once

#include <opencv2/opencv.hpp>

#include "core/GL.h"
#include "core/Types.h"


namespace dv { namespace core {


class GLViewer {
 public:
  /**
   * Creates a GL window.
   */
  GLViewer();

  /**
   * Convenience constructor that takes a lambda performing draw calls.
   */
  explicit GLViewer(std::function<void()> func);

  /**
   * Cleans up th GL window.
   */
  ~GLViewer();

  /**
   * Checks if the viewer is still running.
   */
  bool isRunning();

  /**
   * Starts rendering a frame.
   */
  void frame();

  /**
   * Renders a plane.
   */
  void drawPlane(const Plane &plane);

  /**
   * Draws an array of points.
   */
  void drawPoints(
      const std::vector<cv::Point3f> &worldPoints,
      const std::vector<cv::Point2f> &projPoints,
      const cv::Size &size);

  /**
   * Draws another array of points.
   */
  void drawPoints(
      const std::vector<std::pair<cv::Point3f, cv::Point3f>> &points,
      const cv::Point3f &centroid);

  /**
   * Draws a camera frustum.
   */
  void drawCamera(
      const cv::Mat &cam,
      const cv::Mat &r,
      const cv::Mat &t);

 private:
  /**
   * Destroys GL resources.
   */
  void destroy();

  /**
   * Returns a vector for arcball rotation.
   */
  glm::vec3 getArcballVector(const glm::ivec2 &pos) const;

  /// GLFW window handle.
  GLFWwindow *window_;
  /// Size of the framebuffer.
  glm::ivec2 fbSize_;
  /// Size of the window.
  glm::ivec2 wndSize_;

  /// Last mouse position.
  glm::ivec2 lastMouse_;
  /// View matrix.
  glm::mat4 view_;
  /// Inverse view matrix.
  glm::mat4 invView_;
  /// Projection matrix.
  glm::mat4 proj_;
  /// Inverse projection matrix.
  glm::mat4 invProj_;
  /// Model matrix.
  glm::mat4 model_;
  /// True when rotating.
  bool isRotating_;

  /// Callbacks.
  void onLeftMouseDown();
  void onLeftMouseUp();
  void onMouseMove(double, double);

  /// GLFW callbacks.
  static void onKeyCallback(GLFWwindow*, int, int, int, int);
  static void onMouseClickCallback(GLFWwindow*, int, int, int);
  static void onMouseMoveCallback(GLFWwindow*, double, double);
};

}}
