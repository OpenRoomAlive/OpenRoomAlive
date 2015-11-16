// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once

#include "core/GL.h"
#include "core/Types.h"


namespace dv { namespace master {

class GLViewer {
 public:
  GLViewer();
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

 private:
  /**
   * Destroys GL resources.
   */
  void destroy();

  /// GLFW window handle.
  GLFWwindow *window_;
  /// Width of the window.
  int width_;
  /// Height of the window.
  int height_;

  /// GLFW callbacks.
  static void onKeyCallback(GLFWwindow*, int, int, int, int);
  static void onMouseClickCallback(GLFWwindow*, int, int, int);
  static void onMouseMoveCallback(GLFWwindow*, double, double);
};

}}
