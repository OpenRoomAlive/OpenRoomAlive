// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once

#include <mutex>

#ifdef __APPLE__
# include <OpenGL/gl.h>
#else
# include <GL/gl.h>
#endif
#include <GLFW/glfw3.h>

#include "slave/Display.h"


namespace dv { namespace slave {


/**
 * Class encapsulating all the OpenGL rendering logic.
 */
class GLDisplay : public Display {
 public:
  GLDisplay();
  ~GLDisplay();

  /**
   * Main display loop.
   */
  void run() override;

  /**
   * Stops the displauy.
   */
  void stop() override;

  /**
   * Sets the image displayed on the screen.
   * @param image [description]
   */
  void displayImage(const cv::Mat &image) override;

  /**
   * Retrieves the parameters of the display.
   */
  DisplayParams getParameters() override;

  /**
   * Retrieves the width.
   */
  virtual size_t getWidth() override;

  /**
   * Retrieves the height.
   */
  virtual size_t getHeight() override;

 private:
  /**
   * Open a new GLFW window for this display.
   */
  void openWindow();

  /**
   * GLFW cleanup.
   */
  void destroy();

  /**
   * Called when a key was pressed.
   */
  void onKeyPressed(int key);

  /**
   * Main rendering loop.
   */
  void loop();

 private:
  /// Lock protecting the data.
  std::mutex lock_;
  /// Image (gray code) to display. Initialized to a single pixel RGBA image.
  cv::Mat image_;
  /// Image being displayed (copy to make multithreading easiery).
  cv::Mat displayedImage_;
  /// OpenGL texture acting as an intermediary between OpenCV and OpenGL.
  GLuint texture_;
  /// GLFW window handle.
  GLFWwindow *window_;
  /// Display parameters.
  DisplayParams params_;

  // GLFW callback functions.
  static void onKeyCallback(GLFWwindow*, int, int, int, int);
};

}}
