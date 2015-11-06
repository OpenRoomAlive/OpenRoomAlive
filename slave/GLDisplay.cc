// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include "core/Exception.h"
#include "slave/GLDisplay.h"

using namespace dv;
using namespace dv::slave;


void GLDisplay::onKeyCallback(
    GLFWwindow* window,
    int key,
    int scancode,
    int action,
    int mods)
{
  (void) scancode;
  (void) mods;

  // Check if the window has a pointer to our display class attached.
  GLDisplay *display = static_cast<GLDisplay*>(glfwGetWindowUserPointer(window));
  if (!display) {
    return;
  }

  // Demux events.
  switch (action) {
    case GLFW_PRESS: {
      display->onKeyPressed(key);
      return;
    }
    default: {
      return;
    }
  }
}


GLDisplay::GLDisplay()
try
  : image_(1, 1, CV_8UC3, cv::Scalar(0))
  , texture_(0)
  , window_(nullptr)
{
  if (!glfwInit()) {
    throw EXCEPTION() << "Cannot initialize GLFW.";
  }

  int monitorCount;
  GLFWmonitor **monitors = glfwGetMonitors(&monitorCount);
  if (!monitors || monitorCount < 1) {
    throw EXCEPTION() << "No monitors found.";
  }

  // Find the projector or a fallback monitor if no projector is conencted.
  GLFWmonitor *projector = monitors[monitorCount - 1];
  if (monitorCount == 1) {
    std::cerr << "Only primary monitor found." << std::endl;
  } else if (monitorCount > 2) {
    std::cerr << "Multiple secondary monitors were found." << std::endl;
  }

  const GLFWvidmode* mode = glfwGetVideoMode(projector);
  glfwWindowHint(GLFW_RED_BITS, mode->redBits);
  glfwWindowHint(GLFW_GREEN_BITS, mode->greenBits);
  glfwWindowHint(GLFW_BLUE_BITS, mode->blueBits);
  glfwWindowHint(GLFW_REFRESH_RATE, mode->refreshRate);

  window_ = glfwCreateWindow(
      mode->width,
      mode->height,
      "OpenRoom",
      projector,
      nullptr
  );

  if (!window_) {
    throw EXCEPTION() << "Cannot open GLFW window.";
  }

  glfwHideWindow(window_);
  glfwSetWindowUserPointer(window_, this);
  glfwSetKeyCallback(window_, onKeyCallback);
  glfwSetInputMode(window_, GLFW_CURSOR, GLFW_CURSOR_HIDDEN);
  glfwMakeContextCurrent(window_);

  // Fetch the window size.
  {
    int frameWidth = 0;
    int frameHeight = 0;
    glfwGetFramebufferSize(window_, &frameWidth, &frameHeight);
    if (frameWidth == 0 || frameHeight == 0) {
      throw EXCEPTION() << "Could not retrieve dimensions of the display.";
    }
    resolution_.width = frameWidth;
    resolution_.height = frameHeight;
    glViewport(0, 0, frameWidth, frameHeight);
  }

  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

  glEnable(GL_TEXTURE_2D);
  glGenTextures(1, &texture_);
  glBindTexture(GL_TEXTURE_2D, texture_);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
} catch (...) {
  destroy();
  throw;
}

GLDisplay::~GLDisplay() {
  destroy();
}

Resolution GLDisplay::getParameters() {
  return resolution_;
}

size_t GLDisplay::getWidth() {
  return resolution_.width;
}

size_t GLDisplay::getHeight() {
  return resolution_.height;
}

bool GLDisplay::isRunning() {
  return !glfwWindowShouldClose(window_);
}

void GLDisplay::update() {
  {
    std::lock_guard<std::mutex> locker(lock_);
    displayedImage_ = image_;
  }

  glfwShowWindow(window_);

  glBindTexture(GL_TEXTURE_2D, texture_);
  switch (displayedImage_.elemSize()) {
    case 1: {
      glTexImage2D(
          GL_TEXTURE_2D,
          0,
          GL_RGB,
          displayedImage_.cols,
          displayedImage_.rows,
          0,
          GL_LUMINANCE,
          GL_UNSIGNED_BYTE,
          displayedImage_.data
      );
      break;
    }
    case 3: {
      glTexImage2D(
          GL_TEXTURE_2D,
          0,
          GL_RGB,
          displayedImage_.cols,
          displayedImage_.rows,
          0,
          GL_BGR,
          GL_UNSIGNED_BYTE,
          displayedImage_.data
      );
      break;
    }
    case 4: {
      glTexImage2D(
          GL_TEXTURE_2D,
          0,
          GL_RGB,
          displayedImage_.cols,
          displayedImage_.rows,
          0,
          GL_BGRA,
          GL_UNSIGNED_BYTE,
          displayedImage_.data
      );
      break;
    }
    default: {
      throw EXCEPTION() << "Unsupported image type .";
    }
  }

  glClear(GL_COLOR_BUFFER_BIT);
  glBegin(GL_QUADS);
    glTexCoord2f(1.0f, 1.0f); glVertex2f(-1.0f, -1.0f);
    glTexCoord2f(1.0f, 0.0f); glVertex2f(-1.0f, +1.0f);
    glTexCoord2f(0.0f, 0.0f); glVertex2f(+1.0f, +1.0f);
    glTexCoord2f(0.0f, 1.0f); glVertex2f(+1.0f, -1.0f);
  glEnd();

  glfwSwapBuffers(window_);
  glfwPollEvents();
}

void GLDisplay::displayImage(const cv::Mat &image) {
  std::lock_guard<std::mutex> locker(lock_);
  image_ = image;
}

void GLDisplay::destroy() {
  if (texture_) {
    glDeleteTextures(1, &texture_);
    texture_ = 0;
  }
  if (window_) {
    glfwDestroyWindow(window_);
    window_ = nullptr;
  }
  glfwTerminate();
}

void GLDisplay::stop() {
  glfwSetWindowShouldClose(window_, true);
}

void GLDisplay::onKeyPressed(int key) {
  switch (key) {
    case GLFW_KEY_ESCAPE: {
      glfwSetWindowShouldClose(window_, true);
      return;
    }
    default: {
      return;
    }
  }
}
