// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include "core/Exception.h"
#include "slave/GLDisplay.h"

using namespace dv::slave;


void GLDisplay::onKeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
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
  : image_(1, 1, CV_8UC3)
  , texture_(0)
  , window_(nullptr)
{
}


GLDisplay::~GLDisplay() {
  destroy();
}


void GLDisplay::run() {
  try {
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
    window_ = glfwCreateWindow(
        mode->width,
        mode->height,
        "OpenRoom",
        projector,
        nullptr
    );
    glfwSetWindowUserPointer(window_, this);
    glfwSetKeyCallback(window_, onKeyCallback);

    if (!window_) {
      throw EXCEPTION() << "Cannot open GLFW window.";
    }

    glfwMakeContextCurrent(window_);
    glViewport(0, 0, mode->width, mode->height);

    glEnable(GL_TEXTURE_2D);
    glGenTextures(1, &texture_);
    glBindTexture(GL_TEXTURE_2D, texture_);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

    loop();

    destroy();
  } catch (...) {
    destroy();
    throw;
  }
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

void GLDisplay::loop() {
  while (!glfwWindowShouldClose(window_)) {
    {
      std::lock_guard<std::mutex> locker(lock_);
      displayedImage_ = image_;
    }

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
            GL_RGB,
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
      glTexCoord2f(0.0f, 0.0f); glVertex2f(-1.0f, -1.0f);
      glTexCoord2f(0.0f, 1.0f); glVertex2f(-1.0f, +1.0f);
      glTexCoord2f(1.0f, 1.0f); glVertex2f(+1.0f, +1.0f);
      glTexCoord2f(1.0f, 0.0f); glVertex2f(+1.0f, -1.0f);
    glEnd();

    glfwSwapBuffers(window_);
    glfwPollEvents();
  }
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
