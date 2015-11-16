// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include "core/Exception.h"
#include "master/GLViewer.h"

using namespace dv;
using namespace dv::master;


void GLViewer::onKeyCallback(
    GLFWwindow *window,
    int key,
    int scancode,
    int action,
    int mods)
{
  (void) key;
  (void) scancode;
  (void) action;
  (void) mods;

  // Check if the window has a pointer to our display class attached.
  auto viewer = static_cast<GLViewer*>(glfwGetWindowUserPointer(window));
  if (!viewer) {
    return;
  }
}


void GLViewer::onMouseClickCallback(
    GLFWwindow *window,
    int button,
    int action,
    int mods)
{
  (void) button;
  (void) action;
  (void) mods;

  // Check if the window has a pointer to our display class attached.
  auto viewer = static_cast<GLViewer*>(glfwGetWindowUserPointer(window));
  if (!viewer) {
    return;
  }
}



void GLViewer::onMouseMoveCallback(
    GLFWwindow *window,
    double x,
    double y)
{
  (void) x;
  (void) y;

  // Check if the window has a pointer to our display class attached.
  auto viewer = static_cast<GLViewer*>(glfwGetWindowUserPointer(window));
  if (!viewer) {
    return;
  }
}


GLViewer::GLViewer()
try
  : window_(nullptr)
  , width_(640)
  , height_(480)
{
  if (!glfwInit()) {
    throw EXCEPTION() << "Cannot initialize GLFW.";
  }

  window_ = glfwCreateWindow(width_, height_, "Simple example", NULL, NULL);
  if (!window_) {
    throw EXCEPTION() << "Cannot create GLFW window.";
  }

  // Register event handlers.
  glfwSetKeyCallback(window_, onKeyCallback);
  glfwSetCursorPosCallback(window_, onMouseMoveCallback);
  glfwSetMouseButtonCallback(window_, onMouseClickCallback);

  glfwMakeContextCurrent(window_);
  glfwSwapInterval(1);
} catch(...) {
  destroy();
  throw;
}


GLViewer::~GLViewer() {
}


void GLViewer::destroy() {
  if (window_) {
    glfwDestroyWindow(window_);
    window_ = nullptr;
  }
  glfwTerminate();
}


bool GLViewer::isRunning() {
  return window_ != nullptr && !glfwWindowShouldClose(window_);
}


void GLViewer::frame() {
  glfwGetFramebufferSize(window_, &width_, &height_);
  glfwSwapBuffers(window_);
  glfwPollEvents();

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(45.0f, static_cast<float>(width_) / height_, 0.1f, 100.0f);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}


void GLViewer::drawPlane(const Plane &plane) {
  (void) plane;
}


