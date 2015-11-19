// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include "core/Exception.h"
#include "master/GLViewer.h"

using namespace dv;
using namespace dv::master;


constexpr float kGridSize = 2.0f;


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
  (void) mods;

  auto viewer = static_cast<GLViewer*>(glfwGetWindowUserPointer(window));
  if (!viewer) {
    return;
  }

  switch (button) {
    case GLFW_MOUSE_BUTTON_1: {
      switch (action) {
        case GLFW_PRESS: viewer->onLeftMouseDown(); break;
        case GLFW_RELEASE: viewer->onLeftMouseUp(); break;
      }
      break;
    }
  }
}


void GLViewer::onMouseMoveCallback(
    GLFWwindow *window,
    double x,
    double y)
{
  auto viewer = static_cast<GLViewer*>(glfwGetWindowUserPointer(window));
  if (!viewer) {
    return;
  }
  viewer->onMouseMove(x, y);
}


GLViewer::GLViewer()
try
  : window_(nullptr)
  , wndSize_({640, 480})
  , view_(glm::lookAt(
        glm::vec3{5.0f},
        glm::vec3{0.0f},
        glm::vec3{0.0f, 1.0f, 0.0f}))
  , invView_(glm::inverse(view_))
  , isRotating_(false)
{
  if (!glfwInit()) {
    throw EXCEPTION() << "Cannot initialize GLFW.";
  }

  window_ = glfwCreateWindow(
      wndSize_.x,
      wndSize_.y,
      "Viewer",
      nullptr,
      nullptr);
  if (!window_) {
    throw EXCEPTION() << "Cannot create GLFW window.";
  }

  // Register event handlers.
  glfwSetWindowUserPointer(window_, this);
  glfwSetKeyCallback(window_, onKeyCallback);
  glfwSetCursorPosCallback(window_, onMouseMoveCallback);
  glfwSetMouseButtonCallback(window_, onMouseClickCallback);

  // GLFW setup.
  glfwMakeContextCurrent(window_);
  glfwSwapInterval(1);

  // OpenGL setup.
  glClear(GL_COLOR_BUFFER_BIT);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
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
  // Draw the ground plane.
  glColor4f(1.0f, 1.0f, 1.0f, 0.5f);
  glBegin(GL_QUADS);
    glVertex3f(-kGridSize, 0.0f, -kGridSize);
    glVertex3f(+kGridSize, 0.0f, -kGridSize);
    glVertex3f(+kGridSize, 0.0f, +kGridSize);
    glVertex3f(-kGridSize, 0.0f, +kGridSize);
  glEnd();

  // Process window events.
  glfwSwapBuffers(window_);
  glfwPollEvents();
  glfwGetFramebufferSize(window_, &fbSize_.x, &fbSize_.y);
  glfwGetWindowSize(window_, &wndSize_.x, &wndSize_.y);

  // Update matrices.
  {
    const float aspect = static_cast<float>(fbSize_.x) / fbSize_.y;
    proj_ = glm::perspective(45.0f, aspect, 0.1f, 100.0f);
    invProj_ = glm::inverse(proj_);
  }

  // Clear the framebuffer.
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // Set the view & proj matrices in OpenGL.
  glMatrixMode(GL_PROJECTION);
  glLoadMatrixf(glm::value_ptr(proj_));
  glMatrixMode(GL_MODELVIEW);
  glLoadMatrixf(glm::value_ptr(view_));
  glMultMatrixf(glm::value_ptr(model_));
  glScalef(5.0f, 5.0f, 5.0f);

  // Draw the grid along x, y and z.
  glLineWidth(2.0f);
  glBegin(GL_LINES);
  glColor3f(0.6f, 0.6f, 0.6f);
  for (float i = -kGridSize; i <= kGridSize; i += 0.1f) {
    glVertex3f(-kGridSize,  0.0f,          i);
    glVertex3f(+kGridSize,  0.0f,          i);
    glVertex3f(         i,  0.0f, -kGridSize);
    glVertex3f(         i,  0.0f, +kGridSize);
  }
  glEnd();

  // Draw the principal axis.
  glLineWidth(3.0f);
  glBegin(GL_LINES);
    // Red for X.
    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3f(-kGridSize, 0.0f, 0.0f);
    glVertex3f(+kGridSize, 0.0f, 0.0f);
    // Green for Y.
    glColor3f(0.0f, 1.0f, 0.0f);
    glVertex3f(0.0f,       0.0f, 0.0f);
    glVertex3f(0.0f, +kGridSize, 0.0f);
    // Blue for Z.
    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3f(0.0f, 0.0f, -kGridSize);
    glVertex3f(0.0f, 0.0f, +kGridSize);
  glEnd();
}


void GLViewer::drawPlane(const Plane &plane) {
  (void) plane;
}

void GLViewer::drawPoints(const std::vector<cv::Point3f> &points) {
  glPointSize(3.0f);
  glBegin(GL_POINTS);
  glColor3f(1.0f, 0.0f, 0.0f);
  for (const auto &point : points) {
    glVertex3f(point.x, point.y, point.z);
  }
  glEnd();
}


glm::vec3 GLViewer::getArcballVector(const glm::ivec2 &pos) const {
  const float px = 2.0f * static_cast<float>(pos.x) / wndSize_.x - 1.0f;
  const float py = 2.0f * static_cast<float>(pos.y) / wndSize_.y - 1.0f;
  const float l2 = px * px + py * py;

  if (l2 < 1.0f) {
    return { -px, py, std::sqrt(1.0f - l2) };
  } else {
    const float l = std::sqrt(l2);
    return { -px / l, py / l, 0.0f };
  }
}


void GLViewer::onLeftMouseDown() {
  isRotating_ = true;
}


void GLViewer::onLeftMouseUp() {
  isRotating_ = false;
}


void GLViewer::onMouseMove(double x, double y) {
  if (!isRotating_ || (x == lastMouse_.x && y == lastMouse_.y)) {
    lastMouse_ = {x, y};
    return;
  }

  // Find the two arcball vectors.
  const auto arcA = getArcballVector({x, y});
  const auto arcB = getArcballVector(lastMouse_);
  lastMouse_ = {x, y};

  // Compute the rotation quaternion between the two arcball vectors.
  const auto angle = std::acos(std::min(1.0f, glm::dot(arcA, arcB)));
  const auto axis = glm::vec3(invView_ * glm::vec4(glm::cross(arcB, arcA), 0));
  const auto rot = glm::angleAxis(glm::degrees(-angle * 0.5f), axis);

  // Concatenate rotations.
  model_ = glm::toMat4(glm::normalize(rot)) * model_;
}
