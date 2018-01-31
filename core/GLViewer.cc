// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include <functional>

#include "core/Exception.h"
#include "core/GLViewer.h"

using namespace dv;
using namespace dv::core;


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
  : window_(nullptr)
  , wndSize_({640, 480})
  , view_(glm::lookAt(
        glm::vec3{10.0f},
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
    glfwTerminate();
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
  glEnable(GL_DEPTH_TEST);
}

GLViewer::GLViewer(std::function<void()> func)
  : GLViewer()
{
  while (isRunning()) {
    func();
    frame();
  }
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
    glVertex3f(-kGridSize,        0.0f, 0.0f);
    glVertex3f(+kGridSize + 1.0f, 0.0f, 0.0f);
    // Green for Y.
    glColor3f(0.0f, 1.0f, 0.0f);
    glVertex3f(0.0f, -kGridSize,        0.0f);
    glVertex3f(0.0f, +kGridSize + 1.0f, 0.0f);
    // Blue for Z.
    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3f(0.0f, 0.0f, -kGridSize       );
    glVertex3f(0.0f, 0.0f, +kGridSize + 1.0f);
  glEnd();
}


void GLViewer::drawPlane(const Plane &plane) {
  (void) plane;
}


void GLViewer::drawPoints(
    const std::vector<cv::Point3f> &worldPoints,
    const std::vector<cv::Point2f> &projPoints,
    const cv::Size &size)
{
  if (worldPoints.size() != projPoints.size()) {
    throw EXCEPTION() << "Vector lengths are not equal.";
  }

  glPointSize(3.0f);
  glBegin(GL_POINTS);
  glColor3f(1.0f, 0.0f, 0.0f);
  for (size_t i = 0; i < worldPoints.size(); ++i) {
    glColor3f(projPoints[i].x / size.width, projPoints[i].y / size.height, 0);
    glVertex3f(worldPoints[i].x, worldPoints[i].y, worldPoints[i].z);
  }
  glEnd();
}


void GLViewer::drawPoints(
      const std::vector<Vertex> &points,
      const cv::Point3f &centroid)
{
  glPushMatrix();

  glColor3f(1.0f, 1.0f, 1.0f);
  glPointSize(3.0f);
  glTranslatef(-centroid.x, centroid.y, -centroid.z);

  glBegin(GL_POINTS);
  for (const auto &point : points) {
    const auto &position = point.position;
    const auto &color = point.color;

    glColor3f(color.x, color.y, color.z);
    glVertex3f(position.x, -position.y, position.z);
  }
  glEnd();

  glPopMatrix();
}


void GLViewer::drawCamera(
    const cv::Mat &cam,
    const cv::Mat &r,
    const cv::Mat &t)
{
  // Extract camera parameters.
  float fx = cam.at<float>(0, 0);
  float fy = cam.at<float>(1, 1);
  float cx = cam.at<float>(0, 2);
  float cy = cam.at<float>(1, 2);
  float n  = 0.1f;
  float f  = 4.0f;

  // Compute the intrinsic matrix.
  glm::mat4 mk(
      fx / cx,    0.0f,                 0.0f,  0.0f,
         0.0f, fy / cy,                 0.0f,  0.0f,
         0.0f,    0.0f,   -(f + n) / (f - n), -1.0f,
         0.0f,    0.0f, -2 * f * n / (f - n),  0.0f
  );

  // Compute the rotation & translation matrix.
  cv::Mat rm;
  cv::Rodrigues(r, rm);
  rm.convertTo(rm, CV_32F);
  glm::mat4 mr(
    rm.at<float>(0, 0), rm.at<float>(1, 0), rm.at<float>(2, 0), 0.0f,
    rm.at<float>(0, 1), rm.at<float>(1, 1), rm.at<float>(2, 1), 0.0f,
    rm.at<float>(0, 2), rm.at<float>(1, 2), rm.at<float>(2, 2), 0.0f,
    t.at<float>(0, 0), t.at<float>(1, 0), t.at<float>(2, 0), 1.0f
  );

  // Compute the inverse MV matrix.
  glm::mat4 inv = glm::inverse(mk * mr);

  glm::vec4 corners[] = {
      {-1, -1, -1, 1},
      {-1, -1, +1, 1},
      {-1, +1, -1, 1},
      {-1, +1, +1, 1},
      {+1, -1, -1, 1},
      {+1, -1, +1, 1},
      {+1, +1, -1, 1},
      {+1, +1, +1, 1}
  };
  for (auto &corner : corners) {
    corner = inv * corner;
    corner.x /= corner.w;
    corner.y /= corner.w;
    corner.z /= corner.w;
    corner.w = 1.0f;
  }

  {
    glColor3f(1.0f, 1.0f, 1.0f);
    glBegin(GL_LINES);
      glVertex3fv(glm::value_ptr(corners[0b000]));
      glVertex3fv(glm::value_ptr(corners[0b001]));
      glVertex3fv(glm::value_ptr(corners[0b010]));
      glVertex3fv(glm::value_ptr(corners[0b011]));
      glVertex3fv(glm::value_ptr(corners[0b100]));
      glVertex3fv(glm::value_ptr(corners[0b101]));
      glVertex3fv(glm::value_ptr(corners[0b110]));
      glVertex3fv(glm::value_ptr(corners[0b111]));
    glEnd();
    glBegin(GL_LINE_LOOP);
      glVertex3fv(glm::value_ptr(corners[0b000]));
      glVertex3fv(glm::value_ptr(corners[0b010]));
      glVertex3fv(glm::value_ptr(corners[0b110]));
      glVertex3fv(glm::value_ptr(corners[0b100]));
    glEnd();
    glBegin(GL_LINE_LOOP);
      glVertex3fv(glm::value_ptr(corners[0b001]));
      glVertex3fv(glm::value_ptr(corners[0b011]));
      glVertex3fv(glm::value_ptr(corners[0b111]));
      glVertex3fv(glm::value_ptr(corners[0b101]));
    glEnd();
  }
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
