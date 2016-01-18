// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once


namespace dv { namespace master {

class ConnectionHandler;
class PointCloud;
class ProCamSystem;

/**
 * Performs drawing of the enviroment.
 */
class EnviromentDrawer {
 public:
  EnviromentDrawer(
      const std::vector<ConnectionID> &ids,
      const std::shared_ptr<ProCamSystem> system,
      const boost::shared_ptr<ConnectionHandler> connectionHandler,
      const std::shared_ptr<PointCloud> pointCloud);
  ~EnviromentDrawer();

  /**
   * Draws carpet on the floor of the room.
   */
  void drawCarpet();

 private:
  /**
   * Computes the points in the floor plane.
   */
  void findFloor();

 private:
  /// ProCam ids.
  const std::vector<ConnectionID> ids_;
  /// ProCam system.
  const std::shared_ptr<ProCamSystem> system_;
  /// Connection handler.
  const boost::shared_ptr<ConnectionHandler> connectionHandler_;
  /// Point cloud.
  const std::shared_ptr<PointCloud> pointCloud_;
  /// Floor.
  Plane floor_;
};

}}

