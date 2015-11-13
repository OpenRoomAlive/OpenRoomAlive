// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once

#include <atomic>
#include <condition_variable>
#include <future>
#include <memory>
#include <mutex>
#include <thread>

#include <boost/enable_shared_from_this.hpp>

#include <opencv2/opencv.hpp>

#include <thrift/transport/TTransportUtils.h>

#include "core/Conv.h"
#include "core/Exception.h"
#include "core/Master.h"
#include "core/ProCam.h"
#include "master/ConnectionHandler.h"
#include "master/EventStream.h"
#include "master/MasterServer.h"

namespace dv { namespace master {

/**
 * Manages ProCam connections.
 */
class MasterConnectionHandler
  : public ConnectionHandler
  , public MasterIfFactory
  , public boost::enable_shared_from_this<MasterConnectionHandler>
{
 private:
  using TBufferedTransport = apache::thrift::transport::TBufferedTransport;
  using TConnectionInfo = apache::thrift::TConnectionInfo;

 public:
  MasterConnectionHandler(
      uint16_t proCamPort,
      const std::shared_ptr<EventStream>& stream);
  ~MasterConnectionHandler();

  /**
   * Retrieves the handler and sets up the inverse connection.
   */
  MasterServer* getHandler(const TConnectionInfo& connInfo);

  /**
   * Free the handler.
   */
  void releaseHandler(MasterIf* handler);

  /**
   * Blocks until a specific number of procams connect.
   * Returns the IDs of the ProCams that connected.
   */
  std::vector<ConnectionID> waitForConnections(size_t count) override;

  /**
   * Sends signal to given ProCam to display specified gray code.
   */
  void displayGrayCode(
      ConnectionID id,
      Orientation::type orientation,
      int16_t level,
      bool invertedGrayCode) override {
    InvokeOne<Orientation::type, int16_t, bool>(
        id,
        &ProCamClient::displayGrayCode,
        orientation,
        level,
        invertedGrayCode);
  }

  /**
   * Sends signal to given ProCam to display white image.
   */
  void displayWhite(ConnectionID id) override {
    InvokeOne(id, &ProCamClient::displayWhite);
  }

  /**
   * Clears the display of a procam.
   */
  void clearDisplay(ConnectionID id) override {
    InvokeOne(id, &ProCamClient::clearDisplay);
  }

  /**
   * Disconnects all procams.
   */
  void stop() override;

  /**
   * Clears the displays of all procams.
   */
  void clearDisplays() override {
    return InvokeAll(&ProCamClient::clearDisplay);
  }

  /**
   * Invokes getCameraParams on all clients.
   */
  std::unordered_map<ConnectionID, CameraParams> getCamerasParams() override {
    return InvokeAll(&ProCamClient::getCameraParams);
  }

  /**
   * Invokes getDisplayParams on all clients.
   */
  std::unordered_map<ConnectionID, DisplayParams> getDisplaysParams() override {
    return InvokeAll(&ProCamClient::getDisplayParams);
  }

  /**
   * Invokes getColorImage on all clients.
   */
  FrameMap getColorImages() override {
    return conv::thriftFrameToCvMatMap(
        InvokeAll(&ProCamClient::getColorImage));
  }

  /**
   * Retrieves the depth image of a procam.
   */
  cv::Mat getDepthImage(ConnectionID id) override {
    cv::Mat image;
    conv::thriftFrameToCvMat(
        InvokeOne(id, &ProCamClient::getDepthImage), image);
    return image;
  }

  /**
   * Invokes getDepthImage on all clients.
   */
  FrameMap getDepthImages() override {
    return conv::thriftFrameToCvMatMap(
        InvokeAll(&ProCamClient::getDepthImage));
  }

  /**
   * Invokes getUndistortedColorImage on all clients.
   */
  FrameMap getUndistortedColorImages() override
  {
    return conv::thriftFrameToCvMatMap(
        InvokeAll(&ProCamClient::getUndistortedColorImage));
  }

  /**
   * Invokes getColorBaseline on all clients.
   */
  FrameMap getColorBaselines() override {
    return conv::thriftFrameToCvMatMap(
        InvokeAll(&ProCamClient::getColorBaseline));
  }

  /**
   * Invokes getDepthBaseline on all clients.
   */
  FrameMap getDepthBaselines() override {
    return conv::thriftFrameToCvMatMap(
        InvokeAll(&ProCamClient::getDepthBaseline));
  }

 private:
  /**
   * Information about a connection.
   */
  struct Connection {
    /// Connection to the procam unit.
    const boost::shared_ptr<TBufferedTransport> transport;
    /// Procam client.
    const std::shared_ptr<ProCamClient> client;

    /**
     * Constructor for emplace_back.
     */
    Connection(
        const boost::shared_ptr<TBufferedTransport> &_transport,
        const std::shared_ptr<ProCamClient> &_client)
      : transport(_transport)
      , client(_client)
    {
    }
  };

  /**
   * Invokes a client RPC method on some clients.
   *
   * This RPC call is specialized for methods without any return values.
   *
   * @tparam Args   List of arguments to the RPC call.
   * @param ids     List of all connection IDs to call.
   * @param func    Pointer to the ProCam API method.
   * @param args... List of arguments.
   */
  template<typename ...Args>
  void InvokeGroup(
      const std::vector<ConnectionID>& ids,
      void (ProCamClient::* func) (Args...),
      Args... args);

  /**
   * Invokes a client RCP method on some clients.
   *
   * The RPC calls are performed in parrallel, using one thread for each call.
   * This call only works if the return type is a primitive value.
   *
   * @tparam Ret    Return type of the RPC call.
   * @tparam Args   List of arguments to the RPC call.
   * @param ids     List of all connection IDs to call.
   * @param func    Pointer to the ProCam API method.
   * @param args... List of arguments.
   */
  template<typename Ret, typename ...Args>
  std::unordered_map<ConnectionID, Ret> InvokeGroup(
      const std::vector<ConnectionID>& ids,
      Ret (ProCamClient::* func) (Args...),
      Args... args);

  /**
   * Invokes a client RCP method on some clients.
   *
   * The RPC calls are performed in parrallel, using one thread for each call.
   * This call only works if the return type is a thrift structure.
   *
   * @tparam Ret    Return type of the RPC call.
   * @tparam Args   List of arguments to the RPC call.
   * @param ids     List of all connection IDs to call.
   * @param func    Pointer to the ProCam API method.
   * @param args... List of arguments.
   */
  template<typename Ret, typename ...Args>
  std::unordered_map<ConnectionID, Ret> InvokeGroup(
      const std::vector<ConnectionID>& ids,
      void (ProCamClient::* func) (Ret&, Args...),
      Args... args);


  /**
   * Invokes a client RPC method on all clients.
   */
  template<typename ...Args>
  void InvokeAll(
      void (ProCamClient::* func) (Args...),
      Args... args) {
    return InvokeGroup(getAllIDs(), func, args...);
  }

  /**
   * Invokes a client RCP method on all clients.
   */
  template<typename Ret, typename ...Args>
  std::unordered_map<ConnectionID, Ret> InvokeAll(
      Ret (ProCamClient::* func) (Args...),
      Args... args) {
    return InvokeGroup(getAllIDs(), func, args...);
  }

  /**
   * Invokes a client RCP method on all clients.
   */
  template<typename Ret, typename ...Args>
  std::unordered_map<ConnectionID, Ret> InvokeAll(
      void (ProCamClient::* func) (Ret&, Args...),
      Args... args) {
    return InvokeGroup(getAllIDs(), func, args...);
  }

  /**
   * Invokes a method on a single ProCam if the given connection ID is found.
   *
   * @tparam Args   List of arguments to the function call.
   * @param id      ConnectionID of the target ProCam unit.
   * @param func    Pointer to the ProCamClient function.
   * @param args... List of arguments.
   */
  template<typename ...Args>
  void InvokeOne(
      ConnectionID id,
      void (ProCamClient::* func) (Args...),
      Args... args);

  /**
   * Invokes a method on a single ProCam if the given connection ID is found.
   *
   * @tparam Args   List of arguments to the function call.
   * @param id      ConnectionID of the target ProCam unit.
   * @param func    Pointer to the ProCamClient function.
   * @param args... List of arguments.
   */
  template<typename Ret, typename ...Args>
  Ret InvokeOne(
      ConnectionID id,
      Ret (ProCamClient::* func) (Args...),
      Args... args);


  /**
   * Invokes a method on a single ProCam if the given connection ID is found.
   *
   * @tparam Args   List of arguments to the function call.
   * @param id      ConnectionID of the target ProCam unit.
   * @param func    Pointer to the ProCamClient function.
   * @param args... List of arguments.
   */
  template<typename Ret, typename ...Args>
  Ret InvokeOne(
      ConnectionID id,
      void (ProCamClient::* func) (Ret&, Args...),
      Args... args);

  /**
   * Returns all connection IDs.
   */
  std::vector<ConnectionID> getAllIDs();

  /**
   * Returns a vector of all connections associated to IDs.
   *
   * @throws dv::Exception if any of the IDs does not have a connection.
   */
  std::vector<std::pair<ConnectionID, Connection>> getConnections(
      const std::vector<ConnectionID> &ids);

 private:
  /// List of connections to procams.
  std::unordered_map<ConnectionID, Connection> connections_;
  /// Mutex protecting the connections.
  std::mutex lock_;
  /// Condition variable to check on client count.
  std::condition_variable connectionCountCondition_;
  /// ID assigned to the next incomming connection to the master.
  std::atomic<ConnectionID> nextID_;
  /// Port on which every ProCam listens to requests from the master.
  const uint16_t proCamPort_;
  /// Stream of events sent by ProCams for processing.
  const std::shared_ptr<EventStream> stream_;
};

}}

#include "MasterConnectionHandler-inl.h"

