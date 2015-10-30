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
#include <unordered_map>
#include <vector>

#include <boost/enable_shared_from_this.hpp>

#include <opencv2/opencv.hpp>

#include <thrift/transport/TTransportUtils.h>

#include "core/Exception.h"
#include "core/Master.h"
#include "core/ProCam.h"
#include "master/MasterServer.h"

namespace dv { namespace master {

/*
 * All connections to procams are identified by ID equal to the ID of the
 * associated ProCam.
 */
using ConnectionID = uint64_t;

/**
 * Manages ProCam connections.
 */
class MasterConnectionHandler
  : public MasterIfFactory
  , public boost::enable_shared_from_this<MasterConnectionHandler>
{
 private:
  using TBufferedTransport = apache::thrift::transport::TBufferedTransport;
  using TConnectionInfo = apache::thrift::TConnectionInfo;

 public:
  MasterConnectionHandler(uint16_t proCamPort);
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
  std::vector<ConnectionID> waitForConnections(size_t count);

  /**
   * Sends signal to given ProCam to display specified gray code.
   */
  void displayGrayCode(
      ConnectionID id,
      Orientation::type orientation,
      int16_t level,
      bool invertedGrayCode);

  /**
   * Disconnects all procams.
   */
  void stop();

  /**
   * Invokes getCameraParams on all clients.
   */
  std::unordered_map<ConnectionID, CameraParams> getCamerasParams() {
    return InvokeParallel(&ProCamClient::getCameraParams);
  }

  /**
   * Invokes getDisplayParams on all clients.
   */
  std::unordered_map<ConnectionID, DisplayParams> getDisplaysParams() {
    return InvokeParallel(&ProCamClient::getDisplayParams);
  }

  /**
   * Invokes getColorImage on all clients.
   */
  std::unordered_map<ConnectionID, cv::Mat> getColorImages();

  /**
   * Invokes getDepthImage on all clients.
   */
  std::unordered_map<ConnectionID, cv::Mat> getDepthImages();

  /**
   * Invokes getUndistortedColorImage on all clients.
   */
  std::unordered_map<ConnectionID, cv::Mat> getUndistortedColorImages();

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
   * Invokes a client RCP method on all clients.
   *
   * The RPC calls are performed in parrallel, using one thread for each call.
   * This call only works if the return type is a primitive value.
   *
   * @tparam Ret    Return type of the RPC call.
   * @tparam Args   List of arguments to the RPC call.
   *
   * @param func    Pointer to the ProCam API method.
   * @param args... List of arguments.
   */
  template<typename Ret, typename ...Args>
  std::vector<Ret> InvokeParallel(
      Ret (ProCamClient::* func) (Args...),
      Args... args)
  {
    // Helper lambda that fulfills the promise by invoking the method
    // on the client with the given arguments. By-reference capture is
    // used in order to capture the variadic template args.
    auto executor = [&] (
        std::promise<Ret> promise,
        std::shared_ptr<ProCamClient> client)
    {
      promise.set_value((client.get()->*func) (args...));
    };

    // Launch all threads & create futures for all results.
    std::vector<std::future<Ret>> futures;
    for (const auto &connection : connections_) {
      std::promise<Ret> promise;
      futures.push_back(promise.get_future());

      // The thread is created detached - it must be joined before
      // the future is fulfilled.
      std::thread(
          executor,
          std::move(promise),
          connection.second.client
      ).detach();
    }

    // Return all results from the futures. If a future is not
    // ready to be retrieved, this thread will block until the
    // RPC call succeeds.
    std::vector<Ret> results;
    for (auto &future : futures) {
      results.push_back(future.get());
    }
    return results;
  }

  /**
   * Invokes a client RCP method on all clients.
   *
   * The RPC calls are performed in parrallel, using one thread for each call.
   * This call only works if the return type is a thrift structure.
   *
   * @tparam Ret    Return type of the RPC call.
   * @tparam Args   List of arguments to the RPC call.
   *
   * @param func    Pointer to the ProCam API method.
   * @param args... List of arguments.
   */
  template<typename Ret, typename ...Args>
  std::unordered_map<ConnectionID, Ret> InvokeParallel(
      void (ProCamClient::* func) (Ret&, Args...),
      Args... args)
  {
    using HashMap = std::unordered_map<ConnectionID, Ret>;

    // Helper lambda that executes the call and store the return
    // value in the ret argument. By-reference capture is
    // used in order to capture the variadic template args.
    auto executor = [&] (
        typename HashMap::iterator it,
        std::shared_ptr<ProCamClient> client)
    {
      (client.get()->*func) (it->second, args...);
    };

    // Launch all threads & create a vector to store results.
    std::vector<std::thread> threads;
    HashMap results;
    for (const auto &connection : connections_) {
      auto it = results.emplace(connection.first, Ret());
      if (!it.second) {
        throw EXCEPTION() << "Cannot create result object.";
      }
      threads.push_back(std::thread(
          executor,
          it.first,
          connection.second.client
      ));
    }

    // Wait for all the threads to execute. They will emplace their
    // return values in the result vector.
    for (auto &thread : threads) {
      thread.join();
    }
    return results;
  }

  /// List of connections to procams.
  std::unordered_map<ConnectionID, Connection> connections_;
  /// Mutex protecting the connections.
  std::mutex lock_;
  /// Condition variable to check on client count.
  std::condition_variable connectionCountCondition_;
  // Port on which every ProCam listens to requests from the master.
  const uint16_t proCamPort_;
  /// ID assigned to the next incomming connection to the master.
  std::atomic<ConnectionID> nextID_;
};

}}
