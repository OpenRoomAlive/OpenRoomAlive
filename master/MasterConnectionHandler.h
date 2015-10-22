// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once

#include <condition_variable>
#include <future>
#include <memory>
#include <mutex>
#include <thread>

#include <boost/enable_shared_from_this.hpp>

#include <thrift/transport/TTransportUtils.h>

#include "core/Master.h"
#include "core/ProCam.h"
#include "master/MasterServer.h"

namespace dv { namespace master {


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
   */
  void waitForConnections(size_t count);

  /**
   * Disconnects all procams.
   */
  void stop();

  /**
   * Invokes test on all clients.
   */
  std::vector<int32_t> test(int32_t a, int32_t b) {
    return InvokeParallel(&ProCamClient::test, a, b);
  }

 private:
  /**
   * Information about a connection.
   */
  struct Connection {
    /// Connection to the procam unit.
    boost::shared_ptr<TBufferedTransport> transport;
    /// Procam client.
    std::shared_ptr<ProCamClient> client;

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
      promise.set_value_at_thread_exit((client.get()->*func) (args...));
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
          connection.client
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

  /// List of connections to procams.
  std::vector<Connection> connections_;
  /// Mutex protecting the connections.
  std::mutex lock_;
  /// Condition variable to check on client count.
  std::condition_variable connectionCountCondition_;
  // Port on which every ProCam listens to requests from the master.
  const uint16_t proCamPort_;
};

}}
