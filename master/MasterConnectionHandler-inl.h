// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

namespace dv { namespace master {


template<typename ...Args>
void MasterConnectionHandler::InvokeParallel(
    void (ProCamClient::* func) (Args...),
    Args... args)
{
  // Launch a thread per connection. They exit after the RPC
  // call is completed.
  std::vector<std::thread> threads;
  for (const auto &connection : connections_) {
    threads.emplace_back(
        [&] (std::shared_ptr<ProCamClient> client) {
          (client.get()->*func) (args...);
        },
        connection.second.client
    );
  }

  // Join all threads.
  for (auto &thread : threads) {
    thread.join();
  }
}


template<typename Ret, typename ...Args>
std::unordered_map<ConnectionID, Ret> MasterConnectionHandler::InvokeParallel(
    Ret (ProCamClient::* func) (Args...),
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
    it->second = (client.get()->*func) (args...);
  };

  // Launch all threads & create a vector to store results.
  std::vector<std::thread> threads;
  HashMap results;
  for (const auto &connection : connections_) {
    auto it = results.emplace(connection.first, Ret());
    if (!it.second) {
      throw EXCEPTION() << "Cannot create result object.";
    }
    threads.emplace_back(
        executor,
        it.first,
        connection.second.client
    );
  }

  // Wait for all the threads to execute. They will emplace their
  // return values in the result vector.
  for (auto &thread : threads) {
    thread.join();
  }
  return results;
}


template<typename Ret, typename ...Args>
std::unordered_map<ConnectionID, Ret> MasterConnectionHandler::InvokeParallel(
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
    threads.emplace_back(
        executor,
        it.first,
        connection.second.client
    );
  }

  // Wait for all the threads to execute. They will emplace their
  // return values in the result vector.
  for (auto &thread : threads) {
    thread.join();
  }
  return results;
}

template<typename ...Args>
void MasterConnectionHandler::InvokeOne(
    ConnectionID id,
    void (ProCamClient::* func) (Args...),
    Args... args)
{
  auto it = connections_.find(id);

  if (it != connections_.end()) {
    (it->second.client.get()->*func) (args...);
  } else {
    throw EXCEPTION() << "Connection with a specified ID was not found.";
  }
}

}}
