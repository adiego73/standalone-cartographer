#pragma once
#ifndef FLATLANDSIMULATOR_SERVER_HPP
#define FLATLANDSIMULATOR_SERVER_HPP

#include <thread>
#include <zmq.hpp>

namespace adiego73 {

using ServerCallback = std::function<void(const std::string& req, std::string& rep)>;

class Server
{
private:
    zmq::context_t zCtx_;
    std::string sAddress_;

    int iConnectedClients_{ 0 };
    bool bStart_{ true };
    std::thread pWorkerThread_;

    ServerCallback callback_;

    void workerCallback();

public:
    explicit Server(std::string address);
    ~Server();

    void registerCallback(ServerCallback cb);
    void shutdown();

    // remove move and copy constructors/assignment op.
    Server(const Server&) = delete;
    Server(Server&&) = delete;
    Server& operator=(const Server&) = delete;
    Server& operator=(Server&&) = delete;
};

}  // namespace adiego73
#endif  // FLATLANDSIMULATOR_SERVER_HPP
