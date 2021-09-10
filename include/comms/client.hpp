#pragma once
#ifndef FLATLANDSIMULATOR_CLIENT_HPP
#define FLATLANDSIMULATOR_CLIENT_HPP

#include <zmq.hpp>

namespace adiego73 {

using ClientCallback = std::function<void(const std::string& rep)>;

class Client
{
private:
    zmq::context_t zCtx_;
    std::unique_ptr<zmq::socket_t> zSocket_{ nullptr };
    std::string sAddress_;
    int iRetries_;
    std::string name;

    void clearSocket();

public:
    explicit Client(const std::string& address, int retries = 1, std::string name_ = "Client");
    void send(const std::string& req, const ClientCallback& callback);

    ~Client();
    // remove move and copy constructors/assignment op.
    Client(const Client&) = delete;
    Client(Client&&) = delete;
    Client& operator=(const Client&) = delete;
    Client& operator=(Client&&) = delete;
};
}  // namespace adiego73
#endif  // FLATLANDSIMULATOR_CLIENT_HPP
