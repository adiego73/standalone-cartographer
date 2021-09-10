#include "comms/server.hpp"

#include <iostream>

using namespace adiego73;

Server::Server(std::string address) : sAddress_(std::move(address)), pWorkerThread_(&Server::workerCallback, this)
{
    callback_ = [&address](const std::string& req, std::string& rep) {
        std::cerr << "Not implemented callback for Server on address " << address << std::endl;
    };
}

Server::~Server()
{
    if (bStart_) {
        shutdown();
    }

    if (pWorkerThread_.joinable()) {
        pWorkerThread_.join();
    }

    zCtx_.shutdown();
    zCtx_.close();
}

void
Server::workerCallback()
{
    zmq::socket_t socket(zCtx_, zmq::socket_type::rep);
    // recv timeout equals to 60 milliseconds
    socket.set(zmq::sockopt::rcvtimeo, 60);
    socket.set(zmq::sockopt::linger, 0);

    socket.bind(sAddress_);

    while (bStart_) {
        zmq::message_t req;
        auto recv = socket.recv(req, zmq::recv_flags::none);

        if (recv.has_value() && recv.value() >= 0) {
            iConnectedClients_++;
            std::string res_str;
            callback_(req.to_string(), res_str);

            zmq::message_t reply(res_str.c_str(), res_str.size());
            socket.send(reply, zmq::send_flags::none);
            iConnectedClients_--;
        }
    }

    socket.close();
}

void
Server::registerCallback(ServerCallback cb)
{
    callback_ = std::move(cb);
}

void
Server::shutdown()
{
    std::cout << "Closing server with address " << sAddress_ << " Pending clients: " << iConnectedClients_ << std::endl;
    while (iConnectedClients_ != 0)
        ;

    bStart_ = false;
};
