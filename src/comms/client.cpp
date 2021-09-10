#include "comms/client.hpp"

#include <iostream>
#include <utility>

using namespace adiego73;
using namespace std::chrono_literals;

Client::Client(const std::string& address, int retries, std::string name_) :
    sAddress_(address),
    iRetries_(retries),
    zSocket_(std::make_unique<zmq::socket_t>(zCtx_, zmq::socket_type::req)),
    name(std::move(name_))
{
    zSocket_->set(zmq::sockopt::linger, 0);
    zSocket_->connect(address);
}

void
Client::send(const std::string& req, const ClientCallback& callback)
{
    auto retries = iRetries_;
    while (retries > 0) {
        zmq::message_t request(req.c_str(), req.size());
        zSocket_->send(request, zmq::send_flags::none);

        bool wait_reply = true;
        while (wait_reply) {
            std::array<zmq::pollitem_t, 1> items{};
            items[0] = { *zSocket_, 0, ZMQ_POLLIN, 0 };

            zmq::poll(items, 5s);

            if (items[0].revents & ZMQ_POLLIN) {
                zmq::message_t res;
                auto recv = zSocket_->recv(res, zmq::recv_flags::none);

                if (recv.has_value() && recv.value() >= 0) {
                    callback(res.to_string());
                    wait_reply = false;
                    retries = 0;
                }
            } else if (retries == 0) {
                std::cerr << "Maximum amount of retries reached when connecting to server " << sAddress_ << std::endl;
                wait_reply = false;
                clearSocket();
            } else {
                std::cerr << "[CLIENT] " << name << " :: Reconnecting to " << sAddress_ << std::endl;
                clearSocket();
                // re send the request
                zSocket_->send(request, zmq::send_flags::none);
            }
            retries--;
        }
    }
}

void
Client::clearSocket()
{
    zSocket_->close();
    // build socket again.
    zSocket_ = std::make_unique<zmq::socket_t>(zCtx_, zmq::socket_type::req);
    zSocket_->set(zmq::sockopt::linger, 0);
    zSocket_->connect(sAddress_);
}

Client::~Client()
{
    zSocket_->close();
    zCtx_.shutdown();
    zCtx_.close();
}
