#include "comms/subscriber.hpp"

#include <iostream>
#include <system_error>
#include <utility>

#include "comms/common.hpp"

using namespace adiego73;

Subscriber::Subscriber(std::string name) : sName_(std::move(name)), tThread_(&Subscriber::worker, this) {}

void
Subscriber::subscribe(std::string topic)
{
    if (!bStart_) {
        bStart_ = true;
        sTopic_ = std::move(topic);
        startCondition_.notify_all();
    }
}

void
Subscriber::notifyTermination()
{
    bStart_ = false;
}

void
Subscriber::worker()
{
    std::unique_lock lock(mutex_);
    startCondition_.wait(lock, [&]() {
        return bStart_;
    });

    zmq::message_t msg;
    zmq::socket_t socket(zCtx_, zmq::socket_type::sub);
    // setup socket
    try {
        // set linger value to zero
        socket.set(zmq::sockopt::linger, 0);
        socket.set(zmq::sockopt::subscribe, sTopic_);

        // 60ms timeout. This is needed so to let subscriber
        // to finish when no message is received for an specific topic.
        socket.set(zmq::sockopt::rcvtimeo, 60);

        // connect
        socket.connect(communication::SUBSCRIBER_ADDRESS);
    } catch (std::exception& e) {
        std::cerr << "Exception connecting to " << communication::SUBSCRIBER_ADDRESS << " on subscriber {" << sName_
                  << "}, \"" << e.what() << "\"" << std::endl;
        return;
    }

    std::string msg_str;
    std::string full_topic;
    while (bStart_) {
        try {
            auto recv = socket.recv(msg, zmq::recv_flags::none);

            if (fCallback_ && recv.has_value() && recv.value() > 0) {
                msg_str = msg.to_string();
                auto pos = msg_str.find(communication::TOPIC_DELIMITER);
                full_topic = msg_str.substr(0, pos);
                msg_str.erase(0, pos + communication::TOPIC_DELIMITER_LENGTH);

                fCallback_(full_topic, msg_str);
            }
        } catch (const zmq::error_t& ex) {
            if (ex.num() != ETERM) {
                std::cerr << "{" << sName_ << "} Subscriber::process(): zmq exception \"" << ex.what() << "\"" << std::endl;
            }
            continue;
        } catch (std::exception& e) {
            std::cerr << "{" << sName_ << "} Subscriber::process(): exception \"" << e.what() << "\"" << std::endl;
            break;
        }
    }
    socket.close();
}

void
Subscriber::registerCallback(SubscriberCallback callback)
{
    fCallback_ = std::move(callback);
}

Subscriber::~Subscriber()
{
    notifyTermination();

    if (tThread_.joinable()) {
        tThread_.join();
    }

    zCtx_.shutdown();
    zCtx_.close();
}
