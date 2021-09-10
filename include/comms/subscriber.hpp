#pragma once
#ifndef FLATLANDSIMULATOR_SUBSCRIBER_HPP
#define FLATLANDSIMULATOR_SUBSCRIBER_HPP

#include <condition_variable>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <zmq.hpp>

#include "comms/common.hpp"

namespace adiego73 {

using SubscriberCallback = std::function<void(const std::string& topic, const std::string& msg)>;

class Subscriber
{
private:
    bool bStart_{ false };

    std::mutex mutex_;
    std::condition_variable startCondition_;
    std::thread tThread_;

    zmq::context_t zCtx_;
    std::string sTopic_;
    std::string sName_;

    SubscriberCallback fCallback_{ nullptr };

    void worker();

public:
    explicit Subscriber(std::string name = "Subscriber");

    ~Subscriber();
    void notifyTermination();
    void subscribe(std::string topic);

    void registerCallback(SubscriberCallback callback);

    // remove move and copy constructors/assignment op.
    Subscriber(const Subscriber&) = delete;
    Subscriber(Subscriber&&) = delete;
    Subscriber& operator=(const Subscriber&) = delete;
    Subscriber& operator=(Subscriber&&) = delete;
};

}  // namespace adiego73

#endif
