#pragma once
#ifndef FLATLANDSIMULATOR_PUBLISHER_HPP
#define FLATLANDSIMULATOR_PUBLISHER_HPP

#include <iostream>
#include <memory>
#include <string>
#include <zmq.hpp>

#include "common.hpp"

namespace adiego73 {

class Publisher
{
private:
    zmq::context_t zCtx_;
    zmq::socket_t zSocket_;

public:
    explicit Publisher(const std::string& address = communication::PUBLISHER_ADDRESS);
    bool publish(const std::string& topic, const std::string& data, const std::string& separator = communication::TOPIC_DELIMITER);

    ~Publisher();

    // remove move and copy constructors/assignment op.
    Publisher(const Publisher&) = delete;
    Publisher(Publisher&&) = delete;
    Publisher& operator=(const Publisher&) = delete;
    Publisher& operator=(Publisher&&) = delete;
};

}  // namespace adiego73

#endif
