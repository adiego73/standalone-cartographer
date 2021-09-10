#include "comms/publisher.hpp"

#include "comms/common.hpp"

using namespace adiego73;

Publisher::Publisher(const std::string& address) : zSocket_(zCtx_, zmq::socket_type::pub)
{
    zSocket_.set(zmq::sockopt::linger, 0);
    zSocket_.bind(address);
}

bool
Publisher::publish(const std::string& topic, const std::string& data, const std::string& separator)
{
    std::string message = topic + separator + data;
    zmq::message_t m(message.c_str(), message.size());
    auto res = zSocket_.send(m, zmq::send_flags::none);

    if (res.has_value() && res.value() >= 0) {
        return true;
    }

    std::cerr << "Publisher::publish(), no value " << std::endl;
    return false;
}
Publisher::~Publisher()
{
    zSocket_.close();
    zCtx_.shutdown();
    zCtx_.close();
}
