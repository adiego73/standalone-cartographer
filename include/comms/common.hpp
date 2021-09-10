#ifndef FLATLANDSIMULATOR_COMMON_HPP
#define FLATLANDSIMULATOR_COMMON_HPP

namespace adiego73::communication {

const std::string TOPIC_DELIMITER = "@@|@@";
constexpr int TOPIC_DELIMITER_LENGTH = 5;

const std::string PUBLISHER_ADDRESS = "tcp://127.0.0.1:3330";
const std::string SUBSCRIBER_ADDRESS = "tcp://127.0.0.1:3331";
const std::string DISCOVERY_SERVER = "tcp://127.0.0.1:6000";

}  // namespace adiego73::communication

#endif  // FLATLANDSIMULATOR_COMMON_HPP
