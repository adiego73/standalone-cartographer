#pragma once
#ifndef MAPPING_CARTOGRAPHER_MAPPING_HPP
#define MAPPING_CARTOGRAPHER_MAPPING_HPP

#include <Odometry.pb.h>
#include <cartographer/io/image.h>
#include <cartographer/mapping/2d/probability_grid_range_data_inserter_2d.h>
#include <cartographer/mapping/map_builder_interface.h>
#include <cartographer/mapping/trajectory_builder_interface.h>
#include <google/protobuf/timestamp.pb.h>

#include <comms/publisher.hpp>
#include <comms/subscriber.hpp>
#include <shared_mutex>

using namespace cartographer;
using namespace cartographer::mapping;

using SensorId = TrajectoryBuilderInterface::SensorId;
namespace adiego73 {

class CartographerMapping
{
public:
    CartographerMapping() = default;

    ~CartographerMapping() = default;

    void configure(const std::string &config_file_path, const std::string &filename);
    bool loadStoredState(const std::string &state_file_path);
    void run();
    void doFinalOptimization();
    void createMap(const std::string &filename);
    void storeCurrentState(bool include_submaps, const std::string &filename);
    void stop();

private:
    struct LocalSLAMResultData
    {
        int trajectory_id{ 0 };
        common::Time time;
        transform::Rigid3d local_pose;
        sensor::RangeData range_data_in_local;
    };

    std::mutex mtx_trajectory_builder_;
    std::shared_mutex mtx_odometry_;

    Subscriber sub_odom_{ "OdometrySubscriber" };
    Subscriber sub_lidar_{ "LidarSubscriber" };

    msgs::common::Odometry last_odometry_;

    std::vector<LocalSLAMResultData> local_slam_result_poses_;
    std::unique_ptr<common::LuaParameterDictionary> parameter_dictionary_;
    std::unique_ptr<ProbabilityGridRangeDataInserter2D> range_data_inserter_;
    std::unique_ptr<ProbabilityGrid> probability_grid_;
    std::unique_ptr<MapBuilderInterface> map_builder_;
    std::set<SensorId> sensors_ids_;

    int trajectory_id_{ 0 };
    float map_resolution_{ 0.05 };
    bool start_{ true };

    static common::Time FromProtoTimestamp(const google::protobuf::Timestamp &stamp);
    void OnOdometryMessage(const std::string &topic, const std::string &message, const std::string &sensor);
    void OnLidarMessage(const std::string &topic, const std::string &message, const std::string &sensor);
    void OnSLAMResultCallback(int trajectory_id, const common::Time &time, const transform::Rigid3d &local_pose,
                              sensor::RangeData range_data_in_local);

    void publishMapData();
    void publishOdometryData();

    std::unique_ptr<io::Image> createImage(Eigen::Array2i &offset);
    void writePGM(const std::string &filename, const std::unique_ptr<io::Image> &image) const;
    void writePNG(const std::string &filename, const std::unique_ptr<io::Image> &image) const;
    void writeYAML(const std::string &filename, const std::unique_ptr<io::Image> &image, const Eigen::Array2i &offset) const;
};

}  // namespace adiego73
#endif  // MAPPING_CARTOGRAPHER_MAPPING_HPP
