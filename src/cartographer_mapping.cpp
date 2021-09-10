#include "cartographer_mapping.hpp"

#include <Lidar.pb.h>
#include <Odometry.pb.h>
#include <cartographer/common/configuration_file_resolver.h>
#include <cartographer/io/probability_grid_points_processor.h>
#include <cartographer/io/proto_stream.h>
#include <cartographer/mapping/map_builder.h>
#include <yaml-cpp/yaml.h>

#include <boost/filesystem.hpp>
#include <json/json.hpp>
#include <utility>

using namespace adiego73;

void
CartographerMapping::configure(const std::string& config_file_path, const std::string& filename)
{
    // load parameters
    auto file_resolver = std::make_unique<common::ConfigurationFileResolver>(std::vector<std::string>{ config_file_path });
    const std::string code = file_resolver->GetFileContentOrDie(filename);
    parameter_dictionary_ = std::make_unique<common::LuaParameterDictionary>(code, std::move(file_resolver));

    // range data inserter
    auto range_data_inserter_options = mapping::CreateProbabilityGridRangeDataInserterOptions2D(
    parameter_dictionary_->GetDictionary("range_data_inserter").get());
    range_data_inserter_ = std::make_unique<ProbabilityGridRangeDataInserter2D>(range_data_inserter_options);

    // map resolution
    map_resolution_ = parameter_dictionary_->GetDouble("map_resolution");

    // probability grid
    mapping::ValueConversionTables conversion_tables;
    probability_grid_ = std::make_unique<ProbabilityGrid>(io::CreateProbabilityGrid(map_resolution_, &conversion_tables));

    // map builder
    auto opts_map_builder = mapping::CreateMapBuilderOptions(parameter_dictionary_->GetDictionary("map_builder").get());
    map_builder_ = mapping::CreateMapBuilder(opts_map_builder);

    // sensors
    // TODO: parametrize this, subscribers depend on this too.
    SensorId odom_sensor;
    odom_sensor.type = TrajectoryBuilderInterface::SensorId::SensorType::ODOMETRY;
    odom_sensor.id = "odom";
    sensors_ids_.insert(odom_sensor);

    SensorId lidar_sensor;
    lidar_sensor.type = TrajectoryBuilderInterface::SensorId::SensorType::RANGE;
    lidar_sensor.id = "lidar";
    sensors_ids_.insert(lidar_sensor);

    // trajectory builder
    auto opts_trajectory =
    mapping::CreateTrajectoryBuilderOptions(parameter_dictionary_->GetDictionary("trajectory_builder").get());
    trajectory_id_ = map_builder_->AddTrajectoryBuilder(
    sensors_ids_, opts_trajectory,
    [this](const int trajectory_id, const common::Time& time, const transform::Rigid3d& local_pose,
           sensor::RangeData range_data_in_local, const std::unique_ptr<const TrajectoryBuilderInterface::InsertionResult>) {
        this->OnSLAMResultCallback(trajectory_id, time, local_pose, std::move(range_data_in_local));
    });

    // subscribers callback
    sub_odom_.registerCallback([this, odom_sensor](const std::string& topic, const std::string& message) {
        this->OnOdometryMessage(topic, message, odom_sensor.id);
    });

    sub_lidar_.registerCallback([this, lidar_sensor](const std::string& topic, const std::string& message) {
        this->OnLidarMessage(topic, message, lidar_sensor.id);
    });
}

void
CartographerMapping::OnOdometryMessage(const std::string& topic, const std::string& message, const std::string& sensor)
{
    if (!start_)
        return;

    msgs::common::Odometry odom;

    if (odom.ParseFromString(message)) {
        {
            // store odometry because it is used later on for the lidar transform
            std::unique_lock lock(mtx_odometry_);
            last_odometry_.CopyFrom(odom);
        }
        auto trajectory_builder = map_builder_->GetTrajectoryBuilder(trajectory_id_);

        proto::OdometryData protoOdom;
        protoOdom.mutable_odometry_data()->mutable_pose()->mutable_translation()->set_x(odom.pose().x());
        protoOdom.mutable_odometry_data()->mutable_pose()->mutable_translation()->set_y(odom.pose().y());
        protoOdom.mutable_odometry_data()->mutable_pose()->mutable_translation()->set_z(0.0);

        auto quaternion = transform::AngleAxisVectorToRotationQuaternion<float>({ 0.0f, 0.0f, odom.pose().orientation() });
        protoOdom.mutable_odometry_data()->mutable_pose()->mutable_rotation()->set_x(quaternion.x());
        protoOdom.mutable_odometry_data()->mutable_pose()->mutable_rotation()->set_y(quaternion.y());
        protoOdom.mutable_odometry_data()->mutable_pose()->mutable_rotation()->set_z(quaternion.z());
        protoOdom.mutable_odometry_data()->mutable_pose()->mutable_rotation()->set_w(quaternion.w());

        sensor::OdometryData data{ FromProtoTimestamp(odom.stamp()), transform::ToRigid3(protoOdom.odometry_data().pose()) };

        {
            // add the odometry information to the trajectory builder
            std::lock_guard lock(mtx_trajectory_builder_);
            trajectory_builder->AddSensorData(sensor, data);
        }
    }
}

void
CartographerMapping::OnLidarMessage(const std::string& topic, const std::string& message, const std::string& sensor)
{
    if (!start_)
        return;

    msgs::sensor::Lidar lidar;

    if (lidar.ParseFromString(message)) {
        msgs::common::Pose last_pose;
        {
            // get the last robot pose
            std::shared_lock lock(mtx_odometry_);
            last_pose.CopyFrom(last_odometry_.pose());
        }

        auto trajectory_builder = map_builder_->GetTrajectoryBuilder(trajectory_id_);
        // this ranges variable is used for the trajectory builder
        sensor::TimedPointCloud ranges;
        // while points batch is used to build the grid map
        auto points_batch = std::make_unique<io::PointsBatch>();
        points_batch->origin << last_pose.x(), last_pose.y(), 0;

        for (const auto& point : lidar.cloud()) {
            sensor::TimedRangefinderPoint p;
            // Lidar information contains the distance and the angle of the ray, so we have to transform this information,
            // which is in the lidar reference frame, into the map reference frame. To do this, we translate the point in the
            // local frame to the odometry/map frame.
            p.position.x() = last_pose.x() + cosf(point.angle()) * point.distance();
            p.position.y() = last_pose.y() + sinf(point.angle()) * point.distance();
            p.position.z() = 0.0f;

            // time is set to 0 because there is no time information for this simulated lidar.
            // if the information is available, time refers to the timestamp the laser rage was taken.
            p.time = 0.0f;

            points_batch->points.push_back({ p.position });
            ranges.push_back(std::move(p));
        }

        // create the point cloud which will be used later by the trajectory builder
        sensor::TimedPointCloudData point_cloud{ FromProtoTimestamp(lidar.stamp()),
                                                 { last_pose.x(), last_pose.y(), 0 },
                                                 ranges };

        {
            // add point cloud to the trajectory builder
            std::lock_guard lock(mtx_trajectory_builder_);
            trajectory_builder->AddSensorData(sensor, point_cloud);
        }
        // insert the lidar data to the grid map.
        range_data_inserter_->Insert({ points_batch->origin, sensor::PointCloud(points_batch->points), {} },
                                     probability_grid_.get());
    }
}

common::Time
CartographerMapping::FromProtoTimestamp(const google::protobuf::Timestamp& stamp)
{
    // https://github.com/cartographer-project/cartographer_ros/blob/master/cartographer_ros/cartographer_ros/time_conversion.cc#L37
    return common::FromUniversal((stamp.seconds() + common::kUtsEpochOffsetFromUnixEpochInSeconds) * 10000000ll +
                                 (stamp.nanos() + 50) / 100);
}

void
CartographerMapping::OnSLAMResultCallback(int trajectory_id, const common::Time& time, const transform::Rigid3d& local_pose,
                                          sensor::RangeData range_data_in_local)
{
    if (!start_)
        return;

    local_slam_result_poses_.push_back({ trajectory_id, time, local_pose, std::move(range_data_in_local) });
}

void
CartographerMapping::run()
{
    sub_odom_.subscribe("odom");
    sub_lidar_.subscribe("lidar");

    while (start_) {
        // sleep for half a second
        std::this_thread::sleep_for(
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<float>(5)));
    }

    sub_lidar_.notifyTermination();
    sub_odom_.notifyTermination();

    map_builder_->FinishTrajectory(trajectory_id_);
}

void
CartographerMapping::doFinalOptimization()
{
//    map_builder_->FinishTrajectory(trajectory_id_);
    map_builder_->pose_graph()->RunFinalOptimization();
}

std::unique_ptr<io::Image>
CartographerMapping::createImage(Eigen::Array2i& offset)
{
    std::unique_ptr<io::Image> image = io::DrawProbabilityGrid(*probability_grid_, &offset);
    if (image != nullptr) {
        image->Rotate90DegreesClockwise();
    }
    return image;
}

void
CartographerMapping::createMap(const std::string& filename)
{
    Eigen::Array2i offset;
    auto image = createImage(offset);
    if (image != nullptr) {
        writePGM(filename, image);
        writePNG(filename, image);
        writeYAML(filename, image, offset);
    }
}

void
CartographerMapping::writePGM(const std::string& filename, const std::unique_ptr<io::Image>& image) const
{
    io::StreamFileWriter pgm_writer(filename + ".pgm");
    const std::string header = "P5\n# Cartographer map; " + std::to_string(map_resolution_) + " m/pixel\n" +
                               std::to_string(image->width()) + " " + std::to_string(image->height()) + "\n255\n";
    pgm_writer.Write(header.data(), header.size());
    for (int y = 0; y < image->height(); ++y) {
        for (int x = 0; x < image->width(); ++x) {
            const char color = image->GetPixel(x, y)[0];
            pgm_writer.Write(&color, 1);
        }
    }
    pgm_writer.Close();
}

void
CartographerMapping::writePNG(const std::string& filename, const std::unique_ptr<io::Image>& image) const
{
    io::FileWriterFactory file_writer_factory = [](const std::string& name) {
        std::string file_path = "./";
        return std::make_unique<io::StreamFileWriter>(file_path + name);
    };
    image->WritePng(file_writer_factory(filename + ".png").get());
}

void
CartographerMapping::writeYAML(const std::string& filename, const std::unique_ptr<io::Image>& image,
                               const Eigen::Array2i& offset) const
{
    std::ofstream yaml_File;
    yaml_File.open(filename + ".yaml");

    YAML::Node yaml_node;
    yaml_node["image"] = filename + ".pgm";
    yaml_node["resolution"] = map_resolution_;
    yaml_node["origin"].push_back(offset.x());
    yaml_node["origin"].push_back(offset.y());
    yaml_node["origin"].push_back(0.0);
    yaml_node["negate"] = false;
    yaml_node["occupied_thresh"] = 0.65;
    yaml_node["free_thresh"] = 0.196;

    yaml_File << YAML::Dump(yaml_node);
    yaml_File.close();
}
void
CartographerMapping::storeCurrentState(bool include_submaps, const std::string& filename)
{
    map_builder_->SerializeStateToFile(include_submaps, filename + ".pbstream");
}

void
CartographerMapping::stop()
{
    start_ = false;
}

bool
CartographerMapping::loadStoredState(const std::string& state_file_path)
{
    if (boost::filesystem::exists(state_file_path)) {
        io::ProtoStreamReader stream(state_file_path);
        map_builder_->LoadState(&stream, true);
        return true;
    }
    std::cerr << "\n*********** ERROR *********** \n"
              << "There is no file " << state_file_path  //
              << "\n*****************************" << std::endl;
    return false;
}
