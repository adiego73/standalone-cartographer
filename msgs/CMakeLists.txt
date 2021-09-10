find_package(Protobuf REQUIRED)

protobuf_generate_cpp(PROTO_SRC PROTO_HEADER
        Lidar.proto
        LaserRange.proto
        Path.proto
        Pose.proto
        Robot.proto
        Velocity.proto
        Odometry.proto
        )

set(PROTOBUF_INCLUDE_DIRS ${PROTOBUF_INCLUDE_DIRS} PARENT_SCOPE)
set(PROTOBUF_LIBRARIES ${PROTOBUF_LIBRARIES} PARENT_SCOPE)
set(PROTO_SRC ${PROTO_SRC} PARENT_SCOPE)
set(PROTO_HEADER ${PROTO_HEADER} PARENT_SCOPE)

add_custom_target(protobuf_compilation DEPENDS ${PROTO_SRC} ${PROTO_HEADER})