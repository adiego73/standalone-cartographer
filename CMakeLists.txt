cmake_minimum_required(VERSION 3.16)
project(standalone_cartographer)

set(CMAKE_CXX_STANDARD 17)
set(BUILD_SHARED_LIBS ON)

find_package(Boost REQUIRED iostreams system serialization filesystem thread)
find_package(ZeroMQ REQUIRED)
find_package(Protobuf REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(cartographer REQUIRED)

add_subdirectory(msgs)

add_definitions(${EIGEN3_DEFINITIONS})

set_source_files_properties(${PROTO_SRC} ${PROTO_HEADER} PROPERTIES GENERATED TRUE)

include_directories(include
        external
        build/msgs
        ${EIGEN3_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        )

#############################
## CARTOGRAPHER TARGET

add_executable(cartographer_mapping app/map_with_cartographer_main.cpp
        ${PROTO_SRC}
        ${PROTO_HEADER}
        src/cartographer_mapping.cpp
        include/cartographer_mapping.hpp

        include/comms/client.hpp
        include/comms/publisher.hpp
        include/comms/common.hpp
        include/comms/server.hpp
        include/comms/subscriber.hpp
        src/comms/client.cpp
        src/comms/publisher.cpp
        src/comms/server.cpp
        src/comms/subscriber.cpp

        external/json/json.hpp
        external/pgma/pgma_io.hpp
        external/pgma/pgma_io.cpp
        )

add_dependencies(cartographer_mapping protobuf_compilation)

target_link_libraries(cartographer_mapping PUBLIC
        protobuf::libprotobuf
        ${ZeroMQ_LIBRARY}
        ${Boost_LIBRARIES}
        Eigen3::Eigen
        yaml-cpp
        cartographer
        )

add_custom_command(TARGET cartographer_mapping POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/config/ $<TARGET_FILE_DIR:cartographer_mapping>/config)