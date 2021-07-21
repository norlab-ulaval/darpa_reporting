#include "NodeParameters.h"

NodeParameters::NodeParameters(ros::NodeHandle privateNodeHandle)
{
    retrieveParameters(privateNodeHandle);
    validateParameters();
}

void NodeParameters::retrieveParameters(const ros::NodeHandle& nodeHandle)
{
    nodeHandle.getParam("robot_frames", robotFrames);
    nodeHandle.param<std::string>("darpa_frame", darpaFrame, "darpa");
    nodeHandle.param<std::string>("map_topic", mapTopic, "map");
    nodeHandle.param<std::string>("map_publish_topic", mapPublishTopic, "map_relay");
    nodeHandle.param<std::string>("poses_publish_topic", posesPublishTopic, "poses_relay");
    nodeHandle.param<float>("map_publish_rate", mapPublishRate, 0.1);
    nodeHandle.param<float>("poses_publish_rate", posesPublishRate, 0.1);
    nodeHandle.param<int>("transform_dimension", transformDimension, 4);
}

void NodeParameters::validateParameters() const
{
    if (robotFrames.empty())
        throw std::runtime_error("Empty robot frames");

    if (darpaFrame.empty())
        throw std::runtime_error("Empty darpa frame");

    if (mapTopic.empty())
        throw std::runtime_error("Empty map topic");

    if (mapPublishTopic.empty())
        throw std::runtime_error("Empty map publish topic");

    if (posesPublishTopic.empty())
        throw std::runtime_error("Empty poses publish topic");

    if (mapPublishRate <= 0 || mapPublishRate > 1)
        throw std::runtime_error("Invalid map publish rate: " + std::to_string(mapPublishRate));

    if (posesPublishRate <= 0 || posesPublishRate > 1)
        throw std::runtime_error("Invalid poses publish rate: " + std::to_string(posesPublishRate));

    if (transformDimension != 3 && transformDimension != 4)
        throw std::runtime_error(
            "Invalid transform dimension: Must be 3 for 2D point cloud or 4 for 3D point cloud");
}
