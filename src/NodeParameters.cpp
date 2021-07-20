#include "NodeParameters.h"

NodeParameters::NodeParameters(ros::NodeHandle privateNodeHandle)
{
    retrieveParameters(privateNodeHandle);
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
