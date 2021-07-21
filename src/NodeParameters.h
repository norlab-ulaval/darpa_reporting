#ifndef NODE_PARAMETERS_H
#define NODE_PARAMETERS_H

#include <ros/ros.h>

class NodeParameters
{
  private:
    void retrieveParameters(const ros::NodeHandle& nodeHandle);
    void validateParameters() const;

  public:
    NodeParameters(ros::NodeHandle privateNodeHandle);

    std::vector<std::string> robotFrames;
    std::string darpaFrame;
    std::string mapTopic;
    std::string mapPublishTopic;
    std::string posesPublishTopic;
    float mapPublishRate;
    float posesPublishRate;
    int transformDimension;
};

#endif
