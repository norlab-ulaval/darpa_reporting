#include "NodeParameters.h"

#include <memory>
#include <mutex>
#include <thread>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <pointmatcher_ros/PointMatcher_ROS.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

typedef PointMatcher<float> PM;

bool isMapReady;
sensor_msgs::PointCloud2 map;
geometry_msgs::PoseArray poses;
ros::Publisher mapPublisher;
ros::Publisher posesPublisher;
std::mutex mapLock;
std::shared_ptr<PM::Transformation> transformation;
std::unique_ptr<NodeParameters> params;
std::unique_ptr<tf2_ros::Buffer> tfBuffer;

void mapRelayCallback(const sensor_msgs::PointCloud2& cloudMsg)
{
    geometry_msgs::TransformStamped tf;

    try
    {
        tf = tfBuffer->lookupTransform(params->darpaFrame,
                                       cloudMsg.header.frame_id,
                                       cloudMsg.header.stamp,
                                       ros::Duration(0.1));
    }
    catch (const tf2::TransformException& ex)
    {
        ROS_WARN("%s", ex.what());
        return;
    }

    PM::TransformationParameters mapToDarpa =
        PointMatcher_ROS::rosTfToPointMatcherTransformation<float>(tf, params->transformDimension);
    PM::DataPoints cloud = PointMatcher_ROS::rosMsgToPointMatcherCloud<float>(cloudMsg);

    transformation->inPlaceCompute(mapToDarpa, cloud);

    mapLock.lock();
    map = PointMatcher_ROS::pointMatcherCloudToRosMsg<float>(
        cloud, params->darpaFrame, cloudMsg.header.stamp);
    isMapReady = true;
    mapLock.unlock();
}

void mapPublisherLoop()
{
    ros::Rate mapPublishRate(params->mapPublishRate);

    while (ros::ok())
    {
        mapPublishRate.sleep();

        mapLock.lock();
        if (isMapReady)
        {
            mapPublisher.publish(map);
            isMapReady = false;
        }
        mapLock.unlock();
    }
}

void posesPublisherLoop()
{
    ros::Rate posesPublishRate(params->posesPublishRate);

    while (ros::ok())
    {
        posesPublishRate.sleep();
        poses.header.stamp = ros::Time::now();

        for (auto i = 0; i < params->robotFrames.size(); ++i)
        {
            geometry_msgs::TransformStamped tf;

            try
            {
                tf = tfBuffer->lookupTransform(
                    params->darpaFrame, params->robotFrames[i], ros::Time(0), ros::Duration(0.1));
            }
            catch (const tf2::TransformException& ex)
            {
                ROS_WARN("%s", ex.what());
                continue;
            }

            tf2::Transform t;
            tf2::fromMsg(tf.transform, t);
            geometry_msgs::Pose pose;
            tf2::toMsg(t, pose);

            poses.poses[i] = pose;
        }
        posesPublisher.publish(poses);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "darpa_reporting_node");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    isMapReady = false;
    transformation = PM::get().TransformationRegistrar.create("RigidTransformation");

    params = std::unique_ptr<NodeParameters>(new NodeParameters(pn));
    mapPublisher = n.advertise<sensor_msgs::PointCloud2>(params->mapPublishTopic, 1, true);
    posesPublisher = n.advertise<geometry_msgs::PoseArray>(params->posesPublishTopic, 1, true);

    poses.header.frame_id = params->darpaFrame;
    poses.poses.assign(params->robotFrames.size(), geometry_msgs::Pose());

    for (auto& pose : poses.poses)
        pose.orientation.w = 1;

    ros::Subscriber mapSubscriber(n.subscribe(params->mapTopic, 1, mapRelayCallback));

    tfBuffer = std::unique_ptr<tf2_ros::Buffer>(new tf2_ros::Buffer);
    tf2_ros::TransformListener tfListener(*tfBuffer);

    std::thread mapPublisherThread(mapPublisherLoop);
    std::thread posePublisherThread(posesPublisherLoop);

    ros::spin();

    mapPublisherThread.join();
    posePublisherThread.join();

    return 0;
}
