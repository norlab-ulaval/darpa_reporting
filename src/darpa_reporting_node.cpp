#include "NodeParameters.h"

#include <memory>
#include <mutex>
#include <thread>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <pointmatcher_ros/point_cloud.h>
#include <pointmatcher_ros/transform.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

typedef PointMatcher<float> PM;

bool isMapReady;
sensor_msgs::PointCloud2 map;
geometry_msgs::PoseArray poses;
ros::Publisher mapPublisher;
ros::Publisher posesPublisher;
std::mutex mapLock;
std::shared_ptr<PM::Transformation> transformation;
std::unique_ptr<NodeParameters> params;
std::unique_ptr<tf::TransformListener> tfListener;

void mapRelayCallback(const sensor_msgs::PointCloud2& cloudMsg)
{
    PM::TransformationParameters mapToDarpa;

    try
    {
        mapToDarpa = PointMatcher_ros::transformListenerToEigenMatrix<float>(
            *tfListener, params->darpaFrame, cloudMsg.header.frame_id, cloudMsg.header.stamp);
    }
    catch (const tf::TransformException& ex)
    {
        ROS_WARN("%s", ex.what());
        return;
    }

    PM::DataPoints cloud = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(cloudMsg, true);

    transformation->inPlaceCompute(mapToDarpa, cloud);

    mapLock.lock();
    map = PointMatcher_ros::pointMatcherCloudToRosMsg<float>(
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
        poses.poses.clear();
        poses.header.stamp = ros::Time::now();

        for (const auto& robotFrame : params->robotFrames)
        {
            tf::StampedTransform tf;

            try
            {
                tfListener->lookupTransform(params->darpaFrame, robotFrame, ros::Time(0), tf);
            }
            catch (const tf::TransformException& ex)
            {
                ROS_WARN("%s", ex.what());
                break;
            }

            geometry_msgs::TransformStamped robotToDarpaTfMsg;
            tf::transformStampedTFToMsg(tf, robotToDarpaTfMsg);
            geometry_msgs::Pose pose;
            pose.position.x = robotToDarpaTfMsg.transform.translation.x;
            pose.position.y = robotToDarpaTfMsg.transform.translation.y;
            pose.position.z = robotToDarpaTfMsg.transform.translation.z;
            pose.orientation.x = robotToDarpaTfMsg.transform.rotation.x;
            pose.orientation.y = robotToDarpaTfMsg.transform.rotation.y;
            pose.orientation.z = robotToDarpaTfMsg.transform.rotation.z;
            pose.orientation.w = robotToDarpaTfMsg.transform.rotation.w;

            poses.poses.push_back(pose);
        }

        if (poses.poses.size() == params->robotFrames.size())
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
    poses.poses.reserve(params->robotFrames.size());

    ros::Subscriber mapSubscriber(n.subscribe(params->mapTopic, 1, mapRelayCallback));

    tfListener = std::unique_ptr<tf::TransformListener>(new tf::TransformListener());

    std::thread mapPublisherThread(mapPublisherLoop);
    std::thread posePublisherThread(posesPublisherLoop);

    ros::spin();

    mapPublisherThread.join();
    posePublisherThread.join();

    return 0;
}
