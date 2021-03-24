/*
 * Author: Johan Vertens (vertensj@informatik.uni-freiburg.de)
 * Project: FreiCAR
 * Do NOT distribute this code to anyone outside the FreiCAR project
 */

#ifndef ROS_VIS_H
#define ROS_VIS_H

#include <Eigen/Dense>

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include "data_types.hpp"
#include <list>

class ros_vis
{
public:
    ros_vis(std::shared_ptr<ros::NodeHandle> n);
    void SendPoints(PointCloud<float> pts, const std::string ns, const std::string frame, float r=1.0f, float g=0.0f, float b=0.0f);

    void SendPoses(std::vector<Particle > poses, const std::string ns, const std::string frame);

    void VisualizeDataAssociations(std::vector<Eigen::Vector3f> src, std::vector<Eigen::Vector3f> target, const std::string ns, const std::string frame, float r=1.0f, float g=0.0f, float b=0.0f);
    void VisualizeDataAssociations(std::vector<Sign> src, std::vector<Eigen::Vector3f> target, const std::string ns, const std::string frame, float r, float g, float b);

    void SendBestParticle(const Particle &poses, const std::string frame);

    void SendSigns(std::vector<Sign> signs, const std::string ns, const std::string frame);

    template <class T>
    void SendPoints(std::vector<Eigen::Matrix<T, 3, 1> > pts, const std::string ns, const std::string frame, float r=1.0f, float g=0.0f, float b=0.0f){
        static size_t cnt = 0;
        visualization_msgs::Marker points;
        points.header.frame_id = frame;
        points.header.stamp =  ros::Time::now();
        points.ns = ns;
        points.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = 1.0;
        points.id = 0;
        points.type = visualization_msgs::Marker::SPHERE_LIST;
        points.scale.x = 0.1;
        points.scale.y = 0.1;
        points.scale.z = 0.1;

        points.color.r = r;
        points.color.g = g;
        points.color.b = b;
        points.color.a = 1.0;

        for(size_t i = 0; i < pts.size(); i++){
            geometry_msgs::Point p;
            p.x = static_cast<double>(pts.at(i).x());
            p.y = static_cast<double>(pts.at(i).y());
            p.z = 0;
            points.points.push_back(p);
        }

        marker_pub_.publish(points);
    }

private:
        ros::Publisher marker_pub_;
        ros::Publisher poses_pub_;
        ros::Publisher best_particle_pub_;
        std::shared_ptr<ros::NodeHandle> n_;
};

#endif // ROS_VIS_H
