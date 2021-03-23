/*
 * Author: Johan Vertens (vertensj@informatik.uni-freiburg.de)
 * Project: FreiCAR
 * Do NOT distribute this code to anyone outside the FreiCAR project
 */

#include "ros_vis.h"

/*
 * This class comprises of many visualization functions that can send points or lines as ros msgs to rviz
 */
ros_vis::ros_vis(std::shared_ptr<ros::NodeHandle> n):n_(n)
{
    marker_pub_ = n_->advertise<visualization_msgs::Marker>("particlep_filter_info_marker", 10);
    poses_pub_ = n_->advertise<geometry_msgs::PoseArray>("particles", 10);
    best_particle_pub_ = n_->advertise<geometry_msgs::PoseArray>("best_particle", 10);
}

void ros_vis::SendPoints(PointCloud<float> pts, const std::string ns, const std::string frame, float r, float g, float b){
    static size_t cnt = 0;
    visualization_msgs::Marker points;
    points.header.frame_id = frame;
    points.header.stamp =  ros::Time::now();
    points.ns = ns;
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.id = 0;
    points.type = visualization_msgs::Marker::POINTS;
    points.scale.x = 0.1;
    points.scale.y = 0.1;
    points.scale.z = 0.1;

    points.color.r = r;
    points.color.g = g;
    points.color.b = b;
    points.color.a = 1.0;

    for(size_t i = 0; i < pts.pts.size(); i++){
        geometry_msgs::Point p;
        p.x = static_cast<double>(pts.pts.at(i).x);
        p.y = static_cast<double>(pts.pts.at(i).y);
        p.z = 0;
        points.points.push_back(p);
    }

    marker_pub_.publish(points);
    marker_pub_.publish(points);
}

void ros_vis::SendSigns(std::vector<Sign> signs, const std::string ns, const std::string frame){
    static size_t cnt = 0;
    visualization_msgs::Marker points;
    points.header.frame_id = frame;
    points.header.stamp =  ros::Time::now();
    points.ns = ns;
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.id = 0;
    points.type = visualization_msgs::Marker::POINTS;
    points.scale.x = 0.1;
    points.scale.y = 0.1;
    points.scale.z = 0.1;

    points.color.r = 1.0;
    points.color.g = 0.5;
    points.color.b = 0.3;
    points.color.a = 1.0;

    for(size_t i = 0; i < signs.size(); i++){
        geometry_msgs::Point p;
        const Sign& s = signs.at(i);
        p.x = static_cast<double>(s.position[0]);
        p.y = static_cast<double>(s.position[1]);
        p.z = 0;
        points.points.push_back(p);
    }

    marker_pub_.publish(points);
    marker_pub_.publish(points);
}

void ros_vis::SendPoses(std::vector<Particle > poses, const std::string ns, const std::string frame){
    geometry_msgs::PoseArray poses_msg;

    poses_msg.header.frame_id = frame;
    poses_msg.header.stamp =  ros::Time::now();
    poses_msg.header.seq = 0;

    //    for(size_t i = 0; i < poses.size(); i++){
    for(auto p_i = poses.begin(); p_i != poses.end(); p_i++){
        Eigen::Transform<float,3,Eigen::Affine> transform = p_i->transform;

        geometry_msgs::Pose pose_msg;
        pose_msg.position.x = transform.translation().x();
        pose_msg.position.y = transform.translation().y();
        pose_msg.position.z = transform.translation().z();

        Eigen::Quaternionf rot(transform.rotation());
        pose_msg.orientation.w = rot.w();
        pose_msg.orientation.x = rot.x();
        pose_msg.orientation.y = rot.y();
        pose_msg.orientation.z = rot.z();

        poses_msg.poses.push_back(pose_msg);

    }

    poses_pub_.publish(poses_msg);

}

void ros_vis::SendBestParticle(const Particle& pose, const std::string frame){
    geometry_msgs::PoseArray poses_msg;

    poses_msg.header.frame_id = frame;
    poses_msg.header.stamp =  ros::Time::now();
    poses_msg.header.seq = 0;

    //    for(size_t i = 0; i < poses.size(); i++){
    Eigen::Transform<float,3,Eigen::Affine> transform = pose.transform;

    geometry_msgs::Pose pose_msg;
    pose_msg.position.x = transform.translation().x();
    pose_msg.position.y = transform.translation().y();
    pose_msg.position.z = transform.translation().z();

    Eigen::Quaternionf rot(transform.rotation());
    pose_msg.orientation.w = rot.w();
    pose_msg.orientation.x = rot.x();
    pose_msg.orientation.y = rot.y();
    pose_msg.orientation.z = rot.z();

    poses_msg.poses.push_back(pose_msg);

    best_particle_pub_.publish(poses_msg);

}

void ros_vis::VisualizeDataAssociations(std::vector<Eigen::Vector3f> src, std::vector<Eigen::Vector3f> target, const std::string ns, const std::string frame, float r, float g, float b){
    static size_t cnt = 0;
    visualization_msgs::Marker points;
    points.header.frame_id = frame;
    points.header.stamp =  ros::Time::now();
    points.ns = ns;
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.id = 0;
    points.type = visualization_msgs::Marker::LINE_LIST;
    points.scale.x = 0.04;
    points.scale.y = 0.04;
    points.scale.z = 0.04;

    points.color.r = r;
    points.color.g = g;
    points.color.b = b;
    points.color.a = 1.0;

    for(size_t i = 0; i < src.size(); i++){
        geometry_msgs::Point p;
        p.x = static_cast<double>(src.at(i).x());
        p.y = static_cast<double>(src.at(i).y());
        p.z = 0;
        points.points.push_back(p);

        geometry_msgs::Point q;
        q.x = static_cast<double>(target.at(i).x());
        q.y = static_cast<double>(target.at(i).y());
        q.z = 0;
        points.points.push_back(q);
    }

    marker_pub_.publish(points);
}

void ros_vis::VisualizeDataAssociations(std::vector<Sign> src, std::vector<Eigen::Vector3f> target, const std::string ns, const std::string frame, float r, float g, float b){
    static size_t cnt = 0;
    visualization_msgs::Marker points;
    points.header.frame_id = frame;
    points.header.stamp =  ros::Time::now();
    points.ns = ns;
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.id = 0;
    points.type = visualization_msgs::Marker::LINE_LIST;
    points.scale.x = 0.04;
    points.scale.y = 0.04;
    points.scale.z = 0.04;

    points.color.r = r;
    points.color.g = g;
    points.color.b = b;
    points.color.a = 1.0;

    for(size_t i = 0; i < src.size(); i++){
        geometry_msgs::Point p;
        p.x = static_cast<double>(src.at(i).position.x());
        p.y = static_cast<double>(src.at(i).position.y());
        p.z = 0;
        points.points.push_back(p);

        geometry_msgs::Point q;
        q.x = static_cast<double>(target.at(i).x());
        q.y = static_cast<double>(target.at(i).y());
        q.z = 0;
        points.points.push_back(q);
    }

    marker_pub_.publish(points);
}



