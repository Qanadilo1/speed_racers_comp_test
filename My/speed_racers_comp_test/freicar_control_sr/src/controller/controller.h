/*
 * Author: Johan Vertens (vertensj@informatik.uni-freiburg.de)
 * Project: FreiCAR
 * Do NOT distribute this code to anyone outside the FreiCAR project
 */

#ifndef FREICAR_CONTROL_CONTROLLER_H
#define FREICAR_CONTROL_CONTROLLER_H


#include <string>
#include <cmath>
#include <algorithm>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/transform_datatypes.h>
#include <tf2/transform_storage.h>
#include <tf2/buffer_core.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include "raiscar_msgs/ControlCommand.h"
#include "std_msgs/Bool.h"
#include "raiscar_msgs/ControllerPath.h"
#include "std_msgs/Float64.h"

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>

//#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/point_cloud_conversion.h>
//#include <boost/foreach.hpp>
//#include <pcl_ros/point_cloud.h>
//#include <pcl_ros/point_cloud.h>
//#include <boost/foreach.hpp>
//HERE
#include <freicar_common/FreiCarControl.h>
//HERE STOP


class PID{
public:
    PID(double p, double i, double d){
        p_ = p;
        i_ = i;
        d_ = d;
        prev_t = ros::Time::now();
        prev_e = 0.0;
        integral = 0.0;

    };

    float step(const float error, const ros::Time stamp);
    void resetIntegral();

private:
    double p_;
    double i_;
    double d_;
    double integral;
    double prev_e;
    ros::Time prev_t;

};

class controller {
public:
    controller();
    virtual void controller_step(nav_msgs::Odometry odom);
    void min_depth(std_msgs::Float64 msg);
    void sub_stop(std_msgs::Bool msg);
    void receivePath(raiscar_msgs::ControllerPath new_path);
    void sendGoalMsg(const bool reached);
    void sub_overtake(std_msgs::Bool msg);
    void sub_lidar(sensor_msgs::PointCloud2 data);
    //HERE
    void sub_HLC(freicar_common::FreiCarControl msg);
    //STOP HERE
    std::vector<tf2::Transform> transformPath(nav_msgs::Path &path, const std::string target_frame);
    std::vector<tf2::Transform> discretizePath(std::vector<tf2::Transform> &path, float dist);
    // Vehicle parameters
    double L_;
    // Algorithm variables
    // Position tolerace is measured along the x-axis of the robot!
    double pos_tol_;
    // Control variables for Ackermann steering
    // Steering angle is denoted by delta
    double delta_max_;
    std::vector<tf2::Transform> path_;
    unsigned idx_;
    bool goal_reached_;
    bool completion_advertised_;
    raiscar_msgs::ControlCommand cmd_control_;
    bool stop_sign;
    bool overtake;
    //HERE
    bool HLC_bool;
    float min_depth_dist;
    
    freicar_common::FreiCarControl HLC_msg;
    //STOP HERE

    // Ros infrastructure
    ros::NodeHandle nh_, nh_private_;
    //HERE
    ros::Subscriber sub_odom_, sub_path_, sub_stop_, sub_lidar_,sub_overtake_,min_depth_,sub_hlc_;
    //STOP HERE
    ros::Publisher pub_acker_, pub_goal_reached_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    geometry_msgs::TransformStamped target_p_;
    geometry_msgs::TransformStamped lookahead_;
    std::string map_frame_id_, front_axis_frame_id_, rear_axis_frame_id_, target_frame_id_, tracker_frame_id;
    float current_steering_angle_;

    PID vel_pid;
    float vmax_;
    float des_v_;
    float throttle_limit_, curvature_vel_limit_factor, steering_vel_limit_factor, dist_vel_limit_factor, minimum_throttle_limit;
};



#endif //FREICAR_CONTROL_CONTROLLER_H
