/*
 * Author: Johan Vertens (vertensj@informatik.uni-freiburg.de)
 * Project: FreiCAR
 * Do NOT distribute this code to anyone outside the FreiCAR project
 */

#ifndef ROS_PF_LOCALIZATION_NODE
#define ROS_PF_LOCALIZATION_NODE

#include <random>
#include <unordered_set>

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/Twist.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <tf/transform_listener.h>

#include "data_types.hpp"
#include "ros_vis.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "sensor_model.h"
#include <list>
#include <random>
#include "math.h"
#include <map>
#include <initializer_list>
#include <freicar_common/FreiCarSign.h>
#include <freicar_common/FreiCarSigns.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "particle_filter.h"

#include <future>
#include <thread>
#include <chrono>

#include <freicar_map/thrift_map_proxy.h>
#include <freicar_map/planning/lane_star.h>
#include <freicar_map/logic/right_of_way.h>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, freicar_common::FreiCarSigns> MySyncPolicy;

class Localizer{
public:
    Localizer(std::shared_ptr<ros::NodeHandle> n);
    bool InitMap();

    void RegCallback(const sensor_msgs::ImageConstPtr& msg);
    void OdoCallback(const nav_msgs::OdometryConstPtr& msg);
    void regMarkerCallback(const sensor_msgs::ImageConstPtr& reg_msg, const freicar_common::FreiCarSignsConstPtr& markers_msg);
    void markerCallback(const freicar_common::FreiCarSignsConstPtr& markers_msg);
    void StartObservationStep();
    ~Localizer();

private:

    std::string SignIdToTypeString(int id);
    Eigen::Transform<float, 3, Eigen::Affine> GetTf(ros::Time time);

    static freicar::map::Map& map_;

    std::shared_ptr<particle_filter> p_filter;

    tf::TransformListener listener;
    tf::TransformBroadcaster broadcaster;
    std::shared_ptr<ros::NodeHandle> n_;

    ros::Subscriber odo_sub_;
    ros::Subscriber marker_sub_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    std::shared_ptr<ros_vis> visualizer_;

    freicar_common::FreiCarSigns last_sign_msg_;
    Eigen::Transform<float,3,Eigen::Affine> prev_odo_;
    void ApplyMotion(Eigen::Transform<float,3,Eigen::Affine> motion_inc);

    //std::unique_ptr<std::thread> sensor_model_thread_;

    std::future<bool> sensor_model_thread_;

    std::pair<std::vector<cv::Mat>, ros::Time> latest_lane_reg_;
    std::pair<std::vector<Sign>,ros::Time> latest_signs_;
    ros::Time last_odo_update_;
    bool use_lane_reg_;
    const char *path="/home/freicar/freicar_ws/src/freicar-2020-exercises//02-01-localization-exercise/freicar_localization/src/position_error.txt";
    // For evaluation
    bool evaluate_;
    double aggregated_error_;
    size_t num_measurements_;
    bool first_observation_received_;

};

#endif

