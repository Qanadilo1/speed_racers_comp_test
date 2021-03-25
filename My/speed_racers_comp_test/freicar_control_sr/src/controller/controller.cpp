/*
 * Author: Johan Vertens (vertensj@informatik.uni-freiburg.de)
 * Project: FreiCAR
 * Do NOT distribute this code to anyone outside the FreiCAR project
 */

#include "controller.h"



/*
 * Executes one step of the PID controller
 */
float PID::step(const float error, const ros::Time stamp){
    ros::Duration dt = stamp - prev_t;

    double delta_e = (error - prev_e) / dt.toSec();
    integral += error * dt.toSec();
    integral = std::min(integral, 1.0);

    double out = p_ * error + i_ * integral + d_ * delta_e; // TODO  : Optimise later

//    std::cout << "P value:  " << p_ * error << " D value: " << d_ * delta_e << " I value: " << i_ * integral << std::endl;

    prev_t = stamp;
    prev_e = error;
    return out;
}

/*
 * Resets the integral part to avoid biases
 */
void PID::resetIntegral(){
    integral = 0.0;
}

/*
 * Sends a boolean true as a topic message if the final goal has been reached
 */
void controller::sendGoalMsg(const bool reached){
    std_msgs::Bool msg;
    msg.data = reached;
    if (!completion_advertised_) {
        completion_advertised_ = true;

        pub_goal_reached_.publish(msg);
    }
    else{
        completion_advertised_ = true;

        pub_goal_reached_.publish(msg);

    }
}

/*
 * Rediscretize a given path so that there is at least "dist" meters between each point
 */
std::vector<tf2::Transform> controller::discretizePath(std::vector<tf2::Transform> &path, float dist){
    std::vector<tf2::Transform> disc_path;
    disc_path.push_back(path.at(0));

//    tf2::Vector3 last_dir;
    for(int i=0; i< (path.size()-1); i++){
        float current_d = dist;
        tf2::Vector3 t1p = path.at(i).getOrigin();
        tf2::Vector3 t2p = path.at(i + 1).getOrigin();
        tf2::Vector3 dir = (t2p - t1p);
        float dir_len = dir.length();

        if(dir_len < 0.05)
            continue;

        while(dir_len > current_d){
            tf2::Transform new_t;
            new_t.setIdentity();
            new_t.setOrigin(current_d * dir.normalize() + t1p);
            disc_path.push_back(new_t);
            current_d += dist;
        }
        disc_path.push_back(path.at(i+1));
    }
    return disc_path;
}


/*
 * Transforms a given path to any target frame
 */
std::vector<tf2::Transform> controller::transformPath(nav_msgs::Path &path, const std::string target_frame){
    std::vector<tf2::Transform> t_path;
    if(path.header.frame_id != target_frame){
//        std::cout << "Transforming path to " << target_frame << std::endl;

        geometry_msgs::TransformStamped tf_msg;
        tf2::Stamped<tf2::Transform> transform;
        tf_msg = tf_buffer_.lookupTransform(path.header.frame_id, target_frame, ros::Time(0));
        tf2::convert(tf_msg, transform);

        for (auto & i : path.poses){
            tf2::Transform t_pose;
            tf2::convert(i.pose, t_pose);
            t_pose = transform * t_pose;
            t_pose.getOrigin().setZ(0);
            t_path.push_back(t_pose);
        }
    }else{
        for (auto & i : path.poses){
            tf2::Transform t_pose;
            tf2::convert(i.pose, t_pose);
            t_pose.getOrigin().setZ(0);
            t_path.push_back(t_pose);
        }
    }
    return t_path;
}

/*
 * Callback function that receives a path
 */
void controller::receivePath(raiscar_msgs::ControllerPath new_path)
{
    // When a new path received, the previous one is simply discarded
    // It is up to the planner/motion manager to make sure that the new
    // path is feasible.
    // ROS_INFO("Received new path");


    //vel_override_ = new_path.des_vel;
    if (new_path.path_segment.poses.size() > 0)
    {
        path_ = transformPath(new_path.path_segment, map_frame_id_);
        goal_reached_ = false;
        completion_advertised_ = false;
    }
    else
    {
        path_ = std::vector<tf2::Transform>();
        goal_reached_ = true;
        completion_advertised_ = true;
        // ROS_WARN_STREAM("Received empty path!");
    }

//    path_ = discretizePath(path_, 0.5);
}
void controller::sub_stop(std_msgs::Bool msg)
{
    stop_sign = msg.data;
}

void controller::min_depth(std_msgs::Float64 msg)
{
    min_depth_dist = msg.data;
    std::cout << "min_depth for skipping stop sign" << min_depth_dist << std::endl;
}

void controller::sub_overtake(std_msgs::Bool msg)
{
    overtake = msg.data;
}

void controller::sub_lidar(sensor_msgs::PointCloud2 data)
{
//    std::cout << "subscribed to lidar !!!" << std::endl;
//    BOOST_FOREACH (const pcl::PointXYZ& pt, data)

    sensor_msgs::PointCloud out_pc;
    sensor_msgs::convertPointCloud2ToPointCloud(data, out_pc);
    int counter = 0;
    for (int i =0; i<out_pc.points.size();i++)
    {
        geometry_msgs::Point32 point;
        point.z = out_pc.points[i].z ;
//        std::cout<<"point z"<<point.z<<std::endl;
        if(out_pc.points[i].z > 2.9)
        {
            counter = counter + 1;
        }
        


//        pcl::toROSMsg (*cloud, image_)
//        pcl::

    }
//    std::cout<<"counter"<<counter<<std::endl;



}
//HERE
void controller::sub_HLC (freicar_common::FreiCarControl msg)
{
    HLC_msg.command = msg.command;
    HLC_msg.name = msg.name;
    HLC_bool = true;
}
//STOP HERE
/*
 * Virtual function that needs to be reimplemented
 */
void controller::controller_step(nav_msgs::Odometry odom)
{
    std::cout << "No action implemented ..." << std::endl;
}

controller::controller():pos_tol_(0.1), idx_(0),
                         goal_reached_(true), nh_private_("~"), tf_listener_(tf_buffer_), vel_pid(0.05, 0.15, 0.000){
    // Get parameters from the parameter server
    nh_private_.param<double>("wheelbase", L_, 0.36);

    nh_private_.param<double>("position_tolerance", pos_tol_, 0.1);
    nh_private_.param<double>("steering_angle_limit", delta_max_, 1.22173);
    nh_private_.param<float>("desired_velocity", des_v_, 0.1);

    nh_private_.param<std::string>("map_frame_id", map_frame_id_, "map");
    nh_private_.param<float>("vmax", vmax_, 5.0);
    nh_private_.param<std::string>("robot_frame_id", tracker_frame_id, "freicar");
    nh_private_.param<std::string>("target_frame_id", target_frame_id_, tracker_frame_id + "/lookahead");

    front_axis_frame_id_ = tracker_frame_id + "/front_axis";
    rear_axis_frame_id_ = tracker_frame_id + "/rear_axis";

    // Populate messages with static data
    target_p_.header.frame_id = map_frame_id_;
    target_p_.child_frame_id = target_frame_id_;
    target_p_.transform.rotation.w = 1.0;

    lookahead_.header.frame_id = map_frame_id_;
    lookahead_.child_frame_id = target_frame_id_;
    lookahead_.transform.rotation.w = 1.0;


    // Subscribers to path segment and odometry
    sub_path_ = nh_.subscribe("path_segment", 1, &controller::receivePath, this);
    sub_odom_ = nh_.subscribe("odometry", 1, &controller::controller_step, this);
    sub_stop_ = nh_.subscribe("Stop_sign", 1, &controller::sub_stop,this);
    min_depth_ = nh_.subscribe("min_depth", 1, &controller::min_depth,this);
    sub_lidar_ = nh_.subscribe("freicar_1/sim/lidar", 1, &controller::sub_lidar,this);
    sub_overtake_ = nh_.subscribe("/overtake", 1000, &controller::sub_overtake,this);
    //HERE
    sub_hlc_ = nh_.subscribe("/overtake", 1000, &controller::sub_overtake,this);
    //STOP HERE
//    ros::Subscriber sub3 = node_handle->subscribe("car_localization", 1, callback_car_localization);
    // Publishers for the control command and the "reached" message
    pub_acker_ = nh_.advertise<raiscar_msgs::ControlCommand>("control", 1);
    pub_goal_reached_ = nh_.advertise<std_msgs::Bool>("goal_reached", 1);
    completion_advertised_ = false;
    current_steering_angle_ = 0;

    std::cout << "Pure Pursuit Controller started for frame : " << tracker_frame_id << std::endl;
}
