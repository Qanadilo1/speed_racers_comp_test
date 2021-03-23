/*
 * Author: Johan Vertens (vertensj@informatik.uni-freiburg.de)
 * Project: FreiCAR
 * Do NOT distribute this code to anyone outside the FreiCAR project
 */

/*
 * This file is the main file of the particle filter. It sets up ROS publisher/subscriber, defines the callbacks and
 * creates the particle filter class
 */

#include "ros/ros.h"
#include "ros_pf_localization.h"
#include <iostream>
#include <fstream>
#include <sstream>

// The freicar map class is a singleton, get the instance of that singleton here
std::vector<cv::Mat> observation_reg2;
freicar::map::Map &Localizer::map_ = freicar::map::Map::GetInstance();

/*
 * Entry class of ROS node. It does the following:
 * 1. Sets up all subscribers and publishers
 * 2. Initializes Map
 * 3. Initializes all particles of the particle filter class
 */
Localizer::Localizer(std::shared_ptr<ros::NodeHandle> n) : n_(n), it_(*n) {
    std::ofstream file(path);
    ros::Duration sleep_time(1);
    odo_sub_ = n_->subscribe("/freicar_1/odometry", 1, &Localizer::OdoCallback, this);
    marker_sub_ = n_->subscribe("traffic_signs", 1, &Localizer::markerCallback, this);
    last_odo_update_ = ros::Time::now();

    freicar::map::ThriftMapProxy map_proxy("127.0.0.1", 9091, 9090);
    std::string map_path;
    if (!ros::param::get("/map_path", map_path)) {
        ROS_ERROR("could not find parameter: map_path! map initialization failed.");
        return;
    }

    if (!ros::param::get("~use_lane_regression", use_lane_reg_)) {
        ROS_INFO("could not find parameter: use_lane_regression! default: do not use lane_regression.");
        use_lane_reg_ = false;
    }

    if (!ros::param::get("~evaluate", evaluate_)) {
        ROS_INFO("could not find parameter: evaluate! default: do not evaluate.");
        use_lane_reg_ = false;
    }

    if (use_lane_reg_) {
        image_sub_ = it_.subscribe("/freicar_1/sim/camera/rgb/front/reg_bev", 1, &Localizer::RegCallback, this);
        std::cout << "lane regression is running ";
    }
// if the map can't be loaded
    if (!map_proxy.LoadMapFromFile(map_path)) {
        ROS_INFO("could not find thriftmap file: %s, starting map server...", map_path.c_str());
        map_proxy.StartMapServer();
        // stalling main thread until map is received
        while (freicar::map::Map::GetInstance().status() == freicar::map::MapStatus::UNINITIALIZED) {
            ROS_INFO("waiting for map...");
            ros::Duration(1.0).sleep();
        }
        ROS_INFO("map received!");
        // Thrift creates a corrupt file on failed reads, removing it
        remove(map_path.c_str());
        map_proxy.WriteMapToFile(map_path);
        ROS_INFO("saved new map");
    }
    ros::Duration(2.0).sleep();
    freicar::map::Map::GetInstance().PostProcess(0.22);  // densifies the lane points

    visualizer_ = std::make_shared<ros_vis>(n_);
    p_filter = std::make_shared<particle_filter>(&map_, visualizer_, use_lane_reg_);
    sleep_time.sleep();
//    visualizer_->SendPoints(p_filter->getMapKDPoints(), "map_points", "/map");
    ROS_INFO("Sent map points...");

}

/*
 * Receives /base_link relative to /map as Eigen transformation matrix.
 * WARNING, IMPORTANT: This is ground truth, so don't use it anywhere in the particle filter. This is just for reference
 */
Eigen::Transform<float, 3, Eigen::Affine> Localizer::GetTf(ros::Time time) {
    Eigen::Transform<float, 3, Eigen::Affine> gt_t = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
    tf::StampedTransform transform;
    try {
//        listener.lookupTransform("/map", "/freicar_1/base_link",
//                                 time, transform);

    }
    catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }

    gt_t.translate(Eigen::Vector3f(transform.getOrigin().x(), transform.getOrigin().y(), 0.));
    gt_t.rotate(Eigen::Quaternionf(transform.getRotation().getW(), transform.getRotation().getX(),
                                   transform.getRotation().getY(), transform.getRotation().getZ()));
    return gt_t;
}

// Destructor
Localizer::~Localizer(){
    std::cout << "Shutting down localizer..." << std::endl;
    if (evaluate_){
        double mean_error = aggregated_error_ / num_measurements_;
        //std::cout << "aggregated_error_ " << aggregated_error_ << std::endl;
        //std::cout << "num_measurements_ " << num_measurements_ << std::endl;
        std::fstream file;
        file.open("/home/freicar/freicar_ws/src/freicar-2020-exercises//02-01-localization-exercise/freicar_localization/src/pos_aver.txt", std::fstream::app);
        if(!file.is_open()){
            std::cout << "File doesnt exist ";
        }
        file << mean_error << ",";
        file.close();
        std::cout << "Average error during this run [m]: " << mean_error << std::endl;

    }
}

// Callback function for getting the FreiCAR Sign detections
void Localizer::markerCallback(const freicar_common::FreiCarSignsConstPtr &markers_msg) {
    last_sign_msg_ = *markers_msg;
    std::vector<Sign> sign_observations;

    for (size_t i = 0; i < last_sign_msg_.signs.size(); i++) {
        freicar_common::FreiCarSign s_msg = last_sign_msg_.signs.at(i);
        unsigned int in_id = s_msg.type;
        // check valid range of ids (AudiCup 18)
        if (in_id >= 0 && in_id <= 17) {
            Sign new_sign;
            new_sign.id = s_msg.type;
            new_sign.position = Eigen::Vector3f(s_msg.x, s_msg.y, s_msg.z);
            // Convert id into type string
            new_sign.type = SignIdToTypeString(new_sign.id);
            sign_observations.push_back(new_sign);
        }
    }
    latest_signs_ = std::pair<std::vector<Sign>,ros::Time>(sign_observations, markers_msg->header.stamp);
}


/*
 * Callback function for getting the odometry in order to process the motion model later.
 * This function also visualizes the particles. You have the option to either just show the best particle or
 * showing a mean particle consisting out of the k best particles
 */
void Localizer::OdoCallback(const nav_msgs::OdometryConstPtr &msg) {
    // Apply motion model to all particles
    p_filter->MotionStep(*msg);

    //visualizer_->SendBestParticle(p_filter->getBestParticle(), "/map");
    Particle best_particle = p_filter->getMeanParticle(100);

    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "freicar_1/handle";
    transformStamped.transform.translation.x = best_particle.transform.translation().x();
    transformStamped.transform.translation.y = best_particle.transform.translation().y();
    transformStamped.transform.translation.z = best_particle.transform.translation().z();


//    ROS_INFO("Broadcaster done");

    Eigen::Quaternionf q(best_particle.transform.rotation().matrix());

    transformStamped.transform.rotation.x = q.y();
    transformStamped.transform.rotation.y = q.x();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    broadcaster.sendTransform(transformStamped);

    visualizer_->SendBestParticle(best_particle, "map");



    last_odo_update_ = msg->header.stamp;
    if (evaluate_ && first_observation_received_) {
        Eigen::Transform<float, 3, Eigen::Affine> gt_pose = this->GetTf(ros::Time(0));
	Eigen::Vector3f particle_pos = Eigen::Vector3f(best_particle.transform.translation().x(), best_particle.transform.translation().y(), 0.);
        float position_error = std::sqrt((gt_pose.translation() - particle_pos).squaredNorm());
        aggregated_error_ += position_error;
        num_measurements_++;
        std::cout << "Position error [m]: " << position_error << std::endl;
        std::fstream file;
        file.open(path, std::fstream::app);
        if(!file.is_open()){
            std::cout << "File doesnt exist ";
        }
        file << position_error << ",";
        file.close();


    }
}

/*
 * Converts the sign id to the corresponding type string
 * E.g The sign with id 0 belongs to the type string "CrossingRight"
 */
std::string Localizer::SignIdToTypeString(int id) {
    switch (id) {
        case 0:
            return "CrossingRight";
        case 1:
            return "Stop";
        case 2:
            return "Parking";
        case 3:
            return "RightOfWay";
        case 4:
            return "Straight";
        case 5:
            return "GiveWay";
        case 6:
            return "PedestrianCrossing";
        case 7:
            return "Roundabout";
        case 8:
            return "NoOvertaking";
        case 9:
            return "NoEntry";
        case 10:
            return "Position";
        case 11:
            return "OneWay";
        case 12:
            return "Roadworks";
        case 13:
            return "Speed50";
        case 14:
            return "Speed100";
        case 15:
            return "Spare1";
        case 16:
            return "Position";
        case 17:
            return "Position";
        default:
            return "Position";
    }
    return "";
}

/*
 * This function collects all latest observation messages and passes it to the sensor model
 */
void Localizer::StartObservationStep(){
    // Got observation -> weight particles
    if (sensor_model_thread_.valid()) {
        if (sensor_model_thread_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
            std::cout << "Sensor model is busy, waiting..." << std::endl;
            return;
        } else {
            //std::cout << "Sensor model is ready..." << std::endl;
        }
    }

    std::vector<Sign> observation_signs;
    std::vector<cv::Mat> observation_reg;

    if(!latest_signs_.first.empty()){
        if(abs((latest_signs_.second - last_odo_update_).toSec()) < 0.05){
            observation_signs = (latest_signs_.first);
            first_observation_received_ = true;
        }
        latest_signs_.first.clear();
    }
    if (use_lane_reg_) {
        if (!latest_lane_reg_.first.empty()) {
            if (abs((latest_lane_reg_.second - last_odo_update_).toSec()) < 0.05) {
                observation_reg = (latest_lane_reg_.first);
                first_observation_received_ = true;
            }
            latest_lane_reg_.first.clear();
        }
    }
    if(!observation_reg.empty() || !observation_signs.empty()) {
        sensor_model_thread_ = std::async(std::launch::async, &particle_filter::ObservationStep, p_filter,
                                          observation_reg2, //without
                                          observation_signs);
    }

}


/*
 * Lane Regression callback (BEV view). If the lane regression is provided it will go into the sensor-model and
 * improves the overall performance
 */
void Localizer::RegCallback(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat reg = cv_ptr->image;
    std::vector<cv::Mat> mat_vec;
    mat_vec.push_back(reg);
    latest_lane_reg_ = std::pair<std::vector<cv::Mat>, ros::Time>(mat_vec, msg->header.stamp);
    observation_reg2 = (latest_lane_reg_.first);

}

// Main function that inits ROS, creates the localizer class and spins the system
int main(int argc, char **argv) {
    ros::init(argc, argv, "freicar_particle_filter");

    std::shared_ptr<ros::NodeHandle> n = std::make_shared<ros::NodeHandle>();

    ros::Rate loop_rate(100);

    std::cout << "Particle filter localization node started: " << std::endl;

    Localizer loc(n);

    while (ros::ok()) {
        ros::spinOnce();
        loc.StartObservationStep();
        loop_rate.sleep();
    }

    return 0;
}

