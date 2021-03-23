/*
 * Author: Johan Vertens (vertensj@informatik.uni-freiburg.de)
 * Project: FreiCAR
 * Do NOT distribute this code to anyone outside the FreiCAR project
 */

#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include <list>
#include <random>
#include <chrono>
#include <mutex>
#include <algorithm>
#include <map>
#include "math.h"
#include "data_types.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "map_helper.h"
#include "sensor_model.h"
#include "ros_vis.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <freicar_map/planning/lane_star.h>
#include <freicar_map/logic/right_of_way.h>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#define NUM_PARTICLES 5000
#define BEST_PARTICLE_HISTORY 20
#define QUALITY_RELOC_THRESH 0.65

using namespace std::chrono;

class particle_filter
{
public:

    // Particles has a mutex so we need to implement the init constructor our own
    // Move initialization
    particle_filter(const particle_filter& other) {
        std::lock_guard<std::mutex> lock(other.particle_mutex_);
        particles_ = other.particles_;
        generator_ = other.generator_;
        kd_data_ = other.kd_data_;
        sign_data_ = other.sign_data_;
        sensor_model_ = other.sensor_model_;
        particles_init_ = other.particles_init_;
        odo_init_ = other.odo_init_;
        map_init_ = other.map_init_;
        prev_odo_ = other.prev_odo_;
        visualizer_ = other.visualizer_;
        map_ = other.map_;
        // Vector of weights of all particles
        //std::vector<double> weights;
      }

    particle_filter(freicar::map::Map* map, std::shared_ptr<ros_vis> vis, bool use_lane_reg);

    float getSumWeights();
    Eigen::Vector4f rotationAverage(std::vector<Eigen::Vector4f> quaternions);
    void ImportanceSampling();
    void LowVarianceSampling();
    void InitParticles();
    void InitParticlesAroundPose(const Eigen::Transform<float,3,Eigen::Affine> pose);

    // relative motion model
    void ConstantVelMotionModel(nav_msgs::Odometry odometry, float time_step);
    bool InitMap(freicar::map::Map* map_data);
    bool ObservationStep(const std::vector<cv::Mat> reg, const std::vector<Sign> observed_signs);
    void MotionStep(nav_msgs::Odometry odometry);
    PointCloud<float>& getMapKDPoints();
    int getBestParticleIndex();
    int getWorstParticleIndex();
    Particle getBestParticle();
    Particle getMeanParticle(int k_mean);
    float getSpread();
    bool getQuality(float& quality);
    void NormalizeParticles(const float norm_val);

    std::vector<Sign> transformSignsToCarBaseLink(const std::vector<Sign> &signs);

    float j,u,c,s,r;




private:
    mutable std::mutex particle_mutex_;

    std::vector<Particle> particles_;

    std::default_random_engine generator_;

    // Map data storages
    PointCloud<float> kd_data_;
    std::map<std::string, PointCloud<float> > sign_data_;
    std::shared_ptr<sensor_model> sensor_model_;

    bool particles_init_;
    bool odo_init_;
    bool map_init_;

    nav_msgs::Odometry prev_odo_;

    // ROS DEPENDENCIES
    std::shared_ptr<ros_vis> visualizer_;

    freicar::map::Map* map_;

    float latest_x_vel;

    std::deque<float> particle_memory_;
    bool memory_init_;
    int memory_insert_cnt_;
    bool use_lane_reg_;


};

#endif // PARTICLE_FILTER_H
