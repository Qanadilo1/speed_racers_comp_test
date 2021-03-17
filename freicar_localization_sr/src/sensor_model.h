/*
 * Author: Johan Vertens (vertensj@informatik.uni-freiburg.de)
 * Project: FreiCAR
 * Do NOT distribute this code to anyone outside the FreiCAR project
 */

#ifndef SENSOR_MODEL_H
#define SENSOR_MODEL_H

#include "nanoflann.hpp"
#include "unordered_set"
#include "ros_vis.h"
#include "data_types.hpp"
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <random>
#include <list>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#define NUM_SAMPLES 50
#define REG_THRESH 150

class sensor_model
{
    typedef nanoflann::KDTreeSingleIndexAdaptor<
            nanoflann::L2_Simple_Adaptor<float, PointCloud<float> > ,
            PointCloud<float>,
            2 /* dim */
            > map_kd_tree;

public:    
    sensor_model(PointCloud<float> map_data, std::map<std::string, PointCloud<float> > sign_data, std::shared_ptr<ros_vis> visualizer, bool use_lane_reg);

    std::vector<Eigen::Vector3f> getNearestPoints(std::vector<Eigen::Vector3f> sampled_points);
    std::vector<Eigen::Vector3f> getNearestPoints(std::vector<Sign> observed_signs);
    std::vector<Eigen::Vector3f> transformPoints(std::vector<Eigen::Vector3f> points, const Eigen::Transform<float, 3, Eigen::Affine> transform);
    bool calculatePoseProbability(const std::vector<cv::Mat> lane_regression, const std::vector<Sign> observed_signs, std::vector<Particle>& particles, float& max_prob);
    float SignMeasurementPoseProbabilityNearest(const std::vector<Sign>& observed_signs, const std::vector<Eigen::Vector3f>& data_associations_signs);
    float SignMeasurementPoseProbability(const std::vector<Sign>& observed_signs, Eigen::Transform<float,3,Eigen::Affine> particle_transform);
    std::vector<Sign> transformSigns(const std::vector<Sign>& signs, const Eigen::Transform<float,3,Eigen::Affine>& particle_pose);
    float LaneMeasurementPoseProbability(const std::vector<Eigen::Vector3f>& observed_points, const std::vector<Eigen::Vector3f>& data_associations, const std::vector<float>& weights, const float total_weight);
    std::vector<float> weights;
private:
    float sumWeights(const std::vector<float>& weights);

    PointCloud<float> map_data_;
    std::map<std::string, PointCloud<float> > sign_data_;
    std::map<std::string, std::unique_ptr<map_kd_tree> > sign_indeces_;
    map_kd_tree map_index_;
    std::shared_ptr<ros_vis> visualizer_;
    bool use_lane_reg_;
};

#endif // SENSOR_MODEL_H
