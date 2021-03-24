/*
 * Author: Johan Vertens (vertensj@informatik.uni-freiburg.de)
 * Project: FreiCAR
 * Do NOT distribute this code to anyone outside the FreiCAR project
 */

#include "sensor_model.h"
#include <sstream>

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

visualization_msgs::MarkerArray markerArray;
std::unordered_set<int> pickSet(int N, int k, std::mt19937& gen)
{
    std::unordered_set<int> elems;
    for (int r = N - k; r < N; ++r) {
        int v = std::uniform_int_distribution<>(1, r)(gen);

        if (!elems.insert(v).second) {
            elems.insert(r);
        }
    }
    return elems;
}

/*
 * Returns k random indeces between 0 and N
 */
std::vector<int> pick(int N, int k) {
    std::random_device rd;
    std::mt19937 gen(rd());

    std::unordered_set<int> elems = pickSet(N, k, gen);

    std::vector<int> result(elems.begin(), elems.end());
    std::shuffle(result.begin(), result.end(), gen);
    return result;
}

/*
 * Constructor of sensor model. Builds KD-tree indeces
 */
sensor_model::sensor_model(PointCloud<float> map_data, std::map<std::string, PointCloud<float> > sign_data, std::shared_ptr<ros_vis> visualizer, bool use_lane_reg):map_data_(map_data), sign_data_(sign_data), map_index_(2 /*dim*/, map_data_, nanoflann::KDTreeSingleIndexAdaptorParams(10 /* max leaf */) ), visualizer_(visualizer)
{
    // Creating KD Tree indeces for fast nearest neighbor search
    map_index_.buildIndex();
    for(auto ds = sign_data_.begin(); ds != sign_data_.end(); ds++){
        std::cout << "Creating kd tree for sign type: " << ds->first << " with " << ds->second.pts.size() << " elements..." << std::endl;
        sign_indeces_[ds->first] = std::unique_ptr<map_kd_tree>(new map_kd_tree(2, ds->second, nanoflann::KDTreeSingleIndexAdaptorParams(10)));
        sign_indeces_[ds->first]->buildIndex();
    }
    use_lane_reg_ = use_lane_reg;
}

/*
 * For any given observed lane center points return the nearest lane-center points in the map
 * This is using a KD-Tree for efficient match retrieval
 */
std::vector<Eigen::Vector3f> sensor_model::getNearestPoints(std::vector<Eigen::Vector3f> sampled_points){
    // Get data association
    assert(map_data_.pts.size() > 0);
    std::vector<Eigen::Vector3f> corr_map_associations;
    for(size_t i=0; i < sampled_points.size(); i++){
        // search nearest neighbor for sampled point in map
        float query_pt[2] = { static_cast<float>(sampled_points.at(i).x()), static_cast<float>(sampled_points.at(i).y())};

        const size_t num_results = 1;
        size_t ret_index;
        float out_dist_sqr;
        nanoflann::KNNResultSet<float> resultSet(num_results);
        resultSet.init(&ret_index, &out_dist_sqr );
        map_index_.findNeighbors(resultSet, &query_pt[0], nanoflann::SearchParams(10));

        // Gather map vector
        Point_KD<float> corr_p = map_data_.pts.at(ret_index);
        corr_map_associations.push_back(Eigen::Vector3f((corr_p.x), (corr_p.y), 0.0));
    }
    return corr_map_associations;
}

/*
 * For given observed signs return nearest sign positions with the same type.
 * This is using a KD-Tree for efficient match retrieval.
 * Returns a empty vector if not possible
 */
std::vector<Eigen::Vector3f> sensor_model::getNearestPoints(std::vector<Sign> observed_signs){
    // Get data association
    std::vector<Eigen::Vector3f> corr_map_associations;
    for(size_t i=0; i < observed_signs.size(); i++){
        const Sign& s = observed_signs.at(i);
        if(sign_indeces_.find(s.type) != sign_indeces_.end()){
            // search nearest neighbor for sampled point in map
            float query_pt[2] = {s.position[0], s.position[1]};

            const size_t num_results = 1;
            size_t ret_index;
            float out_dist_sqr;
            nanoflann::KNNResultSet<float> resultSet(num_results);
            resultSet.init(&ret_index, &out_dist_sqr );
            sign_indeces_[s.type]->findNeighbors(resultSet, &query_pt[0], nanoflann::SearchParams(10));

            // Gather sign position from map
            if(out_dist_sqr < 1e30){
                Point_KD<float> corr_p = sign_data_[s.type].pts.at(ret_index);
                corr_map_associations.push_back(Eigen::Vector3f((corr_p.x), (corr_p.y), 0.0));
            }else{
                std::cerr << "Invalid query..." << std::endl;
                return std::vector<Eigen::Vector3f>();
            }
        }else{
            std::cerr << "No corrensponding sign in map kd indeces..." << std::endl;
            return std::vector<Eigen::Vector3f>();
        }
    }
    return corr_map_associations;
}

/*
 * Transforms a given list of 3D points by a given affine transformation matrix
 */
std::vector<Eigen::Vector3f> sensor_model::transformPoints(const std::vector<Eigen::Vector3f> points, const Eigen::Transform<float,3,Eigen::Affine> transform){
    std::vector<Eigen::Vector3f> transformed;
    for(size_t i =0; i < points.size(); i++){
        Eigen::Vector3f p_world = transform * points.at(i);
        transformed.push_back(p_world);
    }
    return transformed;
}

/*
 * Returns sum of given float-vector
 */
float sensor_model::sumWeights(const std::vector<float>& weights){
    float sum = 0.0f;
    for(auto i = weights.begin(); i != weights.end(); i++){
        sum += *i;
    }
    return sum;
}

float probability_density_function(float mean, float stddev, float pos){
    float variance = stddev * stddev;
    return 1./(sqrt(2.*M_PI*variance)) * exp(-((std::pow(pos-mean, 2.))/(2.*variance)));
}

float sensor_model::SignMeasurementPoseProbabilityNearest(const std::vector<Sign>& observed_signs, const std::vector<Eigen::Vector3f>& data_associations_signs){
    size_t N = observed_signs.size();

    float error_signs = 0.0f;
    // Observed Signs #####################################
    for(size_t i=0; i < observed_signs.size(); i++){
        const Eigen::Vector3f& p = observed_signs.at(i).position;
        const Eigen::Vector3f& q = data_associations_signs.at(i);
        error_signs +=  std::sqrt((q-p).squaredNorm());
    }

    error_signs /= static_cast<float>(N);
    return probability_density_function(0, 0.4, error_signs);
//    return std::max(std::min(double(exp(-error_signs)), 1.0), 0.1);
}

float sensor_model::SignMeasurementPoseProbability(const std::vector<Sign>& observed_signs_map, Eigen::Transform<float,3,Eigen::Affine> particle_transform){
    float prob = 0.0f;
    float norm_val = 0;
    // Observed Signs #####################################
    for(size_t i=0; i < observed_signs_map.size(); i++){
        for (size_t k=0; k < sign_data_[observed_signs_map.at(i).type].pts.size(); k++) {

            Point_KD<float> corr_p = sign_data_[observed_signs_map.at(i).type].pts.at(k);
            const Eigen::Vector3f &p = observed_signs_map.at(i).position;
            const Eigen::Vector3f &q = Eigen::Vector3f(corr_p.x, corr_p.y, 0.);

            Eigen::Vector3f q_in_p_pose = particle_transform.inverse() * q;
            float angle = abs(atan2(q_in_p_pose.y(), q_in_p_pose.x()) * (180.0 / M_PI));
            if (angle < 85. / 2) {   // This condition is essentially a prior (setting those priors to
                                     // 0 that do not fall in the condition)
                float error_sign = std::sqrt((q - p).squaredNorm());
                // exp(-error_sign) is another prior that says the further the signs are away, the less likely they get
                // detected
                prob += exp(-error_sign) * probability_density_function(0, 0.4, error_sign);
                norm_val += exp(-error_sign);
            }
        }
    }
    if(norm_val > 0) {
        prob /= norm_val;
    }

    return prob;
}

float sensor_model::LaneMeasurementPoseProbability(const std::vector<Eigen::Vector3f>& observed_points, const std::vector<Eigen::Vector3f>& data_associations, const std::vector<float>& weights, const float total_weight){

    float prob = 0.0f;
    float norm_val = 0.0f;
    // Observed Lane Points ###############################
    for(size_t i=0; i < data_associations.size(); i++){
        const float weight = weights.at(i);
        const Eigen::Vector3f& p = observed_points.at(i);
        const Eigen::Vector3f& q = data_associations.at(i);
        float error = std::sqrt((q-p).squaredNorm());
        prob += weight * probability_density_function(0, 0.2, error);
        norm_val += weight;
    }

    prob /= norm_val;
    return prob;
}

/*
 * Transforms sign position by a given affine transformation matrix
 */
std::vector<Sign> sensor_model::transformSigns(const std::vector<Sign>& signs, const Eigen::Transform<float,3,Eigen::Affine>& particle_pose){
    std::vector<Sign> transformed_signs;
    for(size_t i =0; i < signs.size(); i++){
        const Sign& s = signs.at(i);
        Sign t_s = s;
        t_s.position = particle_pose * s.position;
        transformed_signs.push_back(t_s);
    }
    return transformed_signs;
}



/*
 * ##########################IMPLEMENT ME###############################################################################
 * Sensor-model. This function does the following:
 * --Calculates the likelihood of every particle being at its respective pose.
 * The likelihood should be stored in the particles weight member variable
 * The observed_signs variable contain all observed signs at the current timestep. They are relative to freicar_X/base_link.
 * The current particles are given with the variable "particles"
 * The true positions of all signs for a given type are stored in: sign_data_[observed_signs.at(i).type].pts , where
 * observed_signs.at(i).type is the sign_type of the i'th observed sign and pts is a array of positions (that have
 * the member x and y)
 * For lane regression data: The function getNearestPoints() might come in handy for getting the closest points to the
 * sampled and observed lane center points.
 *
 * The variable max_prob must be filled with the highest likelihood among all particles. If the average
 * of the last BEST_PARTICLE_HISTORY (defined in particle_filter.h) max_prob values is under the value
 * QUALITY_RELOC_THRESH (defined in particle_filter.h) a resampling will be initiated. So you may want to adjust the threshold.
 *
 * The function needs to return True if everything was successfull and False otherwise.

 */
/*/
bool sensor_model::calculatePoseProbability(const std::vector<cv::Mat> lane_regression, const std::vector<Sign> observed_signs, std::vector<Particle>& particles, float& max_prob){
    // Check if there is a bev lane regression matrix available. If so, use it in the observation step
    Particle best_particle;
    max_prob = 0; // Dummy for compilation
    bool success = false; // Dummy for compilation
    // Hint: The following code line can be used for transforming the sign positions using the particles pose.
    // const std::vector<Sign> world_signs = transformSigns(observed_signs, particles[0].transform);
    // ############################################################## implementation #######################
    weights.clear();
    float temp_weight = 0;
    for (int i_p = 0; i_p < particles.size(); i_p++) {
        success = true;
        const std::vector<Sign> world_signs = transformSigns(observed_signs, particles[i_p].transform);

        for (int j_o = 0; j_o < world_signs.size(); j_o++) {
            float prob = SignMeasurementPoseProbability(world_signs, particles[i_p].transform);
            particles[i_p].weight = prob;
            temp_weight += prob;

            if (prob > max_prob){
                max_prob = prob;
                best_particle = particles[i_p];
            }

            weights.push_back(prob);
        }

        }
    //weight normalization

    float tot_prob = sumWeights(weights);
    for (int w=0; w < particles.size(); w++){
        particles[w].weight = particles[w].weight/tot_prob;
    }

    return success;
}
/*/
bool sensor_model::calculatePoseProbability(const std::vector<cv::Mat> lane_regression, const std::vector<Sign> observed_signs, std::vector<Particle>& particles, float& max_prob){
    // Check if there is a bev lane regression matrix available. If so, use it in the observation step
    if(use_lane_reg_){
        Particle best_particle;
        max_prob = 0; // Dummy for compilation
        bool success = false; // Dummy for compilation
// Hint: The following code line can be used for transforming the sign positions using the particles pose.
// const std::vector<Sign> world_signs = transformSigns(observed_signs, particles[0].transform);
// ############################################################## implementation #######################
        weights.clear();

        std::vector<Eigen::Vector3f> center_points;
        int counter_id = 0;

        std::vector<float> reg_weights;

        for (int i=0; i<600; i++)
        {
            for (int j=0; j<600; j++)
            {
                if (lane_regression.at(0).at<float>(i,j) > 170)
                {
                    center_points.push_back(Eigen::Vector3f(i, j, 0.0));
                    reg_weights.push_back(lane_regression.at(0).at<float>(i,j)/255);
//                    std::cout <<"lane regression"<< lane_regression.at(0).at<float>(i,j)<< std::endl;

                }
            }
        }



        float tot_prob2 = sumWeights(weights);
        for (int w=0; w < particles.size(); w++){
            particles[w].weight = particles[w].weight/tot_prob2;
        }

        float temp_weight = 0;
        for (int i_p = 0; i_p < particles.size(); i_p++) {
            success = true;

            const std::vector<Sign> world_signs = transformSigns(observed_signs, particles[i_p].transform);
            float prob = SignMeasurementPoseProbability(world_signs, particles[i_p].transform);
            std::vector<Eigen::Vector3f> points_trans = transformPoints(center_points,particles[i_p].transform);
            std::vector<Eigen::Vector3f> corr_center_points = getNearestPoints(points_trans);
            float prob_lane1 = LaneMeasurementPoseProbability(points_trans,  corr_center_points , reg_weights , tot_prob2);
            if (abs(prob_lane1+prob)<=prob )
            {
                particles[i_p].weight = prob;
                temp_weight += prob;
                weights.push_back(prob);
                std::cout << "Using only signs"<< std::endl;
            }
            else
            {
                particles[i_p].weight = (prob+prob_lane1)/2;
                temp_weight += (prob+prob_lane1)/2;
                weights.push_back((prob+prob_lane1)/2);
                std::cout << "Using Both"<< std::endl;
            }

//        particles[i_p].weight = prob;
//        temp_weight += prob;

            if (prob > max_prob){
                max_prob = prob;
                best_particle = particles[i_p];
            }

//        weights.push_back(prob);

//        }

        }



        return success;
    }


    else{
        Particle best_particle;
        max_prob = 0; // Dummy for compilation
        bool success = false; // Dummy for compilation

        weights.clear();

        int counter_id = 0;



        float tot_prob2 = sumWeights(weights);
        for (int w=0; w < particles.size(); w++){
            particles[w].weight = particles[w].weight/tot_prob2;
        }

        float temp_weight = 0;
        const float no_need = 0;

        for (int i_p = 0; i_p < particles.size(); i_p++) {
            success = true;

            const std::vector<Sign> world_signs = transformSigns(observed_signs, particles[i_p].transform);
            float prob = SignMeasurementPoseProbability(world_signs, particles[i_p].transform);

            particles[i_p].weight = prob;
            temp_weight += prob;
            weights.push_back(prob);

            if (prob > max_prob){
                max_prob = prob;
                best_particle = particles[i_p];
            }

        }



        return success;
    }
}
