/*
 * Author: Johan Vertens (vertensj@informatik.uni-freiburg.de)
 * Project: FreiCAR
 * Do NOT distribute this code to anyone outside the FreiCAR project
 */

#include <map_core/freicar_map.h>
#include "map_helper.h"

namespace map_helper{

Eigen::Vector4f getMaxesMap(freicar::map::Map* map_data){
    float min_x = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::min();
    float min_y = std::numeric_limits<float>::max();
    float max_y = std::numeric_limits<float>::min();

    freicar::mapobjects::Pivot pivot = map_data->pivot();

    if(map_data->status() != freicar::map::MapStatus::UNINITIALIZED){
        freicar::mapobjects::Pivot pivot = map_data->pivot();
        std::vector<freicar::mapobjects::Lane> lane_storage = map_data->getLanes();
        for(auto i=0; i< lane_storage.size(); i++){
            const auto points = lane_storage.at(i).GetPoints();
            for(auto p_i=0; p_i<points.size(); p_i++){
                freicar::mapobjects::Point3D p = points.at(p_i);
                double map_x = (p.x() - pivot.GetPose().x());
                double map_y = (p.y() - pivot.GetPose().y());

                if(map_x < min_x){
                    min_x = map_x;
                }

                if(map_x > max_x){
                    max_x = map_x;
                }

                if(map_y < min_y){
                    min_y = map_y;
                }

                if(map_y > max_y){
                    max_y = map_y;
                }

            }

        }

    }else{
        std::cerr << "Map seems not to be initialized..." << std::endl;
    }
    Eigen::Vector4f out_bb(min_x, max_x, min_y, max_y);
    return out_bb;
}

double getLaneLenght(freicar::mapobjects::Lane lane){
    const auto points = lane.GetPoints();
    freicar::mapobjects::Point3D p_start = points.at(0);
    Eigen::Vector3d vec_start(p_start.x(), p_start.y(), p_start.z());

    double dist = 0.0;
    if(points.size() > 1){

        for(size_t i=1; i < points.size(); i++){
            const freicar::mapobjects::Point3D p = points.at(i);
            Eigen::Vector3d v(p.x(), p.y(), p.z());
            dist += std::sqrt((v - vec_start).squaredNorm());
            vec_start = v;
        }
    }else{
        std::cerr << "Lane has less than 2 points... Can't calculate lenght" << std::endl;
    }

    return dist;
}

Eigen::Vector3d interpolateLane(freicar::mapobjects::Lane lane, const double offset){
    auto points = lane.GetPoints();
    freicar::mapobjects::Point3D p_start = points.at(0);
    Eigen::Vector3d vec_start(p_start.x(), p_start.y(), p_start.z());

    Eigen::Vector3d inter_p;

    double dist = 0.0;
    if(points.size() > 1){

        for(size_t i=1; i < points.size(); i++){
            freicar::mapobjects::Point3D p = points.at(i);
            Eigen::Vector3d v(p.x(), p.y(), p.z());
            double n_dist = dist + std::sqrt((v - vec_start).squaredNorm());

            if(n_dist > offset){
                // interpolate point
                double togo = offset - dist;
                Eigen::Vector3d norm_dir = (v - vec_start).normalized();
                inter_p = vec_start + togo * norm_dir;
                return inter_p;
            }else{
                dist = n_dist;
            }

            vec_start = v;
        }
    }else{
        std::cerr << "Lane has less than 2 points... Can't calculate interpolated point" << std::endl;
    }

    std::cerr << "Offset is beyond length of lane..." << std::endl;
    return inter_p;
}

std::vector<Eigen::Vector3d> discretizeLane(freicar::mapobjects::Lane lane, const double discretize_step){
    std::vector<Eigen::Vector3d> out_points;
    auto points = lane.GetPoints();
    double lane_length = getLaneLenght(lane);

    if(points.empty()){
        std::cerr << "DiscretizeLane: Points empty..." << std::endl;
        return out_points;
    }

    if(points.size() == 1){
        freicar::mapobjects::Point3D p = points.at(0);
        out_points.push_back(Eigen::Vector3d(p.x(), p.y(), p.z()));
    }else{
        for(double current_dist = 0.0; current_dist < lane_length; current_dist+=discretize_step){
            Eigen::Vector3d p = interpolateLane(lane, current_dist);
            out_points.push_back(p);
        }
        // Add last point of lane
        freicar::mapobjects::Point3D last_p = points.back();
        out_points.push_back(Eigen::Vector3d(last_p.x(), last_p.y(), last_p.z()));
    }

    return out_points;
}
}
