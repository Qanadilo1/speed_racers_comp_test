/*
 * Author: Johan Vertens (vertensj@informatik.uni-freiburg.de)
 * Project: FreiCAR
 * Do NOT distribute this code to anyone outside the FreiCAR project
 */

#ifndef MAP_HELPER_H
#define MAP_HELPER_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <freicar_map/planning/lane_star.h>
#include <freicar_map/logic/right_of_way.h>

namespace map_helper{

Eigen::Vector4f getMaxesMap(freicar::map::Map* map_data);
double getLaneLenght(freicar::mapobjects::Lane lane);
Eigen::Vector3d interpolateLane(freicar::mapobjects::Lane lane, const double offset);
std::vector<Eigen::Vector3d> discretizeLane(freicar::mapobjects::Lane lane, const double discretize_step);

}

#endif // MAP_HELPER_H
