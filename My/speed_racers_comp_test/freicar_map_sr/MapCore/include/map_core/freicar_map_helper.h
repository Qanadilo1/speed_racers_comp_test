#ifndef MAP_HELPER_H
#define MAP_HELPER_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "map_core/freicar_map.h"

namespace freicar
{
namespace helper
{
float NormalizedDot(const mapobjects::Point3D &base, const mapobjects::Point3D &p1, const mapobjects::Point3D &p2);

} // namespace map_helper
} // namespace freicar

#endif // MAP_HELPER_H