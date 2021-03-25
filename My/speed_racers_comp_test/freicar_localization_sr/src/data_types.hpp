/*
 * Author: Johan Vertens (vertensj@informatik.uni-freiburg.de)
 * Project: FreiCAR
 * Do NOT distribute this code to anyone outside the FreiCAR project
 */

#ifndef TREE_DATA_TYPES_HPP
#define TREE_DATA_TYPES_HPP

#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <atomic>

template <typename T>
struct Point_KD
{
    T  x,y;
};

template <typename T>
struct PointCloud
{
    std::vector<Point_KD<T>>  pts;

    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return pts.size(); }

    // Returns the dim'th component of the idx'th point in the class:
    // Since this is inlined and the "dim" argument is typically an immediate value, the
    //  "if/else's" are actually solved at compile time.
    inline T kdtree_get_pt(const size_t idx, const size_t dim) const
    {
        if (dim == 0) return pts[idx].x;
        else return pts[idx].y;
    }

    // Optional bounding-box computation: return false to default to a standard bbox computation loop.
    //   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
    //   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }
};

class Particle{
public:
    Particle(){
        weight = 1.0f;
        transform = Eigen::Transform<float,3,Eigen::Affine>::Identity();
    }

    Particle(const Particle& p){
        weight = p.weight.load();
        transform = p.transform;
    }

    Particle& operator=(const Particle& p){
        this->transform = p.transform;
        this->weight = p.weight.load();
        return *this;
    }

    Eigen::Transform<float,3,Eigen::Affine> transform;
    std::atomic<float> weight;
};

class Sign{
public:
    Sign(){
        id = -1;
        type = "";
    }

    int id;
    std::string type;
    Eigen::Vector3f position;
};

#endif // TREE_DATA_TYPES_HPP
