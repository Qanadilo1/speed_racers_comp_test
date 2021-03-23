/*
 * Author: Johan Vertens (vertensj@informatik.uni-freiburg.de)
 * Project: FreiCAR
 * Do NOT distribute this code to anyone outside the FreiCAR project
 */

/* A ROS implementation of the Pure pursuit path tracking algorithm (Coulter 1992).
   Terminology (mostly :) follows:
   Coulter, Implementation of the pure pursuit algoritm, 1992 and
   Sorniotti et al. Path tracking for Automated Driving, 2017.
 */

#include <string>
#include <cmath>
#include <algorithm>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/transform_datatypes.h>
#include <tf2/transform_storage.h>
#include <tf2/buffer_core.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <kdl/frames.hpp>
#include <raiscar_msgs/ControlReport.h>
#include "raiscar_msgs/ControlCommand.h"
#include "std_msgs/Bool.h"
#include "controller.h"
#include <fstream>
#include <sstream>
#include <iostream>

#include <kdl/frames.hpp>




int main(int argc, char**argv)
{
    ros::init(argc, argv, "test_cpp");
    std::vector<double> segment_end = {3, 0};
    std::vector<double> segment_start = {0, 3};
    std::vector<double> robot_pose = {3.0,2.0};

    std::cout << "segment_end = " << segment_end[0] <<   std::endl;
    std::cout << "segment_end = " << segment_end[1] <<   std::endl;

    std::cout << "segment_start = " << segment_start[0] <<std::endl;
    std::cout << "segment_start = " << segment_start[1] <<std::endl;

//
    std::vector<double> p1 = {segment_start[0] - robot_pose[0] ,
                              segment_start[1] - robot_pose[1]};
    std::vector<double> p2 = {segment_end[0] - robot_pose[0],
                              segment_end[1] - robot_pose[1]};


    std::cout << "p1 = " << p1[0] <<   std::endl;
    std::cout << "p1 = " << p1[1] <<   std::endl;

    std::cout << "p2 = " << p2[0] <<   std::endl;
    std::cout << "p2 = " << p2[1] <<   std::endl;



    double dx = p2[0] - p1[0];
    double dy = p2[1] - p1[1];

    std::cout << "dx = " << dx <<   std::endl;
    std::cout << "dy = " << dy <<   std::endl;

    double d = (double) sqrt(dx * dx + dy * dy);
    double D = p1[0] * p2[1] - p2[0] * p1[1];

    std::cout << "d = " << d <<   std::endl;
    std::cout << "D = " << D <<   std::endl;

    double ld_dist_ = 2.0;
    double discriminant =  abs(ld_dist_ * ld_dist_ * d * d - D * D);

    std::cout << "discriminat = " << discriminant <<   std::endl;


    double dy2 = 0;
    if (dy > 0)
    {
        dy2 = 1;

    }
    else if (dy < 0)
    {
        dy2 = -1;
    }
    else
        dy2 = 1;

    std::cout << "dy = " << dy <<   std::endl;

    double x1 = (double) (D * dy + dy2 * dx * sqrt(discriminant)) / (d * d);
    double x2 = (double) (D * dy - dy2 * dx * sqrt(discriminant)) / (d * d);
    // the y components of the intersecting points
    double y1 = (double) (-D * dx + abs(dy) * sqrt(discriminant)) / (d * d);
    double y2 = (double) (-D * dx - abs(dy) * sqrt(discriminant)) / (d * d);
    std::cout << "x1 = " << x1  << "or  " <<  x2<<   std::endl;
    std::cout << "x1 = " << y1  << "or  " <<  y2<<   std::endl;



    // whether each of the intersections are within the segment (and not the entire line)
    bool validIntersection1 = ( std::min(p1[0], p2[0]) < x1 && x1 < std::max(p1[0], p2[0]) )
                              ||( std::min(p1[1], p2[1]) < y1 && y1 < std::max(p1[1], p2[1]));
    bool validIntersection2 = (std::min(p1[0], p2[0]) < x2 && x2 < std::max(p1[0], p2[0]))
                              || (std::min(p1[1], p2[1]) < y2 && y2 < std::max(p1[1], p2[1]));

    std::cout << "intersection 1 = " << validIntersection1 <<   std::endl;

    std::cout << "intersection 2 = " << validIntersection2 <<   std::endl;




    std::vector<double> gxgy = {0.0,0.0};

    if (validIntersection1) {
        gxgy[0] = x1 + robot_pose[0];
        gxgy[1] = y1 + robot_pose[1];
    }
//    if (validIntersection2) {
//        if (gxgy.empty() || abs(x1 - p2[0]) > abs(x2 - p2[0]) || abs(y1 - p2[1]) > abs(y2 - p2[1])) {
//            gxgy[0] = x2 + robot_pose[0];
//            gxgy[1] = y2 + robot_pose[1];
//        }
//    }

    std::cout << "gx= " << gxgy[0] <<   std::endl;

    std::cout << "gy = " << gxgy[1] <<   std::endl;

//    std::cout << "Valid Intersection 1 =  " << validIntersection1 <<  std::endl;
//    std::cout << "Valid Intersection 1 =  " << validIntersection1 <<  std::endl;
//
//    std::cout << "gx = " << gxgy[0] << "  Should be 1" <<   std::endl;
//    std::cout << "gy = " << gxgy[1] <<  "  Should be 2  " <<std::endl;






    return 0;
}
