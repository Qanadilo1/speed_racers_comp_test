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
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"

#include <kdl/frames.hpp>



std::vector<double> find_inter(std::vector<tf2::Transform> path, tf2::Stamped<tf2::Transform> robot_pose)
{
    int ind = 0;
    std::vector<double> gxgy = {0.0,0.0};
    for (; ind<path.size() -1;ind++) {
        std::vector<double> segment_start = {path[ind].getOrigin().x(), path[ind].getOrigin().y()};
        std::vector<double> segment_end = {path[ind + 1].getOrigin().x(), path[ind + 1].getOrigin().y()};

        std::vector<double> p1 = {segment_start[0] - robot_pose.getOrigin().x(),
                                  segment_start[1] - robot_pose.getOrigin().y()};
        std::vector<double> p2 = {segment_end[0] - robot_pose.getOrigin().x(),
                                  segment_end[1] - robot_pose.getOrigin().y()};

        double dx = p2[0] - p1[0];
        double dy = p2[1] - p1[1];

        double d = (double) sqrt(dx * dx + dy * dy);
        double D = p1[0] * p2[1] - p2[0] * p1[1];

        double ld_dist_ = 0.7;
        double discriminant = abs(ld_dist_ * ld_dist_ * d * d - D * D);


        double dy2 = 0;
        if (dy > 0) {
            dy2 = 1;

        } else if (dy < 0) {
            dy2 = -1;
        } else
            dy2 = 1;


        double x1 = (double) (D * dy + dy2 * dx * sqrt(discriminant)) / (d * d);
        double x2 = (double) (D * dy - dy2 * dx * sqrt(discriminant)) / (d * d);
        // the y components of the intersecting points
        double y1 = (double) (-D * dx + abs(dy) * sqrt(discriminant)) / (d * d);
        double y2 = (double) (-D * dx - abs(dy) * sqrt(discriminant)) / (d * d);




        // whether each of the intersections are within the segment (and not the entire line)
        bool validIntersection1 = (std::min(p1[0], p2[0]) < x1 && x1 < std::max(p1[0], p2[0]))
                                  || (std::min(p1[1], p2[1]) < y1 && y1 < std::max(p1[1], p2[1]));
        bool validIntersection2 = (std::min(p1[0], p2[0]) < x2 && x2 < std::max(p1[0], p2[0]))
                                  || (std::min(p1[1], p2[1]) < y2 && y2 < std::max(p1[1], p2[1]));

        std::cout << "intersection 1 = " << validIntersection1 << std::endl;

        std::cout << "intersection 2 = " << validIntersection2 << std::endl;


        if (validIntersection1) {
            gxgy[0] = x1 + robot_pose.getOrigin().x();
            gxgy[1] = y1 + robot_pose.getOrigin().y();
        }
        if (validIntersection1 || validIntersection2) {
            ind = ind + 1;
            break;
        }

    }

    std::cout<<"gx"<<gxgy[0]<<std::endl;
    std::cout<<"gy"<<gxgy[1]<<std::endl;
    std::cout<<"index"<<ind<<std::endl;
    double fuckyou = 1.0 * ind;
    std::vector<double> sol = {gxgy[0],gxgy[1], fuckyou};
    return sol;
}

std::vector<double> ftest(int a )
{
    std::vector<double> qwe = {0,0};
    return qwe;
}

int main(int argc, char**argv)
{

    int i = 0;
    int b=0;
    ftest(b);




    std::list<double> path;
//    path.push_back(0.43812255541599776, 0.8186631601507567)

    std::list<std::vector<std::vector<int>>> mylist;
//    mylist.push_back([0.43812255541599776, 0.8186631601507567])







    ros::init(argc, argv, "test_path");

    visualization_msgs::Marker marker1;
    marker1.pose.position.x = 1.5;
    marker1.pose.position.y = 0.0;

    visualization_msgs::Marker marker2;
    marker2.pose.position.x = 2.2245700359344482;
    marker2.pose.position.y = 0.26467326283454895;

    visualization_msgs::Marker marker3;
    marker3.pose.position.x = 3.2243354320526123;
    marker3.pose.position.y = 0.28548383712768555;

    visualization_msgs::Marker marker4;
    marker4.pose.position.x = 4.224156379699707;
    marker4.pose.position.y = 0.2953287959098816;


    visualization_msgs::MarkerArray markerArray;
    markerArray.markers.push_back(marker1);
    markerArray.markers.push_back(marker2);
    markerArray.markers.push_back(marker3);

//    std::vector<double> segment_end = {3, 0};
//    std::vector<double> segment_start = {0, 3};
    std::vector<double> robot_pose = {1.5,0.0};
//
//    std::vector<double> path1 = {1.5, 0.0};
//    std::vector<double> path3 = {, };
//    std::vector<double> path4 = {, };
//    std::vector<double> path5 = {5.222776412963867, 0.3283778727054596};
//    std::vector<double> path6 = {6.1570329666137695, 0.653523862361908};
//    std::vector<double> path7 = {6.7193145751953125, 1.4582167863845825};
//    std::vector<double> path8 = {6.728407382965088, 2.4463889598846436};
//    std::vector<double> path9 = {6.2758564949035645, 3.3265469074249268};
//    std::vector<double> path10 = {5.39455509185791, 3.765422821044922};
//
//    std::list<std::vector<double>> list_p ;
//    std::list<std::vector<double>>::iterator iter = list_p.begin();
//    list_p.push_back(path1);
//    list_p.push_back(path2);
//    list_p.push_back(path3);
//    list_p.push_back(path4);
//    list_p.push_back(path5);
//    list_p.push_back(path6);
//    list_p.push_back(path7);
//    list_p.push_back(path8);
//    list_p.push_back(path9);
//    list_p.push_back(path10);





    std::vector<double> gxgy = {0.0,0.0};


    for (; i<markerArray.markers.size() -1;i++) {
        std::vector<double> segment_end = {markerArray.markers.at(i).pose.position.x,
                                           markerArray.markers.at(i).pose.position.y};
        std::vector<double> segment_start = {markerArray.markers.at(i + 1).pose.position.x,
                                             markerArray.markers.at(i + 1).pose.position.y};

        std::vector<double> p1 = {segment_start[0] - robot_pose[0],
                                  segment_start[1] - robot_pose[1]};
        std::vector<double> p2 = {segment_end[0] - robot_pose[0],
                                  segment_end[1] - robot_pose[1]};

        double dx = p2[0] - p1[0];
        double dy = p2[1] - p1[1];

        double d = (double) sqrt(dx * dx + dy * dy);
        double D = p1[0] * p2[1] - p2[0] * p1[1];

        double ld_dist_ = 0.7;
        double discriminant = abs(ld_dist_ * ld_dist_ * d * d - D * D);


        double dy2 = 0;
        if (dy > 0) {
            dy2 = 1;

        } else if (dy < 0) {
            dy2 = -1;
        } else
            dy2 = 1;


        double x1 = (double) (D * dy + dy2 * dx * sqrt(discriminant)) / (d * d);
        double x2 = (double) (D * dy - dy2 * dx * sqrt(discriminant)) / (d * d);
        // the y components of the intersecting points
        double y1 = (double) (-D * dx + abs(dy) * sqrt(discriminant)) / (d * d);
        double y2 = (double) (-D * dx - abs(dy) * sqrt(discriminant)) / (d * d);




        // whether each of the intersections are within the segment (and not the entire line)
        bool validIntersection1 = (std::min(p1[0], p2[0]) < x1 && x1 < std::max(p1[0], p2[0]))
                                  || (std::min(p1[1], p2[1]) < y1 && y1 < std::max(p1[1], p2[1]));
        bool validIntersection2 = (std::min(p1[0], p2[0]) < x2 && x2 < std::max(p1[0], p2[0]))
                                  || (std::min(p1[1], p2[1]) < y2 && y2 < std::max(p1[1], p2[1]));

        std::cout << "intersection 1 = " << validIntersection1 << std::endl;

        std::cout << "intersection 2 = " << validIntersection2 << std::endl;


        if (validIntersection1) {
            gxgy[0] = x1 + robot_pose[0];
            gxgy[1] = y1 + robot_pose[1];
        }
        if (validIntersection1 || validIntersection2) {
            i = i + 1;
            break;
        }

    }

    std::cout<<"gx"<<gxgy[0]<<std::endl;
    std::cout<<"gy"<<gxgy[1]<<std::endl;
    std::cout<<"index"<<i<<std::endl;



//
//
//    std::cout << "p1 = " << p1[0] <<   std::endl;
//    std::cout << "p1 = " << p1[1] <<   std::endl;
//
//    std::cout << "p2 = " << p2[0] <<   std::endl;
//    std::cout << "p2 = " << p2[1] <<   std::endl;





//    std::cout << "dx = " << dx <<   std::endl;
//    std::cout << "dy = " << dy <<   std::endl;
//
//    double d = (double) sqrt(dx * dx + dy * dy);
//    double D = p1[0] * p2[1] - p2[0] * p1[1];
//
//    std::cout << "d = " << d <<   std::endl;
//    std::cout << "D = " << D <<   std::endl;
//
//    double ld_dist_ = 2.0;
//    double discriminant =  abs(ld_dist_ * ld_dist_ * d * d - D * D);
//
//    std::cout << "discriminat = " << discriminant <<   std::endl;
//
//
//    double dy2 = 0;
//    if (dy > 0)
//    {
//        dy2 = 1;
//
//    }
//    else if (dy < 0)
//    {
//        dy2 = -1;
//    }
//    else
//        dy2 = 1;
//
//    std::cout << "dy = " << dy <<   std::endl;
//
//    double x1 = (double) (D * dy + dy2 * dx * sqrt(discriminant)) / (d * d);
//    double x2 = (double) (D * dy - dy2 * dx * sqrt(discriminant)) / (d * d);
//    // the y components of the intersecting points
//    double y1 = (double) (-D * dx + abs(dy) * sqrt(discriminant)) / (d * d);
//    double y2 = (double) (-D * dx - abs(dy) * sqrt(discriminant)) / (d * d);
//    std::cout << "x1 = " << x1  << "or  " <<  x2<<   std::endl;
//    std::cout << "x1 = " << y1  << "or  " <<  y2<<   std::endl;
//
//
//
//    // whether each of the intersections are within the segment (and not the entire line)
//    bool validIntersection1 = ( std::min(p1[0], p2[0]) < x1 && x1 < std::max(p1[0], p2[0]) )
//                              ||( std::min(p1[1], p2[1]) < y1 && y1 < std::max(p1[1], p2[1]));
//    bool validIntersection2 = (std::min(p1[0], p2[0]) < x2 && x2 < std::max(p1[0], p2[0]))
//                              || (std::min(p1[1], p2[1]) < y2 && y2 < std::max(p1[1], p2[1]));
//
//    std::cout << "intersection 1 = " << validIntersection1 <<   std::endl;
//
//    std::cout << "intersection 2 = " << validIntersection2 <<   std::endl;
//
//
//
//
//    std::vector<double> gxgy = {0.0,0.0};
//
//    if (validIntersection1) {
//        gxgy[0] = x1 + robot_pose[0];
//        gxgy[1] = y1 + robot_pose[1];
//    }
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
