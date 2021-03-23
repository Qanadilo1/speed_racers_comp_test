#include "freicar_map/thrift_map_proxy.h"
#include "freicar_map/planning/lane_follower.h"
#include "freicar_map/planning/lane_star.h"
#include "yaml-cpp/yaml.h"

//YAML::Node config = YAML::LoadFile("freicar_2_spawn.yaml");
//Added by us
#include "freicar_map/logic/right_of_way.h"
//Added by us

#include "map_core/freicar_map_helper.h"
#include "map_core/freicar_map_config.h"

#include "freicar_common/shared/planner_cmd.h"
#include "freicar_common/WayPoint.h"

#include <ros/package.h>
#include <ros/ros.h>
#include <cstdio>
#include <thread>
#include <ctime>

#include <visualization_msgs/MarkerArray.h>
#include "std_msgs/Bool.h"
#include "yaml.h"


#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/transform_datatypes.h>
#include <tf2/transform_storage.h>
#include <tf2/buffer_core.h>
#include <tf2/convert.h>
#include <tf2/utils.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <freicar_common/FreiCarAgentLocalization.h>
#include <freicar_common/FreiCarControl.h>


#include <map_core/freicar_map.h>
#include <freicar_map/planning/lane_follower.h>
#include <raiscar_msgs/ControlCommand.h>

freicar_common::FreiCarAgentLocalization p_current_row;
freicar_common::FreiCarAgentLocalization p_observed_agent;


//std::string yaml_path = ros::package::getPath("freicar_launch") + "/spawn_positions/" + map_name + "_spawn.yaml";
//YAML::Node base = YAML::LoadFile(yaml_path);

//tf2_ros::Buffer tf_buffer_;
#define DENSIFICATION_LIMIT 0.22 // meters
ros::Subscriber goal_reached;
std_msgs::Bool goal_bool;
std_msgs::Bool overtake;
freicar::mapobjects::Point3D p_current;
freicar::mapobjects::Lane *current_lane;
ros::Publisher snm_pub;
ros::Publisher jun_pub;
freicar_common::FreiCarControl HLC_msg;
bool HLC_bool;
bool junction_arrived;
//freicar::mapobjects::Uuid uuid_;
//float offset_;
//freicar::mapobjects::;  //velocity
auto& map_instance = freicar::map::Map::GetInstance();


int flag_1 = 1;

static geometry_msgs::Point ToGeometryPoint(const freicar::mapobjects::Point3D& pt) {
    geometry_msgs::Point rt;
    rt.x = pt.x();
    rt.y = pt.y();
    rt.z = pt.z();
    return rt;
}
/* WayPoint service request handler
   Edge cases: if the closest point to current_pos is at index i of lane L
		1) current_pos -> L[i] goes backward, L[i+1] goes forward, L goes on for a couple of steps
		2) current_pos -> L[i] goes backward, L ends at index i
*/
bool HandleWayPointRequest(freicar_common::WayPointRequest &req, freicar_common::WayPointResponse &resp) {
    auto command = static_cast<freicar::enums::PlannerCommand>(req.command);
    auto current_pos = freicar::mapobjects::Point3D(req.current_position.x, req.current_position.y, req.current_position.z);
    auto plan = freicar::planning::lane_follower::GetPlan(current_pos, command, req.distance,req.node_count);
    for (size_t i = 0; i < plan.steps.size(); ++i) {
        resp.points.emplace_back(ToGeometryPoint(plan.steps[i].position));
        resp.description.emplace_back(static_cast<unsigned char>(plan.steps[i].path_description));
    }
    return plan.success;
}
//void PublishStop ( , ros::Publisher& pub)
/* debugging function to publish plans (either from the lane_star or lane_follower planners) */
void PublishPlan (freicar::planning::Plan& plan, double r, double g, double b, int id, const std::string& name, ros::Publisher& pub) {
    visualization_msgs::MarkerArray list;
    visualization_msgs::Marker *step_number = new visualization_msgs::Marker[plan.size()];
    int num_count = 0;
    visualization_msgs::Marker plan_points;
    plan_points.id = id;
    plan_points.ns = name;
    plan_points.header.stamp = ros::Time();
    plan_points.header.frame_id = "map";
    plan_points.action = visualization_msgs::Marker::ADD;
    plan_points.type = visualization_msgs::Marker::POINTS;
    plan_points.scale.x = 0.03;
    plan_points.scale.y = 0.03;
    plan_points.pose.orientation = geometry_msgs::Quaternion();
    plan_points.color.b = b;
    plan_points.color.a = 0.7;
    plan_points.color.g = g;
    plan_points.color.r = r;
    geometry_msgs::Point p;
    for (size_t i = 0; i < plan.size(); ++i) {
        step_number[i].id = ++num_count + id;
        step_number[i].pose.position.x = p.x = plan[i].position.x();
        step_number[i].pose.position.y = p.y = plan[i].position.y();
        p.z = plan[i].position.z();
        step_number[i].pose.position.z = plan[i].position.z() + 0.1;

        step_number[i].pose.orientation = geometry_msgs::Quaternion();
        step_number[i].ns = name + "_nums";
        step_number[i].header.stamp = ros::Time();
        step_number[i].header.frame_id = "map";
        step_number[i].action = visualization_msgs::Marker::ADD;
        step_number[i].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        step_number[i].text = std::to_string(i);
        step_number[i].scale.z = 0.055;
        step_number[i].color = plan_points.color;
        list.markers.emplace_back(step_number[i]);
        plan_points.points.emplace_back(p);
    }
    list.markers.emplace_back(plan_points);
    pub.publish(list);
    delete[] step_number;
};

void callback_car_localization (freicar_common::FreiCarAgentLocalization msg)
{
//    return msg;
//
    p_current.SetX(msg.current_pose.transform.translation.x);
    p_current.SetY(msg.current_pose.transform.translation.y);
    p_current_row = msg;

//    auto p_closest = freicar::mapobjects::FindClosestLanePoints(p_current.x(),
//                                                        p_current.y(),
//                                                        p_current.z(),
//                                                        1)[0].first;
//    current_lane = msg.FindLaneByUuid(p_closest.GetLaneUuid());

}

//OVERTAKE CHECK
void callback_overtake (std_msgs:: Bool msg){
    if (msg.data == true)
    {
        overtake.data = true;
        //std::cout<<"overtake"<<std::endl;
    }
    else
    {
        //std::cout<<"not overtake"<<std::endl;
        overtake.data = false;
    }
}

void callback_HLC (freicar_common::FreiCarControl msg)
{
    HLC_msg.command = msg.command;
    HLC_msg.name = msg.name;
    HLC_bool = true;
}

void callback_goal_reached (std_msgs::Bool msg)
{
//    return msg;

    //       bool goal_bool2 = true;
    if (msg.data == true)
    {
        goal_bool.data = true;
//        std::cout<<"Goal REACHED"<<std::endl;
    }
    else
    {
//        std::cout<<"Goal is not REACHED"<<std::endl;
        goal_bool.data = false;
    }

}

int main(int argc, char **argv)
{


    ros::init(argc, argv, "map_framework_sr");
    std::shared_ptr<ros::NodeHandle> node_handle = std::make_shared<ros::NodeHandle>();
//    ros::NodeHandle n;
//    goal_reached=  n.subscribe("freicar_1/goal_reached", 1, callback_goal_reached);
    ROS_INFO("map framework node started...");

//    goal_reached = node_handle->subscribe("freicar_1/goal_reached", 1, callback_goal_reached,);




//    ros::Subscriber sub = node_handle->subscribe("freicar_1/goal_reached", 1, callback_goal_reached);


    // starting map proxy
    freicar::map::ThriftMapProxy map_proxy("127.0.0.1", 9091, 9090);
    bool new_map = false;
    //std::string filename = ros::package::getPath("freicar_map") + "/maps/thriftmap_fix.aismap";
    std::string filename;

    std::cout <<"print map: "<<std::endl;

    std::cout << ros::param::get("/map_path", filename)<<std::endl;

    if (!ros::param::get("/map_path", filename)) {
        ROS_ERROR("could not find parameter: map_path! map initialization failed.");
        return 0;
    }

    if (!map_proxy.LoadMapFromFile(filename)) {
        ROS_INFO("could not find thriftmap file: %s", filename.c_str());
        map_proxy.StartMapServer();
        // stalling main thread until map is received
        ROS_INFO("waiting for map...");
        while(freicar::map::Map::GetInstance().status() == freicar::map::MapStatus::UNINITIALIZED) {
            ros::Duration(1.0).sleep();
        }
        ROS_INFO("map received");
        new_map = true;
    }
    srand(300);
    // needed some delay before using the map
    ros::Duration(2.0).sleep();
    if (new_map) {
        // thrift creates a file on failed reads
        remove(filename.c_str());
        map_proxy.WriteMapToFile(filename);
        ROS_INFO("saved new map");
    }
    // NOTES:
    // 1) NEVER post process before saving. kinda obvious but still
    // 2) increasing densification limit past a certain point or turning it off
    //    WILL introduce bugs. 22cm seems to be balanced.
    // 	  Possible bug with densification disabled:
    // 		  2 closest lane point to (3.30898, 1.46423, 0) belong to a junction
    // 		  despite obviously belonging to d4c7ecc5-0aa9-49a8-8642-5f8ebb965592
    freicar::map::Map::GetInstance().PostProcess(DENSIFICATION_LIMIT);




    srand(time(NULL));
    using namespace freicar::planning::lane_star;
    ros::Publisher tf = node_handle->advertise<visualization_msgs::MarkerArray>("planner_debug", 10, true);

    using freicar::mapobjects::Point3D;
    LaneStar planner(100);
    // auto varz = freicar::map::Map::GetInstance().FindClosestLanePoints(0.0f, 0.0f, 0.0f, 2);
//     auto plan1 = planner.GetPlan(Point3D(1.95487, 3.73705, 0), -3.13996f, Point3D(3.3805, 0.721756, 0), -1.14473f, 0.30);

    planner.ResetContainers();
//     auto plan2 = planner.GetPlan(Point3D(3.3805, 0.721756, 0), -2.99003f, Point3D(6.10168, 1.2007, 0), -2.21812f, 0.30);

    using freicar::mapobjects::Point3D;
//    LaneStar planner(100);
    using freicar::enums::PlannerCommand;

//    auto plan2 = freicar::planning::lane_follower::GetPlan(Point3D(plan[plan.size()-1].position.x(), plan[plan.size()-1].position.y() , plan[plan.size()-1].position.z()), freicar::enums::STRAIGHT, 10,40);

    std::thread debug_thread([&]() {
        using namespace std::chrono_literals;
        while (ros::ok()) {

            ros::Subscriber sub3 = node_handle->subscribe("car_localization", 1, callback_car_localization);
            ros::Subscriber sub = node_handle->subscribe("freicar_1/goal_reached", 1, callback_goal_reached);
            ros::Subscriber HL_sub = node_handle->subscribe("/freicar_commands", 1, callback_HLC);
            ros::Subscriber sub_overtake = node_handle->subscribe("/overtake", 1, callback_overtake);

            auto &map = freicar::map::Map::GetInstance();


            //// TEST
            auto previous_lane_points = map.FindClosestLanePoints(9.2,
                                                                  -1.1,
                                                                  0,
                                                                  1)[0].first;
            const freicar::mapobjects::Lane *previous_lane;
            previous_lane = map.FindLaneByUuid(previous_lane_points.GetLaneUuid());



            //// TEST

            const freicar::mapobjects::Lane *current_lane;
            std::vector<freicar::mapobjects::Point3D> current_points;

            // if a new agent is spawning, get a random start position, go from there
            //p_current.SetCoords(p_current.x(), p_current.y(), p_current.z());

            auto p_closest = map.FindClosestLanePoints(p_current.x(),
                                                       p_current.y(),
                                                       p_current.z(),
                                                       1)[0].first;
            current_lane = map.FindLaneByUuid(p_closest.GetLaneUuid());

            unsigned char HLC_enum;

            if(HLC_msg.command == "start" )
            {
                HLC_enum = 3;
            }

            else if (HLC_msg.command == "straight")
            {
                HLC_enum = 3;
            }
            else if (HLC_msg.command == "left" )
            {
                HLC_enum = 1;
            }
            else if (HLC_msg.command == "right")
            {
                HLC_enum = 2;
            }
            else
            {
                HLC_enum = 3;
            }

            auto  stop_line = current_lane->GetStopLine();
//            auto  junction_line = current_lane->IsJunctionLane();
            auto Road_sign = current_lane->GetRoadSigns();
//            auto sign_type = current_lane->GetRoadSigns().at(0)->GetSignType();

//            std::cout << " stop_line" << stop_line << std::endl;

            snm_pub = node_handle->advertise<std_msgs::Bool>("Stop_sign", 1);
            if(!Road_sign.empty()) {
//                std::cout << " Road_sign" << Road_sign.at(0) << std::endl;
                auto sign_type = current_lane->GetRoadSigns().at(0)->GetSignType();
//                std::cout << " Sign Type" << sign_type << std::endl;
                if((sign_type == "Stop" && stop_line != 0) || (sign_type == "Stop" && stop_line != 0 && HLC_msg.command == "stop")){   //TODO STOP
//                    std::cout << "  I want to stop" << std::endl;
                    std_msgs::Bool stop_flag;
//                    ros::Duration(0.2).sleep();
                    stop_flag.data = true;
//                    ros::Duration(0.5).sleep();
//                    rate.sleep();
                    snm_pub.publish(stop_flag);
                    ros::Duration(1.5).sleep();
                    stop_flag.data = false;

                    snm_pub.publish(stop_flag);
                    ros::Duration(5).sleep();
                    //continue;
                }
            }

            jun_pub = node_handle->advertise<std_msgs::Bool>("JUNC_sign", 1);
            std::string current_l_uuid = p_closest.GetLaneUuid();
            std_msgs::Bool junction_flag;
            /// GET INFO ABOUT JUNCTION
            auto junction_by_id = map.GetUpcomingJunctionID(current_l_uuid);

            auto current_junction_id = map.GetCurrentJunctionID(current_l_uuid);
            auto out_lane = map.GetJunctionByID(current_junction_id);
            //std::cout << "UPCOMING JUNCTION UUID IS " << out_lane[0].GetUuidValue()  << out_lane[1].GetUuidValue()  << out_lane[2].GetUuidValue()  <<std::endl;
//            bool check = map.GetUpcomingJunctionID(current_l_uuid)   == freicar::mapobjects::Junction::Type::CROSS_INTESERSECTION;
//            bool check1 = map.GetUpcomingJunctionID(current_l_uuid)   == freicar::mapobjects::Junction::Type::T_INTESERSECTION;
//            bool check2 = map.GetUpcomingJunctionID(current_l_uuid)   == freicar::mapobjects::Junction::Type::ROUNDABOUT;
//            bool check3 = <map.GetUpcomingJunctionID(current_l_uuid)  == freicar::mapobjects::Junction::Type::NONE;
//            auto type1 = freicar::mapobjects::Junction::GetType(map.GetUpcomingJunctionID(current_l_uuid));
//            map.GetUpcomingJunctionID(current_l_uuid)
//            std::cout << "check " << check << "check1 " << check1 << "check2 " << check2 <<"check3 " << check3 <<std::endl;
//            ///
            std::cout << "check "<< map.GetPastJunctionID(current_l_uuid) << std::endl;
            std::cout << "UPCOMING JUNCTION UUID IS " << map.GetUpcomingJunctionID(current_l_uuid) << std::endl;
            std::cout << "CURRENT JUNCTION UUID IS " << map.GetCurrentJunctionID(current_l_uuid) << std::endl;

            //std::cout << "CURRENT JUNCTION UUID IS " << out_lane << std::endl;
            if (map.GetUpcomingJunctionID(current_l_uuid) != -1) {
                //if (map.GetCurrentJunctionID(p_closest.GetLaneUuid()) != -1) {
//                    auto  junction_left = current_lane->JUNCTION_LEFT;
                    if ( map.GetUpcomingJunctionID(current_l_uuid)   == freicar::mapobjects::Junction::Type::CROSS_INTESERSECTION)
                   std::cout << "UPCOMING JUNCTION UUID IS " << map.GetUpcomingJunctionID(current_l_uuid) << std::endl;
                   std::cout << "CURRENT JUNCTION UUID IS " << map.GetCurrentJunctionID(current_l_uuid) << std::endl;
//                    junction_flag.data = true;
//                    jun_pub.publish(junction_flag);
//                    junction_flag.data = false;
//                    jun_pub.publish(junction_flag);
                    junction_arrived = true;
                    //std::cout << " junction_lrs" <<  junction_left << junction_right << junction_straight <<std::endl;
                    std::cout << "on a lane that leads to a junction" << std::endl;

                }//}

             else {
                auto* current_lane = map.FindLaneByUuid(current_l_uuid);
                auto* next_lane = current_lane->GetConnection(freicar::mapobjects::Lane::STRAIGHT);
                auto next_l_uuid = next_lane->GetUuid().GetUuidValue();
                if (next_lane && map.GetUpcomingJunctionID(next_l_uuid) != -1)
                {
//                    std::cout << "two lanes away from a junction" << std::endl;
                    //  junction_arrived = true;
                } else {
//                    std::cout << "not really close to a junction" << std::endl;
                }
            }

            //Position of all the roadsign
//            auto var = map.getSigns();
//            for (int i = 0; i < var.size(); ++i) {
//                std::cout << "Map signs string "<< i << " "<< var[i].GetUuid().GetUuidValue() << std::endl;
//            std::cout << "x signs: " << var[i].GetPosition().x() << std::endl;
//            std::cout << "y signs: " << var[i].GetPosition().y() << std::endl;
//            std::cout << "z signs: " << var[i].GetPosition().z() << std::endl;
//            }

//            float g = sog->GetOffset();

//            std::cout << "should start planning because "<<flag_1 << std::endl;

            // WRITE LOGIC FOR STOPPPING  PLANNIN IN OPPOSITE DIRECTION
            if (flag_1)
            {
                flag_1 = 0;
//                auto plan1 = freicar::planning::lane_follower::GetPlan(Point3D(10.866, 7.49 , 0.0), freicar::enums::STRAIGHT, 10,20);
                std::cout << "so it planned" << std::endl;
                double x_value_init, y_value_init;
                ros::param::get("/freicar_freicar_1_carla_proxy/spawn/x",x_value_init );
                ros::param::get("/freicar_freicar_1_carla_proxy/spawn/y",y_value_init );

                auto plan1 = freicar::planning::lane_follower::GetPlan(Point3D(9.2,   -1.1 , 0.0), freicar::enums::STRAIGHT, 15,25);

//                auto plan1 = freicar::planning::lane_follower::GetPlan(Point3D(3.5, 2.1 , 0.0), freicar::enums::STRAIGHT, 10,20);
//                auto plan1 = freicar::planning::lane_follower::GetPlan(Point3D(1.5, 0.0 , 0.0), freicar::enums::LEFT, 10,20);
//                auto plan1 = freicar::planning::lane_follower::GetPlan(Point3D(1.5, 0.0 , 0.0), freicar::enums::PlannerCommand{HLC_enum}, 10,20);
                PublishPlan(plan1, 1.0, 0.1, 0.4, 300, "plan_1", tf);

            }

            auto p_closest_plan = map.FindClosestLanePoints(p_current.x(),
                                                            p_current.y(),
                                                            p_current.z(),
                                                            1)[0].first;

            if (junction_arrived && HLC_bool)
            {

                std::cout<< " Replan because i reached a junction"<<std::endl;
                //// YOU NEED TO STICK TO PREVIOUS LANE

//                auto p_closest = map.FindClosestLanePoints(p_current.x(),
//                                                           p_current.y(),
//                                                           p_current.z(),
//                                                           1)[0].first;
//                current_lane = map.FindLaneByUuid(p_closest.GetLaneUuid());

                //// TEST

                auto plan2 = freicar::planning::lane_follower::GetPlan(Point3D(p_closest_plan.x(),p_closest_plan.y() , 0), freicar::enums::PlannerCommand{HLC_enum}, 15,25);
                PublishPlan(plan2, 1.0, 0.1, 0.4, 300, "plan_1", tf);
                junction_arrived = false;
                HLC_bool = false;

            }

            if (junction_arrived == true)
            {
                std::cout << "so it planned in junction arrived loop" << std::endl;
                auto plan2_2 = freicar::planning::lane_follower::GetPlan(Point3D(p_closest_plan.x(),p_closest_plan.y() , 0), freicar::enums::LEFT, 15,25);
                PublishPlan(plan2_2, 1.0, 0.1, 0.4, 300, "plan_1", tf);
                junction_arrived = false;

            }

            //else{
            if(goal_bool.data == true ) {

//                auto plan = freicar::planning::lane_follower::GetPlan(Point3D(p_closest_plan.x(),p_closest_plan.y() , 0), freicar::enums::LEFT, 15,25); //TODO
                std::cout << "so it planned in reset goal loop" << std::endl;
                auto plan = freicar::planning::lane_follower::GetPlan(Point3D(p_closest_plan.x(),p_closest_plan.y() , 0), freicar::enums::LEFT, 15,25); //TODO

                    ///TEST
//                    auto previous_lane_points = map.FindClosestLanePoints(p_closest_plan.x(),  //current lane points
//                                                                          p_closest_plan.y(),
//                                                                          0,
//                                                                          1)[0].first;
//                    const freicar::mapobjects::Lane *previous_lane;
//                    previous_lane = map.FindLaneByUuid(previous_lane_points.GetLaneUuid());  //current lane uuid
//                    auto stay_on_lane =  previous_lane->GetClosestLanePoint(Point3D(previous_lane_points.x(),previous_lane_points.y() , 0), false,  -1).first;
//                    // lane points
                    //auto plan2 = freicar::planning::lane_follower::GetPlan(Point3D(stay_on_lane.x()  ,stay_on_lane.y() , 0), freicar::enums::PlannerCommand{HLC_enum}, 15,25);
                    //auto plan2 = freicar::planning::lane_follower::GetPlan(Point3D(p_closest_plan.x()  ,p_closest_plan.y() , 0), freicar::enums::PlannerCommand{HLC_enum}, 15,25);
                    ///TEST

//                auto plan = freicar::planning::lane_follower::GetPlan(Point3D(p_closest_plan.x(),p_closest_plan.y() , 0), freicar::enums::PlannerCommand{HLC_enum}, 15,25); //TODO
                    PublishPlan(plan, 1.0, 0.1, 0.4, 300, "plan_1", tf);
//                    HLC_bool = false;
//                    junction_arrived = false;
                }
            //}

            if (overtake.data == true) {
                overtake.data = false;
                //std::cout << "x postion " << p_current.x() << std::endl;
                //std::cout << "current lane " << current_lane->GetUuid().GetUuidValue() << std::endl;
                //std::cout << "I'm here or not" << goal_bool.data << std::endl;
                //OPPOSITE LANE FROM CAR
                auto *opposite_lane = current_lane->GetConnection(freicar::mapobjects::Lane::OPPOSITE);
                auto opposite_l_uuid = opposite_lane->GetUuid().GetUuidValue();
                std::cout << "opposite lane " << opposite_l_uuid << std::endl;
                //float width = opposite_lane->GetWidth();
                //auto opposite_points = opposite_lane->GetPoints();
                //fake plan
                //CREATE A PLAN
                auto plan4 = freicar::planning::lane_follower::GetPlan(
                        Point3D(p_closest.x(), p_closest.y(), p_closest.z()), freicar::enums::STRAIGHT, 10, 20);
                std::cout << "postion x  before starting points " << plan4[0].position.x() << std::endl;
                //2.5 METERS FROM MY POSITION
                Point3D starting_plan = Point3D(plan4[4].position.x(), plan4[4].position.y(),
                                                plan4[4].position.z());

                auto q_closest = map.FindClosestLanePoints(starting_plan.x(),
                                                           starting_plan.y(),
                                                           starting_plan.z(),
                                                           1)[0].first;
                //CURRENT LANE OF THE POINT AT 2.5 METERS AWAY
                const freicar::mapobjects::Lane *current_lane2;
                current_lane2 = map.FindLaneByUuid(p_closest.GetLaneUuid());
                //OPPOSITE LANE OF THAT POINT
                auto *opposite_lane2 = current_lane2->GetConnection(freicar::mapobjects::Lane::OPPOSITE);
                auto opposite_points = opposite_lane2->GetPoints();

                std::cout << starting_plan.x() << std::endl;
                Point3D closest_point;
                //std::cout<< closest_point.x() << std::endl;
                std::cout << opposite_points.size() << std::endl;
                float dist = 2.0f;
                auto opposite_l_uuid2 = opposite_lane2->GetUuid().GetUuidValue();
                std::cout << "uuid other lane: " << opposite_l_uuid2 << std::endl;
                std::cout << "dist" << dist << std::endl;
                //ITERATE OVER ALL POINTS IN THE OPPOSITE LANE TO GET THE CLOSEST
                for (auto &opposite_point : opposite_points) {
                    std::cout << "dist inside "
                              << opposite_point.ComputeDistance(starting_plan.x(), starting_plan.y(), starting_plan.z()) << std::endl;
                    if (opposite_point.ComputeDistance(starting_plan.x(), starting_plan.y(), starting_plan.z()) < dist) {
                        closest_point = opposite_point;
                        dist = opposite_point.ComputeDistance(starting_plan.x(), starting_plan.y(), starting_plan.z());
                        std::cout << "dist inside " << dist << std::endl;

                    }

                }


                //std::cout << "closest" << closest_point.x() << std::endl;
                //FAKE PLAN TO OVERTAKE
                auto planovertake2 = freicar::planning::lane_follower::GetPlan(
                        Point3D(closest_point.x(), closest_point.y(), closest_point.z()), freicar::enums::STRAIGHT,
                        2.5, 5);

                //SET THE NEW POINT TO THE PLAN
                plan4[0].position.SetX(planovertake2[4].position.x());
                plan4[0].position.SetY(planovertake2[4].position.y());
                plan4[0].position.SetZ(planovertake2[4].position.z());
                plan4[1].position.SetX(planovertake2[3].position.x());
                plan4[1].position.SetY(planovertake2[3].position.y());
                plan4[1].position.SetZ(planovertake2[3].position.z());
                plan4[2].position.SetX(planovertake2[2].position.x());
                plan4[2].position.SetY(planovertake2[2].position.y());
                plan4[2].position.SetZ(planovertake2[2].position.z());
                plan4[3].position.SetX(planovertake2[1].position.x());
                plan4[3].position.SetY(planovertake2[1].position.y());
                plan4[3].position.SetZ(planovertake2[1].position.z());
                plan4[4].position.SetX(planovertake2[0].position.x());
                plan4[4].position.SetY(planovertake2[0].position.y());
                plan4[4].position.SetZ(planovertake2[0].position.z());
                //PUBLISH PLAN
                PublishPlan(plan4, 1.0, 0.1, 0.4, 300, "plan_1", tf);
                //overtaking_done = false;
            }
//            PublishPlan(plan2, 1.0, 0.1, 0.4, 300, "plan_2", tf);
//     		PublishPlan(plan2, 0.5, 0.7, 0.1, 300, "plan_2", tf);
            // ROS_INFO("visualized map");

            std::this_thread::sleep_for(1s);
            freicar::logic::JunctionAgent::Intent{0};
            freicar::logic::JunctionAgent agent_junction = freicar::logic::JunctionAgent(p_current_row) ;


            //observed ROW
            p_observed_agent.current_pose = p_current_row.current_pose;
            p_observed_agent.current_pose.transform.translation.x = 2.1;
            p_observed_agent.current_pose.transform.translation.y = 3.26;
            p_observed_agent.current_pose.transform.translation.z = 0.0f;

            p_observed_agent.current_pose.transform.rotation.x = 0.0;
            p_observed_agent.current_pose.transform.rotation.y = 0.0;
            p_observed_agent.current_pose.transform.rotation.z = 0.0;
            p_observed_agent.current_pose.transform.rotation.w = 1.0;

            //observed STOP
//            p_observed_agent.current_pose = p_current_row.current_pose;
//            p_observed_agent.current_pose.transform.translation.x = 3.60;
//            p_observed_agent.current_pose.transform.translation.y = 1.48;
//            p_observed_agent.current_pose.transform.translation.z = 0.0f;
//
//            p_observed_agent.current_pose.transform.rotation.x = 0.0;
//            p_observed_agent.current_pose.transform.rotation.y = 0.0;
//            p_observed_agent.current_pose.transform.rotation.z = 0.755;
//            p_observed_agent.current_pose.transform.rotation.w = 0.65;

            p_observed_agent.velocity = p_current_row.velocity ;
            p_observed_agent.velocity.x = 0.0 ;
            p_observed_agent.velocity.y =  0.0;
            p_observed_agent.velocity.z =  0.0;

            p_observed_agent.name = "observed_1";
            p_observed_agent.lane_offset = 0.0f;


            auto p_closest_observed = map.FindClosestLanePoints(p_observed_agent.current_pose.transform.translation.x,
                                                                p_observed_agent.current_pose.transform.translation.y,
                                                                p_observed_agent.current_pose.transform.translation.z,
                                                                1)[0].first;
            std::string current_l_uuid_observed = p_closest_observed.GetLaneUuid();
            p_observed_agent.lane_uuid = current_l_uuid_observed;

//            std::cout<<"our agent name"<<p_current_row.name<<std::endl;
//            std::cout<<"observed agent name"<<p_observed_agent.name<<std::endl;

            freicar::logic::JunctionAgent observed_agent = freicar::logic::JunctionAgent(p_observed_agent) ;



            std::vector<freicar::logic::JunctionAgent> observed_agents;
            observed_agents.push_back(observed_agent);

            std::vector<freicar::logic::JunctionAgent> agents;
            agents.push_back(agent_junction);

            if (freicar::enums::PlannerCommand{HLC_enum} == 1)
            {
                agent_junction.intent = freicar::logic::JunctionAgent::Intent::GOING_LEFT;
            }
            else if (freicar::enums::PlannerCommand{HLC_enum} == 2){
                agent_junction.intent = freicar::logic::JunctionAgent::Intent::GOING_RIGHT;
            }
            else {
                agent_junction.intent = freicar::logic::JunctionAgent::Intent::GOING_STRAIGHT;
            }
//            std::tuple<bool, bool, std::string> test_return = freicar::logic::GetRightOfWay(agent_junction, observed_agents, false, true);
            //           agent_junction.intent = freicar::logic::JunctionAgent::Intent::GOING_LEFT;
            observed_agent.intent = freicar::logic::JunctionAgent::Intent::GOING_RIGHT;

//            std::cout<<"current junction"<<map.GetCurrentJunctionID(agent_junction.lane_uuid)<<std::endl;
//            std::cout<<"upcoming junction"<<map.GetUpcomingJunctionID(agent_junction.lane_uuid)<<std::endl;

            bool one ;
            bool two;
            std::string three;
            std::tie(one, two, three) = freicar::logic::GetRightOfWay(agent_junction, observed_agents, false, false);

//            bool one_ob ;
//            bool two_ob;
//            std::string three_ob;
//            std::tie(one_ob, two_ob, three_ob) = freicar::logic::GetRightOfWay(observed_agent, agents, false, false);
//            std::cout<<"we are" << observed_agent.name <<std::endl;
//




//            std::cout<<"one" << one <<std::endl;
//            std::cout<<"two" << two <<std::endl;
//            std::cout<<"three" <<three <<std::endl;

//            std::cout << "freicar intent:  "<<agent_junction.intent<<std::endl;
//            std::cout << "observed agent intent:  "<<observed_agent.intent<<std::endl;

//
////
//            if (one & !two)
//            {
//                std::cout<<": CASE 10:  we always have ROW and junction not occupied by another car"<<std::endl;
//                std::cout<<three <<": has ROW"<<std::endl;
//
//
//            }
//            else if (one && two)
//            {
//                std::cout<<"CASE 11: we have ROW and junction is occupied by another car"<<std::endl;
//                std::cout<<three <<": has ROW"<<std::endl;
//            }
//            else if (!one && two)
//            {
//                std::cout<<"CASE 01: we dont have ROW and junction is occupied by another car"<<std::endl;
//                std::cout<<three <<": has ROW"<<std::endl;
//            }
//            else if (!one && !two)
//            {
//                std::cout<<"CASE 00: we dont have ROW and junction is not occupied by another car"<<std::endl;
//                std::cout<<three <<": has ROW"<<std::endl;
//            }
//
//
//            if (one_ob & !two_ob)
//            {
//                std::cout<<": OB CASE 10:  we always have ROW and junction not occupied by another car"<<std::endl;
//                std::cout<<three_ob <<": has ROW"<<std::endl;
//
//
//            }
//            else if (one_ob && two_ob)
//            {
//                std::cout<<"OB CASE 11: we have ROW and junction is occupied by another car"<<std::endl;
//                std::cout<<three_ob <<": has ROW"<<std::endl;
//            }
//            else if (!one_ob && two_ob)
//            {
//                std::cout<<"OB CASE 01: we dont have ROW and junction is occupied by another car"<<std::endl;
//                std::cout<<three_ob <<": has ROW"<<std::endl;
//            }
//            else if (!one_ob && !two_ob)
//            {
//                std::cout<<"OB CASE 00: we dont have ROW and junction is not occupied by another car"<<std::endl;
//                std::cout<<three_ob<<": has ROW"<<std::endl;
//            }

//            std::cout<<"tuple= "<<  test_return(0)<<std::endl;
//            std::cout<<"agent pose vector = "<<observed_agents.at(0).current_pose.transform.translation.x<<std::endl;

//            observed_agent.HasRightOfWay()
//            bool row_observed = freicar::logic::JunctionAgent::HasRightOfWay(observed_agent, true)
//            bool test_row = agent_junction.HasRightOfWay(observed_agent,true){JunctionSign::NONE};
//            std::cout<<"do we have row = "<<test_row<<std::endl;

//            freicar::logic::JunctionAgent::HasRightOfWay()
//           bool yesno  = agent_junction.HasRightOfWay(observed_agent, true);


            // get right of way

//            std::vector<freicar::logic::JunctionAgent>& observed_agents = {};

            //Dummy agent
            //const freicar_common::FreiCarAgentLocalization& dummy1 =

            // std::cout<<"Junction Agent  =" << agent_junction.IsOnRightHandSideOf(agent_junction) << std::endl;
//            std::cout<<"Junction Agent Offset = "<<freicar::logic::JunctionAgent::IsOnRightHandSideOf(freicar::logic::JunctionAgent(p_current_row))<<std::endl;
//            std::cout<<"Junction Agent  =" << p_current_row.current_pose << p_current_row.velocity << std::endl;
        }
    });


    ROS_INFO("visualizing map @ 0.1 Hz");
    std::thread map_vis_thread([&]() {
        using namespace std::chrono_literals;
        while (ros::ok()) {

            freicar::map::Map::GetInstance().SendAsRVIZMessage(0.01, 0.01, node_handle);
            // ROS_INFO("visualized map");
            std::this_thread::sleep_for(10s);
        }
    });
    // starting waypoint service
    ros::ServiceServer waypoint_service = node_handle->advertiseService("waypoint_service", HandleWayPointRequest);
    std::cout << "waypoint_service active" << std::endl;

    ros::spin();
    std::cout << "\njoining threads ..." << std::endl;
    map_vis_thread.join();

    return 0;
}
