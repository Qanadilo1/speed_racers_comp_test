#ifndef __AISCAR_MAP_H__
#define __AISCAR_MAP_H__

#include <mutex>
#include <cmath>
#include <vector>

#include <ros/ros.h>
#include <nanoflann.hpp>

#include "gen-cpp/map_data_structure_types.h"
#include "map_core/freicar_map_objects.h"


namespace freicar
{
namespace map
{

enum MapStatus
{
	UNINITIALIZED = 1,
	LOADED_FROM_FILE = 3,
	UPDATED_FROM_REMOTE = 4
};

/* singleton map class */
class Map
{
public:
    Map(const Map&) = delete;
    void operator=(const Map&) = delete;
    
	// basic map functions
	bool ClearMap();
	std::tuple<int, int, int> GetMapStats() const;
	// thrift interface functions
	void ProcessMapMessage(const map_thrift::MapMessage& message, MapStatus status);
	map_thrift::MapMessage AsThriftMessagae() const;
	// members
	MapStatus status() const;
    static Map& GetInstance();
	mapobjects::Pivot pivot() const;

    std::vector<mapobjects::Lane> getLanes();
    std::vector<mapobjects::Roadsign> getSigns();
	
	// look up functions
	mapobjects::Lane* FindLaneByUuid(const mapobjects::Uuid& lane_uuid);
	mapobjects::Lane* FindLaneByUuid(const std::string& lane_uuid);
	mapobjects::LaneGroup* FindLaneGroupByUuid(const mapobjects::Uuid& lanegroup_uuid);
	int GetUpcomingJunctionID(const std::string& lane_uuid_value) const;
	int GetPastJunctionID(const std::string& lane_uuid_value) const;
	int GetCurrentJunctionID(const std::string& lane_uuid_value) const;
	std::vector<mapobjects::Junction> GetJunctions();
	mapobjects::Junction GetJunctionByID(int junction_id);
	std::vector<std::pair<mapobjects::LanePoint3D, float>>
	FindClosestLanePoints(float local_x, float local_y, float local_z, unsigned int count) const;
	std::vector<std::pair<mapobjects::LanePoint3D, float>>
	FindClosestLanePointsWithHeading(float local_x, float local_y, float local_z, unsigned int count, float heading) const;
	mapobjects::LanePoint3D GetRandomLanePoint() const;
	mapobjects::Lane& GetRandomLane();
	// utility
	std::vector<mapobjects::Uuid> GetPureOutgoingConnections(const mapobjects::Lane& lane);
	std::vector<mapobjects::Uuid> GetPureIncomingConnections(const mapobjects::Lane& lane);
	std::tuple<int, int, int> GetNeighboringLanes(const mapobjects::Lane& lane, const mapobjects::LaneGroup& lane_group);
	// post process
	void PostProcess(float density_m);
	// stats
	void PrintMap();
	void SendAsRVIZMessage(float point_size, float lanemarking_size, std::shared_ptr<ros::NodeHandle> n);
	// kd-tree stuff
	inline size_t kdtree_get_point_count() const { return kd_points_.size(); }
	inline float kdtree_get_pt(const size_t idx, const size_t dim) const {
		if (dim == 0)
			return kd_points_[idx].x();
		else if (dim == 1)
			return kd_points_[idx].y();
		else
			return kd_points_[idx].z();
	}
	template<class BBox>
	bool kdtree_get_bbox(BBox& /* bb */) const { return false; }

private:
	Map();
	void IntializeMapContainers();
	// adding structures to map -----------------------------------------
	void SetPivotFromThrift(const map_thrift::Pivot& pivot);
	void AppendMessageToMap(const map_thrift::MapMessage& message);
	void AddThriftLaneObjectList(const map_thrift::LaneObjectList& lo_list);
	void AddLaneMarking(const mapobjects::LaneMarking& lanemakring);
	void AddLane(const mapobjects::Lane& lane);
	void AddLaneGroup(const mapobjects::LaneGroup& lane_group);
	// lookup functions -----------------------------------------
	void AddToStoplinesLookup(mapobjects::Stopline stopline);
	void AddToCrosswalksLookup(mapobjects::Crosswalk crosswalk);
	void AddToParkingLookup(mapobjects::Parking parking);
	void AddToRoadsignLookup(mapobjects::Roadsign roadsign);
	// post-processing -----------------------------------------
	void AdjustCoordinates();
	void SetLaneParents();
	void AddLaneObjectPointers();
	void InitializeJunctions();
	void UpdateNodes(float density_cm);
	void FillLaneHashMaps();
	void CreateKDIndex();
	
	// kd-tree
	nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<float, Map>,
										Map, 3> *kd_tree_;
	// utility functions
	int FindDuplicate(const mapobjects::MapObject& mo);

	// node graph of the map for global planning
	std::vector<mapobjects::LanePoint3D> kd_points_;
	// map variables
	mapobjects::Pivot pivot_point_;
	// map state
	std::unique_ptr<ros::Publisher> marker_pub_;
	MapStatus status_;
	mutable std::mutex access_mutex_;
	bool map_changed_;
	// structure containers
	std::vector<mapobjects::Lane> lane_storage_;
	std::vector<mapobjects::LaneGroup> lanegroup_storage_;
	std::vector<mapobjects::LaneMarking> lanemarking_storage_;
	std::vector<mapobjects::Junction> junction_storage_;
	std::vector<mapobjects::Parking> parking_storage_;
	std::vector<mapobjects::Stopline> stopline_storage_;
	std::vector<mapobjects::Crosswalk> crosswalk_storage_;
	std::vector<mapobjects::Roadsign> roadsign_storage_;
	// uuid containers
	std::unordered_map<std::string, int> lanemarking_uuids_;
	std::unordered_map<std::string, int> lane_uuids_;
	std::unordered_map<std::string, int> lanegroup_uuids_;
	std::unordered_map<std::string, int> laneobject_uuids_;
	// search hashmaps
	std::unordered_map<std::string, std::vector<mapobjects::Uuid>> lane_stoplines_lookup_;
	std::unordered_map<std::string, std::vector<mapobjects::Uuid>> lane_crosswalks_lookup_;
	std::unordered_map<std::string, std::vector<mapobjects::Uuid>> lane_parking_lookup_;
	std::unordered_map<std::string, std::vector<mapobjects::Uuid>> lane_roadsign_lookup_;
	std::unordered_map<std::string, int> lane_junction_lookup_;
	std::unordered_map<std::string, int> incoming_lane_junction_lookup_;
	std::unordered_map<std::string, int> outgoing_lane_junction_lookup_;

};

} // namespace map
} // namespace freicar

#endif
