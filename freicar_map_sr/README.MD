# ROS Map Framework
This node first tries to load a thrift map from a file. If not successful, it'll start a thrift map server that waits for an external source to send the map.
Once loaded, a couple of post porcessing steps are done. The node is also responsible for serving WayPoint requests.

## 1. Post-Processing
###	1.1. AdjustCoordinates
All coordinates are relative to a pivot point that's not used after the design stage. They are adjusted to be relative to the map origin (to be published later by a VR tracker).
### 1.2. SetLaneParents
Each lane will get a reference to its parent lanegroup.

### 1.3. InitializeJunctions
The thrift map does not contain a struct for junctions. Each lane however has a junction ID starting from `0`, `-1` if it doesn't belong to one. This ID is used to create a struct (`Junction` class) that keeps track of all the lanes that enter, exit or are inside it. 3 separate lookup hash maps are also created for these 3 cases. They can be accessed via the following functions. They return the junction ID or -1 if the `lane_uuid` doesn't fulfill the criterion

* `int GetUpcomingJunctionID(string lane_uuid)`
* `int GetPastJunctionID(string lane_uuid)`
* `int GetCurrentJunctionID(string lane_uuid)`

### 1.4. UpdateNodes(density_cm)
Densifies the lane nodes in the map, based on the supplied value in cm. Each lane node also gets a reference to its lane uuid. It's inefficient but necessary because we have overlapping nodes & need to be able to distinguish between them.

### 1.5.FillLaneHashMaps()
Each lane gets a hash map from Connection to Lane* that is filled with information about accessible destination lanes. The Lane::Connection enum has the following values:
```cpp
enum Connection : unsigned char {
	JUNCTION_STRAIGHT = 0,
	JUNCTION_LEFT = 1,
	JUNCTION_RIGHT = 2,
	STRAIGHT = 3,
	OPPOSITE = 4,
	ADJACENT_LEFT = 5,
	ADJACENT_RIGHT = 6,
	EMPTY = 7
};
```
Difference between STRAIGHT and JUNCTION_STRAIGHT is that JUNCTION_STRAIGHT is the straight connection in the upcoming junction. STRAIGHT shows the next lane in the next lanegroup which is not a junction, i.e. there is only the possibility of going stragiht, not counting the adjacent or opposite lanes.

### 1.6.CreateKDIndex()
A kd-tree is built based on a more detailed structure called [LanePoint3D](#5-lanepoint3d) to allow for fast x,y,z lookup on the map. The tree is used in `FindClosestLanePoints` and `FindClosestLanePointsWithHeading`.
### 1.7. Offroad
a dummy `Lane` object is added to the lane storage with uuid `?`. this should be used by agents to indicate they're offroad. `IsOffroad` function checks the uuid value and returns a boolean.
## 2. Junction Logic
Each agent can fill out a struct named `JunctionAgent` for itself & the agents it observes at the junction. The algorithm uses the street signs for each agent, followed by the right hand rule & as a last resort, the agent's intent to determine the right of way. The intent is usually only filled by the calling agent, as the intent of other agents is not known. In case it is (e.g. via turn signals), the logic can be added to `JunctionAgent::HasRightOfWay()`.

## 3. Planning
### 3.1. Plan
Each plan is made up of equidistant nodes, but contains much more information than just their position:
```cpp
struct PlanStep {
	std::string lane_uuid;
	float plan_offset;
	float lane_offset;
	freicar::mapobjects::Point3D position;
	freicar::mapobjects::Lane::Connection path_description;
}
```
### 3.2. Lane Follower
A basic planner that accepts a start point, a command, the distance & the number of equidistant nodes that are used to cover this route. The algorithm has to also cover a couple of weird edge cases that are explained in the code. This planner is used by scripted agents. It can also partially be utilized by other agents through the WayPoint service (the WayPoint service does not provide the full functionality of the planner).
This information is very useful when determining the right of way.
### 3.3. LaneStar
Unlike `lane_follower`, this planner requires an instance. The `GetPlan()` function takes two `Point3D` objects for start and goal nodes. It finds the closest `LaneNode3D`s to them using the kd-tree. It uses a lane-based A-star algorithm to find a path consisting of lanes, then uses a similar algorithm to create a plan with equidistant nodes.

#### possible change
Right now only the distance is considered for locating the closest lane nodes to the given start and goal points. Heading could also be considered using `FindClosestLanePointsWithHeading`.
## 4. WayPoint Service
This is a deprected service that returns a plan based on given parameters. Check the service files for more info. Currently the agents directly call the planner itself.

## 5. LanePoint3D
A child of the Point3D class which functions as a wrapper for lane points. it contains lane uuid, lane offset, heading and the index of the lane point in the lane's vector. the kd-tree is built using a `LanePoint3D` vector. the two `FindClosestLanePoints` functions return this type.
## 6. Notes
### 6.1. Map::FindLaneByUuid()
To avoid implicit copies of lanes, this function returns a `mapobjects::Lane*` instead of a reference. Removing the implicit copy constructor was not possible because building the map from thrift messages depended on it.
### 6.2. Move from `double` to `float`
all structures now work on float values instead of double. we don't need that level of accuracy and would just waste memory by using double.

## 7. Ongoing Problems

### 7.1. Right-Of-Way

#### line 111
A stops at junction, B is on his RHS but not at the junction yet. A wants to steer right anyway => no reason to wait for B. A starts going, B arrives at the junction sees there's an agent on the junction, gets blocked, even though the path is clear.

#### Two on RoW lane
A is on B's tail, both on a RoW lane. B enters junction, A calls GetRightOfWay() and is blocked because B is on the junction, even though A is clear to go.