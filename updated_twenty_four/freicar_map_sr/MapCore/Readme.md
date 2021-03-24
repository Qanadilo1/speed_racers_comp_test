# MapCore
## AADC 2018, team frAIsers

authors: Fabien Jenne

## Description
This folder holds the Map Core data structure, namely:
* the map itself, implemented as singleton.
* the MapObjects, the different objects the map consists of.

## Map 
### Description
* singleton reference
  * <https://gist.github.com/pazdera/1098119>  
  * <http://www.yolinux.com/TUTORIALS/C++Singleton.html>  

### Implementation
* The map is implemented as a shared library `libglobalmap.so`.
* ADTF lives in one process, so the memory for the library is only allocated once -> all filters use the same Map singleton.
* For integraton of the map, simply link to `globalmap`.
* You have to extend the `LD_LIBRARY_PATH` either globally or directly before calling the ADTF Configuration Editor.
* Hint: add this line to the ADTF start script `start_adtf_ce_.sh` and to `~/.bashrc`  
```
export LD_LIBRARY_PATH=/home/aadc/AADC/_build_user/Map/MapCore/:$LD_LIBRARY_PATH
```

### Coordinate frames
* **global map** frame of reference is consistent with AIS-Streetdesigner coordinate system:
  * **X**: to the right
  * **Y**: down
  * **Z**: upwards, 0 in xy plane
  * **rotation**: clockwise with angle=0 in x-direction
  * **units**: [x,y,z] in **cm**, [angle] in **radian**
  * position is relative to the map pivot
* **discretized map**:
  * same coordinate system as the global map
  * **units**: dynamic, depending on the discretization level, standard is 1px/cm
* **ADTF** frame of reference, e.g. from MarkerPositioning
  * **X**: to the right
  * **Y**: up
  * **Z**: upwards
  * **rotation**: counter-clockwise with angle=0 in x-direction
  * **units**: [x,y,z] in **m**, [angle] in **radian**
#### For interaction with ADTF:  
The position given to the map and the poses returned from the map have to be transformed to/from the map frame
## Map discretization

## MapObjects
* LaneGroup
* Lane
* LaneMarking
* Car
* Pivot
* LaneObjects:
  * Roadsigns
  * Crosswalks
  * Stoplines
  * Parking lots

### LaneInfo encoding:
Some special lane information is encoded in its height parameter
* 0: height level 0
* 1: height level 1
* 2: ramp up
* 3: ramp down
* 4: merging (to the left)
* 5: tunnel

## New Structures/Functionalities
All new structures and functionalities are all initialized as a post-processing step after the map is loaded (in the `PostProcess()` function).

### Map
The map now has a kd-tree to enable fast searching for lane points.

### LaneGroup
Each LaneGroup now has a boolean that shows whether it's a part of a junction.

### Lane
Each lane has a hashmap of outgoing connections it has. This is used for planning:
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

### Junction
Each lane has a junction_id. It's used to create a junction object containing all the lanes in that junction, all the incoming/outgoing lanes & the lanegroups that lead to it. It also calculates a polygon that represents the junction. It's further used for determining the right of way at junctions.

### Point3D
Some level of vector math is added to the class to avoid back & forth conversions between Point3D & Eigen::Vector3f. A `std::string` is also added to the class to indicate the uuid of the lane it belongs to. This was needed because there are duplicate nodes on the map & hash like structures wouldn't have been able to differentiate between them.
