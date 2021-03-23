
# Competition Test 17.03.2021
## Start the simulator
`roslaunch freicar_launch_sr sim_base2_row.launch`.

## run the map
`rosrun freicar_map_sr freicar_map_sr_node `

## Start the localizer:
`rosrun freicar_localization_sr freicar_localization_sr_node `
`rosrun freicar_sign_detect freicar_sign_detect_node `

## Start the controller:
`roslaunch freicar_control_sr start_controller.launch `

## Publish the path:
`roscd freicar_control_sr/scripts/`
`anaconda`
`python save_path.py `

## to run overtake:
## first run the bounding boxes publisher:
`roscd freicar_control_sr`
`cd ..`
`cd 01-01-object-detection-exercise`
`anaconda`
`python img_sub_bb_pub.py `

## second run the depth camera:
`roscd freicar_control_sr`
`cd ..`
`cd 01-01-object-detection-exercise`
`anaconda`
`python depth_nodereal.py `


## to reverse after hitting a sign:
`roscd freicar_control_sr`
`cd ..`
`cd 01-01-object-detection-exercise`
`anaconda`
`depth_nodereal_sign_goback.py `

## PS for Davide
add yaml file please, that initialize the position for the localizer and the planner
add your launch file
