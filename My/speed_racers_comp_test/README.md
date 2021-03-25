
# Competition Test 17.03.2021
## Start the simulator
`roslaunch freicar_launch_sr sim_base2_row.launch`.

## Start the localizer:
`rosrun freicar_localization_sr freicar_localization_sr_node `
`rosrun freicar_sign_detect freicar_sign_detect_node `

## Start the controller:
`roslaunch freicar_control_sr start_controller.launch `

## Publish the path:
`roscd freicar_control_sr/scripts/`
`anaconda`
`python save_path.py `

