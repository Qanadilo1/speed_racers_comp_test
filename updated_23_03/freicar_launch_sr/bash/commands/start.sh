#!/bin/bash
rostopic pub -1 /freicar_commands freicar_common/FreiCarControl  '{name:  "greatteam", command: "start"}'
