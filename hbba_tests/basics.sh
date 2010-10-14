#!/usr/bin/env sh

roslaunch hbba_tests basics.launch &
echo "You have 5 sec to launch the translator in gdb..."
sleep 5

rosservice call /add_strategy '{strategy: {id: "goto1", bringup: "a = 1; se_log(\"goto1_bup\")", utility: [{id: "goto", value: 1}], cost: [{id: "cpu", value: 10}], bringdown: ""}}'
rosservice call /set_resource_max '{id: "cpu", value: 100}'
rosservice call /add_desire '{desire: {id: "test", type: "goto", utility: 1}}'

