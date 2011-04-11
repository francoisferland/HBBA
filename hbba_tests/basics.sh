#!/usr/bin/env sh

# WARNING: No longer valid! see basics.py instead.

roslaunch hbba_tests basics.launch &
echo "You have 5 sec to launch the translator in gdb..."
sleep 5

rosservice call /add_strategy '{strategy: {id: "goto1", bringup: "a = 1; se_log(\"goto1_bup\")", utility: [{id: "goto", value: 1}], cost: [{id: "cpu", value: 10}, {id: "planner", value: 1}], bringdown: ""}}'
rosservice call /add_strategy '{strategy: {id: "goto2", bringup: "se_log(\"goto2\")", utility: [{id: "goto", value: 1}], cost: [{id: "cpu", value: 2}, {id: "planner", value: 1}], bringdown: ""}}'
rosservice call /set_resource_max '{id: "cpu", value: 100}'
rosservice call /set_resource_max '{id: "planner", value: 1}'
rosservice call /add_desire '{desire: {id: "test", type: "goto", utility: 1}}'

