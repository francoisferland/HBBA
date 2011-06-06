#!/usr/bin/env sh
rosrun hbba_test filter_reg.py __ns:=/hbba
rosrun hbba_test strategies.py __ns:=/hbba
roslaunch hbba_test scenario_june_2011.launch
