#!/usr/bin/env bash
# Modified for jn0_h12:

roslaunch hbba_test jn0_h12_hbba.launch &
rosrun hbba_test jn0_h12_run.py
roslaunch hbba_test jn0_h12_motivation.launch

read -p "Press ENTER to stop..."
kill %+ # Job on line 4

