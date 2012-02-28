#!/usr/bin/env bash
# Modified for jn0_h12:

roslaunch hbba_test jn0_h12_hbba.launch &
rosrun hbba_test jn0_h12_run.py

read -p "Press any key to stop..."

