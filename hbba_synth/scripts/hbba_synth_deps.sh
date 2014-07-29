#!/usr/bin/env bash
rosrun hbba_synth hbba_synth_deps.py $1 | sed 's/$/\;/g' | tr -d "\n"
