#!/usr/bin/env bash
rosrun hbba_synth hbba_synth -i $1 | sed 's/$/\;/g' | tr -d "\n"
