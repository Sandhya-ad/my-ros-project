#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch subscriber
rosrun my_package straight_line.py

# wait for app to end
dt-launchfile-join