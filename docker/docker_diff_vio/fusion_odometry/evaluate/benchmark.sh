#!/bin/bash

# Global configs.
kCatkinWsPath=/home/jiapengli/Desktop/workspace/catkin_ws

# Check if catkin ws path exists.
if [ ! -d "$kCatkinWsPath" ]; then
    echo ">> Catkin ws path does not exist: $kCatkinWsPath"
    exit 1
fi

# Compile all packages if not compiled.
if [ ! -d "$kCatkinWsPath" ]; then
    echo ">> Catkin ws path does not exist: $kCatkinWsPath"
    exit 1
fi
cd $kCatkinWsPath
catkin_make -j8
source devel/setup.bash

# Run fusion odometry on each dataset.
python3 ${kCatkinWsPath}/src/fusion_odometry/evaluate/run_on_dataset.py
python3 ${kCatkinWsPath}/src/fusion_odometry/evaluate/evaluate_on_benchmark.py
