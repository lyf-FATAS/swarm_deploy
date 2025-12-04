#!/usr/bin/env zsh
rosbag record --tcpnodelay \
    /mavros/imu/data \
    /imu_data_raw \
    /fisheye/left/image_raw/compressed \
    /fisheye/right/image_raw/compressed \
    /fisheye/bright/image_raw/compressed \
    /fisheye/bleft/image_raw/compressed \
    # /fisheye/left/image_raw \
    # /fisheye/right/image_raw \
    # /fisheye/bright/image_raw \
    # /fisheye/bleft/image_raw \
