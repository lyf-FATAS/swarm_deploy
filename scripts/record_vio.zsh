#!/usr/bin/env zsh
#!/usr/bin/env zsh
rosbag record --tcpnodelay \
    /vrpn_client_node/swarm_drone_0/pose \
    /vrpn_client_node/swarm_drone_0/twist \
    /ekf/ekf_odom \
    /mavros/imu/data \
    /mavros/setpoint_raw/target_attitude \
    /imu_data_raw \
    /fisheye/left/image_raw/compressed \
    /fisheye/right/image_raw/compressed \
    /fisheye/bright/image_raw/compressed \
    /fisheye/bleft/image_raw/compressed \
    /fusion_odometry/current_point_odom \
    /drone_control/position_cmd \
    /debugPx4ctrl \
