rosbag record --tcpnodelay \
    /vrpn_client_node/TYNB/pose \
    /vrpn_client_node/TYNB/twist \
    /ekf/ekf_odom \
    /mavros/imu/data \
    /mavros/setpoint_raw/target_attitude \
    /imu_data_raw \
    /fusion_odometry/current_point_odom \
    /drone_control/position_cmd \
    /debugPx4ctrl \
    /other_drones \