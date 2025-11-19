# [Best Practices when using NVIDIA Jetson](https://docs.ultralytics.com/guides/nvidia-jetson/#best-practices-when-using-nvidia-jetson)

When using NVIDIA Jetson, there are a couple of best practices to follow in order to enable maximum performance on the NVIDIA Jetson.

1. **Enable MAX Power Mode**

   Enabling MAX Power Mode on the Jetson will make sure all CPU, GPU cores are turned on.

   ```bash
   sudo nvpmodel -m 0
   ```

2. **Enable Jetson Clocks**

   Enabling Jetson Clocks will make sure all CPU, GPU cores are clocked at their maximum frequency.

   ```bash
   sudo jetson_clocks
   ```

3. **Install Jetson Stats Application**

   We can use jetson stats application to monitor the temperatures of the system components and check other system details such as view CPU, GPU, RAM utilization, change power modes, set to max clocks, check JetPack information.

   ```bash
   sudo apt update
   sudo pip install jetson-stats
   sudo reboot
   jtop
   ```

# Requirements

1. [Python environment for YOLO11 inference.](https://docs.ultralytics.com/guides/nvidia-jetson/#run-on-jetpack-512)

# Build Issues

1. [AttributeError: module 'em' has no attribute 'RAW_OPT'](https://github.com/introlab/rtabmap_ros/issues/1081)

   ```bash
   pip uninstall empy
   pip install empy==3.3.4
   ```
