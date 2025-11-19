import os
import signal
import time
import subprocess
import threading

# Global configs.
kDatasetRootPath = "/home/jiapengli/Desktop/dataset"
kDatasetPathsWithLaunchFileAndRosbagPlayConfig = [
    [kDatasetRootPath + "/Drone_P100", "vio_pinhole_stereo_p100_front430.launch", ["-r1"]],
    [kDatasetRootPath + "/Drone_C5_250001/company_fly_space_dark_autofly_01", "lvio_pinhole_stereo_c5_250001_front430_mid360.launch", ["-r1"]],
    [kDatasetRootPath + "/Drone_C5_250001/company_fly_space_dark_autofly_01", "lvio_pinhole_stereo_c5_250001_down265_mid360.launch", ["-r1"]],
    [kDatasetRootPath + "/Drone_C5_250001/company_fly_space_dark_autofly_02", "lvio_pinhole_stereo_c5_250001_front430_mid360.launch", ["-r1"]],
    [kDatasetRootPath + "/Drone_C5_250001/company_fly_space_dark_autofly_02", "lvio_pinhole_stereo_c5_250001_down265_mid360.launch", ["-r1"]],
    [kDatasetRootPath + "/Drone_C5_250001/underground_car_park_autofly", "lvio_pinhole_stereo_c5_250001_front430_mid360.launch", ["-r1"]],
    [kDatasetRootPath + "/Drone_C5_250001/underground_car_park_autofly", "lvio_pinhole_stereo_c5_250001_down265_mid360.launch", ["-r1"]],
    [kDatasetRootPath + "/Seeker_v2", "vio_rectify_stereo_seeker_v2.launch", ["-r1", "-s20"]],
    [kDatasetRootPath + "/Seeker_v2", "vio_fisheye_stereo_seeker_v2.launch", ["-r1", "-s20"]],
    [kDatasetRootPath + "/Seeker_v1", "vio_rectify_stereo_seeker_v1.launch", ["-r1"]],
    [kDatasetRootPath + "/Seeker_v1", "vio_fisheye_stereo_seeker_v1.launch", ["-r1"]],
    [kDatasetRootPath + "/Euroc", "vio_pinhole_stereo_euroc.launch", ["-r3"]],
    [kDatasetRootPath + "/Livox_MID360", "lio_livox.launch", ["-r3"]],
    [kDatasetRootPath + "/Drone_C5_250801", "lio_livox.launch", ["-r3"]],
]
kDefaultLogRootPath = "/home/jiapengli/output"
kUserOutputLogRootPath = "/home/jiapengli/Desktop/workspace/catkin_ws/benchmark_logs"

# Global variable to track running processes
running_processes = []
# Signal handler.
def SignalHandler(signum, frame):
    print("\033[91m>> Ctrl+c is pressed. Exiting...\033[0m")
    # Kill all running subprocesses
    for process in running_processes:
        if process.poll() is None:  # Process is still running
            process.terminate()
            try:
                process.wait(timeout=1)  # Wait up to 1 seconds for graceful termination
            except subprocess.TimeoutExpired:
                process.kill()  # Force kill if it doesn't terminate gracefully

    # Kill all fusion odometry processes.
    for _, launch_file, _ in kDatasetPathsWithLaunchFileAndRosbagPlayConfig:
        os.system("pkill -f " + launch_file)
    os.system("pkill -f run_on_ros")
    exit(0)

signal.signal(signal.SIGINT, SignalHandler)

# Main function.
if __name__ == "__main__":
    # Kill all fusion odometry processes.
    for _, launch_file, _ in kDatasetPathsWithLaunchFileAndRosbagPlayConfig:
        os.system("pkill -f " + launch_file)

    # Check if roscore is running.
    bash_result = os.popen("ps -ef | grep roscore").read()
    # If string '/opt/ros/noetic/bin/roscore' is in bash_result, then roscore is running.
    if "/opt/ros/noetic/bin/roscore" in bash_result:
        print(">> Roscore is running.")
    else:
        print(">> Roscore is not running. Please start roscore first.")
        exit(1)

    # Clear default log directory.
    if os.path.exists(kDefaultLogRootPath):
        print("\033[91m>> Succeed to clear default log directory: " + kDefaultLogRootPath + "\033[0m")
        os.system("rm -rf " + kDefaultLogRootPath)
    os.system("mkdir -p " + kDefaultLogRootPath)

    # Clear user output log directory.
    if os.path.exists(kUserOutputLogRootPath):
        print("\033[91m>> Succeed to clear user output log directory: " + kUserOutputLogRootPath + "\033[0m")
        os.system("rm -rf " + kUserOutputLogRootPath)
    os.system("mkdir -p " + kUserOutputLogRootPath)

    # Run fusion odometry on each dataset.
    for dataset_path, launch_file, rosbag_play_config in kDatasetPathsWithLaunchFileAndRosbagPlayConfig:
        if not launch_file.endswith(".launch"):
            continue

        # Iterate each .bag file in the dataset path.
        for bag_file in os.listdir(dataset_path):
            if not bag_file.endswith(".bag"):
                continue
            bag_file_name = bag_file[:-4]
            launch_file_name = launch_file[:-7]

            # Launch fusion odometry with visual frontend.
            roslaunch_cmd = ["roslaunch", "fusion_odometry", launch_file]
            roslaunch_process = None
            try:
                roslaunch_process = subprocess.Popen(roslaunch_cmd, stdout=None, stderr=None)
                running_processes.append(roslaunch_process)
                time.sleep(2)  # Wait for launch to initialize
            except KeyboardInterrupt:
                # This will be handled by the signal handler
                pass

            # Print the dataset path and bag file with yellow bold color.
            print(">> Running fusion odometry on dataset: \033[91m" + dataset_path + "/" + bag_file + "\033[0m with launch file: \033[92m" + launch_file + "\033[0m")

            # Use subprocess instead of os.system for better signal handling
            rosbag_cmd = ["rosbag", "play", *rosbag_play_config, dataset_path + "/" + bag_file]
            try:
                with subprocess.Popen(rosbag_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) as rosbag_process:
                    running_processes.append(rosbag_process)
                    rosbag_process.wait()  # Wait for rosbag to complete
                    running_processes.remove(rosbag_process)
            except KeyboardInterrupt:
                # This will be handled by the signal handler
                pass

            # Terminate the roslaunch process if it's still running
            if roslaunch_process is not None and roslaunch_process.poll() is None:
                roslaunch_process.terminate()
                try:
                    roslaunch_process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    roslaunch_process.kill()
                if roslaunch_process in running_processes:
                    running_processes.remove(roslaunch_process)

            # If no files in default log directory, skip.
            if not os.listdir(kDefaultLogRootPath):
                print("\033[91m>> No files in default log directory: " + kDefaultLogRootPath + "\033[0m")
                continue

            # Copy fusion odometry log to user output log directory. Rename the log directory to the dataset path and bag file.
            temp_user_log_path = kUserOutputLogRootPath + "/" + bag_file_name + "___" + launch_file_name
            while os.path.exists(temp_user_log_path):
                temp_user_log_path += "_1"
            os.system("mkdir -p " + temp_user_log_path)
            # Copy log of fusion odometry.
            os.system("cp -r " + kDefaultLogRootPath + "/0/* " + temp_user_log_path + "/")
            # Copy log of visual frontend.
            for file_or_dir in os.listdir(kDefaultLogRootPath):
                if file_or_dir.endswith(".csv"):
                    os.system("cp " + kDefaultLogRootPath + "/" + file_or_dir + " " + temp_user_log_path + "/visual_frontend.csv")
            print(">> Succeed to copy fusion odometry log to user output log directory: \033[91m" + temp_user_log_path + "\033[0m")

            # Clear default log directory.
            if os.path.exists(kDefaultLogRootPath):
                os.system("rm -rf " + kDefaultLogRootPath)
                print(">> Succeed to clear default log directory: \033[91m" + kDefaultLogRootPath + "\033[0m")
            os.system("mkdir -p " + kDefaultLogRootPath)
