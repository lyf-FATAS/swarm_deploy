import os
import csv
import numpy as np
import matplotlib.pyplot as plt
from textwrap import fill
from matplotlib.backends.backend_pdf import PdfPages

# Global configs.
kBenchmarkLogsRootPath = "/home/jiapengli/Desktop/workspace/catkin_ws/benchmark_logs"
kBenchmarkOutputRootPath = "/home/jiapengli/Desktop/workspace/catkin_ws/benchmark_output"

def ScanAllCsvLogFiles(root_path):
    # Find all path in kBenchmarkLogsRootPath with 'fusion_odometry.csv'
    log_file_paths = []
    for root, dirs, files in os.walk(root_path):
        for file in files:
            if file == "fusion_odometry.csv":
                log_file_paths.append(root)
    return log_file_paths

def LoadCsvData(file_name):
    # If file not exist, return empty dictionary.
    if not os.path.exists(file_name):
        print(">> Warning: File not exist: " + file_name)
        return {}

    # Read csv file.
    with open(file_name, mode="r", encoding="utf-8") as file:
        reader = csv.DictReader(file)
        # Remove useless space in column name.
        reader.fieldnames = [column_name.strip() for column_name in reader.fieldnames]

        # Initialize dictionary, each column name as key, corresponding value initialized as empty list.
        data_dict = {}
        for column_name in reader.fieldnames:
            data_dict[column_name] = []

        # Read each row data, convert each value to double type and add to corresponding column.
        for row in reader:
            for key, value in row.items():
                try:
                    double_value = float(value)
                    data_dict[key].append(double_value)
                except ValueError:
                    continue
    del data_dict['']
    return data_dict

def ConvertQuaternionToEulerAngle(qx, qy, qz, qw):
    nqx = np.array(qx)
    nqy = np.array(qy)
    nqz = np.array(qz)
    nqw = np.array(qw)
    t0 = 2.0 * (nqw * nqx + nqy * nqz)
    t1 = 1.0 - 2.0 * (nqx * nqx + nqy * nqy)
    t2 = 2.0 * (nqw * nqy - nqz * nqx)
    t2 = np.clip(t2, -1.0, 1.0)
    t3 = 2.0 * (nqw * nqz + nqx * nqy)
    t4 = 1.0 - 2.0 * (nqy * nqy + nqz * nqz)
    roll_list = np.arctan2(t0, t1) * 180.0 / np.pi
    pitch_list = np.arcsin(t2) * 180.0 / np.pi
    yaw_list = np.arctan2(t3, t4) * 180.0 / np.pi
    return roll_list, pitch_list, yaw_list

def AppendPdfFileHeader(pdf_file, log_file_path):
    fig = plt.figure()
    text = "Analyzing file: " + log_file_path
    wrapped_text = fill(text, width=80)
    plt.text(0.5, 0.5, wrapped_text, fontsize=16, ha='center', va='center', wrap=True, transform=fig.transFigure)
    plt.axis('off')
    pdf_file.savefig(fig, bbox_inches=None)

def AppendTitleFigure(pdf_file, file_name, title):
    title_fig = plt.figure()
    plt.text(0.5, 0.5, file_name + " - " + title, fontsize=24, ha='center', va='center', transform=title_fig.transFigure)
    plt.axis('off')
    plt.close()
    pdf_file.savefig(title_fig)

def AppendImageFigure(figure, figure_title, pdf_file):
    plt.legend()
    plt.title(figure_title)
    plt.close()
    pdf_file.savefig(figure, bbox_inches=None)

def AnalyzeFusionOdometryData(fusion_odometry_data, pdf_file):
    print(">> Analyzing fusion odometry data...")

    # Plot time cost.
    if "time_cost/process_imu_ms" in fusion_odometry_data:
        AppendTitleFigure(pdf_file, "fusion_odometry", "time cost")
        fig = plt.figure()
        plt.scatter(fusion_odometry_data["state/time_stamp_s"], fusion_odometry_data["time_cost/process_once_ms"], label="process_once_ms")
        plt.scatter(fusion_odometry_data["state/time_stamp_s"], fusion_odometry_data["time_cost/process_lidar_ms"], label="process_lidar_ms")
        plt.scatter(fusion_odometry_data["state/time_stamp_s"], fusion_odometry_data["time_cost/process_visual_ms"], label="process_visual_ms")
        plt.scatter(fusion_odometry_data["state/time_stamp_s"], fusion_odometry_data["time_cost/process_imu_ms"], label="process_imu_ms")
        AppendImageFigure(fig, "Fusion Odometry Time Cost", pdf_file)

    # Plot current state of imu frame.
    if "state/time_stamp_s" in fusion_odometry_data:
        AppendTitleFigure(pdf_file, "fusion_odometry", "current state of imu frame")
        if "state/is_attitude_valid" in fusion_odometry_data:
            fig = plt.figure()
            plt.plot(fusion_odometry_data["state/time_stamp_s"], fusion_odometry_data["state/is_attitude_valid"], label="is_attitude_valid")
            plt.plot(fusion_odometry_data["state/time_stamp_s"], fusion_odometry_data["state/is_motion_valid"], label="is_motion_valid")
            AppendImageFigure(fig, "Fusion Odometry Current State(is_attitude_valid)", pdf_file)
        if "state/p_wi_x" in fusion_odometry_data:
            fig = plt.figure()
            plt.plot(fusion_odometry_data["state/time_stamp_s"], fusion_odometry_data["state/p_wi_x"], label="p_wi_x")
            plt.plot(fusion_odometry_data["state/time_stamp_s"], fusion_odometry_data["state/p_wi_y"], label="p_wi_y")
            plt.plot(fusion_odometry_data["state/time_stamp_s"], fusion_odometry_data["state/p_wi_z"], label="p_wi_z")
            AppendImageFigure(fig, "Fusion Odometry Current State(p_wi)", pdf_file)
        if "state/q_wi_x" in fusion_odometry_data:
            roll_list, pitch_list, yaw_list = ConvertQuaternionToEulerAngle(fusion_odometry_data["state/q_wi_x"], fusion_odometry_data["state/q_wi_y"], fusion_odometry_data["state/q_wi_z"], fusion_odometry_data["state/q_wi_w"])
            fig = plt.figure()
            plt.plot(fusion_odometry_data["state/time_stamp_s"], roll_list, label="roll_deg")
            plt.plot(fusion_odometry_data["state/time_stamp_s"], pitch_list, label="pitch_deg")
            plt.plot(fusion_odometry_data["state/time_stamp_s"], yaw_list, label="yaw_deg")
            AppendImageFigure(fig, "Fusion Odometry Current State(q_wi)", pdf_file)
        if "state/v_wi_x" in fusion_odometry_data:
            fig = plt.figure()
            plt.plot(fusion_odometry_data["state/time_stamp_s"], fusion_odometry_data["state/v_wi_x"], label="v_wi_x")
            plt.plot(fusion_odometry_data["state/time_stamp_s"], fusion_odometry_data["state/v_wi_y"], label="v_wi_y")
            plt.plot(fusion_odometry_data["state/time_stamp_s"], fusion_odometry_data["state/v_wi_z"], label="v_wi_z")
            AppendImageFigure(fig, "Fusion Odometry Current State(v_wi)", pdf_file)
        if "state/bias_a_x" in fusion_odometry_data:
            fig = plt.figure()
            plt.plot(fusion_odometry_data["state/time_stamp_s"], [-0.2] * len(fusion_odometry_data["state/time_stamp_s"]), label="bias_a_limit", color="black", linestyle="--")
            plt.plot(fusion_odometry_data["state/time_stamp_s"], [0.2] * len(fusion_odometry_data["state/time_stamp_s"]), label="bias_a_limit", color="black", linestyle="--")
            plt.plot(fusion_odometry_data["state/time_stamp_s"], fusion_odometry_data["state/bias_a_x"], label="bias_a_x")
            plt.plot(fusion_odometry_data["state/time_stamp_s"], fusion_odometry_data["state/bias_a_y"], label="bias_a_y")
            plt.plot(fusion_odometry_data["state/time_stamp_s"], fusion_odometry_data["state/bias_a_z"], label="bias_a_z")
            AppendImageFigure(fig, "Fusion Odometry Current State(bias_a)", pdf_file)
        if "state/bias_g_x" in fusion_odometry_data:
            fig = plt.figure()
            plt.plot(fusion_odometry_data["state/time_stamp_s"], [-0.02] * len(fusion_odometry_data["state/time_stamp_s"]), label="bias_g_limit", color="black", linestyle="--")
            plt.plot(fusion_odometry_data["state/time_stamp_s"], [0.02] * len(fusion_odometry_data["state/time_stamp_s"]), label="bias_g_limit", color="black", linestyle="--")
            plt.plot(fusion_odometry_data["state/time_stamp_s"], fusion_odometry_data["state/bias_g_x"], label="bias_g_x")
            plt.plot(fusion_odometry_data["state/time_stamp_s"], fusion_odometry_data["state/bias_g_y"], label="bias_g_y")
            plt.plot(fusion_odometry_data["state/time_stamp_s"], fusion_odometry_data["state/bias_g_z"], label="bias_g_z")
            AppendImageFigure(fig, "Fusion Odometry Current State(bias_g)", pdf_file)
        if "state/a_wi_x" in fusion_odometry_data:
            fig = plt.figure()
            plt.plot(fusion_odometry_data["state/time_stamp_s"], fusion_odometry_data["state/a_wi_x"], label="a_wi_x")
            plt.plot(fusion_odometry_data["state/time_stamp_s"], fusion_odometry_data["state/a_wi_y"], label="a_wi_y")
            plt.plot(fusion_odometry_data["state/time_stamp_s"], fusion_odometry_data["state/a_wi_z"], label="a_wi_z")
            AppendImageFigure(fig, "Fusion Odometry Current State(a_wi)", pdf_file)
        if "state/w_i_x" in fusion_odometry_data:
            fig = plt.figure()
            plt.plot(fusion_odometry_data["state/time_stamp_s"], fusion_odometry_data["state/w_i_x"], label="w_i_x")
            plt.plot(fusion_odometry_data["state/time_stamp_s"], fusion_odometry_data["state/w_i_y"], label="w_i_y")
            plt.plot(fusion_odometry_data["state/time_stamp_s"], fusion_odometry_data["state/w_i_z"], label="w_i_z")
            AppendImageFigure(fig, "Fusion Odometry Current State(w_i)", pdf_file)

    # Plot sigma std of states.
    if "state_std/time_stamp_s" in fusion_odometry_data:
        AppendTitleFigure(pdf_file, "fusion_odometry", "sigma std of states")
        if "state_std/p_wi_x" in fusion_odometry_data:
            fig = plt.figure()
            plt.plot(fusion_odometry_data["state_std/time_stamp_s"], fusion_odometry_data["state_std/p_wi_x"], label="p_wi_x")
            plt.plot(fusion_odometry_data["state_std/time_stamp_s"], fusion_odometry_data["state_std/p_wi_y"], label="p_wi_y")
            plt.plot(fusion_odometry_data["state_std/time_stamp_s"], fusion_odometry_data["state_std/p_wi_z"], label="p_wi_z")
            plt.ylim(bottom=0.0)
            AppendImageFigure(fig, "Fusion Odometry Sigma Std of States (p_wi)", pdf_file)
        if "state_std/q_wi_x" in fusion_odometry_data:
            fig = plt.figure()
            plt.plot(fusion_odometry_data["state_std/time_stamp_s"], fusion_odometry_data["state_std/q_wi_x"], label="q_wi_x")
            plt.plot(fusion_odometry_data["state_std/time_stamp_s"], fusion_odometry_data["state_std/q_wi_y"], label="q_wi_y")
            plt.plot(fusion_odometry_data["state_std/time_stamp_s"], fusion_odometry_data["state_std/q_wi_z"], label="q_wi_z")
            plt.ylim(bottom=0.0)
            AppendImageFigure(fig, "Fusion Odometry Sigma Std of States (q_wi)", pdf_file)
        if "state_std/v_wi_x" in fusion_odometry_data:
            fig = plt.figure()
            plt.plot(fusion_odometry_data["state_std/time_stamp_s"], fusion_odometry_data["state_std/v_wi_x"], label="v_wi_x")
            plt.plot(fusion_odometry_data["state_std/time_stamp_s"], fusion_odometry_data["state_std/v_wi_y"], label="v_wi_y")
            plt.plot(fusion_odometry_data["state_std/time_stamp_s"], fusion_odometry_data["state_std/v_wi_z"], label="v_wi_z")
            plt.ylim(bottom=0.0)
            AppendImageFigure(fig, "Fusion Odometry Sigma Std of States (v_wi)", pdf_file)
        if "state_std/bias_a_x" in fusion_odometry_data:
            fig = plt.figure()
            plt.plot(fusion_odometry_data["state_std/time_stamp_s"], fusion_odometry_data["state_std/bias_a_x"], label="bias_a_x")
            plt.plot(fusion_odometry_data["state_std/time_stamp_s"], fusion_odometry_data["state_std/bias_a_y"], label="bias_a_y")
            plt.plot(fusion_odometry_data["state_std/time_stamp_s"], fusion_odometry_data["state_std/bias_a_z"], label="bias_a_z")
            plt.ylim(bottom=0.0)
            AppendImageFigure(fig, "Fusion Odometry Sigma Std of States (bias_a)", pdf_file)
        if "state_std/bias_g_x" in fusion_odometry_data:
            fig = plt.figure()
            plt.plot(fusion_odometry_data["state_std/time_stamp_s"], fusion_odometry_data["state_std/bias_g_x"], label="bias_g_x")
            plt.plot(fusion_odometry_data["state_std/time_stamp_s"], fusion_odometry_data["state_std/bias_g_y"], label="bias_g_y")
            plt.plot(fusion_odometry_data["state_std/time_stamp_s"], fusion_odometry_data["state_std/bias_g_z"], label="bias_g_z")
            plt.ylim(bottom=0.0)
            AppendImageFigure(fig, "Fusion Odometry Sigma Std of States (bias_g)", pdf_file)

    # Plot current state of body frame.
    if "state/time_stamp_s" in fusion_odometry_data:
        AppendTitleFigure(pdf_file, "fusion_odometry", "current state of body frame")
        if "state/p_wb_x" in fusion_odometry_data:
            fig = plt.figure()
            plt.plot(fusion_odometry_data["state/time_stamp_s"], fusion_odometry_data["state/p_wb_x"], label="p_wb_x")
            plt.plot(fusion_odometry_data["state/time_stamp_s"], fusion_odometry_data["state/p_wb_y"], label="p_wb_y")
            plt.plot(fusion_odometry_data["state/time_stamp_s"], fusion_odometry_data["state/p_wb_z"], label="p_wb_z")
            AppendImageFigure(fig, "Fusion Odometry Current State(p_wb)", pdf_file)
        if "state/q_wb_x" in fusion_odometry_data:
            roll_list, pitch_list, yaw_list = ConvertQuaternionToEulerAngle(fusion_odometry_data["state/q_wb_x"], fusion_odometry_data["state/q_wb_y"], fusion_odometry_data["state/q_wb_z"], fusion_odometry_data["state/q_wb_w"])
            fig = plt.figure()
            plt.plot(fusion_odometry_data["state/time_stamp_s"], roll_list, label="roll_deg")
            plt.plot(fusion_odometry_data["state/time_stamp_s"], pitch_list, label="pitch_deg")
            plt.plot(fusion_odometry_data["state/time_stamp_s"], yaw_list, label="yaw_deg")
            AppendImageFigure(fig, "Fusion Odometry Current State(q_wb)", pdf_file)
        if "state/v_wb_x" in fusion_odometry_data:
            fig = plt.figure()
            plt.plot(fusion_odometry_data["state/time_stamp_s"], fusion_odometry_data["state/v_wb_x"], label="v_wb_x")
            plt.plot(fusion_odometry_data["state/time_stamp_s"], fusion_odometry_data["state/v_wb_y"], label="v_wb_y")
            plt.plot(fusion_odometry_data["state/time_stamp_s"], fusion_odometry_data["state/v_wb_z"], label="v_wb_z")
            AppendImageFigure(fig, "Fusion Odometry Current State(v_wb)", pdf_file)

    print(">> Analyzing fusion odometry data done.")

def AnalyzeImuManagerData(imu_manager_data, pdf_file):
    print(">> Analyzing imu manager data...")

    # Plot lazy point states.
    if "state/time_stamp_s" in imu_manager_data:
        AppendTitleFigure(pdf_file, "imu_manager", "lazy point motion states")
        if "state/is_attitude_valid" in imu_manager_data:
            fig = plt.figure()
            plt.plot(imu_manager_data["state/time_stamp_s"], imu_manager_data["state/is_attitude_valid"], label="is_attitude_valid")
            plt.plot(imu_manager_data["state/time_stamp_s"], imu_manager_data["state/is_motion_valid"], label="is_motion_valid")
            AppendImageFigure(fig, "IMU Manager Lazy Point States Validation", pdf_file)
        if "state/p_wi_x" in imu_manager_data:
            fig = plt.figure()
            plt.plot(imu_manager_data["state/time_stamp_s"], imu_manager_data["state/p_wi_x"], label="p_wi_x")
            plt.plot(imu_manager_data["state/time_stamp_s"], imu_manager_data["state/p_wi_y"], label="p_wi_y")
            plt.plot(imu_manager_data["state/time_stamp_s"], imu_manager_data["state/p_wi_z"], label="p_wi_z")
            AppendImageFigure(fig, "IMU Manager Lazy Point States (p_wi)", pdf_file)
        if "state/q_wi_x" in imu_manager_data:
            # Convet quaternion to euler angle.
            roll_list, pitch_list, yaw_list = ConvertQuaternionToEulerAngle(imu_manager_data["state/q_wi_x"], imu_manager_data["state/q_wi_y"], imu_manager_data["state/q_wi_z"], imu_manager_data["state/q_wi_w"])
            # Plot.
            fig = plt.figure()
            plt.plot(imu_manager_data["state/time_stamp_s"], roll_list, label="roll_deg")
            plt.plot(imu_manager_data["state/time_stamp_s"], pitch_list, label="pitch_deg")
            plt.plot(imu_manager_data["state/time_stamp_s"], yaw_list, label="yaw_deg")
            AppendImageFigure(fig, "IMU Manager Lazy Point States (q_wi)", pdf_file)
        if "state/v_wi_x" in imu_manager_data:
            fig = plt.figure()
            plt.plot(imu_manager_data["state/time_stamp_s"], imu_manager_data["state/v_wi_x"], label="v_wi_x")
            plt.plot(imu_manager_data["state/time_stamp_s"], imu_manager_data["state/v_wi_y"], label="v_wi_y")
            plt.plot(imu_manager_data["state/time_stamp_s"], imu_manager_data["state/v_wi_z"], label="v_wi_z")
            AppendImageFigure(fig, "IMU Manager Lazy Point States (v_wi)", pdf_file)
        if "state/bias_a_x" in imu_manager_data:
            fig = plt.figure()
            plt.plot(imu_manager_data["state/time_stamp_s"], [-0.2] * len(imu_manager_data["state/time_stamp_s"]), label="bias_a_limit", color="black", linestyle="--")
            plt.plot(imu_manager_data["state/time_stamp_s"], [0.2] * len(imu_manager_data["state/time_stamp_s"]), label="bias_a_limit", color="black", linestyle="--")
            plt.plot(imu_manager_data["state/time_stamp_s"], imu_manager_data["state/bias_a_x"], label="bias_a_x")
            plt.plot(imu_manager_data["state/time_stamp_s"], imu_manager_data["state/bias_a_y"], label="bias_a_y")
            plt.plot(imu_manager_data["state/time_stamp_s"], imu_manager_data["state/bias_a_z"], label="bias_a_z")
            AppendImageFigure(fig, "IMU Manager Lazy Point States (bias_a)", pdf_file)
        if "state/bias_g_x" in imu_manager_data:
            fig = plt.figure()
            plt.plot(imu_manager_data["state/time_stamp_s"], [-0.02] * len(imu_manager_data["state/time_stamp_s"]), label="bias_g_limit", color="black", linestyle="--")
            plt.plot(imu_manager_data["state/time_stamp_s"], [0.02] * len(imu_manager_data["state/time_stamp_s"]), label="bias_g_limit", color="black", linestyle="--")
            plt.plot(imu_manager_data["state/time_stamp_s"], imu_manager_data["state/bias_g_x"], label="bias_g_x")
            plt.plot(imu_manager_data["state/time_stamp_s"], imu_manager_data["state/bias_g_y"], label="bias_g_y")
            plt.plot(imu_manager_data["state/time_stamp_s"], imu_manager_data["state/bias_g_z"], label="bias_g_z")
            AppendImageFigure(fig, "IMU Manager Lazy Point States (bias_g)", pdf_file)

    # Plot imu measurements.
    if "measure/time_stamp_s" in imu_manager_data:
        AppendTitleFigure(pdf_file, "imu_manager", "raw imu measurements")
        if "measure/gyro_x_rps" in imu_manager_data:
            fig = plt.figure()
            plt.plot(imu_manager_data["measure/time_stamp_s"], imu_manager_data["measure/gyro_x_rps"], label="gyro_x_rps")
            plt.plot(imu_manager_data["measure/time_stamp_s"], imu_manager_data["measure/gyro_y_rps"], label="gyro_y_rps")
            plt.plot(imu_manager_data["measure/time_stamp_s"], imu_manager_data["measure/gyro_z_rps"], label="gyro_z_rps")
            AppendImageFigure(fig, "IMU Manager Gyroscope", pdf_file)
        if "measure/accel_x_mps2" in imu_manager_data:
            fig = plt.figure()
            plt.plot(imu_manager_data["measure/time_stamp_s"], imu_manager_data["measure/accel_x_mps2"], label="accel_x_mps2")
            plt.plot(imu_manager_data["measure/time_stamp_s"], imu_manager_data["measure/accel_y_mps2"], label="accel_y_mps2")
            plt.plot(imu_manager_data["measure/time_stamp_s"], imu_manager_data["measure/accel_z_mps2"], label="accel_z_mps2")
            AppendImageFigure(fig, "IMU Manager Acceleration", pdf_file)

    # Plot imu status.
    if "measure/time_stamp_s" in imu_manager_data:
        AppendTitleFigure(pdf_file, "imu_manager", "imu sensor status")
        if "status/is_stationary" in imu_manager_data:
            # Compute acc norm firstly.
            acc_norm = np.sqrt(np.array(imu_manager_data["measure/accel_x_mps2"])**2 + np.array(imu_manager_data["measure/accel_y_mps2"])**2 + np.array(imu_manager_data["measure/accel_z_mps2"])**2)
            gravity_norm = [9.8077] * len(acc_norm)
            # Plot.
            fig = plt.figure()
            plt.plot(imu_manager_data["state/time_stamp_s"], imu_manager_data["status/is_stationary"], label="is_stationary")
            plt.plot(imu_manager_data["state/time_stamp_s"], imu_manager_data["status/is_over_range"], label="is_over_range")
            plt.plot(imu_manager_data["state/time_stamp_s"], acc_norm, label="acc_norm_mps2")
            plt.plot(imu_manager_data["state/time_stamp_s"], gravity_norm, label="gravity_norm_mps2")
            AppendImageFigure(fig, "IMU Manager Stationary Status", pdf_file)

    # Plot imu updater.
    if "measure/time_stamp_s" in imu_manager_data:
        AppendTitleFigure(pdf_file, "imu_manager", "imu updater")
        if "updater/updated_by_gravity" in imu_manager_data:
            fig = plt.figure()
            plt.scatter(imu_manager_data["measure/time_stamp_s"], np.array(imu_manager_data["updater/updated_by_gravity"]) * 1.0, label="updated_by_gravity")
            plt.scatter(imu_manager_data["measure/time_stamp_s"], np.array(imu_manager_data["updater/updated_by_zero_angular_velocity"]) * 2.0, label="updated_by_zero_angular_velocity")
            plt.scatter(imu_manager_data["measure/time_stamp_s"], np.array(imu_manager_data["updater/updated_by_zero_velocity"]) * 3.0, label="updated_by_zero_velocity")
            AppendImageFigure(fig, "IMU Manager Updater", pdf_file)
        if "updater/norm_of_gravity_residual" in imu_manager_data:
            fig = plt.figure()
            plt.plot(imu_manager_data["measure/time_stamp_s"], imu_manager_data["updater/norm_of_gravity_residual"], label="norm_of_gravity_residual")
            plt.plot(imu_manager_data["measure/time_stamp_s"], imu_manager_data["updater/norm_of_zero_angular_velocity_residual"], label="norm_of_zero_angular_velocity_residual")
            plt.plot(imu_manager_data["measure/time_stamp_s"], imu_manager_data["updater/norm_of_zero_velocity_residual"], label="norm_of_zero_velocity_residual")
            AppendImageFigure(fig, "IMU Manager Updater Residual", pdf_file)
        if "updater/norm_of_gravity_delta_x" in imu_manager_data:
            fig = plt.figure()
            plt.plot(imu_manager_data["measure/time_stamp_s"], imu_manager_data["updater/norm_of_gravity_delta_x"], label="norm_of_gravity_delta_x")
            plt.plot(imu_manager_data["measure/time_stamp_s"], imu_manager_data["updater/norm_of_zero_angular_velocity_delta_x"], label="norm_of_zero_angular_velocity_delta_x")
            plt.plot(imu_manager_data["measure/time_stamp_s"], imu_manager_data["updater/norm_of_zero_velocity_delta_x"], label="norm_of_zero_velocity_delta_x")
            AppendImageFigure(fig, "IMU Manager Updater Delta X", pdf_file)

    print(">> Analyzing imu manager data done.")

def AnalyzeVisualManagerData(visual_manager_data, pdf_file):
    print(">> Analyzing visual manager data...")

    # Plot visual manager states.
    if "states/time_stamp_s" in visual_manager_data:
        AppendTitleFigure(pdf_file, "visual_manager", "visual manager states")
        if "states/is_valid" in visual_manager_data:
            fig = plt.figure()
            plt.plot(visual_manager_data["states/time_stamp_s"], visual_manager_data["states/is_valid"], label="is_valid")
            AppendImageFigure(fig, "Visual Manager States", pdf_file)
        # Plot visual extrinsics of each direction.
        dir = 0
        cam_id = 0
        while True:
            status_root_key = "dir " + str(dir) + " status/num_of_stored_frames"
            if status_root_key not in visual_manager_data:
                break
            while True:
                pos_root_key = "states/dir" + str(dir) + "_p_ic_" + str(cam_id)
                if pos_root_key + "_x" not in visual_manager_data:
                    break
                else:
                    fig, axs = plt.subplots(1, 3, gridspec_kw={'left':0.1, 'right':0.95, 'top':0.9, 'bottom':0.1, 'wspace':0.3, 'hspace':0.1})
                    axs[0].plot(visual_manager_data["states/time_stamp_s"], visual_manager_data[pos_root_key + "_x"], label="p_iv_x")
                    axs[0].legend()
                    axs[1].plot(visual_manager_data["states/time_stamp_s"], visual_manager_data[pos_root_key + "_y"], label="p_ic_y")
                    axs[1].legend()
                    axs[2].plot(visual_manager_data["states/time_stamp_s"], visual_manager_data[pos_root_key + "_z"], label="p_ic_z")
                    axs[2].legend()
                    fig.suptitle("Visual Manager States (p_ic_" + str(cam_id) + ") in direction " + str(dir) + "\n ")
                    pdf_file.savefig(fig, bbox_inches=None)
                    plt.close()

                rot_root_key = "states/dir" + str(dir) + "_q_ic_" + str(cam_id)
                if rot_root_key + "_x" not in visual_manager_data:
                    break
                else:
                    roll_list, pitch_list, yaw_list = ConvertQuaternionToEulerAngle(visual_manager_data[rot_root_key + "_x"], visual_manager_data[rot_root_key + "_y"], visual_manager_data[rot_root_key + "_z"], visual_manager_data[rot_root_key + "_w"])
                    fig, axs = plt.subplots(1, 3, gridspec_kw={'left':0.1, 'right':0.95, 'top':0.9, 'bottom':0.1, 'wspace':0.3, 'hspace':0.1})
                    axs[0].plot(visual_manager_data["states/time_stamp_s"], roll_list, label="roll_deg")
                    axs[0].legend()
                    axs[1].plot(visual_manager_data["states/time_stamp_s"], pitch_list, label="pitch_deg")
                    axs[1].legend()
                    axs[2].plot(visual_manager_data["states/time_stamp_s"], yaw_list, label="yaw_deg")
                    axs[2].legend()
                    fig.suptitle("Visual Manager States (q_ic_" + str(cam_id) + ") in direction " + str(dir) + "\n ")
                    pdf_file.savefig(fig, bbox_inches=None)
                    plt.close()
                cam_id += 1
            dir += 1
            cam_id = 0

    # Plot visual manager status.
    if "states/time_stamp_s" in visual_manager_data:
        AppendTitleFigure(pdf_file, "visual_manager", "visual manager public status")
        if "status/is_keyframe" in visual_manager_data:
            fig = plt.figure()
            plt.plot(visual_manager_data["states/time_stamp_s"], visual_manager_data["status/succeed_to_initialize"], label="succeed_to_initialize")
            plt.plot(visual_manager_data["states/time_stamp_s"], visual_manager_data["status/is_keyframe"], label="is_keyframe")
            plt.plot(visual_manager_data["states/time_stamp_s"], visual_manager_data["status/main_direction_id"], label="main_direction_id")
            AppendImageFigure(fig, "Visual Manager Status", pdf_file)
        # Plot status of each direction.
        dir = 0
        while True:
            status_root_key = "dir " + str(dir) + " status/"
            # Plot points used.
            if status_root_key + "num_of_used_points" not in visual_manager_data:
                break
            else:
                AppendTitleFigure(pdf_file, "visual_manager", "visual manager status (direction " + str(dir) + ")")
                fig = plt.figure()
                plt.plot(visual_manager_data["states/time_stamp_s"], visual_manager_data[status_root_key + "num_of_tracked_points"], label="num_of_tracked_points")
                plt.plot(visual_manager_data["states/time_stamp_s"], visual_manager_data[status_root_key + "num_of_triangulized_points"], label="num_of_triangulized_points")
                plt.plot(visual_manager_data["states/time_stamp_s"], visual_manager_data[status_root_key + "num_of_point_candidates"], label="num_of_point_candidates")
                plt.plot(visual_manager_data["states/time_stamp_s"], visual_manager_data[status_root_key + "num_of_outliers"], label="num_of_outliers")
                plt.plot(visual_manager_data["states/time_stamp_s"], visual_manager_data[status_root_key + "num_of_used_points"], label="num_of_used_points")
                AppendImageFigure(fig, "Visual Manager Status (Points Used) in direction " + str(dir), pdf_file)
            # Plot mean reprojection residual.
            if status_root_key + "mean_reprojection_residual" not in visual_manager_data:
                break
            else:
                fig = plt.figure()
                plt.plot(visual_manager_data["states/time_stamp_s"], visual_manager_data[status_root_key + "mean_reprojection_residual"], label="mean_reprojection_residual")
                AppendImageFigure(fig, "Visual Manager Status (Mean Reprojection Residual) in direction " + str(dir), pdf_file)
            # Plot mean max parallex angle.
            if status_root_key + "mean_max_parallex_angle_deg" not in visual_manager_data:
                break
            else:
                fig = plt.figure()
                plt.plot(visual_manager_data["states/time_stamp_s"], visual_manager_data[status_root_key + "current_two_frames_mean_parallex_angle_deg"], label="current_two_frames_mean_parallex_angle_deg")
                plt.plot(visual_manager_data["states/time_stamp_s"], visual_manager_data[status_root_key + "mean_max_parallex_angle_deg"], label="mean_max_parallex_angle_deg")
                AppendImageFigure(fig, "Visual Manager Status (Mean Max Parallex Angle) in direction " + str(dir), pdf_file)
            dir += 1
    # Plot visual updater.
    if "updater/updated_by_points" in visual_manager_data:
        AppendTitleFigure(pdf_file, "visual_manager", "visual manager updater")
        if "updater/updated_by_points" in visual_manager_data:
            fig = plt.figure()
            plt.scatter(visual_manager_data["states/time_stamp_s"], np.array(visual_manager_data["updater/updated_by_points"]) * 1.0, label="updated_by_points")
            AppendImageFigure(fig, "Visual Manager Updater", pdf_file)
        if "updater/norm_of_points_residual" in visual_manager_data:
            fig = plt.figure()
            plt.plot(visual_manager_data["states/time_stamp_s"], visual_manager_data["updater/norm_of_points_residual"], label="norm_of_points_residual")
            AppendImageFigure(fig, "Visual Manager Updater Residual", pdf_file)
        if "updater/norm_of_points_delta_x" in visual_manager_data:
            fig = plt.figure()
            plt.plot(visual_manager_data["states/time_stamp_s"], visual_manager_data["updater/norm_of_points_delta_x"], label="norm_of_points_delta_x")
            AppendImageFigure(fig, "Visual Manager Updater Delta X", pdf_file)

    print(">> Analyzing visual manager data done.")

def AnalyzeLidarManagerData(lidar_manager_data, pdf_file):
    print(">> Analyzing lidar manager data...")

    # Plot lidar manager state.
    if "state/time_stamp_s" in lidar_manager_data:
        AppendTitleFigure(pdf_file, "lidar_manager", "lidar manager state")
        if "state/is_valid" in lidar_manager_data:
            fig = plt.figure()
            plt.plot(lidar_manager_data["state/time_stamp_s"], lidar_manager_data["state/is_valid"], label="is_valid")
            AppendImageFigure(fig, "Lidar Manager States", pdf_file)
        if "state/p_il_x" in lidar_manager_data:
            fig, axs = plt.subplots(1, 3, gridspec_kw={'left':0.1, 'right':0.95, 'top':0.9, 'bottom':0.1, 'wspace':0.3, 'hspace':0.1})
            axs[0].plot(lidar_manager_data["state/time_stamp_s"], lidar_manager_data["state/p_il_x"], label="p_il_x")
            axs[0].legend()
            axs[1].plot(lidar_manager_data["state/time_stamp_s"], lidar_manager_data["state/p_il_y"], label="p_il_y")
            axs[1].legend()
            axs[2].plot(lidar_manager_data["state/time_stamp_s"], lidar_manager_data["state/p_il_z"], label="p_il_z")
            axs[2].legend()
            fig.suptitle("Lidar Manager States (p_il)")
            pdf_file.savefig(fig, bbox_inches=None)
            plt.close()
        if "state/q_il_x" in lidar_manager_data:
            roll_list, pitch_list, yaw_list = ConvertQuaternionToEulerAngle(lidar_manager_data["state/q_il_x"], lidar_manager_data["state/q_il_y"], lidar_manager_data["state/q_il_z"], lidar_manager_data["state/q_il_w"])
            fig, axs = plt.subplots(1, 3, gridspec_kw={'left':0.1, 'right':0.95, 'top':0.9, 'bottom':0.1, 'wspace':0.3, 'hspace':0.1})
            axs[0].plot(lidar_manager_data["state/time_stamp_s"], roll_list, label="roll_deg")
            axs[0].legend()
            axs[1].plot(lidar_manager_data["state/time_stamp_s"], pitch_list, label="pitch_deg")
            axs[1].legend()
            axs[2].plot(lidar_manager_data["state/time_stamp_s"], yaw_list, label="yaw_deg")
            axs[2].legend()
            fig.suptitle("Lidar Manager States (q_il)")
            pdf_file.savefig(fig, bbox_inches=None)
            plt.close()

    # Plot lidar manager status.
    if "state/time_stamp_s" in lidar_manager_data:
        AppendTitleFigure(pdf_file, "lidar_manager", "lidar manager status")
        if "status/is_keyframe" in lidar_manager_data:
            fig = plt.figure()
            plt.plot(lidar_manager_data["state/time_stamp_s"], lidar_manager_data["status/is_keyframe"], label="is_keyframe")
            plt.plot(lidar_manager_data["state/time_stamp_s"], lidar_manager_data["status/overlap_ratio"], label="overlap_ratio")
            AppendImageFigure(fig, "Lidar Manager Status Keyframe", pdf_file)
        if "status/num_of_current_scan_points" in lidar_manager_data:
            fig = plt.figure()
            plt.plot(lidar_manager_data["state/time_stamp_s"], lidar_manager_data["status/num_of_current_scan_points"], label="num_of_current_scan_points")
            plt.plot(lidar_manager_data["state/time_stamp_s"], lidar_manager_data["status/num_of_used_points"], label="num_of_used_points")
            plt.plot(lidar_manager_data["state/time_stamp_s"], lidar_manager_data["status/num_of_temporal_points"], label="num_of_temporal_points")
            plt.plot(lidar_manager_data["state/time_stamp_s"], lidar_manager_data["status/num_of_local_map_points"], label="num_of_local_map_points")
            AppendImageFigure(fig, "Lidar Manager Status Points", pdf_file)
        if "status/degeneration_factor" in lidar_manager_data:
            fig = plt.figure()
            plt.plot(lidar_manager_data["state/time_stamp_s"], lidar_manager_data["status/degeneration_factor"], label="degeneration_factor")
            AppendImageFigure(fig, "Lidar Manager Status Degeneration Factor", pdf_file)
        if "status/main_degeneration_axis" in lidar_manager_data:
            fig = plt.figure()
            plt.plot(lidar_manager_data["state/time_stamp_s"], lidar_manager_data["status/main_degeneration_axis"], label="main_degeneration_axis")
            AppendImageFigure(fig, "Lidar Manager Status Degeneration Axis", pdf_file)
        if "status/position_constrain_intensity_on_degeneration_direction" in lidar_manager_data:
            fig = plt.figure()
            plt.plot(lidar_manager_data["state/time_stamp_s"], lidar_manager_data["status/position_constrain_intensity_on_degeneration_direction"], label="position_constrain_intensity_on_degeneration_direction")
            plt.plot(lidar_manager_data["state/time_stamp_s"], lidar_manager_data["status/rotation_constrain_intensity_on_degeneration_direction"], label="rotation_constrain_intensity_on_degeneration_direction")
            AppendImageFigure(fig, "Lidar Manager Status Degeneration Constrains", pdf_file)

    # Plot lidar manager updater.
    if "updater/updated_by_scan" in lidar_manager_data:
        AppendTitleFigure(pdf_file, "lidar_manager", "lidar manager updater")
        if "updater/updated_by_scan" in lidar_manager_data:
            fig = plt.figure()
            plt.plot(lidar_manager_data["state/time_stamp_s"], lidar_manager_data["updater/updated_by_scan"], label="updated_by_scan")
            AppendImageFigure(fig, "Lidar Manager Updater", pdf_file)
        if "updater/norm_of_scan_residual" in lidar_manager_data:
            fig = plt.figure()
            plt.plot(lidar_manager_data["state/time_stamp_s"], lidar_manager_data["updater/norm_of_scan_residual"], label="norm_of_scan_residual")
            AppendImageFigure(fig, "Lidar Manager Updater Residual", pdf_file)
        if "updater/norm_of_scan_delta_x" in lidar_manager_data:
            fig = plt.figure()
            plt.plot(lidar_manager_data["state/time_stamp_s"], lidar_manager_data["updater/norm_of_scan_delta_x"], label="norm_of_scan_delta_x")
            AppendImageFigure(fig, "Lidar Manager Updater Delta X", pdf_file)
    print(">> Analyzing lidar manager data done.")

def AnalyzeSigmaBalancerData(sigma_balancer_data, pdf_file):
    print(">> Analyzing sigma balancer data...")

    # Plot sigma balancer status.
    if "status/time_stamp_s" in sigma_balancer_data:
        AppendTitleFigure(pdf_file, "sigma_balancer", "sigma balancer status")
        if "status/imu_gravity_updating" in sigma_balancer_data:
            fig = plt.figure()
            plt.plot(sigma_balancer_data["status/time_stamp_s"], sigma_balancer_data["status/imu_gravity_updating"], label="imu_gravity_updating")
            plt.plot(sigma_balancer_data["status/time_stamp_s"], sigma_balancer_data["status/imu_gravity_sigma_scale"], label="imu_gravity_sigma_scale")
            plt.ylim(bottom=0.0)
            AppendImageFigure(fig, "Sigma Balancer Status IMU Gravity", pdf_file)
        if "status/imu_zero_velocity_updating" in sigma_balancer_data:
            fig = plt.figure()
            plt.plot(sigma_balancer_data["status/time_stamp_s"], sigma_balancer_data["status/imu_zero_velocity_updating"], label="imu_zero_velocity_updating")
            plt.plot(sigma_balancer_data["status/time_stamp_s"], sigma_balancer_data["status/imu_zero_velocity_sigma_scale"], label="imu_zero_velocity_sigma_scale")
            plt.ylim(bottom=0.0)
            AppendImageFigure(fig, "Sigma Balancer Status IMU Zero Velocity", pdf_file)
        if "status/imu_zero_angular_velocity_updating" in sigma_balancer_data:
            fig = plt.figure()
            plt.plot(sigma_balancer_data["status/time_stamp_s"], sigma_balancer_data["status/imu_zero_angular_velocity_updating"], label="imu_zero_angular_velocity_updating")
            plt.plot(sigma_balancer_data["status/time_stamp_s"], sigma_balancer_data["status/imu_zero_angular_velocity_sigma_scale"], label="imu_zero_angular_velocity_sigma_scale")
            plt.ylim(bottom=0.0)
            AppendImageFigure(fig, "Sigma Balancer Status IMU Zero Angular Velocity", pdf_file)
        if "status/visual_points_updating" in sigma_balancer_data:
            fig = plt.figure()
            plt.plot(sigma_balancer_data["status/time_stamp_s"], sigma_balancer_data["status/visual_points_updating"], label="visual_points_updating")
            plt.plot(sigma_balancer_data["status/time_stamp_s"], sigma_balancer_data["status/visual_points_sigma_scale"], label="visual_points_sigma_scale")
            plt.ylim(bottom=0.0)
            AppendImageFigure(fig, "Sigma Balancer Status Visual Points", pdf_file)
        if "status/lidar_scan_updating" in sigma_balancer_data:
            fig = plt.figure()
            plt.plot(sigma_balancer_data["status/time_stamp_s"], sigma_balancer_data["status/lidar_scan_updating"], label="lidar_scan_updating")
            plt.plot(sigma_balancer_data["status/time_stamp_s"], sigma_balancer_data["status/lidar_scan_sigma_scale"], label="lidar_scan_sigma_scale")
            plt.ylim(bottom=0.0)
            AppendImageFigure(fig, "Sigma Balancer Status Lidar Scan", pdf_file)
    print(">> Analyzing sigma balancer data done.")

def AnalyzeMonitorData(monitor_data, pdf_file):
    print(">> Analyzing monitor data...")

    # Plot monitor states.
    if "states/time_stamp_s" in monitor_data:
        AppendTitleFigure(pdf_file, "monitor", "monitor states")
        if "states/fusion_score" in monitor_data:
            fig = plt.figure()
            plt.plot(monitor_data["states/time_stamp_s"], monitor_data["states/fusion_score"], label="fusion_score")
            plt.plot(monitor_data["states/time_stamp_s"], monitor_data["states/imu_score"], label="imu_score")
            plt.plot(monitor_data["states/time_stamp_s"], monitor_data["states/visual_score"], label="visual_score")
            plt.plot(monitor_data["states/time_stamp_s"], monitor_data["states/lidar_score"], label="lidar_score")
            AppendImageFigure(fig, "Monitor States", pdf_file)

    print(">> Analyzing monitor data done.")

if __name__ == "__main__":
    # Validate benchmark file paths.
    all_log_file_paths = ScanAllCsvLogFiles(kBenchmarkLogsRootPath)
    if len(all_log_file_paths) == 0:
        print("\033[91m>> No benchmark file paths found.\033[0m")
        exit(1)
    for csv_log_file_path in all_log_file_paths:
        print("\033[92m>> Succeed to scan benchmark log file path: " + csv_log_file_path + "\033[0m")

    # If path of output exists, clear and create it.
    if os.path.exists(kBenchmarkOutputRootPath):
        os.system("rm -rf " + kBenchmarkOutputRootPath)
    os.system("mkdir -p " + kBenchmarkOutputRootPath)

    # Config plots.
    # print(plt.rcParams.keys())
    plt.rcParams["figure.figsize"] = [12.0, 3.5]
    plt.rcParams["figure.autolayout"] = True
    plt.rcParams["axes.grid"] = True
    plt.rcParams["grid.alpha"] = 0.5
    plt.style.use('dark_background')

    # Iterate each benchmark file path.
    for csv_log_file_path in all_log_file_paths:
        print("\033[91m>> Evaluating on benchmark log file path: " + csv_log_file_path + "\033[0m")

        # Create pdf file.
        leaf_folder_name = csv_log_file_path.split("/")[-1]
        pdf_file_path = kBenchmarkOutputRootPath + "/" + leaf_folder_name + ".pdf"
        pdf_file = PdfPages(pdf_file_path)

        # Plot basic information.
        AppendPdfFileHeader(pdf_file, csv_log_file_path)

        # Read and analyze fusion odometry log.
        csv_log_file_name = csv_log_file_path + "/fusion_odometry.csv"
        data = LoadCsvData(csv_log_file_name)
        if data is not {}:
            AnalyzeFusionOdometryData(data, pdf_file)

        # Read and analyze imu manager log.
        csv_log_file_name = csv_log_file_path + "/imu_manager.csv"
        data = LoadCsvData(csv_log_file_name)
        if data is not {}:
            AnalyzeImuManagerData(data, pdf_file)

        # Read and analyze visual manager log.
        csv_log_file_name = csv_log_file_path + "/visual_manager.csv"
        data = LoadCsvData(csv_log_file_name)
        if data is not {}:
            AnalyzeVisualManagerData(data, pdf_file)

        # Read and analyze lidar manager log.
        csv_log_file_name = csv_log_file_path + "/lidar_manager.csv"
        data = LoadCsvData(csv_log_file_name)
        if data is not {}:
            AnalyzeLidarManagerData(data, pdf_file)

        # Read and analyze sigma balancer log.
        csv_log_file_name = csv_log_file_path + "/sigma_balancer.csv"
        data = LoadCsvData(csv_log_file_name)
        if data is not {}:
            AnalyzeSigmaBalancerData(data, pdf_file)

        # Read and analyze monitor log.
        csv_log_file_name = csv_log_file_path + "/monitor.csv"
        data = LoadCsvData(csv_log_file_name)
        if data is not {}:
            AnalyzeMonitorData(data, pdf_file)

        plt.cla()
        plt.close("all")
        pdf_file.close()
