# Calibration Tool Scripts

Run the commands from the repo root so the relative paths resolve correctly (scripts also derive paths from their own location).

## gen_seeker_vio_frontend_config.zsh
- Purpose: Generate seeker ROS wrapper configs and VIO frontend configs from a Kalibr YAML.
- Inputs: `DRONE_ID` env (defaults to 0); optional Kalibr YAML path. If omitted, the script searches `calib/drone_${DRONE_ID}` for `drone*_cam0123_*hz_*+cam01_*hz_imu_*=result.yaml`.
- Outputs: `src/estimation/seeker/config/seeker_omni_depth/{kalibr_cam_chain.yaml, kalibr_imucam_chain.yaml, cali}`, plus diff_vio frontend configs in `docker/docker_diff_vio/df_visual_frontend/config/f4_v2/`.
- Example (`DRONE_ID` already set):
    ```zsh
    ./scripts/calib/gen_seeker_vio_frontend_config.zsh
    ```
- With explicit inputs:
    ```zsh
    DRONE_ID=1 ./scripts/calib/gen_seeker_vio_frontend_config.zsh calib/drone_1/drone1_cam0123_10hz_1+cam01_5hz_imu_0=result.yaml
    ```

## gen_vio_backend_config.zsh
- Purpose: Generate VIO backend config (`virtual_stereo_seek_v2.yaml`) by pulling extrinsics from a running VIO container.
- Inputs: a running VIO container with `/root/.ros/virtual_stereo_ex_param.txt` available; container name (default `diff_vio`, or override via first argument or `DIFF_VIO_CONTAINER` env); optional target YAML path (default `docker/docker_diff_vio/fusion_odometry/config/virtual_stereo_seek_v2.yaml`).
- Behavior: Replaces `p_ic_of_all_cameras` and `q_ic_of_all_cameras` blocks in the target file; if the file does not exist, writes a full default template.
- Example:
    ```zsh
    ./scripts/calib/gen_vio_backend_config.zsh
    ```
- With explicit container/target:
    ```zsh
    ./scripts/calib/gen_vio_backend_config.zsh my_vio docker/docker_diff_vio/fusion_odometry/config/virtual_stereo_seek_v2.yaml
    ```

## save_calib_results.zsh
- Purpose: Save all calibration artifacts for the current drone to `calib/drone_${DRONE_ID}`.
- Inputs: `DRONE_ID` env (defaults to 0).
- Collected files: seeker `cali` binary, `kalibr_cam_chain.yaml`, `kalibr_imucam_chain.yaml`, diff_vio frontend configs (`cam0~3_fisheye.yaml`, `f4_v2_virtural_stereo.yaml`), and backend config `virtual_stereo_seek_v2.yaml`.
- Example:
    ```zsh
    ./scripts/calib/save_calib_results.zsh
    ```

## load_calib_results.zsh
- Purpose: Restore saved calibration artifacts from `calib/drone_${DRONE_ID}` back into seeker and diff_vio paths.
- Inputs: `DRONE_ID` env (defaults to 0); expects files produced by `save_calib_results.zsh` in `calib/drone_${DRONE_ID}`.
- Example:
    ```zsh
    ./scripts/calib/load_calib_results.zsh
    ```
