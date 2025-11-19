#!/usr/bin/env zsh
set -e
set -u
set -o pipefail

# Save all calibration outputs for a drone into calib/drone_${DRONE_ID}
# Usage: ./scripts/calib/save_calib_results.zsh

SCRIPT_DIR="${0:A:h}"
REPO_ROOT="${SCRIPT_DIR:h:h}"
DRONE_ID="${DRONE_ID:-0}"
DEST="${REPO_ROOT}/calib/drone_${DRONE_ID}"

mkdir -p "${DEST}"

sources=(
  "${REPO_ROOT}/src/estimation/seeker/config/seeker_omni_depth/cali"
  "${REPO_ROOT}/src/estimation/seeker/config/seeker_omni_depth/kalibr_cam_chain.yaml"
  "${REPO_ROOT}/src/estimation/seeker/config/seeker_omni_depth/kalibr_imucam_chain.yaml"
  "${REPO_ROOT}/docker/docker_diff_vio/df_visual_frontend/config/f4_v2/cam0_fisheye.yaml"
  "${REPO_ROOT}/docker/docker_diff_vio/df_visual_frontend/config/f4_v2/cam1_fisheye.yaml"
  "${REPO_ROOT}/docker/docker_diff_vio/df_visual_frontend/config/f4_v2/cam2_fisheye.yaml"
  "${REPO_ROOT}/docker/docker_diff_vio/df_visual_frontend/config/f4_v2/cam3_fisheye.yaml"
  "${REPO_ROOT}/docker/docker_diff_vio/df_visual_frontend/config/f4_v2/f4_v2_virtural_stereo.yaml"
  "${REPO_ROOT}/docker/docker_diff_vio/fusion_odometry/config/virtual_stereo_seek_v2.yaml"
)

for src in "${sources[@]}"; do
  if [[ -e "${src}" ]]; then
    cp -rf "${src}" "${DEST}/"
    echo "Saved $(basename "${src}")"
  else
    echo "Warning: ${src} not found, skipped." >&2
  fi
done

echo "Calibration artifacts saved to ${DEST}"
