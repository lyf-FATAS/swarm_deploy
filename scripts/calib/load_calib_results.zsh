#!/usr/bin/env zsh
set -e
set -u
set -o pipefail

# Load calibration outputs from calib/drone_${DRONE_ID} back into seeker & VIO configs
# Usage: ./scripts/calib/load_calib_results.zsh

SCRIPT_DIR="${0:A:h}"
REPO_ROOT="${SCRIPT_DIR:h:h}"
DRONE_ID="${DRONE_ID:-0}"
SRC_DIR="${REPO_ROOT}/calib/drone_${DRONE_ID}"

sources=(
  "${SRC_DIR}/cali::${REPO_ROOT}/src/estimation/seeker/config/seeker_omni_depth/cali"
  "${SRC_DIR}/kalibr_cam_chain.yaml::${REPO_ROOT}/src/estimation/seeker/config/seeker_omni_depth/kalibr_cam_chain.yaml"
  "${SRC_DIR}/kalibr_imucam_chain.yaml::${REPO_ROOT}/src/estimation/seeker/config/seeker_omni_depth/kalibr_imucam_chain.yaml"
  "${SRC_DIR}/cam0_fisheye.yaml::${REPO_ROOT}/docker/docker_diff_vio/df_visual_frontend/config/f4_v2/cam0_fisheye.yaml"
  "${SRC_DIR}/cam1_fisheye.yaml::${REPO_ROOT}/docker/docker_diff_vio/df_visual_frontend/config/f4_v2/cam1_fisheye.yaml"
  "${SRC_DIR}/cam2_fisheye.yaml::${REPO_ROOT}/docker/docker_diff_vio/df_visual_frontend/config/f4_v2/cam2_fisheye.yaml"
  "${SRC_DIR}/cam3_fisheye.yaml::${REPO_ROOT}/docker/docker_diff_vio/df_visual_frontend/config/f4_v2/cam3_fisheye.yaml"
  "${SRC_DIR}/f4_v2_virtural_stereo.yaml::${REPO_ROOT}/docker/docker_diff_vio/df_visual_frontend/config/f4_v2/f4_v2_virtural_stereo.yaml"
  "${SRC_DIR}/virtual_stereo_seek_v2.yaml::${REPO_ROOT}/docker/docker_diff_vio/fusion_odometry/config/virtual_stereo_seek_v2.yaml"
)

if [[ ! -d "${SRC_DIR}" ]]; then
  echo "Source directory ${SRC_DIR} not found." >&2
  exit 1
fi

for entry in "${sources[@]}"; do
  src="${entry%%::*}"
  dst="${entry##*::}"
  if [[ -e "${src}" ]]; then
    mkdir -p "$(dirname "${dst}")"
    cp -rf "${src}" "${dst}"
    echo "Restored $(basename "${src}") -> ${dst}"
  else
    echo "Warning: ${src} not found, skipped." >&2
  fi
done

echo "Calibration artifacts loaded from ${SRC_DIR}"
