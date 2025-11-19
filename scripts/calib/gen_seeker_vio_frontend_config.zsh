#!/usr/bin/env zsh
set -e
set -u
set -o pipefail

SCRIPT_DIR="${0:A:h}"
REPO_ROOT="${SCRIPT_DIR:h:h}"
DRONE_ID="${DRONE_ID:-0}"

CALIB_DIR="${REPO_ROOT}/calib/drone_${DRONE_ID}"
SEEKER_SCRIPT_DIR="${REPO_ROOT}/src/estimation/seeker/script"
OUT_DIR="${REPO_ROOT}/src/estimation/seeker/config/seeker_omni_depth"
DIFFVIO_DIR="${REPO_ROOT}/docker/docker_diff_vio/df_visual_frontend/config/f4_v2"
CONVERT_SCRIPT="${SCRIPT_DIR}/convert_kalibr_to_diffvio.py"

INPUT_YAML="${1:-}"

if [[ -z "${INPUT_YAML}" ]]; then
    yaml_candidates=("${(@f)$(find "${CALIB_DIR}" -maxdepth 1 -type f -name "drone*_cam0123_*hz_*+cam01_*hz_imu_*=result.yaml" | sort)}")
    if (( ${#yaml_candidates[@]} == 0 )); then
        echo "No matching YAML found in ${CALIB_DIR} (pattern: drone*_cam0123_*hz_*+cam01_*hz_imu_*=result.yaml). Pass the file path as an argument or set DRONE_ID." >&2
        exit 1
    fi
    INPUT_YAML="${yaml_candidates[1]}"
fi

if [[ ! -f "${INPUT_YAML}" ]]; then
    echo "Input YAML not found: ${INPUT_YAML}" >&2
    exit 1
fi

mkdir -p "${OUT_DIR}"

pushd "${CALIB_DIR}" >/dev/null
echo "Running 3_get_undistort_kalibr_info.py on ${INPUT_YAML}"
python3 "${SEEKER_SCRIPT_DIR}/3_get_undistort_kalibr_info.py" "${INPUT_YAML}"
mv -f kalibr_cam_chain.yaml "${OUT_DIR}/kalibr_cam_chain.yaml"
mv -f kalibr_imucam_chain.yaml "${OUT_DIR}/kalibr_imucam_chain.yaml"
popd >/dev/null
echo "Wrote kalibr_cam_chain.yaml and kalibr_imucam_chain.yaml to ${OUT_DIR}"

echo ""
echo "Running 89generate_cali.py to produce calib binary"
python3 "${SEEKER_SCRIPT_DIR}/89generate_cali.py" "${OUT_DIR}/kalibr_cam_chain.yaml"
if [[ -f /tmp/cali ]]; then
    mv -f /tmp/cali "${OUT_DIR}/cali"
    echo "Wrote calib binary to ${OUT_DIR}/cali"
else
    echo "Expected /tmp/cali not found after running 89generate_cali.py" >&2
    exit 1
fi

echo ""
echo "Converting kalibr_cam_chain.yaml to diff_vio configs at ${DIFFVIO_DIR}"
python3 "${CONVERT_SCRIPT}" -i "${OUT_DIR}/kalibr_cam_chain.yaml" -o "${DIFFVIO_DIR}"

echo ""
echo "Completed generating configs of Seeker ROS wrapper and VIO frontend for drone ${DRONE_ID}."
