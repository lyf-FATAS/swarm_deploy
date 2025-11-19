#!/usr/bin/env zsh
set -e
set -u
set -o pipefail

# Usage: gen_vio_backend_config.zsh [container_name] [target_yaml]
# Default container: diff_vio. Default target yaml: fusion_odometry/config/virtual_stereo_seek_v2.yaml

SCRIPT_DIR="${0:A:h}"
REPO_ROOT="${SCRIPT_DIR:h:h}"
CONTAINER_NAME="${1:-${DIFF_VIO_CONTAINER:-diff_vio}}"
TARGET_YAML="${2:-${REPO_ROOT}/docker/docker_diff_vio/fusion_odometry/config/virtual_stereo_seek_v2.yaml}"
REMOTE_TXT="/root/.ros/virtual_stereo_ex_param.txt"

if ! docker ps --format '{{.Names}}' | grep -Fx "${CONTAINER_NAME}" >/dev/null; then
    echo "Container ${CONTAINER_NAME} is not running. Start the VIO docker (might consider launching run.zsh) first." >&2
    exit 1
fi

tmpfile="$(mktemp)"
trap 'rm -f "${tmpfile}"' EXIT

mkdir -p "$(dirname "${TARGET_YAML}")"

if ! docker exec "${CONTAINER_NAME}" sh -c "cat '${REMOTE_TXT}'" > "${tmpfile}"; then
    echo "Failed to read ${REMOTE_TXT} from container ${CONTAINER_NAME}; please check the file exists." >&2
    exit 1
fi

python3 - "${tmpfile}" "${TARGET_YAML}" <<'PYCODE'
import re
import sys
from pathlib import Path
from textwrap import dedent

txt_path = Path(sys.argv[1])
yaml_path = Path(sys.argv[2])

raw = txt_path.read_text().splitlines()
pairs = []
for line in raw:
    tokens = [t for t in re.split(r'[,\s]+', line.strip()) if t]
    if len(tokens) >= 7:
        t = list(map(float, tokens[:3]))
        q = list(map(float, tokens[3:7]))
        pairs.append((t, q))

if not pairs:
    sys.exit("No extrinsic data found in virtual_stereo_ex_param.txt")

p_flat = [v for t, _ in pairs for v in t]
q_flat = [v for _, q in pairs for v in q]

base_text = yaml_path.read_text() if yaml_path.exists() else ""

def build_block(values, per_line, key):
    lines = []
    for idx in range(0, len(values), per_line):
        chunk = ",".join(f"{v:.6f}" for v in values[idx: idx + per_line])
        suffix = "," if (idx + per_line) < len(values) else "]"
        prefix = f"    {key}: [" if idx == 0 else "                          "
        lines.append(f"{prefix}{chunk}{suffix}")
    return "\n".join(lines)

p_block = build_block(p_flat, 3, "p_ic_of_all_cameras")
q_block = build_block(q_flat, 4, "q_ic_of_all_cameras")

start_p = base_text.find("p_ic_of_all_cameras:")
start_q = base_text.find("q_ic_of_all_cameras:")
if start_p != -1 and start_q != -1:
    line_start = base_text.rfind("\n", 0, start_p) + 1
    dbg_pos = base_text.find("debug:", start_q)
    if dbg_pos == -1:
        dbg_pos = len(base_text)
    indent_start = base_text.rfind("\n", start_q, dbg_pos)
    end_q = indent_start + 1 if indent_start != -1 else dbg_pos
    new_text = base_text[:line_start] + p_block + "\n" + q_block + "\n" + base_text[end_q:]
else:
    prefix = dedent("""\
    %YAML:1.0
    ---

    imu_topic: ["/mavros/imu/data"]
    visual_topic: ["/visual_track_result"]
    lidar_topic: [""]

    fusion:
        p_ib: [0.0, 0.0, 0.0]
        q_ib: [1.0, 0.0, 0.0, 0.0]

    imu_manager:
        enable_initialize_when_stationary: 1
        lazy_point_time_s: 0.35
        stationary_detect:
            accel_peak_peak: 0.3
            gyro_peak_peak: 0.2
        over_range_detect:
            accel_norm: 30.0
            gyro_norm: 10.0
        imu_model:
            accel_noise_sigma: 0.02
            gyro_noise_sigma: 0.008
            accel_random_walk_sigma: 0.005
            gyro_random_walk_sigma: 0.0008
        state_init_sigma:
            p_wi: 1e-2
            q_wi: 1e-3
            v_wi: 1e-2
            bias_a: 1e-2
            bias_g: 5e-4
        updaters:
            gravity:
                enable_update: 1
                default_sigma: 10.0
            zero_velocity:
                enable_update: 1
                default_sigma: 0.01
            zero_angular_velocity:
                enable_update: 1
                default_sigma: 0.004

    visual_manager:
        enable: 1
        fix_all_p_ic: 1
        max_number_of_cloned_poses: 6
        outliers_rejection:
            max_reprojection_error: 0.008
        max_number_of_points_to_use: 40
        min_number_of_points_to_use: 6
        keyframe_selection:
            min_tracking_ratio: 0.5
            max_mean_parallex_angle: 7.0
            min_number_of_triangulized_points: 15
        state_init_sigma:
            time_delay_s: 0.0
            p_ic: 1e-7
            q_ic: 1e-7
        updaters:
            points:
                enable_update: 1
                default_sigma: 0.008

        number_of_cameras_per_direction: [2, 2, 2, 2]
    """).lstrip("\n")

    suffix = dedent("""\
        debug:
            draw_local_map: 0

    lidar_manager:
        enable: 0
    """).lstrip("\n")

    new_text = prefix + p_block + "\n" + q_block + "\n" + suffix

yaml_path.write_text(new_text)

print(f"Updated {yaml_path} p_ic_of_all_cameras / q_ic_of_all_cameras")
PYCODE
