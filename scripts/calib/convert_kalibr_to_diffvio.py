import yaml
import os
import argparse
import numpy as np

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
DEFAULT_OUTPUT_DIR = os.path.join(
    REPO_ROOT,
    "docker/docker_diff_vio/df_visual_frontend/config/f4_v2",
)


def convert_to_mei(original_data, cam_index):
    """Convert original camera parameters to MEI format."""
    cam_key = f'cam{cam_index}'
    if cam_key not in original_data:
        raise ValueError(f"Camera {cam_key} not found in original data")
    
    cam_data = original_data[cam_key]
    intrinsics = cam_data['intrinsics']
    if len(intrinsics) < 5:
        raise ValueError(f"Camera {cam_key} intrinsics length < 5, cannot export MEI.")
    
    # Build MEI-style dict
    mei_data = {
        'model_type': 'MEI',
        'camera_name': f'camera{cam_index}',
        'image_width': cam_data['resolution'][0],
        'image_height': cam_data['resolution'][1],
        'mirror_parameters': {
            'xi': intrinsics[0]
        },
        'distortion_parameters': {
            'k1': cam_data['distortion_coeffs'][0],
            'k2': cam_data['distortion_coeffs'][1],
            'p1': cam_data['distortion_coeffs'][2],
            'p2': cam_data['distortion_coeffs'][3]
        },
        'projection_parameters': {
            'gamma1': intrinsics[1],
            'gamma2': intrinsics[2],
            'u0': intrinsics[3],
            'v0': intrinsics[4]
        }
    }
    
    return mei_data

def save_mei_yaml(mei_data, output_dir, cam_index):
    """Save MEI camera model to YAML."""
    output_path = os.path.join(output_dir, f'cam{cam_index}_fisheye.yaml')
    with open(output_path, 'w') as f:
        # Write YAML header
        f.write("%YAML:1.0\n")
        f.write("---\n")
        # Write YAML content
        yaml.dump(mei_data, f, default_flow_style=False, sort_keys=False)
    print(f"Saved MEI parameters to {output_path}")

def invert_transform_matrix(matrix_data):
    """Invert a 4x4 transform matrix."""
    # Convert list to 4x4 numpy matrix
    matrix = np.array(matrix_data).reshape(4, 4)
    
    # Extract rotation R and translation t
    R = matrix[:3, :3]
    t = matrix[:3, 3]
    
    # Compute inverse
    R_inv = R.T
    t_inv = -R_inv @ t
    
    # Build full 4x4 inverse
    inv_matrix = np.eye(4)
    inv_matrix[:3, :3] = R_inv
    inv_matrix[:3, 3] = t_inv
    
    return inv_matrix.flatten().tolist()

def save_virtual_stereo_yaml(original_data, output_dir):
    """Generate f4_v2_virtural_stereo.yaml matching the *_true.yaml layout."""
    output_path = os.path.join(output_dir, 'f4_v2_virtural_stereo.yaml')
    static_lines = [
        "%YAML:1.0",
        "---",
        "",
        'camera_topic: ["/fisheye/left/image_raw", "/fisheye/right/image_raw", "/fisheye/bright/image_raw", "/fisheye/bleft/image_raw"]  # camera topic names',
        'camera_info:  ["cam0_fisheye.yaml", "cam1_fisheye.yaml", "cam2_fisheye.yaml", "cam3_fisheye.yaml"]                              # camera info file names',
        'camera_mask:  ["240_320_mask/mask-240-320-3.png","240_320_mask/mask-240-320-3.png","240_320_mask/mask-240-320-3.png","240_320_mask/mask-240-320-3.png"]',
        "",
        "# frontend configuration",
        'tracking_type: "virtual_stereo"       # tracking type: ["virtual_stereo", "general_stereo", "EIS_fisheye"]',
        "max_feature_per_frame: 150            # max feature number in feature tracking",
        "min_feature_distance: 20              # min distance between two features",
        "inv_flow_dis_thresh: 0.5              # inverse flow distance threshold",
        "quality_level: 0.01                   # quality level for Shi-Tomasi corner detection",
        "track_show_status: [1, 1]             # tracking results visualization [enable/disable, draw features, draw ids]",
        "always_keyframe: 1                 # always keyframe: [0, 1]",
        "",
        "# virtual stereo configuration",
        "virtual_stereo_image_width: 240             # virtual stereo image width",
        "virtual_stereo_image_height: 320            # virtual stereo image height",
        "virtual_stereo_image_fov: 90.0              # virtual stereo image field of view",
        "virtual_stereo_rotation_angle_left: -45.0   # virtual stereo rotation angle for left camera",
        "virtual_stereo_rotation_angle_right: 45.0   # virtual stereo rotation angle for right camera",
        "",
        "# CAM0------CAM1",
        "# |           |",
        "# |           |",
        "# CAM3------CAM2",
        "",
    ]

    def wrap_flow_sequence(values, per_line=4):
        lines = []
        total = len(values)
        for idx in range(0, total, per_line):
            chunk = values[idx: idx + per_line]
            prefix = "    data: [" if idx == 0 else "      "
            suffix = "]" if (idx + per_line) >= total else ","
            joined = ", ".join(str(v) for v in chunk)
            lines.append(f"{prefix}{joined}{suffix}")
        if not lines:
            lines.append("    data: []")
        return "\n".join(lines)

    def format_matrix(name, values):
        header = [
            f"{name}: !!opencv-matrix",
            "    rows: 4",
            "    cols: 4",
            "    dt: d",
        ]
        data_block = wrap_flow_sequence(values)
        return "\n".join(header + [data_block])

    body_blocks = []
    for cam_index in range(4):
        cam_key = f'cam{cam_index}'
        if cam_key not in original_data:
            raise ValueError(f"Camera {cam_key} not found for body_T export")
        matrix_data = original_data[cam_key]['T_cam_imu']
        flattened = invert_transform_matrix(matrix_data)
        body_blocks.append(format_matrix(f"body_T_cam{cam_index}", flattened))

    with open(output_path, 'w') as f:
        f.write("\n".join(static_lines) + "\n" + "\n\n".join(body_blocks) + "\n")
    print(f"Saved virtual stereo parameters to {output_path}")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-i', '--input', required=True, help='Input Kalibr YAML file path')
    parser.add_argument('-o', '--output', default=DEFAULT_OUTPUT_DIR, help='Output directory')
    args = parser.parse_args()

    # Load input YAML
    with open(args.input, 'r') as f:
        original_data = yaml.safe_load(f)
    
    # Ensure output directory exists
    output_dir = args.output
    os.makedirs(output_dir, exist_ok=True)
    
    # Convert and save all cameras (cam0-cam3)
    for cam_index in range(4):
        try:
            mei_data = convert_to_mei(original_data, cam_index)
            save_mei_yaml(mei_data, output_dir, cam_index)
        except ValueError as e:
            print(f"Skipping camera {cam_index} EMI convert: {str(e)}")

    # Convert and save virtual stereo extrinsics
    try:
        save_virtual_stereo_yaml(original_data, output_dir)
    except ValueError as e:
        print(f"Skipping virtual stereo export: {str(e)}")
    
    print("Conversion completed!")

if __name__ == '__main__':
    main()
