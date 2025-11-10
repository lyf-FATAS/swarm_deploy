import yaml
import os
import argparse
import numpy as np

def convert_to_mei(original_data, cam_index):
    """将原始相机参数转换为MEI格式"""
    cam_key = f'cam{cam_index}'
    if cam_key not in original_data:
        raise ValueError(f"Camera {cam_key} not found in original data")
    
    cam_data = original_data[cam_key]
    
    # 构建MEI格式的字典
    mei_data = {
        'model_type': 'MEI',
        'camera_name': f'camera{cam_index}',
        'image_width': cam_data['resolution'][0],
        'image_height': cam_data['resolution'][1],
        'mirror_parameters': {
            'xi': cam_data['intrinsics'][0]
        },
        'distortion_parameters': {
            'k1': cam_data['distortion_coeffs'][0],
            'k2': cam_data['distortion_coeffs'][1],
            'p1': cam_data['distortion_coeffs'][2],
            'p2': cam_data['distortion_coeffs'][3]
        },
        'projection_parameters': {
            'gamma1': cam_data['intrinsics'][1],
            'gamma2': cam_data['intrinsics'][2],
            'u0': cam_data['intrinsics'][3],
            'v0': cam_data['intrinsics'][4]
        }
    }
    
    return mei_data

def convert_to_camera_tf(original_data):
    """将原始相机 tf 配置转换为目标格式"""
    target_data = {}
    
    # 添加YAML文件头
    target_data['%YAML'] = '1.0'
    
    # 转换每个相机配置
    for cam_index in range(4):
        cam_key = f'cam{cam_index}'
        if cam_key not in original_data:
            raise ValueError(f"Camera {cam_key} not found")
    
        # 获取原始矩阵数据
        matrix_data = original_data[cam_key]['T_cam_imu']

        # inverse
        flattened = invert_transform_matrix(matrix_data)

        # # 将4x4矩阵展平为一维列表
        # flattened = []
        # for row in matrix_data:
        #     flattened.extend(row)
        
        # 构建OpenCV矩阵格式的变换矩阵
        target_data[f'body_T_{cam_key}'] = {
            '!!opencv-matrix': {
                'rows': 4,
                'cols': 4,
                'dt': 'd',
                'data': flattened
            }
        }
    
    return target_data

def save_mei_yaml(mei_data, output_dir, cam_index):
    """保存为MEI格式的YAML文件"""
    output_path = os.path.join(output_dir, f'cam{cam_index}_fisheye.yaml')
    with open(output_path, 'w') as f:
        # 写入YAML文件头
        f.write("%YAML:1.0\n")
        f.write("---\n")
        # 写入YAML内容
        yaml.dump(mei_data, f, default_flow_style=False, sort_keys=False)
    print(f"Saved MEI parameters to {output_path}")

def save_camera_tf_yaml(camera_tf, output_dir):
    """保存为格式YAML文件"""
    output_path = os.path.join(output_dir, f'camera_tf.yaml')
    with open(output_path, 'w') as f:
        # 写入YAML内容
        yaml.dump(camera_tf, f, default_flow_style=None, sort_keys=False)
    print(f"Saved camera tf parameters to {output_path}")

def invert_transform_matrix(matrix_data):
    """对4x4变换矩阵进行求逆运算"""
    # 将列表转换为4x4 numpy矩阵
    matrix = np.array(matrix_data).reshape(4, 4)
    
    # 提取旋转部分R和平移部分t
    R = matrix[:3, :3]
    t = matrix[:3, 3]
    
    # 计算逆矩阵
    R_inv = R.T  # 旋转矩阵的逆就是它的转置
    t_inv = -R_inv @ t
    
    # 构建完整的4x4逆矩阵
    inv_matrix = np.eye(4)
    inv_matrix[:3, :3] = R_inv
    inv_matrix[:3, 3] = t_inv
    
    return inv_matrix.flatten().tolist()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-i', '--input', required=True, help='输入YAML文件路径')
    args = parser.parse_args()

    # 加载原始YAML文件
    with open(args.input, 'r') as f:
        original_data = yaml.safe_load(f)
    
    # 创建输出目录
    output_dir = 'converted_config'
    os.makedirs(output_dir, exist_ok=True)
    
    # 转换并保存所有相机 MEI
    for cam_index in range(4):  # 假设有4个相机(cam0-cam3)
        try:
            mei_data = convert_to_mei(original_data, cam_index)
            save_mei_yaml(mei_data, output_dir, cam_index)
        except ValueError as e:
            print(f"Skipping camera {cam_index} EMI convert: {str(e)}")

    # 转换并保存相机 TF
    try:
        camera_tf_data = convert_to_camera_tf(original_data)
        save_camera_tf_yaml(camera_tf_data, output_dir)
    except ValueError as e:
        print(f"Skipping camera tf convert: {str(e)}")
    
    print("Conversion completed!")

if __name__ == '__main__':
    main()