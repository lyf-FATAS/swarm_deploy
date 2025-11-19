// Auto-generated. Do not edit!

// (in-package message_typedef.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class LocatorFdiMsg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.is_locator_odom_error = null;
      this.is_imu_over_range = null;
      this.is_visual_track_result_error = null;
      this.is_lidar_scan_error = null;
      this.is_cpu_usage_over_range = null;
      this.odom_horizontal_velocity_m_s = null;
      this.odom_vertical_velocity_m_s = null;
      this.odom_roll_deg = null;
      this.odom_pitch_deg = null;
      this.imu_norm_accel_m_s2 = null;
      this.lidar_ratio_of_invalid_points = null;
      this.cpu_usage_percentage = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('is_locator_odom_error')) {
        this.is_locator_odom_error = initObj.is_locator_odom_error
      }
      else {
        this.is_locator_odom_error = false;
      }
      if (initObj.hasOwnProperty('is_imu_over_range')) {
        this.is_imu_over_range = initObj.is_imu_over_range
      }
      else {
        this.is_imu_over_range = false;
      }
      if (initObj.hasOwnProperty('is_visual_track_result_error')) {
        this.is_visual_track_result_error = initObj.is_visual_track_result_error
      }
      else {
        this.is_visual_track_result_error = false;
      }
      if (initObj.hasOwnProperty('is_lidar_scan_error')) {
        this.is_lidar_scan_error = initObj.is_lidar_scan_error
      }
      else {
        this.is_lidar_scan_error = false;
      }
      if (initObj.hasOwnProperty('is_cpu_usage_over_range')) {
        this.is_cpu_usage_over_range = initObj.is_cpu_usage_over_range
      }
      else {
        this.is_cpu_usage_over_range = false;
      }
      if (initObj.hasOwnProperty('odom_horizontal_velocity_m_s')) {
        this.odom_horizontal_velocity_m_s = initObj.odom_horizontal_velocity_m_s
      }
      else {
        this.odom_horizontal_velocity_m_s = 0.0;
      }
      if (initObj.hasOwnProperty('odom_vertical_velocity_m_s')) {
        this.odom_vertical_velocity_m_s = initObj.odom_vertical_velocity_m_s
      }
      else {
        this.odom_vertical_velocity_m_s = 0.0;
      }
      if (initObj.hasOwnProperty('odom_roll_deg')) {
        this.odom_roll_deg = initObj.odom_roll_deg
      }
      else {
        this.odom_roll_deg = 0.0;
      }
      if (initObj.hasOwnProperty('odom_pitch_deg')) {
        this.odom_pitch_deg = initObj.odom_pitch_deg
      }
      else {
        this.odom_pitch_deg = 0.0;
      }
      if (initObj.hasOwnProperty('imu_norm_accel_m_s2')) {
        this.imu_norm_accel_m_s2 = initObj.imu_norm_accel_m_s2
      }
      else {
        this.imu_norm_accel_m_s2 = 0.0;
      }
      if (initObj.hasOwnProperty('lidar_ratio_of_invalid_points')) {
        this.lidar_ratio_of_invalid_points = initObj.lidar_ratio_of_invalid_points
      }
      else {
        this.lidar_ratio_of_invalid_points = 0.0;
      }
      if (initObj.hasOwnProperty('cpu_usage_percentage')) {
        this.cpu_usage_percentage = initObj.cpu_usage_percentage
      }
      else {
        this.cpu_usage_percentage = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type LocatorFdiMsg
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [is_locator_odom_error]
    bufferOffset = _serializer.bool(obj.is_locator_odom_error, buffer, bufferOffset);
    // Serialize message field [is_imu_over_range]
    bufferOffset = _serializer.bool(obj.is_imu_over_range, buffer, bufferOffset);
    // Serialize message field [is_visual_track_result_error]
    bufferOffset = _serializer.bool(obj.is_visual_track_result_error, buffer, bufferOffset);
    // Serialize message field [is_lidar_scan_error]
    bufferOffset = _serializer.bool(obj.is_lidar_scan_error, buffer, bufferOffset);
    // Serialize message field [is_cpu_usage_over_range]
    bufferOffset = _serializer.bool(obj.is_cpu_usage_over_range, buffer, bufferOffset);
    // Serialize message field [odom_horizontal_velocity_m_s]
    bufferOffset = _serializer.float32(obj.odom_horizontal_velocity_m_s, buffer, bufferOffset);
    // Serialize message field [odom_vertical_velocity_m_s]
    bufferOffset = _serializer.float32(obj.odom_vertical_velocity_m_s, buffer, bufferOffset);
    // Serialize message field [odom_roll_deg]
    bufferOffset = _serializer.float32(obj.odom_roll_deg, buffer, bufferOffset);
    // Serialize message field [odom_pitch_deg]
    bufferOffset = _serializer.float32(obj.odom_pitch_deg, buffer, bufferOffset);
    // Serialize message field [imu_norm_accel_m_s2]
    bufferOffset = _serializer.float32(obj.imu_norm_accel_m_s2, buffer, bufferOffset);
    // Serialize message field [lidar_ratio_of_invalid_points]
    bufferOffset = _serializer.float32(obj.lidar_ratio_of_invalid_points, buffer, bufferOffset);
    // Serialize message field [cpu_usage_percentage]
    bufferOffset = _arraySerializer.float32(obj.cpu_usage_percentage, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type LocatorFdiMsg
    let len;
    let data = new LocatorFdiMsg(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [is_locator_odom_error]
    data.is_locator_odom_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [is_imu_over_range]
    data.is_imu_over_range = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [is_visual_track_result_error]
    data.is_visual_track_result_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [is_lidar_scan_error]
    data.is_lidar_scan_error = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [is_cpu_usage_over_range]
    data.is_cpu_usage_over_range = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [odom_horizontal_velocity_m_s]
    data.odom_horizontal_velocity_m_s = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [odom_vertical_velocity_m_s]
    data.odom_vertical_velocity_m_s = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [odom_roll_deg]
    data.odom_roll_deg = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [odom_pitch_deg]
    data.odom_pitch_deg = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [imu_norm_accel_m_s2]
    data.imu_norm_accel_m_s2 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [lidar_ratio_of_invalid_points]
    data.lidar_ratio_of_invalid_points = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [cpu_usage_percentage]
    data.cpu_usage_percentage = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 4 * object.cpu_usage_percentage.length;
    return length + 33;
  }

  static datatype() {
    // Returns string type for a message object
    return 'message_typedef/LocatorFdiMsg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd429453896a3f9392995d3e879586d29';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header             # ROS standard message header
    bool is_locator_odom_error
    bool is_imu_over_range
    bool is_visual_track_result_error
    bool is_lidar_scan_error
    bool is_cpu_usage_over_range
    float32 odom_horizontal_velocity_m_s
    float32 odom_vertical_velocity_m_s
    float32 odom_roll_deg
    float32 odom_pitch_deg
    float32 imu_norm_accel_m_s2
    float32 lidar_ratio_of_invalid_points
    float32[] cpu_usage_percentage
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new LocatorFdiMsg(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.is_locator_odom_error !== undefined) {
      resolved.is_locator_odom_error = msg.is_locator_odom_error;
    }
    else {
      resolved.is_locator_odom_error = false
    }

    if (msg.is_imu_over_range !== undefined) {
      resolved.is_imu_over_range = msg.is_imu_over_range;
    }
    else {
      resolved.is_imu_over_range = false
    }

    if (msg.is_visual_track_result_error !== undefined) {
      resolved.is_visual_track_result_error = msg.is_visual_track_result_error;
    }
    else {
      resolved.is_visual_track_result_error = false
    }

    if (msg.is_lidar_scan_error !== undefined) {
      resolved.is_lidar_scan_error = msg.is_lidar_scan_error;
    }
    else {
      resolved.is_lidar_scan_error = false
    }

    if (msg.is_cpu_usage_over_range !== undefined) {
      resolved.is_cpu_usage_over_range = msg.is_cpu_usage_over_range;
    }
    else {
      resolved.is_cpu_usage_over_range = false
    }

    if (msg.odom_horizontal_velocity_m_s !== undefined) {
      resolved.odom_horizontal_velocity_m_s = msg.odom_horizontal_velocity_m_s;
    }
    else {
      resolved.odom_horizontal_velocity_m_s = 0.0
    }

    if (msg.odom_vertical_velocity_m_s !== undefined) {
      resolved.odom_vertical_velocity_m_s = msg.odom_vertical_velocity_m_s;
    }
    else {
      resolved.odom_vertical_velocity_m_s = 0.0
    }

    if (msg.odom_roll_deg !== undefined) {
      resolved.odom_roll_deg = msg.odom_roll_deg;
    }
    else {
      resolved.odom_roll_deg = 0.0
    }

    if (msg.odom_pitch_deg !== undefined) {
      resolved.odom_pitch_deg = msg.odom_pitch_deg;
    }
    else {
      resolved.odom_pitch_deg = 0.0
    }

    if (msg.imu_norm_accel_m_s2 !== undefined) {
      resolved.imu_norm_accel_m_s2 = msg.imu_norm_accel_m_s2;
    }
    else {
      resolved.imu_norm_accel_m_s2 = 0.0
    }

    if (msg.lidar_ratio_of_invalid_points !== undefined) {
      resolved.lidar_ratio_of_invalid_points = msg.lidar_ratio_of_invalid_points;
    }
    else {
      resolved.lidar_ratio_of_invalid_points = 0.0
    }

    if (msg.cpu_usage_percentage !== undefined) {
      resolved.cpu_usage_percentage = msg.cpu_usage_percentage;
    }
    else {
      resolved.cpu_usage_percentage = []
    }

    return resolved;
    }
};

module.exports = LocatorFdiMsg;
