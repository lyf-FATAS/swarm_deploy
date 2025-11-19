// Auto-generated. Do not edit!

// (in-package message_typedef.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let FrontendOneDirTrackResult = require('./FrontendOneDirTrackResult.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class FrontendTrackResult {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.track_results = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('track_results')) {
        this.track_results = initObj.track_results
      }
      else {
        this.track_results = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FrontendTrackResult
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [track_results]
    // Serialize the length for message field [track_results]
    bufferOffset = _serializer.uint32(obj.track_results.length, buffer, bufferOffset);
    obj.track_results.forEach((val) => {
      bufferOffset = FrontendOneDirTrackResult.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FrontendTrackResult
    let len;
    let data = new FrontendTrackResult(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [track_results]
    // Deserialize array length for message field [track_results]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.track_results = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.track_results[i] = FrontendOneDirTrackResult.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.track_results.forEach((val) => {
      length += FrontendOneDirTrackResult.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'message_typedef/FrontendTrackResult';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2148ce17866256734095bba115cc9847';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    FrontendOneDirTrackResult[] track_results
    
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
    
    ================================================================================
    MSG: message_typedef/FrontendOneDirTrackResult
    bool is_keyframe
    int8 direction_id
    float32 depth
    uint32[] feature_ids
    
    geometry_msgs/Vector3[] pixel_uv_left_cam
    geometry_msgs/Vector3[] norm_xy_left_cam
    geometry_msgs/Vector3[] sphere_xyz_left_cam
    bool[] left_cam_observe
    
    geometry_msgs/Vector3[] pixel_uv_right_cam
    geometry_msgs/Vector3[] norm_xy_right_cam
    geometry_msgs/Vector3[] sphere_xyz_right_cam
    bool[] right_cam_observe
    
    ================================================================================
    MSG: geometry_msgs/Vector3
    # This represents a vector in free space. 
    # It is only meant to represent a direction. Therefore, it does not
    # make sense to apply a translation to it (e.g., when applying a 
    # generic rigid transformation to a Vector3, tf2 will only apply the
    # rotation). If you want your data to be translatable too, use the
    # geometry_msgs/Point message instead.
    
    float64 x
    float64 y
    float64 z
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new FrontendTrackResult(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.track_results !== undefined) {
      resolved.track_results = new Array(msg.track_results.length);
      for (let i = 0; i < resolved.track_results.length; ++i) {
        resolved.track_results[i] = FrontendOneDirTrackResult.Resolve(msg.track_results[i]);
      }
    }
    else {
      resolved.track_results = []
    }

    return resolved;
    }
};

module.exports = FrontendTrackResult;
