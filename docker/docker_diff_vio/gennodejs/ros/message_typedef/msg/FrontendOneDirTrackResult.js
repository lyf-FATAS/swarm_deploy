// Auto-generated. Do not edit!

// (in-package message_typedef.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class FrontendOneDirTrackResult {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.is_keyframe = null;
      this.direction_id = null;
      this.depth = null;
      this.feature_ids = null;
      this.pixel_uv_left_cam = null;
      this.norm_xy_left_cam = null;
      this.sphere_xyz_left_cam = null;
      this.left_cam_observe = null;
      this.pixel_uv_right_cam = null;
      this.norm_xy_right_cam = null;
      this.sphere_xyz_right_cam = null;
      this.right_cam_observe = null;
    }
    else {
      if (initObj.hasOwnProperty('is_keyframe')) {
        this.is_keyframe = initObj.is_keyframe
      }
      else {
        this.is_keyframe = false;
      }
      if (initObj.hasOwnProperty('direction_id')) {
        this.direction_id = initObj.direction_id
      }
      else {
        this.direction_id = 0;
      }
      if (initObj.hasOwnProperty('depth')) {
        this.depth = initObj.depth
      }
      else {
        this.depth = 0.0;
      }
      if (initObj.hasOwnProperty('feature_ids')) {
        this.feature_ids = initObj.feature_ids
      }
      else {
        this.feature_ids = [];
      }
      if (initObj.hasOwnProperty('pixel_uv_left_cam')) {
        this.pixel_uv_left_cam = initObj.pixel_uv_left_cam
      }
      else {
        this.pixel_uv_left_cam = [];
      }
      if (initObj.hasOwnProperty('norm_xy_left_cam')) {
        this.norm_xy_left_cam = initObj.norm_xy_left_cam
      }
      else {
        this.norm_xy_left_cam = [];
      }
      if (initObj.hasOwnProperty('sphere_xyz_left_cam')) {
        this.sphere_xyz_left_cam = initObj.sphere_xyz_left_cam
      }
      else {
        this.sphere_xyz_left_cam = [];
      }
      if (initObj.hasOwnProperty('left_cam_observe')) {
        this.left_cam_observe = initObj.left_cam_observe
      }
      else {
        this.left_cam_observe = [];
      }
      if (initObj.hasOwnProperty('pixel_uv_right_cam')) {
        this.pixel_uv_right_cam = initObj.pixel_uv_right_cam
      }
      else {
        this.pixel_uv_right_cam = [];
      }
      if (initObj.hasOwnProperty('norm_xy_right_cam')) {
        this.norm_xy_right_cam = initObj.norm_xy_right_cam
      }
      else {
        this.norm_xy_right_cam = [];
      }
      if (initObj.hasOwnProperty('sphere_xyz_right_cam')) {
        this.sphere_xyz_right_cam = initObj.sphere_xyz_right_cam
      }
      else {
        this.sphere_xyz_right_cam = [];
      }
      if (initObj.hasOwnProperty('right_cam_observe')) {
        this.right_cam_observe = initObj.right_cam_observe
      }
      else {
        this.right_cam_observe = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FrontendOneDirTrackResult
    // Serialize message field [is_keyframe]
    bufferOffset = _serializer.bool(obj.is_keyframe, buffer, bufferOffset);
    // Serialize message field [direction_id]
    bufferOffset = _serializer.int8(obj.direction_id, buffer, bufferOffset);
    // Serialize message field [depth]
    bufferOffset = _serializer.float32(obj.depth, buffer, bufferOffset);
    // Serialize message field [feature_ids]
    bufferOffset = _arraySerializer.uint32(obj.feature_ids, buffer, bufferOffset, null);
    // Serialize message field [pixel_uv_left_cam]
    // Serialize the length for message field [pixel_uv_left_cam]
    bufferOffset = _serializer.uint32(obj.pixel_uv_left_cam.length, buffer, bufferOffset);
    obj.pixel_uv_left_cam.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Vector3.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [norm_xy_left_cam]
    // Serialize the length for message field [norm_xy_left_cam]
    bufferOffset = _serializer.uint32(obj.norm_xy_left_cam.length, buffer, bufferOffset);
    obj.norm_xy_left_cam.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Vector3.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [sphere_xyz_left_cam]
    // Serialize the length for message field [sphere_xyz_left_cam]
    bufferOffset = _serializer.uint32(obj.sphere_xyz_left_cam.length, buffer, bufferOffset);
    obj.sphere_xyz_left_cam.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Vector3.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [left_cam_observe]
    bufferOffset = _arraySerializer.bool(obj.left_cam_observe, buffer, bufferOffset, null);
    // Serialize message field [pixel_uv_right_cam]
    // Serialize the length for message field [pixel_uv_right_cam]
    bufferOffset = _serializer.uint32(obj.pixel_uv_right_cam.length, buffer, bufferOffset);
    obj.pixel_uv_right_cam.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Vector3.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [norm_xy_right_cam]
    // Serialize the length for message field [norm_xy_right_cam]
    bufferOffset = _serializer.uint32(obj.norm_xy_right_cam.length, buffer, bufferOffset);
    obj.norm_xy_right_cam.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Vector3.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [sphere_xyz_right_cam]
    // Serialize the length for message field [sphere_xyz_right_cam]
    bufferOffset = _serializer.uint32(obj.sphere_xyz_right_cam.length, buffer, bufferOffset);
    obj.sphere_xyz_right_cam.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Vector3.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [right_cam_observe]
    bufferOffset = _arraySerializer.bool(obj.right_cam_observe, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FrontendOneDirTrackResult
    let len;
    let data = new FrontendOneDirTrackResult(null);
    // Deserialize message field [is_keyframe]
    data.is_keyframe = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [direction_id]
    data.direction_id = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [depth]
    data.depth = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [feature_ids]
    data.feature_ids = _arrayDeserializer.uint32(buffer, bufferOffset, null)
    // Deserialize message field [pixel_uv_left_cam]
    // Deserialize array length for message field [pixel_uv_left_cam]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.pixel_uv_left_cam = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.pixel_uv_left_cam[i] = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [norm_xy_left_cam]
    // Deserialize array length for message field [norm_xy_left_cam]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.norm_xy_left_cam = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.norm_xy_left_cam[i] = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [sphere_xyz_left_cam]
    // Deserialize array length for message field [sphere_xyz_left_cam]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.sphere_xyz_left_cam = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.sphere_xyz_left_cam[i] = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [left_cam_observe]
    data.left_cam_observe = _arrayDeserializer.bool(buffer, bufferOffset, null)
    // Deserialize message field [pixel_uv_right_cam]
    // Deserialize array length for message field [pixel_uv_right_cam]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.pixel_uv_right_cam = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.pixel_uv_right_cam[i] = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [norm_xy_right_cam]
    // Deserialize array length for message field [norm_xy_right_cam]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.norm_xy_right_cam = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.norm_xy_right_cam[i] = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [sphere_xyz_right_cam]
    // Deserialize array length for message field [sphere_xyz_right_cam]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.sphere_xyz_right_cam = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.sphere_xyz_right_cam[i] = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [right_cam_observe]
    data.right_cam_observe = _arrayDeserializer.bool(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.feature_ids.length;
    length += 24 * object.pixel_uv_left_cam.length;
    length += 24 * object.norm_xy_left_cam.length;
    length += 24 * object.sphere_xyz_left_cam.length;
    length += object.left_cam_observe.length;
    length += 24 * object.pixel_uv_right_cam.length;
    length += 24 * object.norm_xy_right_cam.length;
    length += 24 * object.sphere_xyz_right_cam.length;
    length += object.right_cam_observe.length;
    return length + 42;
  }

  static datatype() {
    // Returns string type for a message object
    return 'message_typedef/FrontendOneDirTrackResult';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '87020366231c2b8da1d75e57baab7c8e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new FrontendOneDirTrackResult(null);
    if (msg.is_keyframe !== undefined) {
      resolved.is_keyframe = msg.is_keyframe;
    }
    else {
      resolved.is_keyframe = false
    }

    if (msg.direction_id !== undefined) {
      resolved.direction_id = msg.direction_id;
    }
    else {
      resolved.direction_id = 0
    }

    if (msg.depth !== undefined) {
      resolved.depth = msg.depth;
    }
    else {
      resolved.depth = 0.0
    }

    if (msg.feature_ids !== undefined) {
      resolved.feature_ids = msg.feature_ids;
    }
    else {
      resolved.feature_ids = []
    }

    if (msg.pixel_uv_left_cam !== undefined) {
      resolved.pixel_uv_left_cam = new Array(msg.pixel_uv_left_cam.length);
      for (let i = 0; i < resolved.pixel_uv_left_cam.length; ++i) {
        resolved.pixel_uv_left_cam[i] = geometry_msgs.msg.Vector3.Resolve(msg.pixel_uv_left_cam[i]);
      }
    }
    else {
      resolved.pixel_uv_left_cam = []
    }

    if (msg.norm_xy_left_cam !== undefined) {
      resolved.norm_xy_left_cam = new Array(msg.norm_xy_left_cam.length);
      for (let i = 0; i < resolved.norm_xy_left_cam.length; ++i) {
        resolved.norm_xy_left_cam[i] = geometry_msgs.msg.Vector3.Resolve(msg.norm_xy_left_cam[i]);
      }
    }
    else {
      resolved.norm_xy_left_cam = []
    }

    if (msg.sphere_xyz_left_cam !== undefined) {
      resolved.sphere_xyz_left_cam = new Array(msg.sphere_xyz_left_cam.length);
      for (let i = 0; i < resolved.sphere_xyz_left_cam.length; ++i) {
        resolved.sphere_xyz_left_cam[i] = geometry_msgs.msg.Vector3.Resolve(msg.sphere_xyz_left_cam[i]);
      }
    }
    else {
      resolved.sphere_xyz_left_cam = []
    }

    if (msg.left_cam_observe !== undefined) {
      resolved.left_cam_observe = msg.left_cam_observe;
    }
    else {
      resolved.left_cam_observe = []
    }

    if (msg.pixel_uv_right_cam !== undefined) {
      resolved.pixel_uv_right_cam = new Array(msg.pixel_uv_right_cam.length);
      for (let i = 0; i < resolved.pixel_uv_right_cam.length; ++i) {
        resolved.pixel_uv_right_cam[i] = geometry_msgs.msg.Vector3.Resolve(msg.pixel_uv_right_cam[i]);
      }
    }
    else {
      resolved.pixel_uv_right_cam = []
    }

    if (msg.norm_xy_right_cam !== undefined) {
      resolved.norm_xy_right_cam = new Array(msg.norm_xy_right_cam.length);
      for (let i = 0; i < resolved.norm_xy_right_cam.length; ++i) {
        resolved.norm_xy_right_cam[i] = geometry_msgs.msg.Vector3.Resolve(msg.norm_xy_right_cam[i]);
      }
    }
    else {
      resolved.norm_xy_right_cam = []
    }

    if (msg.sphere_xyz_right_cam !== undefined) {
      resolved.sphere_xyz_right_cam = new Array(msg.sphere_xyz_right_cam.length);
      for (let i = 0; i < resolved.sphere_xyz_right_cam.length; ++i) {
        resolved.sphere_xyz_right_cam[i] = geometry_msgs.msg.Vector3.Resolve(msg.sphere_xyz_right_cam[i]);
      }
    }
    else {
      resolved.sphere_xyz_right_cam = []
    }

    if (msg.right_cam_observe !== undefined) {
      resolved.right_cam_observe = msg.right_cam_observe;
    }
    else {
      resolved.right_cam_observe = []
    }

    return resolved;
    }
};

module.exports = FrontendOneDirTrackResult;
