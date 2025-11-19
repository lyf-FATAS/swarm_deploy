; Auto-generated. Do not edit!


(cl:in-package message_typedef-msg)


;//! \htmlinclude FrontendTrackResult.msg.html

(cl:defclass <FrontendTrackResult> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (track_results
    :reader track_results
    :initarg :track_results
    :type (cl:vector message_typedef-msg:FrontendOneDirTrackResult)
   :initform (cl:make-array 0 :element-type 'message_typedef-msg:FrontendOneDirTrackResult :initial-element (cl:make-instance 'message_typedef-msg:FrontendOneDirTrackResult))))
)

(cl:defclass FrontendTrackResult (<FrontendTrackResult>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FrontendTrackResult>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FrontendTrackResult)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name message_typedef-msg:<FrontendTrackResult> is deprecated: use message_typedef-msg:FrontendTrackResult instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <FrontendTrackResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader message_typedef-msg:header-val is deprecated.  Use message_typedef-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'track_results-val :lambda-list '(m))
(cl:defmethod track_results-val ((m <FrontendTrackResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader message_typedef-msg:track_results-val is deprecated.  Use message_typedef-msg:track_results instead.")
  (track_results m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FrontendTrackResult>) ostream)
  "Serializes a message object of type '<FrontendTrackResult>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'track_results))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'track_results))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FrontendTrackResult>) istream)
  "Deserializes a message object of type '<FrontendTrackResult>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'track_results) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'track_results)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'message_typedef-msg:FrontendOneDirTrackResult))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FrontendTrackResult>)))
  "Returns string type for a message object of type '<FrontendTrackResult>"
  "message_typedef/FrontendTrackResult")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FrontendTrackResult)))
  "Returns string type for a message object of type 'FrontendTrackResult"
  "message_typedef/FrontendTrackResult")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FrontendTrackResult>)))
  "Returns md5sum for a message object of type '<FrontendTrackResult>"
  "2148ce17866256734095bba115cc9847")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FrontendTrackResult)))
  "Returns md5sum for a message object of type 'FrontendTrackResult"
  "2148ce17866256734095bba115cc9847")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FrontendTrackResult>)))
  "Returns full string definition for message of type '<FrontendTrackResult>"
  (cl:format cl:nil "Header header~%FrontendOneDirTrackResult[] track_results~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: message_typedef/FrontendOneDirTrackResult~%bool is_keyframe~%int8 direction_id~%float32 depth~%uint32[] feature_ids~%~%geometry_msgs/Vector3[] pixel_uv_left_cam~%geometry_msgs/Vector3[] norm_xy_left_cam~%geometry_msgs/Vector3[] sphere_xyz_left_cam~%bool[] left_cam_observe~%~%geometry_msgs/Vector3[] pixel_uv_right_cam~%geometry_msgs/Vector3[] norm_xy_right_cam~%geometry_msgs/Vector3[] sphere_xyz_right_cam~%bool[] right_cam_observe~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FrontendTrackResult)))
  "Returns full string definition for message of type 'FrontendTrackResult"
  (cl:format cl:nil "Header header~%FrontendOneDirTrackResult[] track_results~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: message_typedef/FrontendOneDirTrackResult~%bool is_keyframe~%int8 direction_id~%float32 depth~%uint32[] feature_ids~%~%geometry_msgs/Vector3[] pixel_uv_left_cam~%geometry_msgs/Vector3[] norm_xy_left_cam~%geometry_msgs/Vector3[] sphere_xyz_left_cam~%bool[] left_cam_observe~%~%geometry_msgs/Vector3[] pixel_uv_right_cam~%geometry_msgs/Vector3[] norm_xy_right_cam~%geometry_msgs/Vector3[] sphere_xyz_right_cam~%bool[] right_cam_observe~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FrontendTrackResult>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'track_results) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FrontendTrackResult>))
  "Converts a ROS message object to a list"
  (cl:list 'FrontendTrackResult
    (cl:cons ':header (header msg))
    (cl:cons ':track_results (track_results msg))
))
