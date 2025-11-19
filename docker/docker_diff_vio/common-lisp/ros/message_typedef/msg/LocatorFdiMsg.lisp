; Auto-generated. Do not edit!


(cl:in-package message_typedef-msg)


;//! \htmlinclude LocatorFdiMsg.msg.html

(cl:defclass <LocatorFdiMsg> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (is_locator_odom_error
    :reader is_locator_odom_error
    :initarg :is_locator_odom_error
    :type cl:boolean
    :initform cl:nil)
   (is_imu_over_range
    :reader is_imu_over_range
    :initarg :is_imu_over_range
    :type cl:boolean
    :initform cl:nil)
   (is_visual_track_result_error
    :reader is_visual_track_result_error
    :initarg :is_visual_track_result_error
    :type cl:boolean
    :initform cl:nil)
   (is_lidar_scan_error
    :reader is_lidar_scan_error
    :initarg :is_lidar_scan_error
    :type cl:boolean
    :initform cl:nil)
   (is_cpu_usage_over_range
    :reader is_cpu_usage_over_range
    :initarg :is_cpu_usage_over_range
    :type cl:boolean
    :initform cl:nil)
   (odom_horizontal_velocity_m_s
    :reader odom_horizontal_velocity_m_s
    :initarg :odom_horizontal_velocity_m_s
    :type cl:float
    :initform 0.0)
   (odom_vertical_velocity_m_s
    :reader odom_vertical_velocity_m_s
    :initarg :odom_vertical_velocity_m_s
    :type cl:float
    :initform 0.0)
   (odom_roll_deg
    :reader odom_roll_deg
    :initarg :odom_roll_deg
    :type cl:float
    :initform 0.0)
   (odom_pitch_deg
    :reader odom_pitch_deg
    :initarg :odom_pitch_deg
    :type cl:float
    :initform 0.0)
   (imu_norm_accel_m_s2
    :reader imu_norm_accel_m_s2
    :initarg :imu_norm_accel_m_s2
    :type cl:float
    :initform 0.0)
   (lidar_ratio_of_invalid_points
    :reader lidar_ratio_of_invalid_points
    :initarg :lidar_ratio_of_invalid_points
    :type cl:float
    :initform 0.0)
   (cpu_usage_percentage
    :reader cpu_usage_percentage
    :initarg :cpu_usage_percentage
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass LocatorFdiMsg (<LocatorFdiMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LocatorFdiMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LocatorFdiMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name message_typedef-msg:<LocatorFdiMsg> is deprecated: use message_typedef-msg:LocatorFdiMsg instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <LocatorFdiMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader message_typedef-msg:header-val is deprecated.  Use message_typedef-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'is_locator_odom_error-val :lambda-list '(m))
(cl:defmethod is_locator_odom_error-val ((m <LocatorFdiMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader message_typedef-msg:is_locator_odom_error-val is deprecated.  Use message_typedef-msg:is_locator_odom_error instead.")
  (is_locator_odom_error m))

(cl:ensure-generic-function 'is_imu_over_range-val :lambda-list '(m))
(cl:defmethod is_imu_over_range-val ((m <LocatorFdiMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader message_typedef-msg:is_imu_over_range-val is deprecated.  Use message_typedef-msg:is_imu_over_range instead.")
  (is_imu_over_range m))

(cl:ensure-generic-function 'is_visual_track_result_error-val :lambda-list '(m))
(cl:defmethod is_visual_track_result_error-val ((m <LocatorFdiMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader message_typedef-msg:is_visual_track_result_error-val is deprecated.  Use message_typedef-msg:is_visual_track_result_error instead.")
  (is_visual_track_result_error m))

(cl:ensure-generic-function 'is_lidar_scan_error-val :lambda-list '(m))
(cl:defmethod is_lidar_scan_error-val ((m <LocatorFdiMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader message_typedef-msg:is_lidar_scan_error-val is deprecated.  Use message_typedef-msg:is_lidar_scan_error instead.")
  (is_lidar_scan_error m))

(cl:ensure-generic-function 'is_cpu_usage_over_range-val :lambda-list '(m))
(cl:defmethod is_cpu_usage_over_range-val ((m <LocatorFdiMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader message_typedef-msg:is_cpu_usage_over_range-val is deprecated.  Use message_typedef-msg:is_cpu_usage_over_range instead.")
  (is_cpu_usage_over_range m))

(cl:ensure-generic-function 'odom_horizontal_velocity_m_s-val :lambda-list '(m))
(cl:defmethod odom_horizontal_velocity_m_s-val ((m <LocatorFdiMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader message_typedef-msg:odom_horizontal_velocity_m_s-val is deprecated.  Use message_typedef-msg:odom_horizontal_velocity_m_s instead.")
  (odom_horizontal_velocity_m_s m))

(cl:ensure-generic-function 'odom_vertical_velocity_m_s-val :lambda-list '(m))
(cl:defmethod odom_vertical_velocity_m_s-val ((m <LocatorFdiMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader message_typedef-msg:odom_vertical_velocity_m_s-val is deprecated.  Use message_typedef-msg:odom_vertical_velocity_m_s instead.")
  (odom_vertical_velocity_m_s m))

(cl:ensure-generic-function 'odom_roll_deg-val :lambda-list '(m))
(cl:defmethod odom_roll_deg-val ((m <LocatorFdiMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader message_typedef-msg:odom_roll_deg-val is deprecated.  Use message_typedef-msg:odom_roll_deg instead.")
  (odom_roll_deg m))

(cl:ensure-generic-function 'odom_pitch_deg-val :lambda-list '(m))
(cl:defmethod odom_pitch_deg-val ((m <LocatorFdiMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader message_typedef-msg:odom_pitch_deg-val is deprecated.  Use message_typedef-msg:odom_pitch_deg instead.")
  (odom_pitch_deg m))

(cl:ensure-generic-function 'imu_norm_accel_m_s2-val :lambda-list '(m))
(cl:defmethod imu_norm_accel_m_s2-val ((m <LocatorFdiMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader message_typedef-msg:imu_norm_accel_m_s2-val is deprecated.  Use message_typedef-msg:imu_norm_accel_m_s2 instead.")
  (imu_norm_accel_m_s2 m))

(cl:ensure-generic-function 'lidar_ratio_of_invalid_points-val :lambda-list '(m))
(cl:defmethod lidar_ratio_of_invalid_points-val ((m <LocatorFdiMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader message_typedef-msg:lidar_ratio_of_invalid_points-val is deprecated.  Use message_typedef-msg:lidar_ratio_of_invalid_points instead.")
  (lidar_ratio_of_invalid_points m))

(cl:ensure-generic-function 'cpu_usage_percentage-val :lambda-list '(m))
(cl:defmethod cpu_usage_percentage-val ((m <LocatorFdiMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader message_typedef-msg:cpu_usage_percentage-val is deprecated.  Use message_typedef-msg:cpu_usage_percentage instead.")
  (cpu_usage_percentage m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LocatorFdiMsg>) ostream)
  "Serializes a message object of type '<LocatorFdiMsg>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_locator_odom_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_imu_over_range) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_visual_track_result_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_lidar_scan_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_cpu_usage_over_range) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'odom_horizontal_velocity_m_s))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'odom_vertical_velocity_m_s))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'odom_roll_deg))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'odom_pitch_deg))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'imu_norm_accel_m_s2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'lidar_ratio_of_invalid_points))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'cpu_usage_percentage))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'cpu_usage_percentage))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LocatorFdiMsg>) istream)
  "Deserializes a message object of type '<LocatorFdiMsg>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'is_locator_odom_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'is_imu_over_range) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'is_visual_track_result_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'is_lidar_scan_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'is_cpu_usage_over_range) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'odom_horizontal_velocity_m_s) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'odom_vertical_velocity_m_s) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'odom_roll_deg) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'odom_pitch_deg) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'imu_norm_accel_m_s2) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'lidar_ratio_of_invalid_points) (roslisp-utils:decode-single-float-bits bits)))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'cpu_usage_percentage) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'cpu_usage_percentage)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LocatorFdiMsg>)))
  "Returns string type for a message object of type '<LocatorFdiMsg>"
  "message_typedef/LocatorFdiMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LocatorFdiMsg)))
  "Returns string type for a message object of type 'LocatorFdiMsg"
  "message_typedef/LocatorFdiMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LocatorFdiMsg>)))
  "Returns md5sum for a message object of type '<LocatorFdiMsg>"
  "d429453896a3f9392995d3e879586d29")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LocatorFdiMsg)))
  "Returns md5sum for a message object of type 'LocatorFdiMsg"
  "d429453896a3f9392995d3e879586d29")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LocatorFdiMsg>)))
  "Returns full string definition for message of type '<LocatorFdiMsg>"
  (cl:format cl:nil "Header header             # ROS standard message header~%bool is_locator_odom_error~%bool is_imu_over_range~%bool is_visual_track_result_error~%bool is_lidar_scan_error~%bool is_cpu_usage_over_range~%float32 odom_horizontal_velocity_m_s~%float32 odom_vertical_velocity_m_s~%float32 odom_roll_deg~%float32 odom_pitch_deg~%float32 imu_norm_accel_m_s2~%float32 lidar_ratio_of_invalid_points~%float32[] cpu_usage_percentage~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LocatorFdiMsg)))
  "Returns full string definition for message of type 'LocatorFdiMsg"
  (cl:format cl:nil "Header header             # ROS standard message header~%bool is_locator_odom_error~%bool is_imu_over_range~%bool is_visual_track_result_error~%bool is_lidar_scan_error~%bool is_cpu_usage_over_range~%float32 odom_horizontal_velocity_m_s~%float32 odom_vertical_velocity_m_s~%float32 odom_roll_deg~%float32 odom_pitch_deg~%float32 imu_norm_accel_m_s2~%float32 lidar_ratio_of_invalid_points~%float32[] cpu_usage_percentage~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LocatorFdiMsg>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     1
     1
     1
     4
     4
     4
     4
     4
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'cpu_usage_percentage) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LocatorFdiMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'LocatorFdiMsg
    (cl:cons ':header (header msg))
    (cl:cons ':is_locator_odom_error (is_locator_odom_error msg))
    (cl:cons ':is_imu_over_range (is_imu_over_range msg))
    (cl:cons ':is_visual_track_result_error (is_visual_track_result_error msg))
    (cl:cons ':is_lidar_scan_error (is_lidar_scan_error msg))
    (cl:cons ':is_cpu_usage_over_range (is_cpu_usage_over_range msg))
    (cl:cons ':odom_horizontal_velocity_m_s (odom_horizontal_velocity_m_s msg))
    (cl:cons ':odom_vertical_velocity_m_s (odom_vertical_velocity_m_s msg))
    (cl:cons ':odom_roll_deg (odom_roll_deg msg))
    (cl:cons ':odom_pitch_deg (odom_pitch_deg msg))
    (cl:cons ':imu_norm_accel_m_s2 (imu_norm_accel_m_s2 msg))
    (cl:cons ':lidar_ratio_of_invalid_points (lidar_ratio_of_invalid_points msg))
    (cl:cons ':cpu_usage_percentage (cpu_usage_percentage msg))
))
