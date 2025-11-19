;; Auto-generated. Do not edit!


(when (boundp 'message_typedef::LocatorFdiMsg)
  (if (not (find-package "MESSAGE_TYPEDEF"))
    (make-package "MESSAGE_TYPEDEF"))
  (shadow 'LocatorFdiMsg (find-package "MESSAGE_TYPEDEF")))
(unless (find-package "MESSAGE_TYPEDEF::LOCATORFDIMSG")
  (make-package "MESSAGE_TYPEDEF::LOCATORFDIMSG"))

(in-package "ROS")
;;//! \htmlinclude LocatorFdiMsg.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass message_typedef::LocatorFdiMsg
  :super ros::object
  :slots (_header _is_locator_odom_error _is_imu_over_range _is_visual_track_result_error _is_lidar_scan_error _is_cpu_usage_over_range _odom_horizontal_velocity_m_s _odom_vertical_velocity_m_s _odom_roll_deg _odom_pitch_deg _imu_norm_accel_m_s2 _lidar_ratio_of_invalid_points _cpu_usage_percentage ))

(defmethod message_typedef::LocatorFdiMsg
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:is_locator_odom_error __is_locator_odom_error) nil)
    ((:is_imu_over_range __is_imu_over_range) nil)
    ((:is_visual_track_result_error __is_visual_track_result_error) nil)
    ((:is_lidar_scan_error __is_lidar_scan_error) nil)
    ((:is_cpu_usage_over_range __is_cpu_usage_over_range) nil)
    ((:odom_horizontal_velocity_m_s __odom_horizontal_velocity_m_s) 0.0)
    ((:odom_vertical_velocity_m_s __odom_vertical_velocity_m_s) 0.0)
    ((:odom_roll_deg __odom_roll_deg) 0.0)
    ((:odom_pitch_deg __odom_pitch_deg) 0.0)
    ((:imu_norm_accel_m_s2 __imu_norm_accel_m_s2) 0.0)
    ((:lidar_ratio_of_invalid_points __lidar_ratio_of_invalid_points) 0.0)
    ((:cpu_usage_percentage __cpu_usage_percentage) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _is_locator_odom_error __is_locator_odom_error)
   (setq _is_imu_over_range __is_imu_over_range)
   (setq _is_visual_track_result_error __is_visual_track_result_error)
   (setq _is_lidar_scan_error __is_lidar_scan_error)
   (setq _is_cpu_usage_over_range __is_cpu_usage_over_range)
   (setq _odom_horizontal_velocity_m_s (float __odom_horizontal_velocity_m_s))
   (setq _odom_vertical_velocity_m_s (float __odom_vertical_velocity_m_s))
   (setq _odom_roll_deg (float __odom_roll_deg))
   (setq _odom_pitch_deg (float __odom_pitch_deg))
   (setq _imu_norm_accel_m_s2 (float __imu_norm_accel_m_s2))
   (setq _lidar_ratio_of_invalid_points (float __lidar_ratio_of_invalid_points))
   (setq _cpu_usage_percentage __cpu_usage_percentage)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:is_locator_odom_error
   (&optional (__is_locator_odom_error :null))
   (if (not (eq __is_locator_odom_error :null)) (setq _is_locator_odom_error __is_locator_odom_error)) _is_locator_odom_error)
  (:is_imu_over_range
   (&optional (__is_imu_over_range :null))
   (if (not (eq __is_imu_over_range :null)) (setq _is_imu_over_range __is_imu_over_range)) _is_imu_over_range)
  (:is_visual_track_result_error
   (&optional (__is_visual_track_result_error :null))
   (if (not (eq __is_visual_track_result_error :null)) (setq _is_visual_track_result_error __is_visual_track_result_error)) _is_visual_track_result_error)
  (:is_lidar_scan_error
   (&optional (__is_lidar_scan_error :null))
   (if (not (eq __is_lidar_scan_error :null)) (setq _is_lidar_scan_error __is_lidar_scan_error)) _is_lidar_scan_error)
  (:is_cpu_usage_over_range
   (&optional (__is_cpu_usage_over_range :null))
   (if (not (eq __is_cpu_usage_over_range :null)) (setq _is_cpu_usage_over_range __is_cpu_usage_over_range)) _is_cpu_usage_over_range)
  (:odom_horizontal_velocity_m_s
   (&optional __odom_horizontal_velocity_m_s)
   (if __odom_horizontal_velocity_m_s (setq _odom_horizontal_velocity_m_s __odom_horizontal_velocity_m_s)) _odom_horizontal_velocity_m_s)
  (:odom_vertical_velocity_m_s
   (&optional __odom_vertical_velocity_m_s)
   (if __odom_vertical_velocity_m_s (setq _odom_vertical_velocity_m_s __odom_vertical_velocity_m_s)) _odom_vertical_velocity_m_s)
  (:odom_roll_deg
   (&optional __odom_roll_deg)
   (if __odom_roll_deg (setq _odom_roll_deg __odom_roll_deg)) _odom_roll_deg)
  (:odom_pitch_deg
   (&optional __odom_pitch_deg)
   (if __odom_pitch_deg (setq _odom_pitch_deg __odom_pitch_deg)) _odom_pitch_deg)
  (:imu_norm_accel_m_s2
   (&optional __imu_norm_accel_m_s2)
   (if __imu_norm_accel_m_s2 (setq _imu_norm_accel_m_s2 __imu_norm_accel_m_s2)) _imu_norm_accel_m_s2)
  (:lidar_ratio_of_invalid_points
   (&optional __lidar_ratio_of_invalid_points)
   (if __lidar_ratio_of_invalid_points (setq _lidar_ratio_of_invalid_points __lidar_ratio_of_invalid_points)) _lidar_ratio_of_invalid_points)
  (:cpu_usage_percentage
   (&optional __cpu_usage_percentage)
   (if __cpu_usage_percentage (setq _cpu_usage_percentage __cpu_usage_percentage)) _cpu_usage_percentage)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bool _is_locator_odom_error
    1
    ;; bool _is_imu_over_range
    1
    ;; bool _is_visual_track_result_error
    1
    ;; bool _is_lidar_scan_error
    1
    ;; bool _is_cpu_usage_over_range
    1
    ;; float32 _odom_horizontal_velocity_m_s
    4
    ;; float32 _odom_vertical_velocity_m_s
    4
    ;; float32 _odom_roll_deg
    4
    ;; float32 _odom_pitch_deg
    4
    ;; float32 _imu_norm_accel_m_s2
    4
    ;; float32 _lidar_ratio_of_invalid_points
    4
    ;; float32[] _cpu_usage_percentage
    (* 4    (length _cpu_usage_percentage)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; bool _is_locator_odom_error
       (if _is_locator_odom_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _is_imu_over_range
       (if _is_imu_over_range (write-byte -1 s) (write-byte 0 s))
     ;; bool _is_visual_track_result_error
       (if _is_visual_track_result_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _is_lidar_scan_error
       (if _is_lidar_scan_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _is_cpu_usage_over_range
       (if _is_cpu_usage_over_range (write-byte -1 s) (write-byte 0 s))
     ;; float32 _odom_horizontal_velocity_m_s
       (sys::poke _odom_horizontal_velocity_m_s (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _odom_vertical_velocity_m_s
       (sys::poke _odom_vertical_velocity_m_s (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _odom_roll_deg
       (sys::poke _odom_roll_deg (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _odom_pitch_deg
       (sys::poke _odom_pitch_deg (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _imu_norm_accel_m_s2
       (sys::poke _imu_norm_accel_m_s2 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _lidar_ratio_of_invalid_points
       (sys::poke _lidar_ratio_of_invalid_points (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32[] _cpu_usage_percentage
     (write-long (length _cpu_usage_percentage) s)
     (dotimes (i (length _cpu_usage_percentage))
       (sys::poke (elt _cpu_usage_percentage i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; bool _is_locator_odom_error
     (setq _is_locator_odom_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _is_imu_over_range
     (setq _is_imu_over_range (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _is_visual_track_result_error
     (setq _is_visual_track_result_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _is_lidar_scan_error
     (setq _is_lidar_scan_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _is_cpu_usage_over_range
     (setq _is_cpu_usage_over_range (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32 _odom_horizontal_velocity_m_s
     (setq _odom_horizontal_velocity_m_s (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _odom_vertical_velocity_m_s
     (setq _odom_vertical_velocity_m_s (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _odom_roll_deg
     (setq _odom_roll_deg (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _odom_pitch_deg
     (setq _odom_pitch_deg (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _imu_norm_accel_m_s2
     (setq _imu_norm_accel_m_s2 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _lidar_ratio_of_invalid_points
     (setq _lidar_ratio_of_invalid_points (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32[] _cpu_usage_percentage
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _cpu_usage_percentage (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _cpu_usage_percentage i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get message_typedef::LocatorFdiMsg :md5sum-) "d429453896a3f9392995d3e879586d29")
(setf (get message_typedef::LocatorFdiMsg :datatype-) "message_typedef/LocatorFdiMsg")
(setf (get message_typedef::LocatorFdiMsg :definition-)
      "Header header             # ROS standard message header
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

")



(provide :message_typedef/LocatorFdiMsg "d429453896a3f9392995d3e879586d29")


