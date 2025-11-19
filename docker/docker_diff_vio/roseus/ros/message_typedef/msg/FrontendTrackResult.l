;; Auto-generated. Do not edit!


(when (boundp 'message_typedef::FrontendTrackResult)
  (if (not (find-package "MESSAGE_TYPEDEF"))
    (make-package "MESSAGE_TYPEDEF"))
  (shadow 'FrontendTrackResult (find-package "MESSAGE_TYPEDEF")))
(unless (find-package "MESSAGE_TYPEDEF::FRONTENDTRACKRESULT")
  (make-package "MESSAGE_TYPEDEF::FRONTENDTRACKRESULT"))

(in-package "ROS")
;;//! \htmlinclude FrontendTrackResult.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass message_typedef::FrontendTrackResult
  :super ros::object
  :slots (_header _track_results ))

(defmethod message_typedef::FrontendTrackResult
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:track_results __track_results) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _track_results __track_results)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:track_results
   (&rest __track_results)
   (if (keywordp (car __track_results))
       (send* _track_results __track_results)
     (progn
       (if __track_results (setq _track_results (car __track_results)))
       _track_results)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; message_typedef/FrontendOneDirTrackResult[] _track_results
    (apply #'+ (send-all _track_results :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; message_typedef/FrontendOneDirTrackResult[] _track_results
     (write-long (length _track_results) s)
     (dolist (elem _track_results)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; message_typedef/FrontendOneDirTrackResult[] _track_results
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _track_results (let (r) (dotimes (i n) (push (instance message_typedef::FrontendOneDirTrackResult :init) r)) r))
     (dolist (elem- _track_results)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get message_typedef::FrontendTrackResult :md5sum-) "2148ce17866256734095bba115cc9847")
(setf (get message_typedef::FrontendTrackResult :datatype-) "message_typedef/FrontendTrackResult")
(setf (get message_typedef::FrontendTrackResult :definition-)
      "Header header
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
")



(provide :message_typedef/FrontendTrackResult "2148ce17866256734095bba115cc9847")


