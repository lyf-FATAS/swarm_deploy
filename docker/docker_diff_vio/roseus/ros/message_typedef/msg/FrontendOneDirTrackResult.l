;; Auto-generated. Do not edit!


(when (boundp 'message_typedef::FrontendOneDirTrackResult)
  (if (not (find-package "MESSAGE_TYPEDEF"))
    (make-package "MESSAGE_TYPEDEF"))
  (shadow 'FrontendOneDirTrackResult (find-package "MESSAGE_TYPEDEF")))
(unless (find-package "MESSAGE_TYPEDEF::FRONTENDONEDIRTRACKRESULT")
  (make-package "MESSAGE_TYPEDEF::FRONTENDONEDIRTRACKRESULT"))

(in-package "ROS")
;;//! \htmlinclude FrontendOneDirTrackResult.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass message_typedef::FrontendOneDirTrackResult
  :super ros::object
  :slots (_is_keyframe _direction_id _depth _feature_ids _pixel_uv_left_cam _norm_xy_left_cam _sphere_xyz_left_cam _left_cam_observe _pixel_uv_right_cam _norm_xy_right_cam _sphere_xyz_right_cam _right_cam_observe ))

(defmethod message_typedef::FrontendOneDirTrackResult
  (:init
   (&key
    ((:is_keyframe __is_keyframe) nil)
    ((:direction_id __direction_id) 0)
    ((:depth __depth) 0.0)
    ((:feature_ids __feature_ids) (make-array 0 :initial-element 0 :element-type :integer))
    ((:pixel_uv_left_cam __pixel_uv_left_cam) ())
    ((:norm_xy_left_cam __norm_xy_left_cam) ())
    ((:sphere_xyz_left_cam __sphere_xyz_left_cam) ())
    ((:left_cam_observe __left_cam_observe) (let (r) (dotimes (i 0) (push nil r)) r))
    ((:pixel_uv_right_cam __pixel_uv_right_cam) ())
    ((:norm_xy_right_cam __norm_xy_right_cam) ())
    ((:sphere_xyz_right_cam __sphere_xyz_right_cam) ())
    ((:right_cam_observe __right_cam_observe) (let (r) (dotimes (i 0) (push nil r)) r))
    )
   (send-super :init)
   (setq _is_keyframe __is_keyframe)
   (setq _direction_id (round __direction_id))
   (setq _depth (float __depth))
   (setq _feature_ids __feature_ids)
   (setq _pixel_uv_left_cam __pixel_uv_left_cam)
   (setq _norm_xy_left_cam __norm_xy_left_cam)
   (setq _sphere_xyz_left_cam __sphere_xyz_left_cam)
   (setq _left_cam_observe __left_cam_observe)
   (setq _pixel_uv_right_cam __pixel_uv_right_cam)
   (setq _norm_xy_right_cam __norm_xy_right_cam)
   (setq _sphere_xyz_right_cam __sphere_xyz_right_cam)
   (setq _right_cam_observe __right_cam_observe)
   self)
  (:is_keyframe
   (&optional (__is_keyframe :null))
   (if (not (eq __is_keyframe :null)) (setq _is_keyframe __is_keyframe)) _is_keyframe)
  (:direction_id
   (&optional __direction_id)
   (if __direction_id (setq _direction_id __direction_id)) _direction_id)
  (:depth
   (&optional __depth)
   (if __depth (setq _depth __depth)) _depth)
  (:feature_ids
   (&optional __feature_ids)
   (if __feature_ids (setq _feature_ids __feature_ids)) _feature_ids)
  (:pixel_uv_left_cam
   (&rest __pixel_uv_left_cam)
   (if (keywordp (car __pixel_uv_left_cam))
       (send* _pixel_uv_left_cam __pixel_uv_left_cam)
     (progn
       (if __pixel_uv_left_cam (setq _pixel_uv_left_cam (car __pixel_uv_left_cam)))
       _pixel_uv_left_cam)))
  (:norm_xy_left_cam
   (&rest __norm_xy_left_cam)
   (if (keywordp (car __norm_xy_left_cam))
       (send* _norm_xy_left_cam __norm_xy_left_cam)
     (progn
       (if __norm_xy_left_cam (setq _norm_xy_left_cam (car __norm_xy_left_cam)))
       _norm_xy_left_cam)))
  (:sphere_xyz_left_cam
   (&rest __sphere_xyz_left_cam)
   (if (keywordp (car __sphere_xyz_left_cam))
       (send* _sphere_xyz_left_cam __sphere_xyz_left_cam)
     (progn
       (if __sphere_xyz_left_cam (setq _sphere_xyz_left_cam (car __sphere_xyz_left_cam)))
       _sphere_xyz_left_cam)))
  (:left_cam_observe
   (&optional __left_cam_observe)
   (if __left_cam_observe (setq _left_cam_observe __left_cam_observe)) _left_cam_observe)
  (:pixel_uv_right_cam
   (&rest __pixel_uv_right_cam)
   (if (keywordp (car __pixel_uv_right_cam))
       (send* _pixel_uv_right_cam __pixel_uv_right_cam)
     (progn
       (if __pixel_uv_right_cam (setq _pixel_uv_right_cam (car __pixel_uv_right_cam)))
       _pixel_uv_right_cam)))
  (:norm_xy_right_cam
   (&rest __norm_xy_right_cam)
   (if (keywordp (car __norm_xy_right_cam))
       (send* _norm_xy_right_cam __norm_xy_right_cam)
     (progn
       (if __norm_xy_right_cam (setq _norm_xy_right_cam (car __norm_xy_right_cam)))
       _norm_xy_right_cam)))
  (:sphere_xyz_right_cam
   (&rest __sphere_xyz_right_cam)
   (if (keywordp (car __sphere_xyz_right_cam))
       (send* _sphere_xyz_right_cam __sphere_xyz_right_cam)
     (progn
       (if __sphere_xyz_right_cam (setq _sphere_xyz_right_cam (car __sphere_xyz_right_cam)))
       _sphere_xyz_right_cam)))
  (:right_cam_observe
   (&optional __right_cam_observe)
   (if __right_cam_observe (setq _right_cam_observe __right_cam_observe)) _right_cam_observe)
  (:serialization-length
   ()
   (+
    ;; bool _is_keyframe
    1
    ;; int8 _direction_id
    1
    ;; float32 _depth
    4
    ;; uint32[] _feature_ids
    (* 4    (length _feature_ids)) 4
    ;; geometry_msgs/Vector3[] _pixel_uv_left_cam
    (apply #'+ (send-all _pixel_uv_left_cam :serialization-length)) 4
    ;; geometry_msgs/Vector3[] _norm_xy_left_cam
    (apply #'+ (send-all _norm_xy_left_cam :serialization-length)) 4
    ;; geometry_msgs/Vector3[] _sphere_xyz_left_cam
    (apply #'+ (send-all _sphere_xyz_left_cam :serialization-length)) 4
    ;; bool[] _left_cam_observe
    (* 1    (length _left_cam_observe)) 4
    ;; geometry_msgs/Vector3[] _pixel_uv_right_cam
    (apply #'+ (send-all _pixel_uv_right_cam :serialization-length)) 4
    ;; geometry_msgs/Vector3[] _norm_xy_right_cam
    (apply #'+ (send-all _norm_xy_right_cam :serialization-length)) 4
    ;; geometry_msgs/Vector3[] _sphere_xyz_right_cam
    (apply #'+ (send-all _sphere_xyz_right_cam :serialization-length)) 4
    ;; bool[] _right_cam_observe
    (* 1    (length _right_cam_observe)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _is_keyframe
       (if _is_keyframe (write-byte -1 s) (write-byte 0 s))
     ;; int8 _direction_id
       (write-byte _direction_id s)
     ;; float32 _depth
       (sys::poke _depth (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint32[] _feature_ids
     (write-long (length _feature_ids) s)
     (dotimes (i (length _feature_ids))
       (write-long (elt _feature_ids i) s)
       )
     ;; geometry_msgs/Vector3[] _pixel_uv_left_cam
     (write-long (length _pixel_uv_left_cam) s)
     (dolist (elem _pixel_uv_left_cam)
       (send elem :serialize s)
       )
     ;; geometry_msgs/Vector3[] _norm_xy_left_cam
     (write-long (length _norm_xy_left_cam) s)
     (dolist (elem _norm_xy_left_cam)
       (send elem :serialize s)
       )
     ;; geometry_msgs/Vector3[] _sphere_xyz_left_cam
     (write-long (length _sphere_xyz_left_cam) s)
     (dolist (elem _sphere_xyz_left_cam)
       (send elem :serialize s)
       )
     ;; bool[] _left_cam_observe
     (write-long (length _left_cam_observe) s)
     (dotimes (i (length _left_cam_observe))
       (if (elt _left_cam_observe i) (write-byte -1 s) (write-byte 0 s))
       )
     ;; geometry_msgs/Vector3[] _pixel_uv_right_cam
     (write-long (length _pixel_uv_right_cam) s)
     (dolist (elem _pixel_uv_right_cam)
       (send elem :serialize s)
       )
     ;; geometry_msgs/Vector3[] _norm_xy_right_cam
     (write-long (length _norm_xy_right_cam) s)
     (dolist (elem _norm_xy_right_cam)
       (send elem :serialize s)
       )
     ;; geometry_msgs/Vector3[] _sphere_xyz_right_cam
     (write-long (length _sphere_xyz_right_cam) s)
     (dolist (elem _sphere_xyz_right_cam)
       (send elem :serialize s)
       )
     ;; bool[] _right_cam_observe
     (write-long (length _right_cam_observe) s)
     (dotimes (i (length _right_cam_observe))
       (if (elt _right_cam_observe i) (write-byte -1 s) (write-byte 0 s))
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _is_keyframe
     (setq _is_keyframe (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int8 _direction_id
     (setq _direction_id (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _direction_id 127) (setq _direction_id (- _direction_id 256)))
   ;; float32 _depth
     (setq _depth (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint32[] _feature_ids
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _feature_ids (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _feature_ids i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; geometry_msgs/Vector3[] _pixel_uv_left_cam
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _pixel_uv_left_cam (let (r) (dotimes (i n) (push (instance geometry_msgs::Vector3 :init) r)) r))
     (dolist (elem- _pixel_uv_left_cam)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; geometry_msgs/Vector3[] _norm_xy_left_cam
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _norm_xy_left_cam (let (r) (dotimes (i n) (push (instance geometry_msgs::Vector3 :init) r)) r))
     (dolist (elem- _norm_xy_left_cam)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; geometry_msgs/Vector3[] _sphere_xyz_left_cam
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _sphere_xyz_left_cam (let (r) (dotimes (i n) (push (instance geometry_msgs::Vector3 :init) r)) r))
     (dolist (elem- _sphere_xyz_left_cam)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; bool[] _left_cam_observe
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _left_cam_observe (make-list n))
     (dotimes (i n)
     (setf (elt _left_cam_observe i) (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
     ))
   ;; geometry_msgs/Vector3[] _pixel_uv_right_cam
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _pixel_uv_right_cam (let (r) (dotimes (i n) (push (instance geometry_msgs::Vector3 :init) r)) r))
     (dolist (elem- _pixel_uv_right_cam)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; geometry_msgs/Vector3[] _norm_xy_right_cam
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _norm_xy_right_cam (let (r) (dotimes (i n) (push (instance geometry_msgs::Vector3 :init) r)) r))
     (dolist (elem- _norm_xy_right_cam)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; geometry_msgs/Vector3[] _sphere_xyz_right_cam
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _sphere_xyz_right_cam (let (r) (dotimes (i n) (push (instance geometry_msgs::Vector3 :init) r)) r))
     (dolist (elem- _sphere_xyz_right_cam)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; bool[] _right_cam_observe
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _right_cam_observe (make-list n))
     (dotimes (i n)
     (setf (elt _right_cam_observe i) (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
     ))
   ;;
   self)
  )

(setf (get message_typedef::FrontendOneDirTrackResult :md5sum-) "87020366231c2b8da1d75e57baab7c8e")
(setf (get message_typedef::FrontendOneDirTrackResult :datatype-) "message_typedef/FrontendOneDirTrackResult")
(setf (get message_typedef::FrontendOneDirTrackResult :definition-)
      "bool is_keyframe
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



(provide :message_typedef/FrontendOneDirTrackResult "87020366231c2b8da1d75e57baab7c8e")


