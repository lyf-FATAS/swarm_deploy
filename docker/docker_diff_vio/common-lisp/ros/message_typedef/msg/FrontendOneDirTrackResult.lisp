; Auto-generated. Do not edit!


(cl:in-package message_typedef-msg)


;//! \htmlinclude FrontendOneDirTrackResult.msg.html

(cl:defclass <FrontendOneDirTrackResult> (roslisp-msg-protocol:ros-message)
  ((is_keyframe
    :reader is_keyframe
    :initarg :is_keyframe
    :type cl:boolean
    :initform cl:nil)
   (direction_id
    :reader direction_id
    :initarg :direction_id
    :type cl:fixnum
    :initform 0)
   (depth
    :reader depth
    :initarg :depth
    :type cl:float
    :initform 0.0)
   (feature_ids
    :reader feature_ids
    :initarg :feature_ids
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0))
   (pixel_uv_left_cam
    :reader pixel_uv_left_cam
    :initarg :pixel_uv_left_cam
    :type (cl:vector geometry_msgs-msg:Vector3)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Vector3 :initial-element (cl:make-instance 'geometry_msgs-msg:Vector3)))
   (norm_xy_left_cam
    :reader norm_xy_left_cam
    :initarg :norm_xy_left_cam
    :type (cl:vector geometry_msgs-msg:Vector3)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Vector3 :initial-element (cl:make-instance 'geometry_msgs-msg:Vector3)))
   (sphere_xyz_left_cam
    :reader sphere_xyz_left_cam
    :initarg :sphere_xyz_left_cam
    :type (cl:vector geometry_msgs-msg:Vector3)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Vector3 :initial-element (cl:make-instance 'geometry_msgs-msg:Vector3)))
   (left_cam_observe
    :reader left_cam_observe
    :initarg :left_cam_observe
    :type (cl:vector cl:boolean)
   :initform (cl:make-array 0 :element-type 'cl:boolean :initial-element cl:nil))
   (pixel_uv_right_cam
    :reader pixel_uv_right_cam
    :initarg :pixel_uv_right_cam
    :type (cl:vector geometry_msgs-msg:Vector3)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Vector3 :initial-element (cl:make-instance 'geometry_msgs-msg:Vector3)))
   (norm_xy_right_cam
    :reader norm_xy_right_cam
    :initarg :norm_xy_right_cam
    :type (cl:vector geometry_msgs-msg:Vector3)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Vector3 :initial-element (cl:make-instance 'geometry_msgs-msg:Vector3)))
   (sphere_xyz_right_cam
    :reader sphere_xyz_right_cam
    :initarg :sphere_xyz_right_cam
    :type (cl:vector geometry_msgs-msg:Vector3)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Vector3 :initial-element (cl:make-instance 'geometry_msgs-msg:Vector3)))
   (right_cam_observe
    :reader right_cam_observe
    :initarg :right_cam_observe
    :type (cl:vector cl:boolean)
   :initform (cl:make-array 0 :element-type 'cl:boolean :initial-element cl:nil)))
)

(cl:defclass FrontendOneDirTrackResult (<FrontendOneDirTrackResult>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FrontendOneDirTrackResult>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FrontendOneDirTrackResult)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name message_typedef-msg:<FrontendOneDirTrackResult> is deprecated: use message_typedef-msg:FrontendOneDirTrackResult instead.")))

(cl:ensure-generic-function 'is_keyframe-val :lambda-list '(m))
(cl:defmethod is_keyframe-val ((m <FrontendOneDirTrackResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader message_typedef-msg:is_keyframe-val is deprecated.  Use message_typedef-msg:is_keyframe instead.")
  (is_keyframe m))

(cl:ensure-generic-function 'direction_id-val :lambda-list '(m))
(cl:defmethod direction_id-val ((m <FrontendOneDirTrackResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader message_typedef-msg:direction_id-val is deprecated.  Use message_typedef-msg:direction_id instead.")
  (direction_id m))

(cl:ensure-generic-function 'depth-val :lambda-list '(m))
(cl:defmethod depth-val ((m <FrontendOneDirTrackResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader message_typedef-msg:depth-val is deprecated.  Use message_typedef-msg:depth instead.")
  (depth m))

(cl:ensure-generic-function 'feature_ids-val :lambda-list '(m))
(cl:defmethod feature_ids-val ((m <FrontendOneDirTrackResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader message_typedef-msg:feature_ids-val is deprecated.  Use message_typedef-msg:feature_ids instead.")
  (feature_ids m))

(cl:ensure-generic-function 'pixel_uv_left_cam-val :lambda-list '(m))
(cl:defmethod pixel_uv_left_cam-val ((m <FrontendOneDirTrackResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader message_typedef-msg:pixel_uv_left_cam-val is deprecated.  Use message_typedef-msg:pixel_uv_left_cam instead.")
  (pixel_uv_left_cam m))

(cl:ensure-generic-function 'norm_xy_left_cam-val :lambda-list '(m))
(cl:defmethod norm_xy_left_cam-val ((m <FrontendOneDirTrackResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader message_typedef-msg:norm_xy_left_cam-val is deprecated.  Use message_typedef-msg:norm_xy_left_cam instead.")
  (norm_xy_left_cam m))

(cl:ensure-generic-function 'sphere_xyz_left_cam-val :lambda-list '(m))
(cl:defmethod sphere_xyz_left_cam-val ((m <FrontendOneDirTrackResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader message_typedef-msg:sphere_xyz_left_cam-val is deprecated.  Use message_typedef-msg:sphere_xyz_left_cam instead.")
  (sphere_xyz_left_cam m))

(cl:ensure-generic-function 'left_cam_observe-val :lambda-list '(m))
(cl:defmethod left_cam_observe-val ((m <FrontendOneDirTrackResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader message_typedef-msg:left_cam_observe-val is deprecated.  Use message_typedef-msg:left_cam_observe instead.")
  (left_cam_observe m))

(cl:ensure-generic-function 'pixel_uv_right_cam-val :lambda-list '(m))
(cl:defmethod pixel_uv_right_cam-val ((m <FrontendOneDirTrackResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader message_typedef-msg:pixel_uv_right_cam-val is deprecated.  Use message_typedef-msg:pixel_uv_right_cam instead.")
  (pixel_uv_right_cam m))

(cl:ensure-generic-function 'norm_xy_right_cam-val :lambda-list '(m))
(cl:defmethod norm_xy_right_cam-val ((m <FrontendOneDirTrackResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader message_typedef-msg:norm_xy_right_cam-val is deprecated.  Use message_typedef-msg:norm_xy_right_cam instead.")
  (norm_xy_right_cam m))

(cl:ensure-generic-function 'sphere_xyz_right_cam-val :lambda-list '(m))
(cl:defmethod sphere_xyz_right_cam-val ((m <FrontendOneDirTrackResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader message_typedef-msg:sphere_xyz_right_cam-val is deprecated.  Use message_typedef-msg:sphere_xyz_right_cam instead.")
  (sphere_xyz_right_cam m))

(cl:ensure-generic-function 'right_cam_observe-val :lambda-list '(m))
(cl:defmethod right_cam_observe-val ((m <FrontendOneDirTrackResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader message_typedef-msg:right_cam_observe-val is deprecated.  Use message_typedef-msg:right_cam_observe instead.")
  (right_cam_observe m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FrontendOneDirTrackResult>) ostream)
  "Serializes a message object of type '<FrontendOneDirTrackResult>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_keyframe) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'direction_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'depth))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'feature_ids))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) ele) ostream))
   (cl:slot-value msg 'feature_ids))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'pixel_uv_left_cam))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'pixel_uv_left_cam))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'norm_xy_left_cam))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'norm_xy_left_cam))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'sphere_xyz_left_cam))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'sphere_xyz_left_cam))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'left_cam_observe))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if ele 1 0)) ostream))
   (cl:slot-value msg 'left_cam_observe))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'pixel_uv_right_cam))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'pixel_uv_right_cam))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'norm_xy_right_cam))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'norm_xy_right_cam))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'sphere_xyz_right_cam))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'sphere_xyz_right_cam))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'right_cam_observe))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if ele 1 0)) ostream))
   (cl:slot-value msg 'right_cam_observe))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FrontendOneDirTrackResult>) istream)
  "Deserializes a message object of type '<FrontendOneDirTrackResult>"
    (cl:setf (cl:slot-value msg 'is_keyframe) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'direction_id) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'depth) (roslisp-utils:decode-single-float-bits bits)))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'feature_ids) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'feature_ids)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:aref vals i)) (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'pixel_uv_left_cam) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'pixel_uv_left_cam)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Vector3))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'norm_xy_left_cam) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'norm_xy_left_cam)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Vector3))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'sphere_xyz_left_cam) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'sphere_xyz_left_cam)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Vector3))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'left_cam_observe) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'left_cam_observe)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:not (cl:zerop (cl:read-byte istream)))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'pixel_uv_right_cam) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'pixel_uv_right_cam)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Vector3))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'norm_xy_right_cam) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'norm_xy_right_cam)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Vector3))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'sphere_xyz_right_cam) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'sphere_xyz_right_cam)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Vector3))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'right_cam_observe) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'right_cam_observe)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:not (cl:zerop (cl:read-byte istream)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FrontendOneDirTrackResult>)))
  "Returns string type for a message object of type '<FrontendOneDirTrackResult>"
  "message_typedef/FrontendOneDirTrackResult")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FrontendOneDirTrackResult)))
  "Returns string type for a message object of type 'FrontendOneDirTrackResult"
  "message_typedef/FrontendOneDirTrackResult")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FrontendOneDirTrackResult>)))
  "Returns md5sum for a message object of type '<FrontendOneDirTrackResult>"
  "87020366231c2b8da1d75e57baab7c8e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FrontendOneDirTrackResult)))
  "Returns md5sum for a message object of type 'FrontendOneDirTrackResult"
  "87020366231c2b8da1d75e57baab7c8e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FrontendOneDirTrackResult>)))
  "Returns full string definition for message of type '<FrontendOneDirTrackResult>"
  (cl:format cl:nil "bool is_keyframe~%int8 direction_id~%float32 depth~%uint32[] feature_ids~%~%geometry_msgs/Vector3[] pixel_uv_left_cam~%geometry_msgs/Vector3[] norm_xy_left_cam~%geometry_msgs/Vector3[] sphere_xyz_left_cam~%bool[] left_cam_observe~%~%geometry_msgs/Vector3[] pixel_uv_right_cam~%geometry_msgs/Vector3[] norm_xy_right_cam~%geometry_msgs/Vector3[] sphere_xyz_right_cam~%bool[] right_cam_observe~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FrontendOneDirTrackResult)))
  "Returns full string definition for message of type 'FrontendOneDirTrackResult"
  (cl:format cl:nil "bool is_keyframe~%int8 direction_id~%float32 depth~%uint32[] feature_ids~%~%geometry_msgs/Vector3[] pixel_uv_left_cam~%geometry_msgs/Vector3[] norm_xy_left_cam~%geometry_msgs/Vector3[] sphere_xyz_left_cam~%bool[] left_cam_observe~%~%geometry_msgs/Vector3[] pixel_uv_right_cam~%geometry_msgs/Vector3[] norm_xy_right_cam~%geometry_msgs/Vector3[] sphere_xyz_right_cam~%bool[] right_cam_observe~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FrontendOneDirTrackResult>))
  (cl:+ 0
     1
     1
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'feature_ids) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'pixel_uv_left_cam) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'norm_xy_left_cam) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'sphere_xyz_left_cam) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'left_cam_observe) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'pixel_uv_right_cam) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'norm_xy_right_cam) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'sphere_xyz_right_cam) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'right_cam_observe) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FrontendOneDirTrackResult>))
  "Converts a ROS message object to a list"
  (cl:list 'FrontendOneDirTrackResult
    (cl:cons ':is_keyframe (is_keyframe msg))
    (cl:cons ':direction_id (direction_id msg))
    (cl:cons ':depth (depth msg))
    (cl:cons ':feature_ids (feature_ids msg))
    (cl:cons ':pixel_uv_left_cam (pixel_uv_left_cam msg))
    (cl:cons ':norm_xy_left_cam (norm_xy_left_cam msg))
    (cl:cons ':sphere_xyz_left_cam (sphere_xyz_left_cam msg))
    (cl:cons ':left_cam_observe (left_cam_observe msg))
    (cl:cons ':pixel_uv_right_cam (pixel_uv_right_cam msg))
    (cl:cons ':norm_xy_right_cam (norm_xy_right_cam msg))
    (cl:cons ':sphere_xyz_right_cam (sphere_xyz_right_cam msg))
    (cl:cons ':right_cam_observe (right_cam_observe msg))
))
