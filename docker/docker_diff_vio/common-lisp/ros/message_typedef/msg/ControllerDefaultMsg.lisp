; Auto-generated. Do not edit!


(cl:in-package message_typedef-msg)


;//! \htmlinclude ControllerDefaultMsg.msg.html

(cl:defclass <ControllerDefaultMsg> (roslisp-msg-protocol:ros-message)
  ((is_valid
    :reader is_valid
    :initarg :is_valid
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ControllerDefaultMsg (<ControllerDefaultMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControllerDefaultMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControllerDefaultMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name message_typedef-msg:<ControllerDefaultMsg> is deprecated: use message_typedef-msg:ControllerDefaultMsg instead.")))

(cl:ensure-generic-function 'is_valid-val :lambda-list '(m))
(cl:defmethod is_valid-val ((m <ControllerDefaultMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader message_typedef-msg:is_valid-val is deprecated.  Use message_typedef-msg:is_valid instead.")
  (is_valid m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControllerDefaultMsg>) ostream)
  "Serializes a message object of type '<ControllerDefaultMsg>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_valid) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControllerDefaultMsg>) istream)
  "Deserializes a message object of type '<ControllerDefaultMsg>"
    (cl:setf (cl:slot-value msg 'is_valid) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControllerDefaultMsg>)))
  "Returns string type for a message object of type '<ControllerDefaultMsg>"
  "message_typedef/ControllerDefaultMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControllerDefaultMsg)))
  "Returns string type for a message object of type 'ControllerDefaultMsg"
  "message_typedef/ControllerDefaultMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControllerDefaultMsg>)))
  "Returns md5sum for a message object of type '<ControllerDefaultMsg>"
  "bf1cc1d88653066e9e865909bc165df6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControllerDefaultMsg)))
  "Returns md5sum for a message object of type 'ControllerDefaultMsg"
  "bf1cc1d88653066e9e865909bc165df6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControllerDefaultMsg>)))
  "Returns full string definition for message of type '<ControllerDefaultMsg>"
  (cl:format cl:nil "bool is_valid~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControllerDefaultMsg)))
  "Returns full string definition for message of type 'ControllerDefaultMsg"
  (cl:format cl:nil "bool is_valid~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControllerDefaultMsg>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControllerDefaultMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'ControllerDefaultMsg
    (cl:cons ':is_valid (is_valid msg))
))
