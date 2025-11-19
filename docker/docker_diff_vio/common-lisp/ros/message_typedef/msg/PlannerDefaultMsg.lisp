; Auto-generated. Do not edit!


(cl:in-package message_typedef-msg)


;//! \htmlinclude PlannerDefaultMsg.msg.html

(cl:defclass <PlannerDefaultMsg> (roslisp-msg-protocol:ros-message)
  ((is_valid
    :reader is_valid
    :initarg :is_valid
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass PlannerDefaultMsg (<PlannerDefaultMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PlannerDefaultMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PlannerDefaultMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name message_typedef-msg:<PlannerDefaultMsg> is deprecated: use message_typedef-msg:PlannerDefaultMsg instead.")))

(cl:ensure-generic-function 'is_valid-val :lambda-list '(m))
(cl:defmethod is_valid-val ((m <PlannerDefaultMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader message_typedef-msg:is_valid-val is deprecated.  Use message_typedef-msg:is_valid instead.")
  (is_valid m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PlannerDefaultMsg>) ostream)
  "Serializes a message object of type '<PlannerDefaultMsg>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_valid) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PlannerDefaultMsg>) istream)
  "Deserializes a message object of type '<PlannerDefaultMsg>"
    (cl:setf (cl:slot-value msg 'is_valid) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PlannerDefaultMsg>)))
  "Returns string type for a message object of type '<PlannerDefaultMsg>"
  "message_typedef/PlannerDefaultMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlannerDefaultMsg)))
  "Returns string type for a message object of type 'PlannerDefaultMsg"
  "message_typedef/PlannerDefaultMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PlannerDefaultMsg>)))
  "Returns md5sum for a message object of type '<PlannerDefaultMsg>"
  "bf1cc1d88653066e9e865909bc165df6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PlannerDefaultMsg)))
  "Returns md5sum for a message object of type 'PlannerDefaultMsg"
  "bf1cc1d88653066e9e865909bc165df6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PlannerDefaultMsg>)))
  "Returns full string definition for message of type '<PlannerDefaultMsg>"
  (cl:format cl:nil "bool is_valid~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PlannerDefaultMsg)))
  "Returns full string definition for message of type 'PlannerDefaultMsg"
  (cl:format cl:nil "bool is_valid~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PlannerDefaultMsg>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PlannerDefaultMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'PlannerDefaultMsg
    (cl:cons ':is_valid (is_valid msg))
))
