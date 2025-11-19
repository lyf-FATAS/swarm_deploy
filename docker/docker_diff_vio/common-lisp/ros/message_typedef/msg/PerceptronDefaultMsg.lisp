; Auto-generated. Do not edit!


(cl:in-package message_typedef-msg)


;//! \htmlinclude PerceptronDefaultMsg.msg.html

(cl:defclass <PerceptronDefaultMsg> (roslisp-msg-protocol:ros-message)
  ((is_valid
    :reader is_valid
    :initarg :is_valid
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass PerceptronDefaultMsg (<PerceptronDefaultMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PerceptronDefaultMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PerceptronDefaultMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name message_typedef-msg:<PerceptronDefaultMsg> is deprecated: use message_typedef-msg:PerceptronDefaultMsg instead.")))

(cl:ensure-generic-function 'is_valid-val :lambda-list '(m))
(cl:defmethod is_valid-val ((m <PerceptronDefaultMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader message_typedef-msg:is_valid-val is deprecated.  Use message_typedef-msg:is_valid instead.")
  (is_valid m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PerceptronDefaultMsg>) ostream)
  "Serializes a message object of type '<PerceptronDefaultMsg>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_valid) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PerceptronDefaultMsg>) istream)
  "Deserializes a message object of type '<PerceptronDefaultMsg>"
    (cl:setf (cl:slot-value msg 'is_valid) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PerceptronDefaultMsg>)))
  "Returns string type for a message object of type '<PerceptronDefaultMsg>"
  "message_typedef/PerceptronDefaultMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PerceptronDefaultMsg)))
  "Returns string type for a message object of type 'PerceptronDefaultMsg"
  "message_typedef/PerceptronDefaultMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PerceptronDefaultMsg>)))
  "Returns md5sum for a message object of type '<PerceptronDefaultMsg>"
  "bf1cc1d88653066e9e865909bc165df6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PerceptronDefaultMsg)))
  "Returns md5sum for a message object of type 'PerceptronDefaultMsg"
  "bf1cc1d88653066e9e865909bc165df6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PerceptronDefaultMsg>)))
  "Returns full string definition for message of type '<PerceptronDefaultMsg>"
  (cl:format cl:nil "bool is_valid~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PerceptronDefaultMsg)))
  "Returns full string definition for message of type 'PerceptronDefaultMsg"
  (cl:format cl:nil "bool is_valid~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PerceptronDefaultMsg>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PerceptronDefaultMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'PerceptronDefaultMsg
    (cl:cons ':is_valid (is_valid msg))
))
