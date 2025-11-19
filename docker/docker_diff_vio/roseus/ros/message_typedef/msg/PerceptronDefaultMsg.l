;; Auto-generated. Do not edit!


(when (boundp 'message_typedef::PerceptronDefaultMsg)
  (if (not (find-package "MESSAGE_TYPEDEF"))
    (make-package "MESSAGE_TYPEDEF"))
  (shadow 'PerceptronDefaultMsg (find-package "MESSAGE_TYPEDEF")))
(unless (find-package "MESSAGE_TYPEDEF::PERCEPTRONDEFAULTMSG")
  (make-package "MESSAGE_TYPEDEF::PERCEPTRONDEFAULTMSG"))

(in-package "ROS")
;;//! \htmlinclude PerceptronDefaultMsg.msg.html


(defclass message_typedef::PerceptronDefaultMsg
  :super ros::object
  :slots (_is_valid ))

(defmethod message_typedef::PerceptronDefaultMsg
  (:init
   (&key
    ((:is_valid __is_valid) nil)
    )
   (send-super :init)
   (setq _is_valid __is_valid)
   self)
  (:is_valid
   (&optional (__is_valid :null))
   (if (not (eq __is_valid :null)) (setq _is_valid __is_valid)) _is_valid)
  (:serialization-length
   ()
   (+
    ;; bool _is_valid
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _is_valid
       (if _is_valid (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _is_valid
     (setq _is_valid (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get message_typedef::PerceptronDefaultMsg :md5sum-) "bf1cc1d88653066e9e865909bc165df6")
(setf (get message_typedef::PerceptronDefaultMsg :datatype-) "message_typedef/PerceptronDefaultMsg")
(setf (get message_typedef::PerceptronDefaultMsg :definition-)
      "bool is_valid

")



(provide :message_typedef/PerceptronDefaultMsg "bf1cc1d88653066e9e865909bc165df6")


