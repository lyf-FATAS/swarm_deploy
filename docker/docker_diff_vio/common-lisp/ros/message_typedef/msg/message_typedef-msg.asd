
(cl:in-package :asdf)

(defsystem "message_typedef-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ControllerDefaultMsg" :depends-on ("_package_ControllerDefaultMsg"))
    (:file "_package_ControllerDefaultMsg" :depends-on ("_package"))
    (:file "CustomMsg" :depends-on ("_package_CustomMsg"))
    (:file "_package_CustomMsg" :depends-on ("_package"))
    (:file "CustomPoint" :depends-on ("_package_CustomPoint"))
    (:file "_package_CustomPoint" :depends-on ("_package"))
    (:file "FrontendOneDirTrackResult" :depends-on ("_package_FrontendOneDirTrackResult"))
    (:file "_package_FrontendOneDirTrackResult" :depends-on ("_package"))
    (:file "FrontendTrackResult" :depends-on ("_package_FrontendTrackResult"))
    (:file "_package_FrontendTrackResult" :depends-on ("_package"))
    (:file "LocatorFdiMsg" :depends-on ("_package_LocatorFdiMsg"))
    (:file "_package_LocatorFdiMsg" :depends-on ("_package"))
    (:file "PerceptronDefaultMsg" :depends-on ("_package_PerceptronDefaultMsg"))
    (:file "_package_PerceptronDefaultMsg" :depends-on ("_package"))
    (:file "PlannerDefaultMsg" :depends-on ("_package_PlannerDefaultMsg"))
    (:file "_package_PlannerDefaultMsg" :depends-on ("_package"))
  ))