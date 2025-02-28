
(cl:in-package :asdf)

(defsystem "tomato_slam-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "TomatoDetection" :depends-on ("_package_TomatoDetection"))
    (:file "_package_TomatoDetection" :depends-on ("_package"))
    (:file "TomatoDetections" :depends-on ("_package_TomatoDetections"))
    (:file "_package_TomatoDetections" :depends-on ("_package"))
  ))