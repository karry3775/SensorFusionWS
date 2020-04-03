
(cl:in-package :asdf)

(defsystem "sensor_fusion_pkg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "SensorMsg" :depends-on ("_package_SensorMsg"))
    (:file "_package_SensorMsg" :depends-on ("_package"))
    (:file "SensorMsgStamped" :depends-on ("_package_SensorMsgStamped"))
    (:file "_package_SensorMsgStamped" :depends-on ("_package"))
  ))