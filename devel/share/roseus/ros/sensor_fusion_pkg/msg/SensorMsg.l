;; Auto-generated. Do not edit!


(when (boundp 'sensor_fusion_pkg::SensorMsg)
  (if (not (find-package "SENSOR_FUSION_PKG"))
    (make-package "SENSOR_FUSION_PKG"))
  (shadow 'SensorMsg (find-package "SENSOR_FUSION_PKG")))
(unless (find-package "SENSOR_FUSION_PKG::SENSORMSG")
  (make-package "SENSOR_FUSION_PKG::SENSORMSG"))

(in-package "ROS")
;;//! \htmlinclude SensorMsg.msg.html


(defclass sensor_fusion_pkg::SensorMsg
  :super ros::object
  :slots (_data ))

(defmethod sensor_fusion_pkg::SensorMsg
  (:init
   (&key
    ((:data __data) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _data __data)
   self)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; float64[] _data
    (* 8    (length _data)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _data
     (write-long (length _data) s)
     (dotimes (i (length _data))
       (sys::poke (elt _data i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _data
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _data (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _data i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get sensor_fusion_pkg::SensorMsg :md5sum-) "788898178a3da2c3718461eecda8f714")
(setf (get sensor_fusion_pkg::SensorMsg :datatype-) "sensor_fusion_pkg/SensorMsg")
(setf (get sensor_fusion_pkg::SensorMsg :definition-)
      "float64[] data

")



(provide :sensor_fusion_pkg/SensorMsg "788898178a3da2c3718461eecda8f714")


