; Auto-generated. Do not edit!


(cl:in-package sensor_fusion_pkg-msg)


;//! \htmlinclude SensorMsg.msg.html

(cl:defclass <SensorMsg> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass SensorMsg (<SensorMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SensorMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SensorMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sensor_fusion_pkg-msg:<SensorMsg> is deprecated: use sensor_fusion_pkg-msg:SensorMsg instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <SensorMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sensor_fusion_pkg-msg:data-val is deprecated.  Use sensor_fusion_pkg-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SensorMsg>) ostream)
  "Serializes a message object of type '<SensorMsg>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SensorMsg>) istream)
  "Deserializes a message object of type '<SensorMsg>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SensorMsg>)))
  "Returns string type for a message object of type '<SensorMsg>"
  "sensor_fusion_pkg/SensorMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SensorMsg)))
  "Returns string type for a message object of type 'SensorMsg"
  "sensor_fusion_pkg/SensorMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SensorMsg>)))
  "Returns md5sum for a message object of type '<SensorMsg>"
  "788898178a3da2c3718461eecda8f714")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SensorMsg)))
  "Returns md5sum for a message object of type 'SensorMsg"
  "788898178a3da2c3718461eecda8f714")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SensorMsg>)))
  "Returns full string definition for message of type '<SensorMsg>"
  (cl:format cl:nil "float64[] data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SensorMsg)))
  "Returns full string definition for message of type 'SensorMsg"
  (cl:format cl:nil "float64[] data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SensorMsg>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SensorMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'SensorMsg
    (cl:cons ':data (data msg))
))
