; Auto-generated. Do not edit!


(cl:in-package tomato_slam-msg)


;//! \htmlinclude TomatoDetections.msg.html

(cl:defclass <TomatoDetections> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (detections
    :reader detections
    :initarg :detections
    :type (cl:vector tomato_slam-msg:TomatoDetection)
   :initform (cl:make-array 0 :element-type 'tomato_slam-msg:TomatoDetection :initial-element (cl:make-instance 'tomato_slam-msg:TomatoDetection))))
)

(cl:defclass TomatoDetections (<TomatoDetections>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TomatoDetections>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TomatoDetections)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tomato_slam-msg:<TomatoDetections> is deprecated: use tomato_slam-msg:TomatoDetections instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <TomatoDetections>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tomato_slam-msg:header-val is deprecated.  Use tomato_slam-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'detections-val :lambda-list '(m))
(cl:defmethod detections-val ((m <TomatoDetections>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tomato_slam-msg:detections-val is deprecated.  Use tomato_slam-msg:detections instead.")
  (detections m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TomatoDetections>) ostream)
  "Serializes a message object of type '<TomatoDetections>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'detections))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'detections))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TomatoDetections>) istream)
  "Deserializes a message object of type '<TomatoDetections>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'detections) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'detections)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'tomato_slam-msg:TomatoDetection))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TomatoDetections>)))
  "Returns string type for a message object of type '<TomatoDetections>"
  "tomato_slam/TomatoDetections")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TomatoDetections)))
  "Returns string type for a message object of type 'TomatoDetections"
  "tomato_slam/TomatoDetections")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TomatoDetections>)))
  "Returns md5sum for a message object of type '<TomatoDetections>"
  "c67131f4672dd62f883ff3c6bb318393")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TomatoDetections)))
  "Returns md5sum for a message object of type 'TomatoDetections"
  "c67131f4672dd62f883ff3c6bb318393")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TomatoDetections>)))
  "Returns full string definition for message of type '<TomatoDetections>"
  (cl:format cl:nil "# 多番茄检测消息~%Header header~%tomato_slam/TomatoDetection[] detections~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: tomato_slam/TomatoDetection~%# 番茄检测消息~%geometry_msgs/Point position  # 番茄中心位置~%float32 radius               # 番茄半径~%float32 confidence           # 检测置信度~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TomatoDetections)))
  "Returns full string definition for message of type 'TomatoDetections"
  (cl:format cl:nil "# 多番茄检测消息~%Header header~%tomato_slam/TomatoDetection[] detections~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: tomato_slam/TomatoDetection~%# 番茄检测消息~%geometry_msgs/Point position  # 番茄中心位置~%float32 radius               # 番茄半径~%float32 confidence           # 检测置信度~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TomatoDetections>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'detections) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TomatoDetections>))
  "Converts a ROS message object to a list"
  (cl:list 'TomatoDetections
    (cl:cons ':header (header msg))
    (cl:cons ':detections (detections msg))
))
