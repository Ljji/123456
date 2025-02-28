; Auto-generated. Do not edit!


(cl:in-package tomato_slam-msg)


;//! \htmlinclude TomatoDetection.msg.html

(cl:defclass <TomatoDetection> (roslisp-msg-protocol:ros-message)
  ((position
    :reader position
    :initarg :position
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (radius
    :reader radius
    :initarg :radius
    :type cl:float
    :initform 0.0)
   (confidence
    :reader confidence
    :initarg :confidence
    :type cl:float
    :initform 0.0))
)

(cl:defclass TomatoDetection (<TomatoDetection>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TomatoDetection>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TomatoDetection)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tomato_slam-msg:<TomatoDetection> is deprecated: use tomato_slam-msg:TomatoDetection instead.")))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <TomatoDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tomato_slam-msg:position-val is deprecated.  Use tomato_slam-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'radius-val :lambda-list '(m))
(cl:defmethod radius-val ((m <TomatoDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tomato_slam-msg:radius-val is deprecated.  Use tomato_slam-msg:radius instead.")
  (radius m))

(cl:ensure-generic-function 'confidence-val :lambda-list '(m))
(cl:defmethod confidence-val ((m <TomatoDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tomato_slam-msg:confidence-val is deprecated.  Use tomato_slam-msg:confidence instead.")
  (confidence m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TomatoDetection>) ostream)
  "Serializes a message object of type '<TomatoDetection>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'radius))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'confidence))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TomatoDetection>) istream)
  "Deserializes a message object of type '<TomatoDetection>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'radius) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'confidence) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TomatoDetection>)))
  "Returns string type for a message object of type '<TomatoDetection>"
  "tomato_slam/TomatoDetection")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TomatoDetection)))
  "Returns string type for a message object of type 'TomatoDetection"
  "tomato_slam/TomatoDetection")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TomatoDetection>)))
  "Returns md5sum for a message object of type '<TomatoDetection>"
  "d9023f05a2180cc99be40075822ec995")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TomatoDetection)))
  "Returns md5sum for a message object of type 'TomatoDetection"
  "d9023f05a2180cc99be40075822ec995")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TomatoDetection>)))
  "Returns full string definition for message of type '<TomatoDetection>"
  (cl:format cl:nil "# 番茄检测消息~%geometry_msgs/Point position  # 番茄中心位置~%float32 radius               # 番茄半径~%float32 confidence           # 检测置信度~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TomatoDetection)))
  "Returns full string definition for message of type 'TomatoDetection"
  (cl:format cl:nil "# 番茄检测消息~%geometry_msgs/Point position  # 番茄中心位置~%float32 radius               # 番茄半径~%float32 confidence           # 检测置信度~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TomatoDetection>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position))
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TomatoDetection>))
  "Converts a ROS message object to a list"
  (cl:list 'TomatoDetection
    (cl:cons ':position (position msg))
    (cl:cons ':radius (radius msg))
    (cl:cons ':confidence (confidence msg))
))
