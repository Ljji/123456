;; Auto-generated. Do not edit!


(when (boundp 'tomato_slam::TomatoDetections)
  (if (not (find-package "TOMATO_SLAM"))
    (make-package "TOMATO_SLAM"))
  (shadow 'TomatoDetections (find-package "TOMATO_SLAM")))
(unless (find-package "TOMATO_SLAM::TOMATODETECTIONS")
  (make-package "TOMATO_SLAM::TOMATODETECTIONS"))

(in-package "ROS")
;;//! \htmlinclude TomatoDetections.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass tomato_slam::TomatoDetections
  :super ros::object
  :slots (_header _detections ))

(defmethod tomato_slam::TomatoDetections
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:detections __detections) (let (r) (dotimes (i 0) (push (instance tomato_slam::TomatoDetection :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _detections __detections)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:detections
   (&rest __detections)
   (if (keywordp (car __detections))
       (send* _detections __detections)
     (progn
       (if __detections (setq _detections (car __detections)))
       _detections)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; tomato_slam/TomatoDetection[] _detections
    (apply #'+ (send-all _detections :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; tomato_slam/TomatoDetection[] _detections
     (write-long (length _detections) s)
     (dolist (elem _detections)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; tomato_slam/TomatoDetection[] _detections
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _detections (let (r) (dotimes (i n) (push (instance tomato_slam::TomatoDetection :init) r)) r))
     (dolist (elem- _detections)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get tomato_slam::TomatoDetections :md5sum-) "c67131f4672dd62f883ff3c6bb318393")
(setf (get tomato_slam::TomatoDetections :datatype-) "tomato_slam/TomatoDetections")
(setf (get tomato_slam::TomatoDetections :definition-)
      "# 多番茄检测消息
Header header
tomato_slam/TomatoDetection[] detections

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: tomato_slam/TomatoDetection
# 番茄检测消息
geometry_msgs/Point position  # 番茄中心位置
float32 radius               # 番茄半径
float32 confidence           # 检测置信度

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :tomato_slam/TomatoDetections "c67131f4672dd62f883ff3c6bb318393")


