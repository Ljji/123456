;; Auto-generated. Do not edit!


(when (boundp 'tomato_slam::TomatoDetection)
  (if (not (find-package "TOMATO_SLAM"))
    (make-package "TOMATO_SLAM"))
  (shadow 'TomatoDetection (find-package "TOMATO_SLAM")))
(unless (find-package "TOMATO_SLAM::TOMATODETECTION")
  (make-package "TOMATO_SLAM::TOMATODETECTION"))

(in-package "ROS")
;;//! \htmlinclude TomatoDetection.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass tomato_slam::TomatoDetection
  :super ros::object
  :slots (_position _radius _confidence ))

(defmethod tomato_slam::TomatoDetection
  (:init
   (&key
    ((:position __position) (instance geometry_msgs::Point :init))
    ((:radius __radius) 0.0)
    ((:confidence __confidence) 0.0)
    )
   (send-super :init)
   (setq _position __position)
   (setq _radius (float __radius))
   (setq _confidence (float __confidence))
   self)
  (:position
   (&rest __position)
   (if (keywordp (car __position))
       (send* _position __position)
     (progn
       (if __position (setq _position (car __position)))
       _position)))
  (:radius
   (&optional __radius)
   (if __radius (setq _radius __radius)) _radius)
  (:confidence
   (&optional __confidence)
   (if __confidence (setq _confidence __confidence)) _confidence)
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Point _position
    (send _position :serialization-length)
    ;; float32 _radius
    4
    ;; float32 _confidence
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Point _position
       (send _position :serialize s)
     ;; float32 _radius
       (sys::poke _radius (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _confidence
       (sys::poke _confidence (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Point _position
     (send _position :deserialize buf ptr-) (incf ptr- (send _position :serialization-length))
   ;; float32 _radius
     (setq _radius (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _confidence
     (setq _confidence (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get tomato_slam::TomatoDetection :md5sum-) "d9023f05a2180cc99be40075822ec995")
(setf (get tomato_slam::TomatoDetection :datatype-) "tomato_slam/TomatoDetection")
(setf (get tomato_slam::TomatoDetection :definition-)
      "# 番茄检测消息
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



(provide :tomato_slam/TomatoDetection "d9023f05a2180cc99be40075822ec995")


