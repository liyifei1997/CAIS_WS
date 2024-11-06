;; Auto-generated. Do not edit!


(when (boundp 'kmriiwa_msgs::JointPosition)
  (if (not (find-package "KMRIIWA_MSGS"))
    (make-package "KMRIIWA_MSGS"))
  (shadow 'JointPosition (find-package "KMRIIWA_MSGS")))
(unless (find-package "KMRIIWA_MSGS::JOINTPOSITION")
  (make-package "KMRIIWA_MSGS::JOINTPOSITION"))

(in-package "ROS")
;;//! \htmlinclude JointPosition.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass kmriiwa_msgs::JointPosition
  :super ros::object
  :slots (_header _a1 _a2 _a3 _a4 _a5 _a6 _a7 ))

(defmethod kmriiwa_msgs::JointPosition
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:a1 __a1) 0.0)
    ((:a2 __a2) 0.0)
    ((:a3 __a3) 0.0)
    ((:a4 __a4) 0.0)
    ((:a5 __a5) 0.0)
    ((:a6 __a6) 0.0)
    ((:a7 __a7) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _a1 (float __a1))
   (setq _a2 (float __a2))
   (setq _a3 (float __a3))
   (setq _a4 (float __a4))
   (setq _a5 (float __a5))
   (setq _a6 (float __a6))
   (setq _a7 (float __a7))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:a1
   (&optional __a1)
   (if __a1 (setq _a1 __a1)) _a1)
  (:a2
   (&optional __a2)
   (if __a2 (setq _a2 __a2)) _a2)
  (:a3
   (&optional __a3)
   (if __a3 (setq _a3 __a3)) _a3)
  (:a4
   (&optional __a4)
   (if __a4 (setq _a4 __a4)) _a4)
  (:a5
   (&optional __a5)
   (if __a5 (setq _a5 __a5)) _a5)
  (:a6
   (&optional __a6)
   (if __a6 (setq _a6 __a6)) _a6)
  (:a7
   (&optional __a7)
   (if __a7 (setq _a7 __a7)) _a7)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32 _a1
    4
    ;; float32 _a2
    4
    ;; float32 _a3
    4
    ;; float32 _a4
    4
    ;; float32 _a5
    4
    ;; float32 _a6
    4
    ;; float32 _a7
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32 _a1
       (sys::poke _a1 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _a2
       (sys::poke _a2 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _a3
       (sys::poke _a3 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _a4
       (sys::poke _a4 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _a5
       (sys::poke _a5 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _a6
       (sys::poke _a6 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _a7
       (sys::poke _a7 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32 _a1
     (setq _a1 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _a2
     (setq _a2 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _a3
     (setq _a3 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _a4
     (setq _a4 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _a5
     (setq _a5 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _a6
     (setq _a6 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _a7
     (setq _a7 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kmriiwa_msgs::JointPosition :md5sum-) "5d0a4f29c23a5c6c3eab418768bb26e4")
(setf (get kmriiwa_msgs::JointPosition :datatype-) "kmriiwa_msgs/JointPosition")
(setf (get kmriiwa_msgs::JointPosition :definition-)
      "Header header
float32 a1
float32 a2
float32 a3
float32 a4
float32 a5
float32 a6
float32 a7
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

")



(provide :kmriiwa_msgs/JointPosition "5d0a4f29c23a5c6c3eab418768bb26e4")


