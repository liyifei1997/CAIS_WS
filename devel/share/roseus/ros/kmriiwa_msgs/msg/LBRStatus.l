;; Auto-generated. Do not edit!


(when (boundp 'kmriiwa_msgs::LBRStatus)
  (if (not (find-package "KMRIIWA_MSGS"))
    (make-package "KMRIIWA_MSGS"))
  (shadow 'LBRStatus (find-package "KMRIIWA_MSGS")))
(unless (find-package "KMRIIWA_MSGS::LBRSTATUS")
  (make-package "KMRIIWA_MSGS::LBRSTATUS"))

(in-package "ROS")
;;//! \htmlinclude LBRStatus.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass kmriiwa_msgs::LBRStatus
  :super ros::object
  :slots (_header _motion_enabled _axes_mastered _axes_gms_referenced _axes_position_referenced _safety_state_enabled ))

(defmethod kmriiwa_msgs::LBRStatus
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:motion_enabled __motion_enabled) nil)
    ((:axes_mastered __axes_mastered) nil)
    ((:axes_gms_referenced __axes_gms_referenced) nil)
    ((:axes_position_referenced __axes_position_referenced) nil)
    ((:safety_state_enabled __safety_state_enabled) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _motion_enabled __motion_enabled)
   (setq _axes_mastered __axes_mastered)
   (setq _axes_gms_referenced __axes_gms_referenced)
   (setq _axes_position_referenced __axes_position_referenced)
   (setq _safety_state_enabled __safety_state_enabled)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:motion_enabled
   (&optional (__motion_enabled :null))
   (if (not (eq __motion_enabled :null)) (setq _motion_enabled __motion_enabled)) _motion_enabled)
  (:axes_mastered
   (&optional (__axes_mastered :null))
   (if (not (eq __axes_mastered :null)) (setq _axes_mastered __axes_mastered)) _axes_mastered)
  (:axes_gms_referenced
   (&optional (__axes_gms_referenced :null))
   (if (not (eq __axes_gms_referenced :null)) (setq _axes_gms_referenced __axes_gms_referenced)) _axes_gms_referenced)
  (:axes_position_referenced
   (&optional (__axes_position_referenced :null))
   (if (not (eq __axes_position_referenced :null)) (setq _axes_position_referenced __axes_position_referenced)) _axes_position_referenced)
  (:safety_state_enabled
   (&optional (__safety_state_enabled :null))
   (if (not (eq __safety_state_enabled :null)) (setq _safety_state_enabled __safety_state_enabled)) _safety_state_enabled)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bool _motion_enabled
    1
    ;; bool _axes_mastered
    1
    ;; bool _axes_gms_referenced
    1
    ;; bool _axes_position_referenced
    1
    ;; bool _safety_state_enabled
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; bool _motion_enabled
       (if _motion_enabled (write-byte -1 s) (write-byte 0 s))
     ;; bool _axes_mastered
       (if _axes_mastered (write-byte -1 s) (write-byte 0 s))
     ;; bool _axes_gms_referenced
       (if _axes_gms_referenced (write-byte -1 s) (write-byte 0 s))
     ;; bool _axes_position_referenced
       (if _axes_position_referenced (write-byte -1 s) (write-byte 0 s))
     ;; bool _safety_state_enabled
       (if _safety_state_enabled (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; bool _motion_enabled
     (setq _motion_enabled (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _axes_mastered
     (setq _axes_mastered (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _axes_gms_referenced
     (setq _axes_gms_referenced (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _axes_position_referenced
     (setq _axes_position_referenced (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _safety_state_enabled
     (setq _safety_state_enabled (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get kmriiwa_msgs::LBRStatus :md5sum-) "d1e9bf004da750115463ceb70e37bbcd")
(setf (get kmriiwa_msgs::LBRStatus :datatype-) "kmriiwa_msgs/LBRStatus")
(setf (get kmriiwa_msgs::LBRStatus :definition-)
      "Header header
bool motion_enabled
bool axes_mastered
bool axes_gms_referenced
bool axes_position_referenced
bool safety_state_enabled
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



(provide :kmriiwa_msgs/LBRStatus "d1e9bf004da750115463ceb70e37bbcd")


