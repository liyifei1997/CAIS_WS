;; Auto-generated. Do not edit!


(when (boundp 'kmriiwa_msgs::KMRStatus)
  (if (not (find-package "KMRIIWA_MSGS"))
    (make-package "KMRIIWA_MSGS"))
  (shadow 'KMRStatus (find-package "KMRIIWA_MSGS")))
(unless (find-package "KMRIIWA_MSGS::KMRSTATUS")
  (make-package "KMRIIWA_MSGS::KMRSTATUS"))

(in-package "ROS")
;;//! \htmlinclude KMRStatus.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass kmriiwa_msgs::KMRStatus
  :super ros::object
  :slots (_header _charge_state_percentage _motion_enabled _warning_field_clear _safety_field_clear _safety_state_enabled ))

(defmethod kmriiwa_msgs::KMRStatus
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:charge_state_percentage __charge_state_percentage) 0)
    ((:motion_enabled __motion_enabled) nil)
    ((:warning_field_clear __warning_field_clear) nil)
    ((:safety_field_clear __safety_field_clear) nil)
    ((:safety_state_enabled __safety_state_enabled) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _charge_state_percentage (round __charge_state_percentage))
   (setq _motion_enabled __motion_enabled)
   (setq _warning_field_clear __warning_field_clear)
   (setq _safety_field_clear __safety_field_clear)
   (setq _safety_state_enabled __safety_state_enabled)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:charge_state_percentage
   (&optional __charge_state_percentage)
   (if __charge_state_percentage (setq _charge_state_percentage __charge_state_percentage)) _charge_state_percentage)
  (:motion_enabled
   (&optional (__motion_enabled :null))
   (if (not (eq __motion_enabled :null)) (setq _motion_enabled __motion_enabled)) _motion_enabled)
  (:warning_field_clear
   (&optional (__warning_field_clear :null))
   (if (not (eq __warning_field_clear :null)) (setq _warning_field_clear __warning_field_clear)) _warning_field_clear)
  (:safety_field_clear
   (&optional (__safety_field_clear :null))
   (if (not (eq __safety_field_clear :null)) (setq _safety_field_clear __safety_field_clear)) _safety_field_clear)
  (:safety_state_enabled
   (&optional (__safety_state_enabled :null))
   (if (not (eq __safety_state_enabled :null)) (setq _safety_state_enabled __safety_state_enabled)) _safety_state_enabled)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int32 _charge_state_percentage
    4
    ;; bool _motion_enabled
    1
    ;; bool _warning_field_clear
    1
    ;; bool _safety_field_clear
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
     ;; int32 _charge_state_percentage
       (write-long _charge_state_percentage s)
     ;; bool _motion_enabled
       (if _motion_enabled (write-byte -1 s) (write-byte 0 s))
     ;; bool _warning_field_clear
       (if _warning_field_clear (write-byte -1 s) (write-byte 0 s))
     ;; bool _safety_field_clear
       (if _safety_field_clear (write-byte -1 s) (write-byte 0 s))
     ;; bool _safety_state_enabled
       (if _safety_state_enabled (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int32 _charge_state_percentage
     (setq _charge_state_percentage (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _motion_enabled
     (setq _motion_enabled (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _warning_field_clear
     (setq _warning_field_clear (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _safety_field_clear
     (setq _safety_field_clear (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _safety_state_enabled
     (setq _safety_state_enabled (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get kmriiwa_msgs::KMRStatus :md5sum-) "4a74515beaa6408bf61b8361cab81069")
(setf (get kmriiwa_msgs::KMRStatus :datatype-) "kmriiwa_msgs/KMRStatus")
(setf (get kmriiwa_msgs::KMRStatus :definition-)
      "Header header
int32 charge_state_percentage
bool motion_enabled
bool warning_field_clear
bool safety_field_clear
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



(provide :kmriiwa_msgs/KMRStatus "4a74515beaa6408bf61b8361cab81069")


