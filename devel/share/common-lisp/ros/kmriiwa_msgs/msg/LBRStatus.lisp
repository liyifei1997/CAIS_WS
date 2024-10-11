; Auto-generated. Do not edit!


(cl:in-package kmriiwa_msgs-msg)


;//! \htmlinclude LBRStatus.msg.html

(cl:defclass <LBRStatus> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (motion_enabled
    :reader motion_enabled
    :initarg :motion_enabled
    :type cl:boolean
    :initform cl:nil)
   (axes_mastered
    :reader axes_mastered
    :initarg :axes_mastered
    :type cl:boolean
    :initform cl:nil)
   (axes_gms_referenced
    :reader axes_gms_referenced
    :initarg :axes_gms_referenced
    :type cl:boolean
    :initform cl:nil)
   (axes_position_referenced
    :reader axes_position_referenced
    :initarg :axes_position_referenced
    :type cl:boolean
    :initform cl:nil)
   (safety_state_enabled
    :reader safety_state_enabled
    :initarg :safety_state_enabled
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass LBRStatus (<LBRStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LBRStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LBRStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kmriiwa_msgs-msg:<LBRStatus> is deprecated: use kmriiwa_msgs-msg:LBRStatus instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <LBRStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kmriiwa_msgs-msg:header-val is deprecated.  Use kmriiwa_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'motion_enabled-val :lambda-list '(m))
(cl:defmethod motion_enabled-val ((m <LBRStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kmriiwa_msgs-msg:motion_enabled-val is deprecated.  Use kmriiwa_msgs-msg:motion_enabled instead.")
  (motion_enabled m))

(cl:ensure-generic-function 'axes_mastered-val :lambda-list '(m))
(cl:defmethod axes_mastered-val ((m <LBRStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kmriiwa_msgs-msg:axes_mastered-val is deprecated.  Use kmriiwa_msgs-msg:axes_mastered instead.")
  (axes_mastered m))

(cl:ensure-generic-function 'axes_gms_referenced-val :lambda-list '(m))
(cl:defmethod axes_gms_referenced-val ((m <LBRStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kmriiwa_msgs-msg:axes_gms_referenced-val is deprecated.  Use kmriiwa_msgs-msg:axes_gms_referenced instead.")
  (axes_gms_referenced m))

(cl:ensure-generic-function 'axes_position_referenced-val :lambda-list '(m))
(cl:defmethod axes_position_referenced-val ((m <LBRStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kmriiwa_msgs-msg:axes_position_referenced-val is deprecated.  Use kmriiwa_msgs-msg:axes_position_referenced instead.")
  (axes_position_referenced m))

(cl:ensure-generic-function 'safety_state_enabled-val :lambda-list '(m))
(cl:defmethod safety_state_enabled-val ((m <LBRStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kmriiwa_msgs-msg:safety_state_enabled-val is deprecated.  Use kmriiwa_msgs-msg:safety_state_enabled instead.")
  (safety_state_enabled m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LBRStatus>) ostream)
  "Serializes a message object of type '<LBRStatus>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'motion_enabled) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'axes_mastered) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'axes_gms_referenced) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'axes_position_referenced) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'safety_state_enabled) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LBRStatus>) istream)
  "Deserializes a message object of type '<LBRStatus>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'motion_enabled) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'axes_mastered) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'axes_gms_referenced) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'axes_position_referenced) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'safety_state_enabled) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LBRStatus>)))
  "Returns string type for a message object of type '<LBRStatus>"
  "kmriiwa_msgs/LBRStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LBRStatus)))
  "Returns string type for a message object of type 'LBRStatus"
  "kmriiwa_msgs/LBRStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LBRStatus>)))
  "Returns md5sum for a message object of type '<LBRStatus>"
  "d1e9bf004da750115463ceb70e37bbcd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LBRStatus)))
  "Returns md5sum for a message object of type 'LBRStatus"
  "d1e9bf004da750115463ceb70e37bbcd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LBRStatus>)))
  "Returns full string definition for message of type '<LBRStatus>"
  (cl:format cl:nil "Header header~%bool motion_enabled~%bool axes_mastered~%bool axes_gms_referenced~%bool axes_position_referenced~%bool safety_state_enabled~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LBRStatus)))
  "Returns full string definition for message of type 'LBRStatus"
  (cl:format cl:nil "Header header~%bool motion_enabled~%bool axes_mastered~%bool axes_gms_referenced~%bool axes_position_referenced~%bool safety_state_enabled~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LBRStatus>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LBRStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'LBRStatus
    (cl:cons ':header (header msg))
    (cl:cons ':motion_enabled (motion_enabled msg))
    (cl:cons ':axes_mastered (axes_mastered msg))
    (cl:cons ':axes_gms_referenced (axes_gms_referenced msg))
    (cl:cons ':axes_position_referenced (axes_position_referenced msg))
    (cl:cons ':safety_state_enabled (safety_state_enabled msg))
))
