; Auto-generated. Do not edit!


(cl:in-package kmriiwa_msgs-msg)


;//! \htmlinclude KMRStatus.msg.html

(cl:defclass <KMRStatus> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (charge_state_percentage
    :reader charge_state_percentage
    :initarg :charge_state_percentage
    :type cl:integer
    :initform 0)
   (motion_enabled
    :reader motion_enabled
    :initarg :motion_enabled
    :type cl:boolean
    :initform cl:nil)
   (warning_field_clear
    :reader warning_field_clear
    :initarg :warning_field_clear
    :type cl:boolean
    :initform cl:nil)
   (safety_field_clear
    :reader safety_field_clear
    :initarg :safety_field_clear
    :type cl:boolean
    :initform cl:nil)
   (safety_state_enabled
    :reader safety_state_enabled
    :initarg :safety_state_enabled
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass KMRStatus (<KMRStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <KMRStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'KMRStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kmriiwa_msgs-msg:<KMRStatus> is deprecated: use kmriiwa_msgs-msg:KMRStatus instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <KMRStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kmriiwa_msgs-msg:header-val is deprecated.  Use kmriiwa_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'charge_state_percentage-val :lambda-list '(m))
(cl:defmethod charge_state_percentage-val ((m <KMRStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kmriiwa_msgs-msg:charge_state_percentage-val is deprecated.  Use kmriiwa_msgs-msg:charge_state_percentage instead.")
  (charge_state_percentage m))

(cl:ensure-generic-function 'motion_enabled-val :lambda-list '(m))
(cl:defmethod motion_enabled-val ((m <KMRStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kmriiwa_msgs-msg:motion_enabled-val is deprecated.  Use kmriiwa_msgs-msg:motion_enabled instead.")
  (motion_enabled m))

(cl:ensure-generic-function 'warning_field_clear-val :lambda-list '(m))
(cl:defmethod warning_field_clear-val ((m <KMRStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kmriiwa_msgs-msg:warning_field_clear-val is deprecated.  Use kmriiwa_msgs-msg:warning_field_clear instead.")
  (warning_field_clear m))

(cl:ensure-generic-function 'safety_field_clear-val :lambda-list '(m))
(cl:defmethod safety_field_clear-val ((m <KMRStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kmriiwa_msgs-msg:safety_field_clear-val is deprecated.  Use kmriiwa_msgs-msg:safety_field_clear instead.")
  (safety_field_clear m))

(cl:ensure-generic-function 'safety_state_enabled-val :lambda-list '(m))
(cl:defmethod safety_state_enabled-val ((m <KMRStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kmriiwa_msgs-msg:safety_state_enabled-val is deprecated.  Use kmriiwa_msgs-msg:safety_state_enabled instead.")
  (safety_state_enabled m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <KMRStatus>) ostream)
  "Serializes a message object of type '<KMRStatus>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'charge_state_percentage)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'motion_enabled) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'warning_field_clear) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'safety_field_clear) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'safety_state_enabled) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <KMRStatus>) istream)
  "Deserializes a message object of type '<KMRStatus>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'charge_state_percentage) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:setf (cl:slot-value msg 'motion_enabled) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'warning_field_clear) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'safety_field_clear) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'safety_state_enabled) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<KMRStatus>)))
  "Returns string type for a message object of type '<KMRStatus>"
  "kmriiwa_msgs/KMRStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'KMRStatus)))
  "Returns string type for a message object of type 'KMRStatus"
  "kmriiwa_msgs/KMRStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<KMRStatus>)))
  "Returns md5sum for a message object of type '<KMRStatus>"
  "4a74515beaa6408bf61b8361cab81069")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'KMRStatus)))
  "Returns md5sum for a message object of type 'KMRStatus"
  "4a74515beaa6408bf61b8361cab81069")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<KMRStatus>)))
  "Returns full string definition for message of type '<KMRStatus>"
  (cl:format cl:nil "Header header~%int32 charge_state_percentage~%bool motion_enabled~%bool warning_field_clear~%bool safety_field_clear~%bool safety_state_enabled~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'KMRStatus)))
  "Returns full string definition for message of type 'KMRStatus"
  (cl:format cl:nil "Header header~%int32 charge_state_percentage~%bool motion_enabled~%bool warning_field_clear~%bool safety_field_clear~%bool safety_state_enabled~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <KMRStatus>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <KMRStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'KMRStatus
    (cl:cons ':header (header msg))
    (cl:cons ':charge_state_percentage (charge_state_percentage msg))
    (cl:cons ':motion_enabled (motion_enabled msg))
    (cl:cons ':warning_field_clear (warning_field_clear msg))
    (cl:cons ':safety_field_clear (safety_field_clear msg))
    (cl:cons ':safety_state_enabled (safety_state_enabled msg))
))
