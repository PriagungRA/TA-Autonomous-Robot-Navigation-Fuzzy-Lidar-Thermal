; Auto-generated. Do not edit!


(cl:in-package tuw_geometry_msgs-msg)


;//! \htmlinclude TwistWithOrientation.msg.html

(cl:defclass <TwistWithOrientation> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (twist
    :reader twist
    :initarg :twist
    :type geometry_msgs-msg:Twist
    :initform (cl:make-instance 'geometry_msgs-msg:Twist))
   (orientation
    :reader orientation
    :initarg :orientation
    :type cl:float
    :initform 0.0))
)

(cl:defclass TwistWithOrientation (<TwistWithOrientation>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TwistWithOrientation>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TwistWithOrientation)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tuw_geometry_msgs-msg:<TwistWithOrientation> is deprecated: use tuw_geometry_msgs-msg:TwistWithOrientation instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <TwistWithOrientation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tuw_geometry_msgs-msg:header-val is deprecated.  Use tuw_geometry_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'twist-val :lambda-list '(m))
(cl:defmethod twist-val ((m <TwistWithOrientation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tuw_geometry_msgs-msg:twist-val is deprecated.  Use tuw_geometry_msgs-msg:twist instead.")
  (twist m))

(cl:ensure-generic-function 'orientation-val :lambda-list '(m))
(cl:defmethod orientation-val ((m <TwistWithOrientation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tuw_geometry_msgs-msg:orientation-val is deprecated.  Use tuw_geometry_msgs-msg:orientation instead.")
  (orientation m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TwistWithOrientation>) ostream)
  "Serializes a message object of type '<TwistWithOrientation>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'twist) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'orientation))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TwistWithOrientation>) istream)
  "Deserializes a message object of type '<TwistWithOrientation>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'twist) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'orientation) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TwistWithOrientation>)))
  "Returns string type for a message object of type '<TwistWithOrientation>"
  "tuw_geometry_msgs/TwistWithOrientation")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TwistWithOrientation)))
  "Returns string type for a message object of type 'TwistWithOrientation"
  "tuw_geometry_msgs/TwistWithOrientation")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TwistWithOrientation>)))
  "Returns md5sum for a message object of type '<TwistWithOrientation>"
  "ad8e7cc3b8974787cc18131989636e17")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TwistWithOrientation)))
  "Returns md5sum for a message object of type 'TwistWithOrientation"
  "ad8e7cc3b8974787cc18131989636e17")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TwistWithOrientation>)))
  "Returns full string definition for message of type '<TwistWithOrientation>"
  (cl:format cl:nil "Header header~%~%geometry_msgs/Twist twist~%float64 orientation~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3 linear~%Vector3 angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TwistWithOrientation)))
  "Returns full string definition for message of type 'TwistWithOrientation"
  (cl:format cl:nil "Header header~%~%geometry_msgs/Twist twist~%float64 orientation~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3 linear~%Vector3 angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TwistWithOrientation>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'twist))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TwistWithOrientation>))
  "Converts a ROS message object to a list"
  (cl:list 'TwistWithOrientation
    (cl:cons ':header (header msg))
    (cl:cons ':twist (twist msg))
    (cl:cons ':orientation (orientation msg))
))
