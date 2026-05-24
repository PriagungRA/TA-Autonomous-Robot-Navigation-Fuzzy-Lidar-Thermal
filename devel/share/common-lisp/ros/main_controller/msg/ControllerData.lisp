; Auto-generated. Do not edit!


(cl:in-package main_controller-msg)


;//! \htmlinclude ControllerData.msg.html

(cl:defclass <ControllerData> (roslisp-msg-protocol:ros-message)
  ((StatusControl
    :reader StatusControl
    :initarg :StatusControl
    :type cl:fixnum
    :initform 0)
   (data
    :reader data
    :initarg :data
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass ControllerData (<ControllerData>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControllerData>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControllerData)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name main_controller-msg:<ControllerData> is deprecated: use main_controller-msg:ControllerData instead.")))

(cl:ensure-generic-function 'StatusControl-val :lambda-list '(m))
(cl:defmethod StatusControl-val ((m <ControllerData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader main_controller-msg:StatusControl-val is deprecated.  Use main_controller-msg:StatusControl instead.")
  (StatusControl m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <ControllerData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader main_controller-msg:data-val is deprecated.  Use main_controller-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControllerData>) ostream)
  "Serializes a message object of type '<ControllerData>"
  (cl:let* ((signed (cl:slot-value msg 'StatusControl)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    ))
   (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControllerData>) istream)
  "Deserializes a message object of type '<ControllerData>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'StatusControl) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControllerData>)))
  "Returns string type for a message object of type '<ControllerData>"
  "main_controller/ControllerData")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControllerData)))
  "Returns string type for a message object of type 'ControllerData"
  "main_controller/ControllerData")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControllerData>)))
  "Returns md5sum for a message object of type '<ControllerData>"
  "244e1cda6d3e37f787f007f1bc662fea")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControllerData)))
  "Returns md5sum for a message object of type 'ControllerData"
  "244e1cda6d3e37f787f007f1bc662fea")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControllerData>)))
  "Returns full string definition for message of type '<ControllerData>"
  (cl:format cl:nil "int16 StatusControl~%int16[] data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControllerData)))
  "Returns full string definition for message of type 'ControllerData"
  (cl:format cl:nil "int16 StatusControl~%int16[] data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControllerData>))
  (cl:+ 0
     2
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControllerData>))
  "Converts a ROS message object to a list"
  (cl:list 'ControllerData
    (cl:cons ':StatusControl (StatusControl msg))
    (cl:cons ':data (data msg))
))
