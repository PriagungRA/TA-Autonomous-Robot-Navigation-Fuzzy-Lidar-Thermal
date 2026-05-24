;; Auto-generated. Do not edit!


(when (boundp 'tuw_geometry_msgs::TwistWithOrientation)
  (if (not (find-package "TUW_GEOMETRY_MSGS"))
    (make-package "TUW_GEOMETRY_MSGS"))
  (shadow 'TwistWithOrientation (find-package "TUW_GEOMETRY_MSGS")))
(unless (find-package "TUW_GEOMETRY_MSGS::TWISTWITHORIENTATION")
  (make-package "TUW_GEOMETRY_MSGS::TWISTWITHORIENTATION"))

(in-package "ROS")
;;//! \htmlinclude TwistWithOrientation.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass tuw_geometry_msgs::TwistWithOrientation
  :super ros::object
  :slots (_header _twist _orientation ))

(defmethod tuw_geometry_msgs::TwistWithOrientation
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:twist __twist) (instance geometry_msgs::Twist :init))
    ((:orientation __orientation) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _twist __twist)
   (setq _orientation (float __orientation))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:twist
   (&rest __twist)
   (if (keywordp (car __twist))
       (send* _twist __twist)
     (progn
       (if __twist (setq _twist (car __twist)))
       _twist)))
  (:orientation
   (&optional __orientation)
   (if __orientation (setq _orientation __orientation)) _orientation)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Twist _twist
    (send _twist :serialization-length)
    ;; float64 _orientation
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Twist _twist
       (send _twist :serialize s)
     ;; float64 _orientation
       (sys::poke _orientation (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Twist _twist
     (send _twist :deserialize buf ptr-) (incf ptr- (send _twist :serialization-length))
   ;; float64 _orientation
     (setq _orientation (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get tuw_geometry_msgs::TwistWithOrientation :md5sum-) "ad8e7cc3b8974787cc18131989636e17")
(setf (get tuw_geometry_msgs::TwistWithOrientation :datatype-) "tuw_geometry_msgs/TwistWithOrientation")
(setf (get tuw_geometry_msgs::TwistWithOrientation :definition-)
      "Header header

geometry_msgs/Twist twist
float64 orientation

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
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3 linear
Vector3 angular

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
")



(provide :tuw_geometry_msgs/TwistWithOrientation "ad8e7cc3b8974787cc18131989636e17")


