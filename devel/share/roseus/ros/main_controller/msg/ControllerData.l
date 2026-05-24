;; Auto-generated. Do not edit!


(when (boundp 'main_controller::ControllerData)
  (if (not (find-package "MAIN_CONTROLLER"))
    (make-package "MAIN_CONTROLLER"))
  (shadow 'ControllerData (find-package "MAIN_CONTROLLER")))
(unless (find-package "MAIN_CONTROLLER::CONTROLLERDATA")
  (make-package "MAIN_CONTROLLER::CONTROLLERDATA"))

(in-package "ROS")
;;//! \htmlinclude ControllerData.msg.html


(defclass main_controller::ControllerData
  :super ros::object
  :slots (_StatusControl _data ))

(defmethod main_controller::ControllerData
  (:init
   (&key
    ((:StatusControl __StatusControl) 0)
    ((:data __data) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _StatusControl (round __StatusControl))
   (setq _data __data)
   self)
  (:StatusControl
   (&optional __StatusControl)
   (if __StatusControl (setq _StatusControl __StatusControl)) _StatusControl)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; int16 _StatusControl
    2
    ;; int16[] _data
    (* 2    (length _data)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16 _StatusControl
       (write-word _StatusControl s)
     ;; int16[] _data
     (write-long (length _data) s)
     (dotimes (i (length _data))
       (write-word (elt _data i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16 _StatusControl
     (setq _StatusControl (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16[] _data
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _data (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _data i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     ))
   ;;
   self)
  )

(setf (get main_controller::ControllerData :md5sum-) "244e1cda6d3e37f787f007f1bc662fea")
(setf (get main_controller::ControllerData :datatype-) "main_controller/ControllerData")
(setf (get main_controller::ControllerData :definition-)
      "int16 StatusControl
int16[] data
")



(provide :main_controller/ControllerData "244e1cda6d3e37f787f007f1bc662fea")


