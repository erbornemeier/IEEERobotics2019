;; Auto-generated. Do not edit!


(when (boundp 'std_msgs::Int16)
  (if (not (find-package "STD_MSGS"))
    (make-package "STD_MSGS"))
  (shadow 'Int16 (find-package "STD_MSGS")))
(unless (find-package "STD_MSGS::INT16")
  (make-package "STD_MSGS::INT16"))

(in-package "ROS")
;;//! \htmlinclude Int16.msg.html


(defclass std_msgs::Int16
  :super ros::object
  :slots (_data ))

(defmethod std_msgs::Int16
  (:init
   (&key
    ((:data __data) 0)
    )
   (send-super :init)
   (setq _data (round __data))
   self)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; int16 _data
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16 _data
       (write-word _data s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16 _data
     (setq _data (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get std_msgs::Int16 :md5sum-) "8524586e34fbd7cb1c08c5f5f1ca0e57")
(setf (get std_msgs::Int16 :datatype-) "std_msgs/Int16")
(setf (get std_msgs::Int16 :definition-)
      "int16 data

")



(provide :std_msgs/Int16 "8524586e34fbd7cb1c08c5f5f1ca0e57")


