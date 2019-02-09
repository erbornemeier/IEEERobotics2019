;; Auto-generated. Do not edit!


(when (boundp 'std_msgs::Int32)
  (if (not (find-package "STD_MSGS"))
    (make-package "STD_MSGS"))
  (shadow 'Int32 (find-package "STD_MSGS")))
(unless (find-package "STD_MSGS::INT32")
  (make-package "STD_MSGS::INT32"))

(in-package "ROS")
;;//! \htmlinclude Int32.msg.html


(defclass std_msgs::Int32
  :super ros::object
  :slots (_data ))

(defmethod std_msgs::Int32
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
    ;; int32 _data
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _data
       (write-long _data s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _data
     (setq _data (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get std_msgs::Int32 :md5sum-) "da5909fbe378aeaf85e547e830cc1bb7")
(setf (get std_msgs::Int32 :datatype-) "std_msgs/Int32")
(setf (get std_msgs::Int32 :definition-)
      "int32 data
")



(provide :std_msgs/Int32 "da5909fbe378aeaf85e547e830cc1bb7")


