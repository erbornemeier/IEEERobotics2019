;; Auto-generated. Do not edit!


(when (boundp 'std_msgs::UInt32)
  (if (not (find-package "STD_MSGS"))
    (make-package "STD_MSGS"))
  (shadow 'UInt32 (find-package "STD_MSGS")))
(unless (find-package "STD_MSGS::UINT32")
  (make-package "STD_MSGS::UINT32"))

(in-package "ROS")
;;//! \htmlinclude UInt32.msg.html


(defclass std_msgs::UInt32
  :super ros::object
  :slots (_data ))

(defmethod std_msgs::UInt32
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
    ;; uint32 _data
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _data
       (write-long _data s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _data
     (setq _data (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get std_msgs::UInt32 :md5sum-) "304a39449588c7f8ce2df6e8001c5fce")
(setf (get std_msgs::UInt32 :datatype-) "std_msgs/UInt32")
(setf (get std_msgs::UInt32 :definition-)
      "uint32 data
")



(provide :std_msgs/UInt32 "304a39449588c7f8ce2df6e8001c5fce")


