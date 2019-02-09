;; Auto-generated. Do not edit!


(when (boundp 'std_msgs::UInt16)
  (if (not (find-package "STD_MSGS"))
    (make-package "STD_MSGS"))
  (shadow 'UInt16 (find-package "STD_MSGS")))
(unless (find-package "STD_MSGS::UINT16")
  (make-package "STD_MSGS::UINT16"))

(in-package "ROS")
;;//! \htmlinclude UInt16.msg.html


(defclass std_msgs::UInt16
  :super ros::object
  :slots (_data ))

(defmethod std_msgs::UInt16
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
    ;; uint16 _data
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint16 _data
       (write-word _data s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint16 _data
     (setq _data (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get std_msgs::UInt16 :md5sum-) "1df79edf208b629fe6b81923a544552d")
(setf (get std_msgs::UInt16 :datatype-) "std_msgs/UInt16")
(setf (get std_msgs::UInt16 :definition-)
      "uint16 data

")



(provide :std_msgs/UInt16 "1df79edf208b629fe6b81923a544552d")


