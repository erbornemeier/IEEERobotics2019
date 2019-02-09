;; Auto-generated. Do not edit!


(when (boundp 'std_msgs::UInt8)
  (if (not (find-package "STD_MSGS"))
    (make-package "STD_MSGS"))
  (shadow 'UInt8 (find-package "STD_MSGS")))
(unless (find-package "STD_MSGS::UINT8")
  (make-package "STD_MSGS::UINT8"))

(in-package "ROS")
;;//! \htmlinclude UInt8.msg.html


(defclass std_msgs::UInt8
  :super ros::object
  :slots (_data ))

(defmethod std_msgs::UInt8
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
    ;; uint8 _data
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _data
       (write-byte _data s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _data
     (setq _data (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get std_msgs::UInt8 :md5sum-) "7c8164229e7d2c17eb95e9231617fdee")
(setf (get std_msgs::UInt8 :datatype-) "std_msgs/UInt8")
(setf (get std_msgs::UInt8 :definition-)
      "uint8 data

")



(provide :std_msgs/UInt8 "7c8164229e7d2c17eb95e9231617fdee")


