;; Auto-generated. Do not edit!


(when (boundp 'std_msgs::Byte)
  (if (not (find-package "STD_MSGS"))
    (make-package "STD_MSGS"))
  (shadow 'Byte (find-package "STD_MSGS")))
(unless (find-package "STD_MSGS::BYTE")
  (make-package "STD_MSGS::BYTE"))

(in-package "ROS")
;;//! \htmlinclude Byte.msg.html


(defclass std_msgs::Byte
  :super ros::object
  :slots (_data ))

(defmethod std_msgs::Byte
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
    ;; byte _data
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; byte _data
       (write-byte _data s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; byte _data
     (setq _data (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _data 127) (setq _data (- _data 256)))
   ;;
   self)
  )

(setf (get std_msgs::Byte :md5sum-) "ad736a2e8818154c487bb80fe42ce43b")
(setf (get std_msgs::Byte :datatype-) "std_msgs/Byte")
(setf (get std_msgs::Byte :definition-)
      "byte data

")



(provide :std_msgs/Byte "ad736a2e8818154c487bb80fe42ce43b")


