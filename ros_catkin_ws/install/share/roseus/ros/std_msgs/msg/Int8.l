;; Auto-generated. Do not edit!


(when (boundp 'std_msgs::Int8)
  (if (not (find-package "STD_MSGS"))
    (make-package "STD_MSGS"))
  (shadow 'Int8 (find-package "STD_MSGS")))
(unless (find-package "STD_MSGS::INT8")
  (make-package "STD_MSGS::INT8"))

(in-package "ROS")
;;//! \htmlinclude Int8.msg.html


(defclass std_msgs::Int8
  :super ros::object
  :slots (_data ))

(defmethod std_msgs::Int8
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
    ;; int8 _data
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _data
       (write-byte _data s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _data
     (setq _data (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _data 127) (setq _data (- _data 256)))
   ;;
   self)
  )

(setf (get std_msgs::Int8 :md5sum-) "27ffa0c9c4b8fb8492252bcad9e5c57b")
(setf (get std_msgs::Int8 :datatype-) "std_msgs/Int8")
(setf (get std_msgs::Int8 :definition-)
      "int8 data

")



(provide :std_msgs/Int8 "27ffa0c9c4b8fb8492252bcad9e5c57b")


