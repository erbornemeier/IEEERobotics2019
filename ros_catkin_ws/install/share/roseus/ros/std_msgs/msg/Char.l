;; Auto-generated. Do not edit!


(when (boundp 'std_msgs::Char)
  (if (not (find-package "STD_MSGS"))
    (make-package "STD_MSGS"))
  (shadow 'Char (find-package "STD_MSGS")))
(unless (find-package "STD_MSGS::CHAR")
  (make-package "STD_MSGS::CHAR"))

(in-package "ROS")
;;//! \htmlinclude Char.msg.html


(defclass std_msgs::Char
  :super ros::object
  :slots (_data ))

(defmethod std_msgs::Char
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
    ;; char _data
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; char _data
       (write-byte _data s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; char _data
     (setq _data (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get std_msgs::Char :md5sum-) "1bf77f25acecdedba0e224b162199717")
(setf (get std_msgs::Char :datatype-) "std_msgs/Char")
(setf (get std_msgs::Char :definition-)
      "char data
")



(provide :std_msgs/Char "1bf77f25acecdedba0e224b162199717")


