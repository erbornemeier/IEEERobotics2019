;; Auto-generated. Do not edit!


(when (boundp 'std_msgs::Bool)
  (if (not (find-package "STD_MSGS"))
    (make-package "STD_MSGS"))
  (shadow 'Bool (find-package "STD_MSGS")))
(unless (find-package "STD_MSGS::BOOL")
  (make-package "STD_MSGS::BOOL"))

(in-package "ROS")
;;//! \htmlinclude Bool.msg.html


(defclass std_msgs::Bool
  :super ros::object
  :slots (_data ))

(defmethod std_msgs::Bool
  (:init
   (&key
    ((:data __data) nil)
    )
   (send-super :init)
   (setq _data __data)
   self)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; bool _data
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _data
       (if _data (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _data
     (setq _data (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get std_msgs::Bool :md5sum-) "8b94c1b53db61fb6aed406028ad6332a")
(setf (get std_msgs::Bool :datatype-) "std_msgs/Bool")
(setf (get std_msgs::Bool :definition-)
      "bool data
")



(provide :std_msgs/Bool "8b94c1b53db61fb6aed406028ad6332a")


