;; Auto-generated. Do not edit!


(when (boundp 'std_msgs::Empty)
  (if (not (find-package "STD_MSGS"))
    (make-package "STD_MSGS"))
  (shadow 'Empty (find-package "STD_MSGS")))
(unless (find-package "STD_MSGS::EMPTY")
  (make-package "STD_MSGS::EMPTY"))

(in-package "ROS")
;;//! \htmlinclude Empty.msg.html


(defclass std_msgs::Empty
  :super ros::object
  :slots ())

(defmethod std_msgs::Empty
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(setf (get std_msgs::Empty :md5sum-) "d41d8cd98f00b204e9800998ecf8427e")
(setf (get std_msgs::Empty :datatype-) "std_msgs/Empty")
(setf (get std_msgs::Empty :definition-)
      "
")



(provide :std_msgs/Empty "d41d8cd98f00b204e9800998ecf8427e")


