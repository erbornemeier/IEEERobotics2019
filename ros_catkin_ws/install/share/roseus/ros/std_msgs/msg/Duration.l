;; Auto-generated. Do not edit!


(when (boundp 'std_msgs::Duration)
  (if (not (find-package "STD_MSGS"))
    (make-package "STD_MSGS"))
  (shadow 'Duration (find-package "STD_MSGS")))
(unless (find-package "STD_MSGS::DURATION")
  (make-package "STD_MSGS::DURATION"))

(in-package "ROS")
;;//! \htmlinclude Duration.msg.html


(defclass std_msgs::Duration
  :super ros::object
  :slots (_data ))

(defmethod std_msgs::Duration
  (:init
   (&key
    ((:data __data) (instance ros::time :init))
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
    ;; duration _data
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; duration _data
       (write-long (send _data :sec) s) (write-long (send _data :nsec) s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; duration _data
     (send _data :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _data :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get std_msgs::Duration :md5sum-) "3e286caf4241d664e55f3ad380e2ae46")
(setf (get std_msgs::Duration :datatype-) "std_msgs/Duration")
(setf (get std_msgs::Duration :definition-)
      "duration data

")



(provide :std_msgs/Duration "3e286caf4241d664e55f3ad380e2ae46")


