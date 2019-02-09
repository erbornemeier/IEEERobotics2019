;; Auto-generated. Do not edit!


(when (boundp 'std_msgs::Time)
  (if (not (find-package "STD_MSGS"))
    (make-package "STD_MSGS"))
  (shadow 'Time (find-package "STD_MSGS")))
(unless (find-package "STD_MSGS::TIME")
  (make-package "STD_MSGS::TIME"))

(in-package "ROS")
;;//! \htmlinclude Time.msg.html


(defclass std_msgs::Time
  :super ros::object
  :slots (_data ))

(defmethod std_msgs::Time
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
    ;; time _data
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; time _data
       (write-long (send _data :sec) s) (write-long (send _data :nsec) s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; time _data
     (send _data :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _data :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get std_msgs::Time :md5sum-) "cd7166c74c552c311fbcc2fe5a7bc289")
(setf (get std_msgs::Time :datatype-) "std_msgs/Time")
(setf (get std_msgs::Time :definition-)
      "time data

")



(provide :std_msgs/Time "cd7166c74c552c311fbcc2fe5a7bc289")


