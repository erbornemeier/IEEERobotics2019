;; Auto-generated. Do not edit!


(when (boundp 'std_msgs::Float64)
  (if (not (find-package "STD_MSGS"))
    (make-package "STD_MSGS"))
  (shadow 'Float64 (find-package "STD_MSGS")))
(unless (find-package "STD_MSGS::FLOAT64")
  (make-package "STD_MSGS::FLOAT64"))

(in-package "ROS")
;;//! \htmlinclude Float64.msg.html


(defclass std_msgs::Float64
  :super ros::object
  :slots (_data ))

(defmethod std_msgs::Float64
  (:init
   (&key
    ((:data __data) 0.0)
    )
   (send-super :init)
   (setq _data (float __data))
   self)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; float64 _data
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _data
       (sys::poke _data (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _data
     (setq _data (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get std_msgs::Float64 :md5sum-) "fdb28210bfa9d7c91146260178d9a584")
(setf (get std_msgs::Float64 :datatype-) "std_msgs/Float64")
(setf (get std_msgs::Float64 :definition-)
      "float64 data
")



(provide :std_msgs/Float64 "fdb28210bfa9d7c91146260178d9a584")


