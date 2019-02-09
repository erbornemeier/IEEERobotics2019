;; Auto-generated. Do not edit!


(when (boundp 'std_msgs::Float32)
  (if (not (find-package "STD_MSGS"))
    (make-package "STD_MSGS"))
  (shadow 'Float32 (find-package "STD_MSGS")))
(unless (find-package "STD_MSGS::FLOAT32")
  (make-package "STD_MSGS::FLOAT32"))

(in-package "ROS")
;;//! \htmlinclude Float32.msg.html


(defclass std_msgs::Float32
  :super ros::object
  :slots (_data ))

(defmethod std_msgs::Float32
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
    ;; float32 _data
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _data
       (sys::poke _data (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _data
     (setq _data (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get std_msgs::Float32 :md5sum-) "73fcbf46b49191e672908e50842a83d4")
(setf (get std_msgs::Float32 :datatype-) "std_msgs/Float32")
(setf (get std_msgs::Float32 :definition-)
      "float32 data
")



(provide :std_msgs/Float32 "73fcbf46b49191e672908e50842a83d4")


