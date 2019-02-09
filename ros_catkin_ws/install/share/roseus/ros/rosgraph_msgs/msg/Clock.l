;; Auto-generated. Do not edit!


(when (boundp 'rosgraph_msgs::Clock)
  (if (not (find-package "ROSGRAPH_MSGS"))
    (make-package "ROSGRAPH_MSGS"))
  (shadow 'Clock (find-package "ROSGRAPH_MSGS")))
(unless (find-package "ROSGRAPH_MSGS::CLOCK")
  (make-package "ROSGRAPH_MSGS::CLOCK"))

(in-package "ROS")
;;//! \htmlinclude Clock.msg.html


(defclass rosgraph_msgs::Clock
  :super ros::object
  :slots (_clock ))

(defmethod rosgraph_msgs::Clock
  (:init
   (&key
    ((:clock __clock) (instance ros::time :init))
    )
   (send-super :init)
   (setq _clock __clock)
   self)
  (:clock
   (&optional __clock)
   (if __clock (setq _clock __clock)) _clock)
  (:serialization-length
   ()
   (+
    ;; time _clock
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; time _clock
       (write-long (send _clock :sec) s) (write-long (send _clock :nsec) s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; time _clock
     (send _clock :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _clock :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get rosgraph_msgs::Clock :md5sum-) "a9c97c1d230cfc112e270351a944ee47")
(setf (get rosgraph_msgs::Clock :datatype-) "rosgraph_msgs/Clock")
(setf (get rosgraph_msgs::Clock :definition-)
      "# roslib/Clock is used for publishing simulated time in ROS. 
# This message simply communicates the current time.
# For more information, see http://www.ros.org/wiki/Clock
time clock

")



(provide :rosgraph_msgs/Clock "a9c97c1d230cfc112e270351a944ee47")


