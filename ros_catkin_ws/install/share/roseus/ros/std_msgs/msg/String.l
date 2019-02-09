;; Auto-generated. Do not edit!


(when (boundp 'std_msgs::String)
  (if (not (find-package "STD_MSGS"))
    (make-package "STD_MSGS"))
  (shadow 'String (find-package "STD_MSGS")))
(unless (find-package "STD_MSGS::STRING")
  (make-package "STD_MSGS::STRING"))

(in-package "ROS")
;;//! \htmlinclude String.msg.html


(defclass std_msgs::String
  :super ros::object
  :slots (_data ))

(defmethod std_msgs::String
  (:init
   (&key
    ((:data __data) "")
    )
   (send-super :init)
   (setq _data (string __data))
   self)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; string _data
    4 (length _data)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _data
       (write-long (length _data) s) (princ _data s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _data
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _data (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get std_msgs::String :md5sum-) "992ce8a1687cec8c8bd883ec73ca41d1")
(setf (get std_msgs::String :datatype-) "std_msgs/String")
(setf (get std_msgs::String :definition-)
      "string data

")



(provide :std_msgs/String "992ce8a1687cec8c8bd883ec73ca41d1")


