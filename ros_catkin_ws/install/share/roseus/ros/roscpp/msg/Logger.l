;; Auto-generated. Do not edit!


(when (boundp 'roscpp::Logger)
  (if (not (find-package "ROSCPP"))
    (make-package "ROSCPP"))
  (shadow 'Logger (find-package "ROSCPP")))
(unless (find-package "ROSCPP::LOGGER")
  (make-package "ROSCPP::LOGGER"))

(in-package "ROS")
;;//! \htmlinclude Logger.msg.html


(defclass roscpp::Logger
  :super ros::object
  :slots (_name _level ))

(defmethod roscpp::Logger
  (:init
   (&key
    ((:name __name) "")
    ((:level __level) "")
    )
   (send-super :init)
   (setq _name (string __name))
   (setq _level (string __level))
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:level
   (&optional __level)
   (if __level (setq _level __level)) _level)
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ;; string _level
    4 (length _level)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; string _level
       (write-long (length _level) s) (princ _level s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _level
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _level (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get roscpp::Logger :md5sum-) "a6069a2ff40db7bd32143dd66e1f408e")
(setf (get roscpp::Logger :datatype-) "roscpp/Logger")
(setf (get roscpp::Logger :definition-)
      "string name
string level

")



(provide :roscpp/Logger "a6069a2ff40db7bd32143dd66e1f408e")


