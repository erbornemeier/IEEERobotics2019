;; Auto-generated. Do not edit!


(when (boundp 'roscpp::GetLoggers)
  (if (not (find-package "ROSCPP"))
    (make-package "ROSCPP"))
  (shadow 'GetLoggers (find-package "ROSCPP")))
(unless (find-package "ROSCPP::GETLOGGERS")
  (make-package "ROSCPP::GETLOGGERS"))
(unless (find-package "ROSCPP::GETLOGGERSREQUEST")
  (make-package "ROSCPP::GETLOGGERSREQUEST"))
(unless (find-package "ROSCPP::GETLOGGERSRESPONSE")
  (make-package "ROSCPP::GETLOGGERSRESPONSE"))

(in-package "ROS")





(defclass roscpp::GetLoggersRequest
  :super ros::object
  :slots ())

(defmethod roscpp::GetLoggersRequest
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

(defclass roscpp::GetLoggersResponse
  :super ros::object
  :slots (_loggers ))

(defmethod roscpp::GetLoggersResponse
  (:init
   (&key
    ((:loggers __loggers) (let (r) (dotimes (i 0) (push (instance roscpp::Logger :init) r)) r))
    )
   (send-super :init)
   (setq _loggers __loggers)
   self)
  (:loggers
   (&rest __loggers)
   (if (keywordp (car __loggers))
       (send* _loggers __loggers)
     (progn
       (if __loggers (setq _loggers (car __loggers)))
       _loggers)))
  (:serialization-length
   ()
   (+
    ;; roscpp/Logger[] _loggers
    (apply #'+ (send-all _loggers :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; roscpp/Logger[] _loggers
     (write-long (length _loggers) s)
     (dolist (elem _loggers)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; roscpp/Logger[] _loggers
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _loggers (let (r) (dotimes (i n) (push (instance roscpp::Logger :init) r)) r))
     (dolist (elem- _loggers)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(defclass roscpp::GetLoggers
  :super ros::object
  :slots ())

(setf (get roscpp::GetLoggers :md5sum-) "32e97e85527d4678a8f9279894bb64b0")
(setf (get roscpp::GetLoggers :datatype-) "roscpp/GetLoggers")
(setf (get roscpp::GetLoggers :request) roscpp::GetLoggersRequest)
(setf (get roscpp::GetLoggers :response) roscpp::GetLoggersResponse)

(defmethod roscpp::GetLoggersRequest
  (:response () (instance roscpp::GetLoggersResponse :init)))

(setf (get roscpp::GetLoggersRequest :md5sum-) "32e97e85527d4678a8f9279894bb64b0")
(setf (get roscpp::GetLoggersRequest :datatype-) "roscpp/GetLoggersRequest")
(setf (get roscpp::GetLoggersRequest :definition-)
      "---
Logger[] loggers

================================================================================
MSG: roscpp/Logger
string name
string level
")

(setf (get roscpp::GetLoggersResponse :md5sum-) "32e97e85527d4678a8f9279894bb64b0")
(setf (get roscpp::GetLoggersResponse :datatype-) "roscpp/GetLoggersResponse")
(setf (get roscpp::GetLoggersResponse :definition-)
      "---
Logger[] loggers

================================================================================
MSG: roscpp/Logger
string name
string level
")



(provide :roscpp/GetLoggers "32e97e85527d4678a8f9279894bb64b0")


