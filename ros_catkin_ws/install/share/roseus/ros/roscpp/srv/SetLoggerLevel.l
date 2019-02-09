;; Auto-generated. Do not edit!


(when (boundp 'roscpp::SetLoggerLevel)
  (if (not (find-package "ROSCPP"))
    (make-package "ROSCPP"))
  (shadow 'SetLoggerLevel (find-package "ROSCPP")))
(unless (find-package "ROSCPP::SETLOGGERLEVEL")
  (make-package "ROSCPP::SETLOGGERLEVEL"))
(unless (find-package "ROSCPP::SETLOGGERLEVELREQUEST")
  (make-package "ROSCPP::SETLOGGERLEVELREQUEST"))
(unless (find-package "ROSCPP::SETLOGGERLEVELRESPONSE")
  (make-package "ROSCPP::SETLOGGERLEVELRESPONSE"))

(in-package "ROS")





(defclass roscpp::SetLoggerLevelRequest
  :super ros::object
  :slots (_logger _level ))

(defmethod roscpp::SetLoggerLevelRequest
  (:init
   (&key
    ((:logger __logger) "")
    ((:level __level) "")
    )
   (send-super :init)
   (setq _logger (string __logger))
   (setq _level (string __level))
   self)
  (:logger
   (&optional __logger)
   (if __logger (setq _logger __logger)) _logger)
  (:level
   (&optional __level)
   (if __level (setq _level __level)) _level)
  (:serialization-length
   ()
   (+
    ;; string _logger
    4 (length _logger)
    ;; string _level
    4 (length _level)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _logger
       (write-long (length _logger) s) (princ _logger s)
     ;; string _level
       (write-long (length _level) s) (princ _level s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _logger
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _logger (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _level
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _level (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass roscpp::SetLoggerLevelResponse
  :super ros::object
  :slots ())

(defmethod roscpp::SetLoggerLevelResponse
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

(defclass roscpp::SetLoggerLevel
  :super ros::object
  :slots ())

(setf (get roscpp::SetLoggerLevel :md5sum-) "51da076440d78ca1684d36c868df61ea")
(setf (get roscpp::SetLoggerLevel :datatype-) "roscpp/SetLoggerLevel")
(setf (get roscpp::SetLoggerLevel :request) roscpp::SetLoggerLevelRequest)
(setf (get roscpp::SetLoggerLevel :response) roscpp::SetLoggerLevelResponse)

(defmethod roscpp::SetLoggerLevelRequest
  (:response () (instance roscpp::SetLoggerLevelResponse :init)))

(setf (get roscpp::SetLoggerLevelRequest :md5sum-) "51da076440d78ca1684d36c868df61ea")
(setf (get roscpp::SetLoggerLevelRequest :datatype-) "roscpp/SetLoggerLevelRequest")
(setf (get roscpp::SetLoggerLevelRequest :definition-)
      "string logger
string level
---
")

(setf (get roscpp::SetLoggerLevelResponse :md5sum-) "51da076440d78ca1684d36c868df61ea")
(setf (get roscpp::SetLoggerLevelResponse :datatype-) "roscpp/SetLoggerLevelResponse")
(setf (get roscpp::SetLoggerLevelResponse :definition-)
      "string logger
string level
---
")



(provide :roscpp/SetLoggerLevel "51da076440d78ca1684d36c868df61ea")


