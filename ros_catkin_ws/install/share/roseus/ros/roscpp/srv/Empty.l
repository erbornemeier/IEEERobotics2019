;; Auto-generated. Do not edit!


(when (boundp 'roscpp::Empty)
  (if (not (find-package "ROSCPP"))
    (make-package "ROSCPP"))
  (shadow 'Empty (find-package "ROSCPP")))
(unless (find-package "ROSCPP::EMPTY")
  (make-package "ROSCPP::EMPTY"))
(unless (find-package "ROSCPP::EMPTYREQUEST")
  (make-package "ROSCPP::EMPTYREQUEST"))
(unless (find-package "ROSCPP::EMPTYRESPONSE")
  (make-package "ROSCPP::EMPTYRESPONSE"))

(in-package "ROS")





(defclass roscpp::EmptyRequest
  :super ros::object
  :slots ())

(defmethod roscpp::EmptyRequest
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

(defclass roscpp::EmptyResponse
  :super ros::object
  :slots ())

(defmethod roscpp::EmptyResponse
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

(defclass roscpp::Empty
  :super ros::object
  :slots ())

(setf (get roscpp::Empty :md5sum-) "d41d8cd98f00b204e9800998ecf8427e")
(setf (get roscpp::Empty :datatype-) "roscpp/Empty")
(setf (get roscpp::Empty :request) roscpp::EmptyRequest)
(setf (get roscpp::Empty :response) roscpp::EmptyResponse)

(defmethod roscpp::EmptyRequest
  (:response () (instance roscpp::EmptyResponse :init)))

(setf (get roscpp::EmptyRequest :md5sum-) "d41d8cd98f00b204e9800998ecf8427e")
(setf (get roscpp::EmptyRequest :datatype-) "roscpp/EmptyRequest")
(setf (get roscpp::EmptyRequest :definition-)
      "---
")

(setf (get roscpp::EmptyResponse :md5sum-) "d41d8cd98f00b204e9800998ecf8427e")
(setf (get roscpp::EmptyResponse :datatype-) "roscpp/EmptyResponse")
(setf (get roscpp::EmptyResponse :definition-)
      "---
")



(provide :roscpp/Empty "d41d8cd98f00b204e9800998ecf8427e")


