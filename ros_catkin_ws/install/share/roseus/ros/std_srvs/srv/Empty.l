;; Auto-generated. Do not edit!


(when (boundp 'std_srvs::Empty)
  (if (not (find-package "STD_SRVS"))
    (make-package "STD_SRVS"))
  (shadow 'Empty (find-package "STD_SRVS")))
(unless (find-package "STD_SRVS::EMPTY")
  (make-package "STD_SRVS::EMPTY"))
(unless (find-package "STD_SRVS::EMPTYREQUEST")
  (make-package "STD_SRVS::EMPTYREQUEST"))
(unless (find-package "STD_SRVS::EMPTYRESPONSE")
  (make-package "STD_SRVS::EMPTYRESPONSE"))

(in-package "ROS")





(defclass std_srvs::EmptyRequest
  :super ros::object
  :slots ())

(defmethod std_srvs::EmptyRequest
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

(defclass std_srvs::EmptyResponse
  :super ros::object
  :slots ())

(defmethod std_srvs::EmptyResponse
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

(defclass std_srvs::Empty
  :super ros::object
  :slots ())

(setf (get std_srvs::Empty :md5sum-) "d41d8cd98f00b204e9800998ecf8427e")
(setf (get std_srvs::Empty :datatype-) "std_srvs/Empty")
(setf (get std_srvs::Empty :request) std_srvs::EmptyRequest)
(setf (get std_srvs::Empty :response) std_srvs::EmptyResponse)

(defmethod std_srvs::EmptyRequest
  (:response () (instance std_srvs::EmptyResponse :init)))

(setf (get std_srvs::EmptyRequest :md5sum-) "d41d8cd98f00b204e9800998ecf8427e")
(setf (get std_srvs::EmptyRequest :datatype-) "std_srvs/EmptyRequest")
(setf (get std_srvs::EmptyRequest :definition-)
      "---
")

(setf (get std_srvs::EmptyResponse :md5sum-) "d41d8cd98f00b204e9800998ecf8427e")
(setf (get std_srvs::EmptyResponse :datatype-) "std_srvs/EmptyResponse")
(setf (get std_srvs::EmptyResponse :definition-)
      "---
")



(provide :std_srvs/Empty "d41d8cd98f00b204e9800998ecf8427e")


