;; Auto-generated. Do not edit!


(when (boundp 'std_srvs::SetBool)
  (if (not (find-package "STD_SRVS"))
    (make-package "STD_SRVS"))
  (shadow 'SetBool (find-package "STD_SRVS")))
(unless (find-package "STD_SRVS::SETBOOL")
  (make-package "STD_SRVS::SETBOOL"))
(unless (find-package "STD_SRVS::SETBOOLREQUEST")
  (make-package "STD_SRVS::SETBOOLREQUEST"))
(unless (find-package "STD_SRVS::SETBOOLRESPONSE")
  (make-package "STD_SRVS::SETBOOLRESPONSE"))

(in-package "ROS")





(defclass std_srvs::SetBoolRequest
  :super ros::object
  :slots (_data ))

(defmethod std_srvs::SetBoolRequest
  (:init
   (&key
    ((:data __data) nil)
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
    ;; bool _data
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _data
       (if _data (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _data
     (setq _data (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass std_srvs::SetBoolResponse
  :super ros::object
  :slots (_success _message ))

(defmethod std_srvs::SetBoolResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:message __message) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _message (string __message))
   self)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass std_srvs::SetBool
  :super ros::object
  :slots ())

(setf (get std_srvs::SetBool :md5sum-) "09fb03525b03e7ea1fd3992bafd87e16")
(setf (get std_srvs::SetBool :datatype-) "std_srvs/SetBool")
(setf (get std_srvs::SetBool :request) std_srvs::SetBoolRequest)
(setf (get std_srvs::SetBool :response) std_srvs::SetBoolResponse)

(defmethod std_srvs::SetBoolRequest
  (:response () (instance std_srvs::SetBoolResponse :init)))

(setf (get std_srvs::SetBoolRequest :md5sum-) "09fb03525b03e7ea1fd3992bafd87e16")
(setf (get std_srvs::SetBoolRequest :datatype-) "std_srvs/SetBoolRequest")
(setf (get std_srvs::SetBoolRequest :definition-)
      "bool data
---
bool success
string message

")

(setf (get std_srvs::SetBoolResponse :md5sum-) "09fb03525b03e7ea1fd3992bafd87e16")
(setf (get std_srvs::SetBoolResponse :datatype-) "std_srvs/SetBoolResponse")
(setf (get std_srvs::SetBoolResponse :definition-)
      "bool data
---
bool success
string message

")



(provide :std_srvs/SetBool "09fb03525b03e7ea1fd3992bafd87e16")


