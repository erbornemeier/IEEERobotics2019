;; Auto-generated. Do not edit!


(when (boundp 'std_srvs::Trigger)
  (if (not (find-package "STD_SRVS"))
    (make-package "STD_SRVS"))
  (shadow 'Trigger (find-package "STD_SRVS")))
(unless (find-package "STD_SRVS::TRIGGER")
  (make-package "STD_SRVS::TRIGGER"))
(unless (find-package "STD_SRVS::TRIGGERREQUEST")
  (make-package "STD_SRVS::TRIGGERREQUEST"))
(unless (find-package "STD_SRVS::TRIGGERRESPONSE")
  (make-package "STD_SRVS::TRIGGERRESPONSE"))

(in-package "ROS")





(defclass std_srvs::TriggerRequest
  :super ros::object
  :slots ())

(defmethod std_srvs::TriggerRequest
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

(defclass std_srvs::TriggerResponse
  :super ros::object
  :slots (_success _message ))

(defmethod std_srvs::TriggerResponse
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

(defclass std_srvs::Trigger
  :super ros::object
  :slots ())

(setf (get std_srvs::Trigger :md5sum-) "937c9679a518e3a18d831e57125ea522")
(setf (get std_srvs::Trigger :datatype-) "std_srvs/Trigger")
(setf (get std_srvs::Trigger :request) std_srvs::TriggerRequest)
(setf (get std_srvs::Trigger :response) std_srvs::TriggerResponse)

(defmethod std_srvs::TriggerRequest
  (:response () (instance std_srvs::TriggerResponse :init)))

(setf (get std_srvs::TriggerRequest :md5sum-) "937c9679a518e3a18d831e57125ea522")
(setf (get std_srvs::TriggerRequest :datatype-) "std_srvs/TriggerRequest")
(setf (get std_srvs::TriggerRequest :definition-)
      "---
bool success
string message

")

(setf (get std_srvs::TriggerResponse :md5sum-) "937c9679a518e3a18d831e57125ea522")
(setf (get std_srvs::TriggerResponse :datatype-) "std_srvs/TriggerResponse")
(setf (get std_srvs::TriggerResponse :definition-)
      "---
bool success
string message

")



(provide :std_srvs/Trigger "937c9679a518e3a18d831e57125ea522")


