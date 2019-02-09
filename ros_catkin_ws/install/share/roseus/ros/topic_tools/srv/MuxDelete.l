;; Auto-generated. Do not edit!


(when (boundp 'topic_tools::MuxDelete)
  (if (not (find-package "TOPIC_TOOLS"))
    (make-package "TOPIC_TOOLS"))
  (shadow 'MuxDelete (find-package "TOPIC_TOOLS")))
(unless (find-package "TOPIC_TOOLS::MUXDELETE")
  (make-package "TOPIC_TOOLS::MUXDELETE"))
(unless (find-package "TOPIC_TOOLS::MUXDELETEREQUEST")
  (make-package "TOPIC_TOOLS::MUXDELETEREQUEST"))
(unless (find-package "TOPIC_TOOLS::MUXDELETERESPONSE")
  (make-package "TOPIC_TOOLS::MUXDELETERESPONSE"))

(in-package "ROS")





(defclass topic_tools::MuxDeleteRequest
  :super ros::object
  :slots (_topic ))

(defmethod topic_tools::MuxDeleteRequest
  (:init
   (&key
    ((:topic __topic) "")
    )
   (send-super :init)
   (setq _topic (string __topic))
   self)
  (:topic
   (&optional __topic)
   (if __topic (setq _topic __topic)) _topic)
  (:serialization-length
   ()
   (+
    ;; string _topic
    4 (length _topic)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _topic
       (write-long (length _topic) s) (princ _topic s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _topic
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _topic (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass topic_tools::MuxDeleteResponse
  :super ros::object
  :slots ())

(defmethod topic_tools::MuxDeleteResponse
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

(defclass topic_tools::MuxDelete
  :super ros::object
  :slots ())

(setf (get topic_tools::MuxDelete :md5sum-) "d8f94bae31b356b24d0427f80426d0c3")
(setf (get topic_tools::MuxDelete :datatype-) "topic_tools/MuxDelete")
(setf (get topic_tools::MuxDelete :request) topic_tools::MuxDeleteRequest)
(setf (get topic_tools::MuxDelete :response) topic_tools::MuxDeleteResponse)

(defmethod topic_tools::MuxDeleteRequest
  (:response () (instance topic_tools::MuxDeleteResponse :init)))

(setf (get topic_tools::MuxDeleteRequest :md5sum-) "d8f94bae31b356b24d0427f80426d0c3")
(setf (get topic_tools::MuxDeleteRequest :datatype-) "topic_tools/MuxDeleteRequest")
(setf (get topic_tools::MuxDeleteRequest :definition-)
      "string topic
---


")

(setf (get topic_tools::MuxDeleteResponse :md5sum-) "d8f94bae31b356b24d0427f80426d0c3")
(setf (get topic_tools::MuxDeleteResponse :datatype-) "topic_tools/MuxDeleteResponse")
(setf (get topic_tools::MuxDeleteResponse :definition-)
      "string topic
---


")



(provide :topic_tools/MuxDelete "d8f94bae31b356b24d0427f80426d0c3")


