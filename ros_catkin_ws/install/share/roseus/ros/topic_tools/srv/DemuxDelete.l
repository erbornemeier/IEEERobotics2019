;; Auto-generated. Do not edit!


(when (boundp 'topic_tools::DemuxDelete)
  (if (not (find-package "TOPIC_TOOLS"))
    (make-package "TOPIC_TOOLS"))
  (shadow 'DemuxDelete (find-package "TOPIC_TOOLS")))
(unless (find-package "TOPIC_TOOLS::DEMUXDELETE")
  (make-package "TOPIC_TOOLS::DEMUXDELETE"))
(unless (find-package "TOPIC_TOOLS::DEMUXDELETEREQUEST")
  (make-package "TOPIC_TOOLS::DEMUXDELETEREQUEST"))
(unless (find-package "TOPIC_TOOLS::DEMUXDELETERESPONSE")
  (make-package "TOPIC_TOOLS::DEMUXDELETERESPONSE"))

(in-package "ROS")





(defclass topic_tools::DemuxDeleteRequest
  :super ros::object
  :slots (_topic ))

(defmethod topic_tools::DemuxDeleteRequest
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

(defclass topic_tools::DemuxDeleteResponse
  :super ros::object
  :slots ())

(defmethod topic_tools::DemuxDeleteResponse
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

(defclass topic_tools::DemuxDelete
  :super ros::object
  :slots ())

(setf (get topic_tools::DemuxDelete :md5sum-) "d8f94bae31b356b24d0427f80426d0c3")
(setf (get topic_tools::DemuxDelete :datatype-) "topic_tools/DemuxDelete")
(setf (get topic_tools::DemuxDelete :request) topic_tools::DemuxDeleteRequest)
(setf (get topic_tools::DemuxDelete :response) topic_tools::DemuxDeleteResponse)

(defmethod topic_tools::DemuxDeleteRequest
  (:response () (instance topic_tools::DemuxDeleteResponse :init)))

(setf (get topic_tools::DemuxDeleteRequest :md5sum-) "d8f94bae31b356b24d0427f80426d0c3")
(setf (get topic_tools::DemuxDeleteRequest :datatype-) "topic_tools/DemuxDeleteRequest")
(setf (get topic_tools::DemuxDeleteRequest :definition-)
      "string topic
---


")

(setf (get topic_tools::DemuxDeleteResponse :md5sum-) "d8f94bae31b356b24d0427f80426d0c3")
(setf (get topic_tools::DemuxDeleteResponse :datatype-) "topic_tools/DemuxDeleteResponse")
(setf (get topic_tools::DemuxDeleteResponse :definition-)
      "string topic
---


")



(provide :topic_tools/DemuxDelete "d8f94bae31b356b24d0427f80426d0c3")


