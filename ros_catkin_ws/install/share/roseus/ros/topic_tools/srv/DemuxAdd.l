;; Auto-generated. Do not edit!


(when (boundp 'topic_tools::DemuxAdd)
  (if (not (find-package "TOPIC_TOOLS"))
    (make-package "TOPIC_TOOLS"))
  (shadow 'DemuxAdd (find-package "TOPIC_TOOLS")))
(unless (find-package "TOPIC_TOOLS::DEMUXADD")
  (make-package "TOPIC_TOOLS::DEMUXADD"))
(unless (find-package "TOPIC_TOOLS::DEMUXADDREQUEST")
  (make-package "TOPIC_TOOLS::DEMUXADDREQUEST"))
(unless (find-package "TOPIC_TOOLS::DEMUXADDRESPONSE")
  (make-package "TOPIC_TOOLS::DEMUXADDRESPONSE"))

(in-package "ROS")





(defclass topic_tools::DemuxAddRequest
  :super ros::object
  :slots (_topic ))

(defmethod topic_tools::DemuxAddRequest
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

(defclass topic_tools::DemuxAddResponse
  :super ros::object
  :slots ())

(defmethod topic_tools::DemuxAddResponse
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

(defclass topic_tools::DemuxAdd
  :super ros::object
  :slots ())

(setf (get topic_tools::DemuxAdd :md5sum-) "d8f94bae31b356b24d0427f80426d0c3")
(setf (get topic_tools::DemuxAdd :datatype-) "topic_tools/DemuxAdd")
(setf (get topic_tools::DemuxAdd :request) topic_tools::DemuxAddRequest)
(setf (get topic_tools::DemuxAdd :response) topic_tools::DemuxAddResponse)

(defmethod topic_tools::DemuxAddRequest
  (:response () (instance topic_tools::DemuxAddResponse :init)))

(setf (get topic_tools::DemuxAddRequest :md5sum-) "d8f94bae31b356b24d0427f80426d0c3")
(setf (get topic_tools::DemuxAddRequest :datatype-) "topic_tools/DemuxAddRequest")
(setf (get topic_tools::DemuxAddRequest :definition-)
      "string topic
---

")

(setf (get topic_tools::DemuxAddResponse :md5sum-) "d8f94bae31b356b24d0427f80426d0c3")
(setf (get topic_tools::DemuxAddResponse :datatype-) "topic_tools/DemuxAddResponse")
(setf (get topic_tools::DemuxAddResponse :definition-)
      "string topic
---

")



(provide :topic_tools/DemuxAdd "d8f94bae31b356b24d0427f80426d0c3")


