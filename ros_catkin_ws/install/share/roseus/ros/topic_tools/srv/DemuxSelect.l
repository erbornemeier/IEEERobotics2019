;; Auto-generated. Do not edit!


(when (boundp 'topic_tools::DemuxSelect)
  (if (not (find-package "TOPIC_TOOLS"))
    (make-package "TOPIC_TOOLS"))
  (shadow 'DemuxSelect (find-package "TOPIC_TOOLS")))
(unless (find-package "TOPIC_TOOLS::DEMUXSELECT")
  (make-package "TOPIC_TOOLS::DEMUXSELECT"))
(unless (find-package "TOPIC_TOOLS::DEMUXSELECTREQUEST")
  (make-package "TOPIC_TOOLS::DEMUXSELECTREQUEST"))
(unless (find-package "TOPIC_TOOLS::DEMUXSELECTRESPONSE")
  (make-package "TOPIC_TOOLS::DEMUXSELECTRESPONSE"))

(in-package "ROS")





(defclass topic_tools::DemuxSelectRequest
  :super ros::object
  :slots (_topic ))

(defmethod topic_tools::DemuxSelectRequest
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

(defclass topic_tools::DemuxSelectResponse
  :super ros::object
  :slots (_prev_topic ))

(defmethod topic_tools::DemuxSelectResponse
  (:init
   (&key
    ((:prev_topic __prev_topic) "")
    )
   (send-super :init)
   (setq _prev_topic (string __prev_topic))
   self)
  (:prev_topic
   (&optional __prev_topic)
   (if __prev_topic (setq _prev_topic __prev_topic)) _prev_topic)
  (:serialization-length
   ()
   (+
    ;; string _prev_topic
    4 (length _prev_topic)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _prev_topic
       (write-long (length _prev_topic) s) (princ _prev_topic s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _prev_topic
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _prev_topic (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass topic_tools::DemuxSelect
  :super ros::object
  :slots ())

(setf (get topic_tools::DemuxSelect :md5sum-) "053052240ca985e1f2eedbb0dae9b1f7")
(setf (get topic_tools::DemuxSelect :datatype-) "topic_tools/DemuxSelect")
(setf (get topic_tools::DemuxSelect :request) topic_tools::DemuxSelectRequest)
(setf (get topic_tools::DemuxSelect :response) topic_tools::DemuxSelectResponse)

(defmethod topic_tools::DemuxSelectRequest
  (:response () (instance topic_tools::DemuxSelectResponse :init)))

(setf (get topic_tools::DemuxSelectRequest :md5sum-) "053052240ca985e1f2eedbb0dae9b1f7")
(setf (get topic_tools::DemuxSelectRequest :datatype-) "topic_tools/DemuxSelectRequest")
(setf (get topic_tools::DemuxSelectRequest :definition-)
      "string topic
---
string prev_topic

")

(setf (get topic_tools::DemuxSelectResponse :md5sum-) "053052240ca985e1f2eedbb0dae9b1f7")
(setf (get topic_tools::DemuxSelectResponse :datatype-) "topic_tools/DemuxSelectResponse")
(setf (get topic_tools::DemuxSelectResponse :definition-)
      "string topic
---
string prev_topic

")



(provide :topic_tools/DemuxSelect "053052240ca985e1f2eedbb0dae9b1f7")


