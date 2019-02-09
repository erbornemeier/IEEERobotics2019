;; Auto-generated. Do not edit!


(when (boundp 'topic_tools::MuxSelect)
  (if (not (find-package "TOPIC_TOOLS"))
    (make-package "TOPIC_TOOLS"))
  (shadow 'MuxSelect (find-package "TOPIC_TOOLS")))
(unless (find-package "TOPIC_TOOLS::MUXSELECT")
  (make-package "TOPIC_TOOLS::MUXSELECT"))
(unless (find-package "TOPIC_TOOLS::MUXSELECTREQUEST")
  (make-package "TOPIC_TOOLS::MUXSELECTREQUEST"))
(unless (find-package "TOPIC_TOOLS::MUXSELECTRESPONSE")
  (make-package "TOPIC_TOOLS::MUXSELECTRESPONSE"))

(in-package "ROS")





(defclass topic_tools::MuxSelectRequest
  :super ros::object
  :slots (_topic ))

(defmethod topic_tools::MuxSelectRequest
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

(defclass topic_tools::MuxSelectResponse
  :super ros::object
  :slots (_prev_topic ))

(defmethod topic_tools::MuxSelectResponse
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

(defclass topic_tools::MuxSelect
  :super ros::object
  :slots ())

(setf (get topic_tools::MuxSelect :md5sum-) "053052240ca985e1f2eedbb0dae9b1f7")
(setf (get topic_tools::MuxSelect :datatype-) "topic_tools/MuxSelect")
(setf (get topic_tools::MuxSelect :request) topic_tools::MuxSelectRequest)
(setf (get topic_tools::MuxSelect :response) topic_tools::MuxSelectResponse)

(defmethod topic_tools::MuxSelectRequest
  (:response () (instance topic_tools::MuxSelectResponse :init)))

(setf (get topic_tools::MuxSelectRequest :md5sum-) "053052240ca985e1f2eedbb0dae9b1f7")
(setf (get topic_tools::MuxSelectRequest :datatype-) "topic_tools/MuxSelectRequest")
(setf (get topic_tools::MuxSelectRequest :definition-)
      "string topic
---
string prev_topic

")

(setf (get topic_tools::MuxSelectResponse :md5sum-) "053052240ca985e1f2eedbb0dae9b1f7")
(setf (get topic_tools::MuxSelectResponse :datatype-) "topic_tools/MuxSelectResponse")
(setf (get topic_tools::MuxSelectResponse :definition-)
      "string topic
---
string prev_topic

")



(provide :topic_tools/MuxSelect "053052240ca985e1f2eedbb0dae9b1f7")


