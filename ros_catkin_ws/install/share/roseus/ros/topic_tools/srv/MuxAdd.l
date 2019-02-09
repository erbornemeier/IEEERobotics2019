;; Auto-generated. Do not edit!


(when (boundp 'topic_tools::MuxAdd)
  (if (not (find-package "TOPIC_TOOLS"))
    (make-package "TOPIC_TOOLS"))
  (shadow 'MuxAdd (find-package "TOPIC_TOOLS")))
(unless (find-package "TOPIC_TOOLS::MUXADD")
  (make-package "TOPIC_TOOLS::MUXADD"))
(unless (find-package "TOPIC_TOOLS::MUXADDREQUEST")
  (make-package "TOPIC_TOOLS::MUXADDREQUEST"))
(unless (find-package "TOPIC_TOOLS::MUXADDRESPONSE")
  (make-package "TOPIC_TOOLS::MUXADDRESPONSE"))

(in-package "ROS")





(defclass topic_tools::MuxAddRequest
  :super ros::object
  :slots (_topic ))

(defmethod topic_tools::MuxAddRequest
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

(defclass topic_tools::MuxAddResponse
  :super ros::object
  :slots ())

(defmethod topic_tools::MuxAddResponse
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

(defclass topic_tools::MuxAdd
  :super ros::object
  :slots ())

(setf (get topic_tools::MuxAdd :md5sum-) "d8f94bae31b356b24d0427f80426d0c3")
(setf (get topic_tools::MuxAdd :datatype-) "topic_tools/MuxAdd")
(setf (get topic_tools::MuxAdd :request) topic_tools::MuxAddRequest)
(setf (get topic_tools::MuxAdd :response) topic_tools::MuxAddResponse)

(defmethod topic_tools::MuxAddRequest
  (:response () (instance topic_tools::MuxAddResponse :init)))

(setf (get topic_tools::MuxAddRequest :md5sum-) "d8f94bae31b356b24d0427f80426d0c3")
(setf (get topic_tools::MuxAddRequest :datatype-) "topic_tools/MuxAddRequest")
(setf (get topic_tools::MuxAddRequest :definition-)
      "string topic
---

")

(setf (get topic_tools::MuxAddResponse :md5sum-) "d8f94bae31b356b24d0427f80426d0c3")
(setf (get topic_tools::MuxAddResponse :datatype-) "topic_tools/MuxAddResponse")
(setf (get topic_tools::MuxAddResponse :definition-)
      "string topic
---

")



(provide :topic_tools/MuxAdd "d8f94bae31b356b24d0427f80426d0c3")


