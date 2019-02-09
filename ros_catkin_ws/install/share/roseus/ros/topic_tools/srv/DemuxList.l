;; Auto-generated. Do not edit!


(when (boundp 'topic_tools::DemuxList)
  (if (not (find-package "TOPIC_TOOLS"))
    (make-package "TOPIC_TOOLS"))
  (shadow 'DemuxList (find-package "TOPIC_TOOLS")))
(unless (find-package "TOPIC_TOOLS::DEMUXLIST")
  (make-package "TOPIC_TOOLS::DEMUXLIST"))
(unless (find-package "TOPIC_TOOLS::DEMUXLISTREQUEST")
  (make-package "TOPIC_TOOLS::DEMUXLISTREQUEST"))
(unless (find-package "TOPIC_TOOLS::DEMUXLISTRESPONSE")
  (make-package "TOPIC_TOOLS::DEMUXLISTRESPONSE"))

(in-package "ROS")





(defclass topic_tools::DemuxListRequest
  :super ros::object
  :slots ())

(defmethod topic_tools::DemuxListRequest
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

(defclass topic_tools::DemuxListResponse
  :super ros::object
  :slots (_topics ))

(defmethod topic_tools::DemuxListResponse
  (:init
   (&key
    ((:topics __topics) (let (r) (dotimes (i 0) (push "" r)) r))
    )
   (send-super :init)
   (setq _topics __topics)
   self)
  (:topics
   (&optional __topics)
   (if __topics (setq _topics __topics)) _topics)
  (:serialization-length
   ()
   (+
    ;; string[] _topics
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _topics)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string[] _topics
     (write-long (length _topics) s)
     (dolist (elem _topics)
       (write-long (length elem) s) (princ elem s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string[] _topics
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _topics (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _topics i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;;
   self)
  )

(defclass topic_tools::DemuxList
  :super ros::object
  :slots ())

(setf (get topic_tools::DemuxList :md5sum-) "b0eef9a05d4e829092fc2f2c3c2aad3d")
(setf (get topic_tools::DemuxList :datatype-) "topic_tools/DemuxList")
(setf (get topic_tools::DemuxList :request) topic_tools::DemuxListRequest)
(setf (get topic_tools::DemuxList :response) topic_tools::DemuxListResponse)

(defmethod topic_tools::DemuxListRequest
  (:response () (instance topic_tools::DemuxListResponse :init)))

(setf (get topic_tools::DemuxListRequest :md5sum-) "b0eef9a05d4e829092fc2f2c3c2aad3d")
(setf (get topic_tools::DemuxListRequest :datatype-) "topic_tools/DemuxListRequest")
(setf (get topic_tools::DemuxListRequest :definition-)
      "---
string[] topics

")

(setf (get topic_tools::DemuxListResponse :md5sum-) "b0eef9a05d4e829092fc2f2c3c2aad3d")
(setf (get topic_tools::DemuxListResponse :datatype-) "topic_tools/DemuxListResponse")
(setf (get topic_tools::DemuxListResponse :definition-)
      "---
string[] topics

")



(provide :topic_tools/DemuxList "b0eef9a05d4e829092fc2f2c3c2aad3d")


