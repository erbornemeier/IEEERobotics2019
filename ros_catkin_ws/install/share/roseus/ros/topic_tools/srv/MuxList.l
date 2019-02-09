;; Auto-generated. Do not edit!


(when (boundp 'topic_tools::MuxList)
  (if (not (find-package "TOPIC_TOOLS"))
    (make-package "TOPIC_TOOLS"))
  (shadow 'MuxList (find-package "TOPIC_TOOLS")))
(unless (find-package "TOPIC_TOOLS::MUXLIST")
  (make-package "TOPIC_TOOLS::MUXLIST"))
(unless (find-package "TOPIC_TOOLS::MUXLISTREQUEST")
  (make-package "TOPIC_TOOLS::MUXLISTREQUEST"))
(unless (find-package "TOPIC_TOOLS::MUXLISTRESPONSE")
  (make-package "TOPIC_TOOLS::MUXLISTRESPONSE"))

(in-package "ROS")





(defclass topic_tools::MuxListRequest
  :super ros::object
  :slots ())

(defmethod topic_tools::MuxListRequest
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

(defclass topic_tools::MuxListResponse
  :super ros::object
  :slots (_topics ))

(defmethod topic_tools::MuxListResponse
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

(defclass topic_tools::MuxList
  :super ros::object
  :slots ())

(setf (get topic_tools::MuxList :md5sum-) "b0eef9a05d4e829092fc2f2c3c2aad3d")
(setf (get topic_tools::MuxList :datatype-) "topic_tools/MuxList")
(setf (get topic_tools::MuxList :request) topic_tools::MuxListRequest)
(setf (get topic_tools::MuxList :response) topic_tools::MuxListResponse)

(defmethod topic_tools::MuxListRequest
  (:response () (instance topic_tools::MuxListResponse :init)))

(setf (get topic_tools::MuxListRequest :md5sum-) "b0eef9a05d4e829092fc2f2c3c2aad3d")
(setf (get topic_tools::MuxListRequest :datatype-) "topic_tools/MuxListRequest")
(setf (get topic_tools::MuxListRequest :definition-)
      "---
string[] topics

")

(setf (get topic_tools::MuxListResponse :md5sum-) "b0eef9a05d4e829092fc2f2c3c2aad3d")
(setf (get topic_tools::MuxListResponse :datatype-) "topic_tools/MuxListResponse")
(setf (get topic_tools::MuxListResponse :definition-)
      "---
string[] topics

")



(provide :topic_tools/MuxList "b0eef9a05d4e829092fc2f2c3c2aad3d")


