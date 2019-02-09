;; Auto-generated. Do not edit!


(when (boundp 'rosgraph_msgs::TopicStatistics)
  (if (not (find-package "ROSGRAPH_MSGS"))
    (make-package "ROSGRAPH_MSGS"))
  (shadow 'TopicStatistics (find-package "ROSGRAPH_MSGS")))
(unless (find-package "ROSGRAPH_MSGS::TOPICSTATISTICS")
  (make-package "ROSGRAPH_MSGS::TOPICSTATISTICS"))

(in-package "ROS")
;;//! \htmlinclude TopicStatistics.msg.html


(defclass rosgraph_msgs::TopicStatistics
  :super ros::object
  :slots (_topic _node_pub _node_sub _window_start _window_stop _delivered_msgs _dropped_msgs _traffic _period_mean _period_stddev _period_max _stamp_age_mean _stamp_age_stddev _stamp_age_max ))

(defmethod rosgraph_msgs::TopicStatistics
  (:init
   (&key
    ((:topic __topic) "")
    ((:node_pub __node_pub) "")
    ((:node_sub __node_sub) "")
    ((:window_start __window_start) (instance ros::time :init))
    ((:window_stop __window_stop) (instance ros::time :init))
    ((:delivered_msgs __delivered_msgs) 0)
    ((:dropped_msgs __dropped_msgs) 0)
    ((:traffic __traffic) 0)
    ((:period_mean __period_mean) (instance ros::time :init))
    ((:period_stddev __period_stddev) (instance ros::time :init))
    ((:period_max __period_max) (instance ros::time :init))
    ((:stamp_age_mean __stamp_age_mean) (instance ros::time :init))
    ((:stamp_age_stddev __stamp_age_stddev) (instance ros::time :init))
    ((:stamp_age_max __stamp_age_max) (instance ros::time :init))
    )
   (send-super :init)
   (setq _topic (string __topic))
   (setq _node_pub (string __node_pub))
   (setq _node_sub (string __node_sub))
   (setq _window_start __window_start)
   (setq _window_stop __window_stop)
   (setq _delivered_msgs (round __delivered_msgs))
   (setq _dropped_msgs (round __dropped_msgs))
   (setq _traffic (round __traffic))
   (setq _period_mean __period_mean)
   (setq _period_stddev __period_stddev)
   (setq _period_max __period_max)
   (setq _stamp_age_mean __stamp_age_mean)
   (setq _stamp_age_stddev __stamp_age_stddev)
   (setq _stamp_age_max __stamp_age_max)
   self)
  (:topic
   (&optional __topic)
   (if __topic (setq _topic __topic)) _topic)
  (:node_pub
   (&optional __node_pub)
   (if __node_pub (setq _node_pub __node_pub)) _node_pub)
  (:node_sub
   (&optional __node_sub)
   (if __node_sub (setq _node_sub __node_sub)) _node_sub)
  (:window_start
   (&optional __window_start)
   (if __window_start (setq _window_start __window_start)) _window_start)
  (:window_stop
   (&optional __window_stop)
   (if __window_stop (setq _window_stop __window_stop)) _window_stop)
  (:delivered_msgs
   (&optional __delivered_msgs)
   (if __delivered_msgs (setq _delivered_msgs __delivered_msgs)) _delivered_msgs)
  (:dropped_msgs
   (&optional __dropped_msgs)
   (if __dropped_msgs (setq _dropped_msgs __dropped_msgs)) _dropped_msgs)
  (:traffic
   (&optional __traffic)
   (if __traffic (setq _traffic __traffic)) _traffic)
  (:period_mean
   (&optional __period_mean)
   (if __period_mean (setq _period_mean __period_mean)) _period_mean)
  (:period_stddev
   (&optional __period_stddev)
   (if __period_stddev (setq _period_stddev __period_stddev)) _period_stddev)
  (:period_max
   (&optional __period_max)
   (if __period_max (setq _period_max __period_max)) _period_max)
  (:stamp_age_mean
   (&optional __stamp_age_mean)
   (if __stamp_age_mean (setq _stamp_age_mean __stamp_age_mean)) _stamp_age_mean)
  (:stamp_age_stddev
   (&optional __stamp_age_stddev)
   (if __stamp_age_stddev (setq _stamp_age_stddev __stamp_age_stddev)) _stamp_age_stddev)
  (:stamp_age_max
   (&optional __stamp_age_max)
   (if __stamp_age_max (setq _stamp_age_max __stamp_age_max)) _stamp_age_max)
  (:serialization-length
   ()
   (+
    ;; string _topic
    4 (length _topic)
    ;; string _node_pub
    4 (length _node_pub)
    ;; string _node_sub
    4 (length _node_sub)
    ;; time _window_start
    8
    ;; time _window_stop
    8
    ;; int32 _delivered_msgs
    4
    ;; int32 _dropped_msgs
    4
    ;; int32 _traffic
    4
    ;; duration _period_mean
    8
    ;; duration _period_stddev
    8
    ;; duration _period_max
    8
    ;; duration _stamp_age_mean
    8
    ;; duration _stamp_age_stddev
    8
    ;; duration _stamp_age_max
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _topic
       (write-long (length _topic) s) (princ _topic s)
     ;; string _node_pub
       (write-long (length _node_pub) s) (princ _node_pub s)
     ;; string _node_sub
       (write-long (length _node_sub) s) (princ _node_sub s)
     ;; time _window_start
       (write-long (send _window_start :sec) s) (write-long (send _window_start :nsec) s)
     ;; time _window_stop
       (write-long (send _window_stop :sec) s) (write-long (send _window_stop :nsec) s)
     ;; int32 _delivered_msgs
       (write-long _delivered_msgs s)
     ;; int32 _dropped_msgs
       (write-long _dropped_msgs s)
     ;; int32 _traffic
       (write-long _traffic s)
     ;; duration _period_mean
       (write-long (send _period_mean :sec) s) (write-long (send _period_mean :nsec) s)
     ;; duration _period_stddev
       (write-long (send _period_stddev :sec) s) (write-long (send _period_stddev :nsec) s)
     ;; duration _period_max
       (write-long (send _period_max :sec) s) (write-long (send _period_max :nsec) s)
     ;; duration _stamp_age_mean
       (write-long (send _stamp_age_mean :sec) s) (write-long (send _stamp_age_mean :nsec) s)
     ;; duration _stamp_age_stddev
       (write-long (send _stamp_age_stddev :sec) s) (write-long (send _stamp_age_stddev :nsec) s)
     ;; duration _stamp_age_max
       (write-long (send _stamp_age_max :sec) s) (write-long (send _stamp_age_max :nsec) s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _topic
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _topic (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _node_pub
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _node_pub (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _node_sub
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _node_sub (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; time _window_start
     (send _window_start :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _window_start :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; time _window_stop
     (send _window_stop :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _window_stop :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _delivered_msgs
     (setq _delivered_msgs (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _dropped_msgs
     (setq _dropped_msgs (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _traffic
     (setq _traffic (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; duration _period_mean
     (send _period_mean :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _period_mean :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; duration _period_stddev
     (send _period_stddev :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _period_stddev :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; duration _period_max
     (send _period_max :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _period_max :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; duration _stamp_age_mean
     (send _stamp_age_mean :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _stamp_age_mean :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; duration _stamp_age_stddev
     (send _stamp_age_stddev :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _stamp_age_stddev :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; duration _stamp_age_max
     (send _stamp_age_max :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _stamp_age_max :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get rosgraph_msgs::TopicStatistics :md5sum-) "10152ed868c5097a5e2e4a89d7daa710")
(setf (get rosgraph_msgs::TopicStatistics :datatype-) "rosgraph_msgs/TopicStatistics")
(setf (get rosgraph_msgs::TopicStatistics :definition-)
      "# name of the topic
string topic

# node id of the publisher
string node_pub

# node id of the subscriber
string node_sub

# the statistics apply to this time window
time window_start
time window_stop

# number of messages delivered during the window
int32 delivered_msgs
# numbers of messages dropped during the window
int32 dropped_msgs

# traffic during the window, in bytes
int32 traffic

# mean/stddev/max period between two messages
duration period_mean
duration period_stddev
duration period_max

# mean/stddev/max age of the message based on the
# timestamp in the message header. In case the
# message does not have a header, it will be 0.
duration stamp_age_mean
duration stamp_age_stddev
duration stamp_age_max

")



(provide :rosgraph_msgs/TopicStatistics "10152ed868c5097a5e2e4a89d7daa710")


