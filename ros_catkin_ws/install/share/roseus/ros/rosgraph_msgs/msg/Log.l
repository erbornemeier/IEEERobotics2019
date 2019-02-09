;; Auto-generated. Do not edit!


(when (boundp 'rosgraph_msgs::Log)
  (if (not (find-package "ROSGRAPH_MSGS"))
    (make-package "ROSGRAPH_MSGS"))
  (shadow 'Log (find-package "ROSGRAPH_MSGS")))
(unless (find-package "ROSGRAPH_MSGS::LOG")
  (make-package "ROSGRAPH_MSGS::LOG"))

(in-package "ROS")
;;//! \htmlinclude Log.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*DEBUG*" (find-package "ROSGRAPH_MSGS::LOG"))
(shadow '*DEBUG* (find-package "ROSGRAPH_MSGS::LOG"))
(defconstant rosgraph_msgs::Log::*DEBUG* 1)
(intern "*INFO*" (find-package "ROSGRAPH_MSGS::LOG"))
(shadow '*INFO* (find-package "ROSGRAPH_MSGS::LOG"))
(defconstant rosgraph_msgs::Log::*INFO* 2)
(intern "*WARN*" (find-package "ROSGRAPH_MSGS::LOG"))
(shadow '*WARN* (find-package "ROSGRAPH_MSGS::LOG"))
(defconstant rosgraph_msgs::Log::*WARN* 4)
(intern "*ERROR*" (find-package "ROSGRAPH_MSGS::LOG"))
(shadow '*ERROR* (find-package "ROSGRAPH_MSGS::LOG"))
(defconstant rosgraph_msgs::Log::*ERROR* 8)
(intern "*FATAL*" (find-package "ROSGRAPH_MSGS::LOG"))
(shadow '*FATAL* (find-package "ROSGRAPH_MSGS::LOG"))
(defconstant rosgraph_msgs::Log::*FATAL* 16)
(defclass rosgraph_msgs::Log
  :super ros::object
  :slots (_header _level _name _msg _file _function _line _topics ))

(defmethod rosgraph_msgs::Log
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:level __level) 0)
    ((:name __name) "")
    ((:msg __msg) "")
    ((:file __file) "")
    ((:function __function) "")
    ((:line __line) 0)
    ((:topics __topics) (let (r) (dotimes (i 0) (push "" r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _level (round __level))
   (setq _name (string __name))
   (setq _msg (string __msg))
   (setq _file (string __file))
   (setq _function (string __function))
   (setq _line (round __line))
   (setq _topics __topics)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:level
   (&optional __level)
   (if __level (setq _level __level)) _level)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:msg
   (&optional __msg)
   (if __msg (setq _msg __msg)) _msg)
  (:file
   (&optional __file)
   (if __file (setq _file __file)) _file)
  (:function
   (&optional __function)
   (if __function (setq _function __function)) _function)
  (:line
   (&optional __line)
   (if __line (setq _line __line)) _line)
  (:topics
   (&optional __topics)
   (if __topics (setq _topics __topics)) _topics)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; byte _level
    1
    ;; string _name
    4 (length _name)
    ;; string _msg
    4 (length _msg)
    ;; string _file
    4 (length _file)
    ;; string _function
    4 (length _function)
    ;; uint32 _line
    4
    ;; string[] _topics
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _topics)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; byte _level
       (write-byte _level s)
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; string _msg
       (write-long (length _msg) s) (princ _msg s)
     ;; string _file
       (write-long (length _file) s) (princ _file s)
     ;; string _function
       (write-long (length _function) s) (princ _function s)
     ;; uint32 _line
       (write-long _line s)
     ;; string[] _topics
     (write-long (length _topics) s)
     (dolist (elem _topics)
       (write-long (length elem) s) (princ elem s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; byte _level
     (setq _level (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _level 127) (setq _level (- _level 256)))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _msg
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _msg (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _file
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _file (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _function
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _function (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint32 _line
     (setq _line (sys::peek buf ptr- :integer)) (incf ptr- 4)
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

(setf (get rosgraph_msgs::Log :md5sum-) "acffd30cd6b6de30f120938c17c593fb")
(setf (get rosgraph_msgs::Log :datatype-) "rosgraph_msgs/Log")
(setf (get rosgraph_msgs::Log :definition-)
      "##
## Severity level constants
##
byte DEBUG=1 #debug level
byte INFO=2  #general level
byte WARN=4  #warning level
byte ERROR=8 #error level
byte FATAL=16 #fatal/critical level
##
## Fields
##
Header header
byte level
string name # name of the node
string msg # message 
string file # file the message came from
string function # function the message came from
uint32 line # line the message came from
string[] topics # topic names that the node publishes

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

")



(provide :rosgraph_msgs/Log "acffd30cd6b6de30f120938c17c593fb")


