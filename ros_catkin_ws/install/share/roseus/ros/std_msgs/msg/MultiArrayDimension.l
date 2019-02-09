;; Auto-generated. Do not edit!


(when (boundp 'std_msgs::MultiArrayDimension)
  (if (not (find-package "STD_MSGS"))
    (make-package "STD_MSGS"))
  (shadow 'MultiArrayDimension (find-package "STD_MSGS")))
(unless (find-package "STD_MSGS::MULTIARRAYDIMENSION")
  (make-package "STD_MSGS::MULTIARRAYDIMENSION"))

(in-package "ROS")
;;//! \htmlinclude MultiArrayDimension.msg.html


(defclass std_msgs::MultiArrayDimension
  :super ros::object
  :slots (_label _size _stride ))

(defmethod std_msgs::MultiArrayDimension
  (:init
   (&key
    ((:label __label) "")
    ((:size __size) 0)
    ((:stride __stride) 0)
    )
   (send-super :init)
   (setq _label (string __label))
   (setq _size (round __size))
   (setq _stride (round __stride))
   self)
  (:label
   (&optional __label)
   (if __label (setq _label __label)) _label)
  (:size
   (&optional __size)
   (if __size (setq _size __size)) _size)
  (:stride
   (&optional __stride)
   (if __stride (setq _stride __stride)) _stride)
  (:serialization-length
   ()
   (+
    ;; string _label
    4 (length _label)
    ;; uint32 _size
    4
    ;; uint32 _stride
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _label
       (write-long (length _label) s) (princ _label s)
     ;; uint32 _size
       (write-long _size s)
     ;; uint32 _stride
       (write-long _stride s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _label
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _label (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint32 _size
     (setq _size (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _stride
     (setq _stride (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get std_msgs::MultiArrayDimension :md5sum-) "4cd0c83a8683deae40ecdac60e53bfa8")
(setf (get std_msgs::MultiArrayDimension :datatype-) "std_msgs/MultiArrayDimension")
(setf (get std_msgs::MultiArrayDimension :definition-)
      "string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension
")



(provide :std_msgs/MultiArrayDimension "4cd0c83a8683deae40ecdac60e53bfa8")


