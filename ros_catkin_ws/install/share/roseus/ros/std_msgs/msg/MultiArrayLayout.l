;; Auto-generated. Do not edit!


(when (boundp 'std_msgs::MultiArrayLayout)
  (if (not (find-package "STD_MSGS"))
    (make-package "STD_MSGS"))
  (shadow 'MultiArrayLayout (find-package "STD_MSGS")))
(unless (find-package "STD_MSGS::MULTIARRAYLAYOUT")
  (make-package "STD_MSGS::MULTIARRAYLAYOUT"))

(in-package "ROS")
;;//! \htmlinclude MultiArrayLayout.msg.html


(defclass std_msgs::MultiArrayLayout
  :super ros::object
  :slots (_dim _data_offset ))

(defmethod std_msgs::MultiArrayLayout
  (:init
   (&key
    ((:dim __dim) (let (r) (dotimes (i 0) (push (instance std_msgs::MultiArrayDimension :init) r)) r))
    ((:data_offset __data_offset) 0)
    )
   (send-super :init)
   (setq _dim __dim)
   (setq _data_offset (round __data_offset))
   self)
  (:dim
   (&rest __dim)
   (if (keywordp (car __dim))
       (send* _dim __dim)
     (progn
       (if __dim (setq _dim (car __dim)))
       _dim)))
  (:data_offset
   (&optional __data_offset)
   (if __data_offset (setq _data_offset __data_offset)) _data_offset)
  (:serialization-length
   ()
   (+
    ;; std_msgs/MultiArrayDimension[] _dim
    (apply #'+ (send-all _dim :serialization-length)) 4
    ;; uint32 _data_offset
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/MultiArrayDimension[] _dim
     (write-long (length _dim) s)
     (dolist (elem _dim)
       (send elem :serialize s)
       )
     ;; uint32 _data_offset
       (write-long _data_offset s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/MultiArrayDimension[] _dim
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _dim (let (r) (dotimes (i n) (push (instance std_msgs::MultiArrayDimension :init) r)) r))
     (dolist (elem- _dim)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; uint32 _data_offset
     (setq _data_offset (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get std_msgs::MultiArrayLayout :md5sum-) "0fed2a11c13e11c5571b4e2a995a91a3")
(setf (get std_msgs::MultiArrayLayout :datatype-) "std_msgs/MultiArrayLayout")
(setf (get std_msgs::MultiArrayLayout :definition-)
      "# The multiarray declares a generic multi-dimensional array of a
# particular data type.  Dimensions are ordered from outer most
# to inner most.

MultiArrayDimension[] dim # Array of dimension properties
uint32 data_offset        # padding elements at front of data

# Accessors should ALWAYS be written in terms of dimension stride
# and specified outer-most dimension first.
# 
# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
#
# A standard, 3-channel 640x480 image with interleaved color channels
# would be specified as:
#
# dim[0].label  = \"height\"
# dim[0].size   = 480
# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
# dim[1].label  = \"width\"
# dim[1].size   = 640
# dim[1].stride = 3*640 = 1920
# dim[2].label  = \"channel\"
# dim[2].size   = 3
# dim[2].stride = 3
#
# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.

================================================================================
MSG: std_msgs/MultiArrayDimension
string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension
")



(provide :std_msgs/MultiArrayLayout "0fed2a11c13e11c5571b4e2a995a91a3")


