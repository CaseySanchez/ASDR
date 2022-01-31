;; Auto-generated. Do not edit!


(when (boundp 'coverage_path_planner::make_plan)
  (if (not (find-package "COVERAGE_PATH_PLANNER"))
    (make-package "COVERAGE_PATH_PLANNER"))
  (shadow 'make_plan (find-package "COVERAGE_PATH_PLANNER")))
(unless (find-package "COVERAGE_PATH_PLANNER::MAKE_PLAN")
  (make-package "COVERAGE_PATH_PLANNER::MAKE_PLAN"))
(unless (find-package "COVERAGE_PATH_PLANNER::MAKE_PLANREQUEST")
  (make-package "COVERAGE_PATH_PLANNER::MAKE_PLANREQUEST"))
(unless (find-package "COVERAGE_PATH_PLANNER::MAKE_PLANRESPONSE")
  (make-package "COVERAGE_PATH_PLANNER::MAKE_PLANRESPONSE"))

(in-package "ROS")



(if (not (find-package "NAV_MSGS"))
  (ros::roseus-add-msgs "nav_msgs"))


(defclass coverage_path_planner::make_planRequest
  :super ros::object
  :slots ())

(defmethod coverage_path_planner::make_planRequest
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

(defclass coverage_path_planner::make_planResponse
  :super ros::object
  :slots (_plan ))

(defmethod coverage_path_planner::make_planResponse
  (:init
   (&key
    ((:plan __plan) (instance nav_msgs::Path :init))
    )
   (send-super :init)
   (setq _plan __plan)
   self)
  (:plan
   (&rest __plan)
   (if (keywordp (car __plan))
       (send* _plan __plan)
     (progn
       (if __plan (setq _plan (car __plan)))
       _plan)))
  (:serialization-length
   ()
   (+
    ;; nav_msgs/Path _plan
    (send _plan :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; nav_msgs/Path _plan
       (send _plan :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; nav_msgs/Path _plan
     (send _plan :deserialize buf ptr-) (incf ptr- (send _plan :serialization-length))
   ;;
   self)
  )

(defclass coverage_path_planner::make_plan
  :super ros::object
  :slots ())

(setf (get coverage_path_planner::make_plan :md5sum-) "0002bc113c0259d71f6cf8cbc9430e18")
(setf (get coverage_path_planner::make_plan :datatype-) "coverage_path_planner/make_plan")
(setf (get coverage_path_planner::make_plan :request) coverage_path_planner::make_planRequest)
(setf (get coverage_path_planner::make_plan :response) coverage_path_planner::make_planResponse)

(defmethod coverage_path_planner::make_planRequest
  (:response () (instance coverage_path_planner::make_planResponse :init)))

(setf (get coverage_path_planner::make_planRequest :md5sum-) "0002bc113c0259d71f6cf8cbc9430e18")
(setf (get coverage_path_planner::make_planRequest :datatype-) "coverage_path_planner/make_planRequest")
(setf (get coverage_path_planner::make_planRequest :definition-)
      "---
nav_msgs/Path plan

================================================================================
MSG: nav_msgs/Path
#An array of poses that represents a Path for a robot to follow
Header header
geometry_msgs/PoseStamped[] poses

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
string frame_id

================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
")

(setf (get coverage_path_planner::make_planResponse :md5sum-) "0002bc113c0259d71f6cf8cbc9430e18")
(setf (get coverage_path_planner::make_planResponse :datatype-) "coverage_path_planner/make_planResponse")
(setf (get coverage_path_planner::make_planResponse :definition-)
      "---
nav_msgs/Path plan

================================================================================
MSG: nav_msgs/Path
#An array of poses that represents a Path for a robot to follow
Header header
geometry_msgs/PoseStamped[] poses

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
string frame_id

================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
")



(provide :coverage_path_planner/make_plan "0002bc113c0259d71f6cf8cbc9430e18")


