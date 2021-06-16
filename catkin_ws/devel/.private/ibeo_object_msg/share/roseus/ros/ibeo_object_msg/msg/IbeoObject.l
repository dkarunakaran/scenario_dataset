;; Auto-generated. Do not edit!


(when (boundp 'ibeo_object_msg::IbeoObject)
  (if (not (find-package "IBEO_OBJECT_MSG"))
    (make-package "IBEO_OBJECT_MSG"))
  (shadow 'IbeoObject (find-package "IBEO_OBJECT_MSG")))
(unless (find-package "IBEO_OBJECT_MSG::IBEOOBJECT")
  (make-package "IBEO_OBJECT_MSG::IBEOOBJECT"))

(in-package "ROS")
;;//! \htmlinclude IbeoObject.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "SHAPE_MSGS"))
  (ros::roseus-add-msgs "shape_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ibeo_object_msg::IbeoObject
  :super ros::object
  :slots (_header _object_id _object_class _object_class_uncertainty _object_age _object_class_age _mobile _motion_model_validated _pose _twist _shape ))

(defmethod ibeo_object_msg::IbeoObject
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:object_id __object_id) 0)
    ((:object_class __object_class) 0)
    ((:object_class_uncertainty __object_class_uncertainty) 0)
    ((:object_age __object_age) 0.0)
    ((:object_class_age __object_class_age) 0.0)
    ((:mobile __mobile) nil)
    ((:motion_model_validated __motion_model_validated) nil)
    ((:pose __pose) (instance geometry_msgs::PoseWithCovariance :init))
    ((:twist __twist) (instance geometry_msgs::TwistWithCovariance :init))
    ((:shape __shape) (instance shape_msgs::SolidPrimitive :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _object_id (round __object_id))
   (setq _object_class (round __object_class))
   (setq _object_class_uncertainty (round __object_class_uncertainty))
   (setq _object_age (float __object_age))
   (setq _object_class_age (float __object_class_age))
   (setq _mobile __mobile)
   (setq _motion_model_validated __motion_model_validated)
   (setq _pose __pose)
   (setq _twist __twist)
   (setq _shape __shape)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:object_id
   (&optional __object_id)
   (if __object_id (setq _object_id __object_id)) _object_id)
  (:object_class
   (&optional __object_class)
   (if __object_class (setq _object_class __object_class)) _object_class)
  (:object_class_uncertainty
   (&optional __object_class_uncertainty)
   (if __object_class_uncertainty (setq _object_class_uncertainty __object_class_uncertainty)) _object_class_uncertainty)
  (:object_age
   (&optional __object_age)
   (if __object_age (setq _object_age __object_age)) _object_age)
  (:object_class_age
   (&optional __object_class_age)
   (if __object_class_age (setq _object_class_age __object_class_age)) _object_class_age)
  (:mobile
   (&optional __mobile)
   (if __mobile (setq _mobile __mobile)) _mobile)
  (:motion_model_validated
   (&optional __motion_model_validated)
   (if __motion_model_validated (setq _motion_model_validated __motion_model_validated)) _motion_model_validated)
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:twist
   (&rest __twist)
   (if (keywordp (car __twist))
       (send* _twist __twist)
     (progn
       (if __twist (setq _twist (car __twist)))
       _twist)))
  (:shape
   (&rest __shape)
   (if (keywordp (car __shape))
       (send* _shape __shape)
     (progn
       (if __shape (setq _shape (car __shape)))
       _shape)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint32 _object_id
    4
    ;; uint8 _object_class
    1
    ;; uint16 _object_class_uncertainty
    2
    ;; float32 _object_age
    4
    ;; float32 _object_class_age
    4
    ;; bool _mobile
    1
    ;; bool _motion_model_validated
    1
    ;; geometry_msgs/PoseWithCovariance _pose
    (send _pose :serialization-length)
    ;; geometry_msgs/TwistWithCovariance _twist
    (send _twist :serialization-length)
    ;; shape_msgs/SolidPrimitive _shape
    (send _shape :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint32 _object_id
       (write-long _object_id s)
     ;; uint8 _object_class
       (write-byte _object_class s)
     ;; uint16 _object_class_uncertainty
       (write-word _object_class_uncertainty s)
     ;; float32 _object_age
       (sys::poke _object_age (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _object_class_age
       (sys::poke _object_class_age (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _mobile
       (if _mobile (write-byte -1 s) (write-byte 0 s))
     ;; bool _motion_model_validated
       (if _motion_model_validated (write-byte -1 s) (write-byte 0 s))
     ;; geometry_msgs/PoseWithCovariance _pose
       (send _pose :serialize s)
     ;; geometry_msgs/TwistWithCovariance _twist
       (send _twist :serialize s)
     ;; shape_msgs/SolidPrimitive _shape
       (send _shape :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint32 _object_id
     (setq _object_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint8 _object_class
     (setq _object_class (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint16 _object_class_uncertainty
     (setq _object_class_uncertainty (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float32 _object_age
     (setq _object_age (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _object_class_age
     (setq _object_class_age (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _mobile
     (setq _mobile (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _motion_model_validated
     (setq _motion_model_validated (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; geometry_msgs/PoseWithCovariance _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;; geometry_msgs/TwistWithCovariance _twist
     (send _twist :deserialize buf ptr-) (incf ptr- (send _twist :serialization-length))
   ;; shape_msgs/SolidPrimitive _shape
     (send _shape :deserialize buf ptr-) (incf ptr- (send _shape :serialization-length))
   ;;
   self)
  )

(setf (get ibeo_object_msg::IbeoObject :md5sum-) "985d2c588d52776ee0b17d51f99bcf86")
(setf (get ibeo_object_msg::IbeoObject :datatype-) "ibeo_object_msg/IbeoObject")
(setf (get ibeo_object_msg::IbeoObject :definition-)
      "Header header
uint32 object_id
uint8 object_class
uint16 object_class_uncertainty
float32 object_age
float32 object_class_age
bool mobile
bool motion_model_validated
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist
shape_msgs/SolidPrimitive shape

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
MSG: geometry_msgs/PoseWithCovariance
# This represents a pose in free space with uncertainty.

Pose pose

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance

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

================================================================================
MSG: geometry_msgs/TwistWithCovariance
# This expresses velocity in free space with uncertainty.

Twist twist

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: shape_msgs/SolidPrimitive
# Define box, sphere, cylinder, cone 
# All shapes are defined to have their bounding boxes centered around 0,0,0.

uint8 BOX=1
uint8 SPHERE=2
uint8 CYLINDER=3
uint8 CONE=4

# The type of the shape
uint8 type


# The dimensions of the shape
float64[] dimensions

# The meaning of the shape dimensions: each constant defines the index in the 'dimensions' array

# For the BOX type, the X, Y, and Z dimensions are the length of the corresponding
# sides of the box.
uint8 BOX_X=0
uint8 BOX_Y=1
uint8 BOX_Z=2


# For the SPHERE type, only one component is used, and it gives the radius of
# the sphere.
uint8 SPHERE_RADIUS=0


# For the CYLINDER and CONE types, the center line is oriented along
# the Z axis.  Therefore the CYLINDER_HEIGHT (CONE_HEIGHT) component
# of dimensions gives the height of the cylinder (cone).  The
# CYLINDER_RADIUS (CONE_RADIUS) component of dimensions gives the
# radius of the base of the cylinder (cone).  Cone and cylinder
# primitives are defined to be circular. The tip of the cone is
# pointing up, along +Z axis.

uint8 CYLINDER_HEIGHT=0
uint8 CYLINDER_RADIUS=1

uint8 CONE_HEIGHT=0
uint8 CONE_RADIUS=1

")



(provide :ibeo_object_msg/IbeoObject "985d2c588d52776ee0b17d51f99bcf86")


