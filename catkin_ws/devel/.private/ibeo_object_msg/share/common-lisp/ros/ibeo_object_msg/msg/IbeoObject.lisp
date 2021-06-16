; Auto-generated. Do not edit!


(cl:in-package ibeo_object_msg-msg)


;//! \htmlinclude IbeoObject.msg.html

(cl:defclass <IbeoObject> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (object_id
    :reader object_id
    :initarg :object_id
    :type cl:integer
    :initform 0)
   (object_class
    :reader object_class
    :initarg :object_class
    :type cl:fixnum
    :initform 0)
   (object_class_uncertainty
    :reader object_class_uncertainty
    :initarg :object_class_uncertainty
    :type cl:fixnum
    :initform 0)
   (object_age
    :reader object_age
    :initarg :object_age
    :type cl:float
    :initform 0.0)
   (object_class_age
    :reader object_class_age
    :initarg :object_class_age
    :type cl:float
    :initform 0.0)
   (mobile
    :reader mobile
    :initarg :mobile
    :type cl:boolean
    :initform cl:nil)
   (motion_model_validated
    :reader motion_model_validated
    :initarg :motion_model_validated
    :type cl:boolean
    :initform cl:nil)
   (pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:PoseWithCovariance
    :initform (cl:make-instance 'geometry_msgs-msg:PoseWithCovariance))
   (twist
    :reader twist
    :initarg :twist
    :type geometry_msgs-msg:TwistWithCovariance
    :initform (cl:make-instance 'geometry_msgs-msg:TwistWithCovariance))
   (shape
    :reader shape
    :initarg :shape
    :type shape_msgs-msg:SolidPrimitive
    :initform (cl:make-instance 'shape_msgs-msg:SolidPrimitive)))
)

(cl:defclass IbeoObject (<IbeoObject>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <IbeoObject>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'IbeoObject)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ibeo_object_msg-msg:<IbeoObject> is deprecated: use ibeo_object_msg-msg:IbeoObject instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <IbeoObject>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ibeo_object_msg-msg:header-val is deprecated.  Use ibeo_object_msg-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'object_id-val :lambda-list '(m))
(cl:defmethod object_id-val ((m <IbeoObject>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ibeo_object_msg-msg:object_id-val is deprecated.  Use ibeo_object_msg-msg:object_id instead.")
  (object_id m))

(cl:ensure-generic-function 'object_class-val :lambda-list '(m))
(cl:defmethod object_class-val ((m <IbeoObject>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ibeo_object_msg-msg:object_class-val is deprecated.  Use ibeo_object_msg-msg:object_class instead.")
  (object_class m))

(cl:ensure-generic-function 'object_class_uncertainty-val :lambda-list '(m))
(cl:defmethod object_class_uncertainty-val ((m <IbeoObject>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ibeo_object_msg-msg:object_class_uncertainty-val is deprecated.  Use ibeo_object_msg-msg:object_class_uncertainty instead.")
  (object_class_uncertainty m))

(cl:ensure-generic-function 'object_age-val :lambda-list '(m))
(cl:defmethod object_age-val ((m <IbeoObject>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ibeo_object_msg-msg:object_age-val is deprecated.  Use ibeo_object_msg-msg:object_age instead.")
  (object_age m))

(cl:ensure-generic-function 'object_class_age-val :lambda-list '(m))
(cl:defmethod object_class_age-val ((m <IbeoObject>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ibeo_object_msg-msg:object_class_age-val is deprecated.  Use ibeo_object_msg-msg:object_class_age instead.")
  (object_class_age m))

(cl:ensure-generic-function 'mobile-val :lambda-list '(m))
(cl:defmethod mobile-val ((m <IbeoObject>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ibeo_object_msg-msg:mobile-val is deprecated.  Use ibeo_object_msg-msg:mobile instead.")
  (mobile m))

(cl:ensure-generic-function 'motion_model_validated-val :lambda-list '(m))
(cl:defmethod motion_model_validated-val ((m <IbeoObject>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ibeo_object_msg-msg:motion_model_validated-val is deprecated.  Use ibeo_object_msg-msg:motion_model_validated instead.")
  (motion_model_validated m))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <IbeoObject>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ibeo_object_msg-msg:pose-val is deprecated.  Use ibeo_object_msg-msg:pose instead.")
  (pose m))

(cl:ensure-generic-function 'twist-val :lambda-list '(m))
(cl:defmethod twist-val ((m <IbeoObject>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ibeo_object_msg-msg:twist-val is deprecated.  Use ibeo_object_msg-msg:twist instead.")
  (twist m))

(cl:ensure-generic-function 'shape-val :lambda-list '(m))
(cl:defmethod shape-val ((m <IbeoObject>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ibeo_object_msg-msg:shape-val is deprecated.  Use ibeo_object_msg-msg:shape instead.")
  (shape m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <IbeoObject>) ostream)
  "Serializes a message object of type '<IbeoObject>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'object_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'object_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'object_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'object_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'object_class)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'object_class_uncertainty)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'object_class_uncertainty)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'object_age))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'object_class_age))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'mobile) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'motion_model_validated) 1 0)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'twist) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'shape) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <IbeoObject>) istream)
  "Deserializes a message object of type '<IbeoObject>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'object_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'object_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'object_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'object_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'object_class)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'object_class_uncertainty)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'object_class_uncertainty)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'object_age) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'object_class_age) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'mobile) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'motion_model_validated) (cl:not (cl:zerop (cl:read-byte istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'twist) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'shape) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<IbeoObject>)))
  "Returns string type for a message object of type '<IbeoObject>"
  "ibeo_object_msg/IbeoObject")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'IbeoObject)))
  "Returns string type for a message object of type 'IbeoObject"
  "ibeo_object_msg/IbeoObject")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<IbeoObject>)))
  "Returns md5sum for a message object of type '<IbeoObject>"
  "985d2c588d52776ee0b17d51f99bcf86")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'IbeoObject)))
  "Returns md5sum for a message object of type 'IbeoObject"
  "985d2c588d52776ee0b17d51f99bcf86")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<IbeoObject>)))
  "Returns full string definition for message of type '<IbeoObject>"
  (cl:format cl:nil "Header header~%uint32 object_id~%uint8 object_class~%uint16 object_class_uncertainty~%float32 object_age~%float32 object_class_age~%bool mobile~%bool motion_model_validated~%geometry_msgs/PoseWithCovariance pose~%geometry_msgs/TwistWithCovariance twist~%shape_msgs/SolidPrimitive shape~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseWithCovariance~%# This represents a pose in free space with uncertainty.~%~%Pose pose~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/TwistWithCovariance~%# This expresses velocity in free space with uncertainty.~%~%Twist twist~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: shape_msgs/SolidPrimitive~%# Define box, sphere, cylinder, cone ~%# All shapes are defined to have their bounding boxes centered around 0,0,0.~%~%uint8 BOX=1~%uint8 SPHERE=2~%uint8 CYLINDER=3~%uint8 CONE=4~%~%# The type of the shape~%uint8 type~%~%~%# The dimensions of the shape~%float64[] dimensions~%~%# The meaning of the shape dimensions: each constant defines the index in the 'dimensions' array~%~%# For the BOX type, the X, Y, and Z dimensions are the length of the corresponding~%# sides of the box.~%uint8 BOX_X=0~%uint8 BOX_Y=1~%uint8 BOX_Z=2~%~%~%# For the SPHERE type, only one component is used, and it gives the radius of~%# the sphere.~%uint8 SPHERE_RADIUS=0~%~%~%# For the CYLINDER and CONE types, the center line is oriented along~%# the Z axis.  Therefore the CYLINDER_HEIGHT (CONE_HEIGHT) component~%# of dimensions gives the height of the cylinder (cone).  The~%# CYLINDER_RADIUS (CONE_RADIUS) component of dimensions gives the~%# radius of the base of the cylinder (cone).  Cone and cylinder~%# primitives are defined to be circular. The tip of the cone is~%# pointing up, along +Z axis.~%~%uint8 CYLINDER_HEIGHT=0~%uint8 CYLINDER_RADIUS=1~%~%uint8 CONE_HEIGHT=0~%uint8 CONE_RADIUS=1~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'IbeoObject)))
  "Returns full string definition for message of type 'IbeoObject"
  (cl:format cl:nil "Header header~%uint32 object_id~%uint8 object_class~%uint16 object_class_uncertainty~%float32 object_age~%float32 object_class_age~%bool mobile~%bool motion_model_validated~%geometry_msgs/PoseWithCovariance pose~%geometry_msgs/TwistWithCovariance twist~%shape_msgs/SolidPrimitive shape~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseWithCovariance~%# This represents a pose in free space with uncertainty.~%~%Pose pose~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/TwistWithCovariance~%# This expresses velocity in free space with uncertainty.~%~%Twist twist~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: shape_msgs/SolidPrimitive~%# Define box, sphere, cylinder, cone ~%# All shapes are defined to have their bounding boxes centered around 0,0,0.~%~%uint8 BOX=1~%uint8 SPHERE=2~%uint8 CYLINDER=3~%uint8 CONE=4~%~%# The type of the shape~%uint8 type~%~%~%# The dimensions of the shape~%float64[] dimensions~%~%# The meaning of the shape dimensions: each constant defines the index in the 'dimensions' array~%~%# For the BOX type, the X, Y, and Z dimensions are the length of the corresponding~%# sides of the box.~%uint8 BOX_X=0~%uint8 BOX_Y=1~%uint8 BOX_Z=2~%~%~%# For the SPHERE type, only one component is used, and it gives the radius of~%# the sphere.~%uint8 SPHERE_RADIUS=0~%~%~%# For the CYLINDER and CONE types, the center line is oriented along~%# the Z axis.  Therefore the CYLINDER_HEIGHT (CONE_HEIGHT) component~%# of dimensions gives the height of the cylinder (cone).  The~%# CYLINDER_RADIUS (CONE_RADIUS) component of dimensions gives the~%# radius of the base of the cylinder (cone).  Cone and cylinder~%# primitives are defined to be circular. The tip of the cone is~%# pointing up, along +Z axis.~%~%uint8 CYLINDER_HEIGHT=0~%uint8 CYLINDER_RADIUS=1~%~%uint8 CONE_HEIGHT=0~%uint8 CONE_RADIUS=1~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <IbeoObject>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     1
     2
     4
     4
     1
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'twist))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'shape))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <IbeoObject>))
  "Converts a ROS message object to a list"
  (cl:list 'IbeoObject
    (cl:cons ':header (header msg))
    (cl:cons ':object_id (object_id msg))
    (cl:cons ':object_class (object_class msg))
    (cl:cons ':object_class_uncertainty (object_class_uncertainty msg))
    (cl:cons ':object_age (object_age msg))
    (cl:cons ':object_class_age (object_class_age msg))
    (cl:cons ':mobile (mobile msg))
    (cl:cons ':motion_model_validated (motion_model_validated msg))
    (cl:cons ':pose (pose msg))
    (cl:cons ':twist (twist msg))
    (cl:cons ':shape (shape msg))
))
