// Auto-generated. Do not edit!

// (in-package ibeo_object_msg.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');
let shape_msgs = _finder('shape_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class IbeoObject {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.object_id = null;
      this.object_class = null;
      this.object_class_uncertainty = null;
      this.object_age = null;
      this.object_class_age = null;
      this.mobile = null;
      this.motion_model_validated = null;
      this.pose = null;
      this.twist = null;
      this.shape = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('object_id')) {
        this.object_id = initObj.object_id
      }
      else {
        this.object_id = 0;
      }
      if (initObj.hasOwnProperty('object_class')) {
        this.object_class = initObj.object_class
      }
      else {
        this.object_class = 0;
      }
      if (initObj.hasOwnProperty('object_class_uncertainty')) {
        this.object_class_uncertainty = initObj.object_class_uncertainty
      }
      else {
        this.object_class_uncertainty = 0;
      }
      if (initObj.hasOwnProperty('object_age')) {
        this.object_age = initObj.object_age
      }
      else {
        this.object_age = 0.0;
      }
      if (initObj.hasOwnProperty('object_class_age')) {
        this.object_class_age = initObj.object_class_age
      }
      else {
        this.object_class_age = 0.0;
      }
      if (initObj.hasOwnProperty('mobile')) {
        this.mobile = initObj.mobile
      }
      else {
        this.mobile = false;
      }
      if (initObj.hasOwnProperty('motion_model_validated')) {
        this.motion_model_validated = initObj.motion_model_validated
      }
      else {
        this.motion_model_validated = false;
      }
      if (initObj.hasOwnProperty('pose')) {
        this.pose = initObj.pose
      }
      else {
        this.pose = new geometry_msgs.msg.PoseWithCovariance();
      }
      if (initObj.hasOwnProperty('twist')) {
        this.twist = initObj.twist
      }
      else {
        this.twist = new geometry_msgs.msg.TwistWithCovariance();
      }
      if (initObj.hasOwnProperty('shape')) {
        this.shape = initObj.shape
      }
      else {
        this.shape = new shape_msgs.msg.SolidPrimitive();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type IbeoObject
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [object_id]
    bufferOffset = _serializer.uint32(obj.object_id, buffer, bufferOffset);
    // Serialize message field [object_class]
    bufferOffset = _serializer.uint8(obj.object_class, buffer, bufferOffset);
    // Serialize message field [object_class_uncertainty]
    bufferOffset = _serializer.uint16(obj.object_class_uncertainty, buffer, bufferOffset);
    // Serialize message field [object_age]
    bufferOffset = _serializer.float32(obj.object_age, buffer, bufferOffset);
    // Serialize message field [object_class_age]
    bufferOffset = _serializer.float32(obj.object_class_age, buffer, bufferOffset);
    // Serialize message field [mobile]
    bufferOffset = _serializer.bool(obj.mobile, buffer, bufferOffset);
    // Serialize message field [motion_model_validated]
    bufferOffset = _serializer.bool(obj.motion_model_validated, buffer, bufferOffset);
    // Serialize message field [pose]
    bufferOffset = geometry_msgs.msg.PoseWithCovariance.serialize(obj.pose, buffer, bufferOffset);
    // Serialize message field [twist]
    bufferOffset = geometry_msgs.msg.TwistWithCovariance.serialize(obj.twist, buffer, bufferOffset);
    // Serialize message field [shape]
    bufferOffset = shape_msgs.msg.SolidPrimitive.serialize(obj.shape, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type IbeoObject
    let len;
    let data = new IbeoObject(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [object_id]
    data.object_id = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [object_class]
    data.object_class = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [object_class_uncertainty]
    data.object_class_uncertainty = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [object_age]
    data.object_age = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [object_class_age]
    data.object_class_age = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [mobile]
    data.mobile = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [motion_model_validated]
    data.motion_model_validated = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [pose]
    data.pose = geometry_msgs.msg.PoseWithCovariance.deserialize(buffer, bufferOffset);
    // Deserialize message field [twist]
    data.twist = geometry_msgs.msg.TwistWithCovariance.deserialize(buffer, bufferOffset);
    // Deserialize message field [shape]
    data.shape = shape_msgs.msg.SolidPrimitive.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += shape_msgs.msg.SolidPrimitive.getMessageSize(object.shape);
    return length + 697;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ibeo_object_msg/IbeoObject';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '985d2c588d52776ee0b17d51f99bcf86';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new IbeoObject(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.object_id !== undefined) {
      resolved.object_id = msg.object_id;
    }
    else {
      resolved.object_id = 0
    }

    if (msg.object_class !== undefined) {
      resolved.object_class = msg.object_class;
    }
    else {
      resolved.object_class = 0
    }

    if (msg.object_class_uncertainty !== undefined) {
      resolved.object_class_uncertainty = msg.object_class_uncertainty;
    }
    else {
      resolved.object_class_uncertainty = 0
    }

    if (msg.object_age !== undefined) {
      resolved.object_age = msg.object_age;
    }
    else {
      resolved.object_age = 0.0
    }

    if (msg.object_class_age !== undefined) {
      resolved.object_class_age = msg.object_class_age;
    }
    else {
      resolved.object_class_age = 0.0
    }

    if (msg.mobile !== undefined) {
      resolved.mobile = msg.mobile;
    }
    else {
      resolved.mobile = false
    }

    if (msg.motion_model_validated !== undefined) {
      resolved.motion_model_validated = msg.motion_model_validated;
    }
    else {
      resolved.motion_model_validated = false
    }

    if (msg.pose !== undefined) {
      resolved.pose = geometry_msgs.msg.PoseWithCovariance.Resolve(msg.pose)
    }
    else {
      resolved.pose = new geometry_msgs.msg.PoseWithCovariance()
    }

    if (msg.twist !== undefined) {
      resolved.twist = geometry_msgs.msg.TwistWithCovariance.Resolve(msg.twist)
    }
    else {
      resolved.twist = new geometry_msgs.msg.TwistWithCovariance()
    }

    if (msg.shape !== undefined) {
      resolved.shape = shape_msgs.msg.SolidPrimitive.Resolve(msg.shape)
    }
    else {
      resolved.shape = new shape_msgs.msg.SolidPrimitive()
    }

    return resolved;
    }
};

module.exports = IbeoObject;
