// Auto-generated. Do not edit!

// (in-package kmriiwa_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class LBRStatus {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.motion_enabled = null;
      this.axes_mastered = null;
      this.axes_gms_referenced = null;
      this.axes_position_referenced = null;
      this.safety_state_enabled = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('motion_enabled')) {
        this.motion_enabled = initObj.motion_enabled
      }
      else {
        this.motion_enabled = false;
      }
      if (initObj.hasOwnProperty('axes_mastered')) {
        this.axes_mastered = initObj.axes_mastered
      }
      else {
        this.axes_mastered = false;
      }
      if (initObj.hasOwnProperty('axes_gms_referenced')) {
        this.axes_gms_referenced = initObj.axes_gms_referenced
      }
      else {
        this.axes_gms_referenced = false;
      }
      if (initObj.hasOwnProperty('axes_position_referenced')) {
        this.axes_position_referenced = initObj.axes_position_referenced
      }
      else {
        this.axes_position_referenced = false;
      }
      if (initObj.hasOwnProperty('safety_state_enabled')) {
        this.safety_state_enabled = initObj.safety_state_enabled
      }
      else {
        this.safety_state_enabled = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type LBRStatus
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [motion_enabled]
    bufferOffset = _serializer.bool(obj.motion_enabled, buffer, bufferOffset);
    // Serialize message field [axes_mastered]
    bufferOffset = _serializer.bool(obj.axes_mastered, buffer, bufferOffset);
    // Serialize message field [axes_gms_referenced]
    bufferOffset = _serializer.bool(obj.axes_gms_referenced, buffer, bufferOffset);
    // Serialize message field [axes_position_referenced]
    bufferOffset = _serializer.bool(obj.axes_position_referenced, buffer, bufferOffset);
    // Serialize message field [safety_state_enabled]
    bufferOffset = _serializer.bool(obj.safety_state_enabled, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type LBRStatus
    let len;
    let data = new LBRStatus(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [motion_enabled]
    data.motion_enabled = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [axes_mastered]
    data.axes_mastered = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [axes_gms_referenced]
    data.axes_gms_referenced = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [axes_position_referenced]
    data.axes_position_referenced = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [safety_state_enabled]
    data.safety_state_enabled = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 5;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kmriiwa_msgs/LBRStatus';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd1e9bf004da750115463ceb70e37bbcd';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    bool motion_enabled
    bool axes_mastered
    bool axes_gms_referenced
    bool axes_position_referenced
    bool safety_state_enabled
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new LBRStatus(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.motion_enabled !== undefined) {
      resolved.motion_enabled = msg.motion_enabled;
    }
    else {
      resolved.motion_enabled = false
    }

    if (msg.axes_mastered !== undefined) {
      resolved.axes_mastered = msg.axes_mastered;
    }
    else {
      resolved.axes_mastered = false
    }

    if (msg.axes_gms_referenced !== undefined) {
      resolved.axes_gms_referenced = msg.axes_gms_referenced;
    }
    else {
      resolved.axes_gms_referenced = false
    }

    if (msg.axes_position_referenced !== undefined) {
      resolved.axes_position_referenced = msg.axes_position_referenced;
    }
    else {
      resolved.axes_position_referenced = false
    }

    if (msg.safety_state_enabled !== undefined) {
      resolved.safety_state_enabled = msg.safety_state_enabled;
    }
    else {
      resolved.safety_state_enabled = false
    }

    return resolved;
    }
};

module.exports = LBRStatus;
