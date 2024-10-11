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

class KMRStatus {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.charge_state_percentage = null;
      this.motion_enabled = null;
      this.warning_field_clear = null;
      this.safety_field_clear = null;
      this.safety_state_enabled = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('charge_state_percentage')) {
        this.charge_state_percentage = initObj.charge_state_percentage
      }
      else {
        this.charge_state_percentage = 0;
      }
      if (initObj.hasOwnProperty('motion_enabled')) {
        this.motion_enabled = initObj.motion_enabled
      }
      else {
        this.motion_enabled = false;
      }
      if (initObj.hasOwnProperty('warning_field_clear')) {
        this.warning_field_clear = initObj.warning_field_clear
      }
      else {
        this.warning_field_clear = false;
      }
      if (initObj.hasOwnProperty('safety_field_clear')) {
        this.safety_field_clear = initObj.safety_field_clear
      }
      else {
        this.safety_field_clear = false;
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
    // Serializes a message object of type KMRStatus
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [charge_state_percentage]
    bufferOffset = _serializer.int32(obj.charge_state_percentage, buffer, bufferOffset);
    // Serialize message field [motion_enabled]
    bufferOffset = _serializer.bool(obj.motion_enabled, buffer, bufferOffset);
    // Serialize message field [warning_field_clear]
    bufferOffset = _serializer.bool(obj.warning_field_clear, buffer, bufferOffset);
    // Serialize message field [safety_field_clear]
    bufferOffset = _serializer.bool(obj.safety_field_clear, buffer, bufferOffset);
    // Serialize message field [safety_state_enabled]
    bufferOffset = _serializer.bool(obj.safety_state_enabled, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type KMRStatus
    let len;
    let data = new KMRStatus(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [charge_state_percentage]
    data.charge_state_percentage = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [motion_enabled]
    data.motion_enabled = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [warning_field_clear]
    data.warning_field_clear = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [safety_field_clear]
    data.safety_field_clear = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [safety_state_enabled]
    data.safety_state_enabled = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kmriiwa_msgs/KMRStatus';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4a74515beaa6408bf61b8361cab81069';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    int32 charge_state_percentage
    bool motion_enabled
    bool warning_field_clear
    bool safety_field_clear
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
    const resolved = new KMRStatus(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.charge_state_percentage !== undefined) {
      resolved.charge_state_percentage = msg.charge_state_percentage;
    }
    else {
      resolved.charge_state_percentage = 0
    }

    if (msg.motion_enabled !== undefined) {
      resolved.motion_enabled = msg.motion_enabled;
    }
    else {
      resolved.motion_enabled = false
    }

    if (msg.warning_field_clear !== undefined) {
      resolved.warning_field_clear = msg.warning_field_clear;
    }
    else {
      resolved.warning_field_clear = false
    }

    if (msg.safety_field_clear !== undefined) {
      resolved.safety_field_clear = msg.safety_field_clear;
    }
    else {
      resolved.safety_field_clear = false
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

module.exports = KMRStatus;
