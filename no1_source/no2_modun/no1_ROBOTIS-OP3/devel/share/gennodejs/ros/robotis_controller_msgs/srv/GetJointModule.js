// Auto-generated. Do not edit!

// (in-package robotis_controller_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class GetJointModuleRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.joint_name = null;
    }
    else {
      if (initObj.hasOwnProperty('joint_name')) {
        this.joint_name = initObj.joint_name
      }
      else {
        this.joint_name = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetJointModuleRequest
    // Serialize message field [joint_name]
    bufferOffset = _arraySerializer.string(obj.joint_name, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetJointModuleRequest
    let len;
    let data = new GetJointModuleRequest(null);
    // Deserialize message field [joint_name]
    data.joint_name = _arrayDeserializer.string(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.joint_name.forEach((val) => {
      length += 4 + _getByteLength(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'robotis_controller_msgs/GetJointModuleRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '01d1dab1fc816c24d1eda912a6b60345';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string[] joint_name
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetJointModuleRequest(null);
    if (msg.joint_name !== undefined) {
      resolved.joint_name = msg.joint_name;
    }
    else {
      resolved.joint_name = []
    }

    return resolved;
    }
};

class GetJointModuleResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.joint_name = null;
      this.module_name = null;
    }
    else {
      if (initObj.hasOwnProperty('joint_name')) {
        this.joint_name = initObj.joint_name
      }
      else {
        this.joint_name = [];
      }
      if (initObj.hasOwnProperty('module_name')) {
        this.module_name = initObj.module_name
      }
      else {
        this.module_name = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetJointModuleResponse
    // Serialize message field [joint_name]
    bufferOffset = _arraySerializer.string(obj.joint_name, buffer, bufferOffset, null);
    // Serialize message field [module_name]
    bufferOffset = _arraySerializer.string(obj.module_name, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetJointModuleResponse
    let len;
    let data = new GetJointModuleResponse(null);
    // Deserialize message field [joint_name]
    data.joint_name = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [module_name]
    data.module_name = _arrayDeserializer.string(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.joint_name.forEach((val) => {
      length += 4 + _getByteLength(val);
    });
    object.module_name.forEach((val) => {
      length += 4 + _getByteLength(val);
    });
    return length + 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'robotis_controller_msgs/GetJointModuleResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1f9dc32600ec95afe667839e777ecbdd';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string[] joint_name
    string[] module_name
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetJointModuleResponse(null);
    if (msg.joint_name !== undefined) {
      resolved.joint_name = msg.joint_name;
    }
    else {
      resolved.joint_name = []
    }

    if (msg.module_name !== undefined) {
      resolved.module_name = msg.module_name;
    }
    else {
      resolved.module_name = []
    }

    return resolved;
    }
};

module.exports = {
  Request: GetJointModuleRequest,
  Response: GetJointModuleResponse,
  md5sum() { return '55513368a0fa6f6cbd22bf1bfb152faa'; },
  datatype() { return 'robotis_controller_msgs/GetJointModule'; }
};
