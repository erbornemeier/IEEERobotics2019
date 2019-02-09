// Auto-generated. Do not edit!

// (in-package roscpp.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class SetLoggerLevelRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.logger = null;
      this.level = null;
    }
    else {
      if (initObj.hasOwnProperty('logger')) {
        this.logger = initObj.logger
      }
      else {
        this.logger = '';
      }
      if (initObj.hasOwnProperty('level')) {
        this.level = initObj.level
      }
      else {
        this.level = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetLoggerLevelRequest
    // Serialize message field [logger]
    bufferOffset = _serializer.string(obj.logger, buffer, bufferOffset);
    // Serialize message field [level]
    bufferOffset = _serializer.string(obj.level, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetLoggerLevelRequest
    let len;
    let data = new SetLoggerLevelRequest(null);
    // Deserialize message field [logger]
    data.logger = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [level]
    data.level = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.logger.length;
    length += object.level.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'roscpp/SetLoggerLevelRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '51da076440d78ca1684d36c868df61ea';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string logger
    string level
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetLoggerLevelRequest(null);
    if (msg.logger !== undefined) {
      resolved.logger = msg.logger;
    }
    else {
      resolved.logger = ''
    }

    if (msg.level !== undefined) {
      resolved.level = msg.level;
    }
    else {
      resolved.level = ''
    }

    return resolved;
    }
};

class SetLoggerLevelResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetLoggerLevelResponse
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetLoggerLevelResponse
    let len;
    let data = new SetLoggerLevelResponse(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'roscpp/SetLoggerLevelResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetLoggerLevelResponse(null);
    return resolved;
    }
};

module.exports = {
  Request: SetLoggerLevelRequest,
  Response: SetLoggerLevelResponse,
  md5sum() { return '51da076440d78ca1684d36c868df61ea'; },
  datatype() { return 'roscpp/SetLoggerLevel'; }
};
