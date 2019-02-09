// Auto-generated. Do not edit!

// (in-package topic_tools.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class MuxSelectRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.topic = null;
    }
    else {
      if (initObj.hasOwnProperty('topic')) {
        this.topic = initObj.topic
      }
      else {
        this.topic = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MuxSelectRequest
    // Serialize message field [topic]
    bufferOffset = _serializer.string(obj.topic, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MuxSelectRequest
    let len;
    let data = new MuxSelectRequest(null);
    // Deserialize message field [topic]
    data.topic = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.topic.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'topic_tools/MuxSelectRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd8f94bae31b356b24d0427f80426d0c3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string topic
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MuxSelectRequest(null);
    if (msg.topic !== undefined) {
      resolved.topic = msg.topic;
    }
    else {
      resolved.topic = ''
    }

    return resolved;
    }
};

class MuxSelectResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.prev_topic = null;
    }
    else {
      if (initObj.hasOwnProperty('prev_topic')) {
        this.prev_topic = initObj.prev_topic
      }
      else {
        this.prev_topic = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MuxSelectResponse
    // Serialize message field [prev_topic]
    bufferOffset = _serializer.string(obj.prev_topic, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MuxSelectResponse
    let len;
    let data = new MuxSelectResponse(null);
    // Deserialize message field [prev_topic]
    data.prev_topic = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.prev_topic.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'topic_tools/MuxSelectResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3db0a473debdbafea387c9e49358c320';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string prev_topic
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MuxSelectResponse(null);
    if (msg.prev_topic !== undefined) {
      resolved.prev_topic = msg.prev_topic;
    }
    else {
      resolved.prev_topic = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: MuxSelectRequest,
  Response: MuxSelectResponse,
  md5sum() { return '053052240ca985e1f2eedbb0dae9b1f7'; },
  datatype() { return 'topic_tools/MuxSelect'; }
};
