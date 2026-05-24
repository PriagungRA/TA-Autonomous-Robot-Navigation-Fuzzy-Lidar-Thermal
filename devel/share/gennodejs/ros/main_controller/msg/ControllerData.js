// Auto-generated. Do not edit!

// (in-package main_controller.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class ControllerData {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.StatusControl = null;
      this.data = null;
    }
    else {
      if (initObj.hasOwnProperty('StatusControl')) {
        this.StatusControl = initObj.StatusControl
      }
      else {
        this.StatusControl = 0;
      }
      if (initObj.hasOwnProperty('data')) {
        this.data = initObj.data
      }
      else {
        this.data = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ControllerData
    // Serialize message field [StatusControl]
    bufferOffset = _serializer.int16(obj.StatusControl, buffer, bufferOffset);
    // Serialize message field [data]
    bufferOffset = _arraySerializer.int16(obj.data, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ControllerData
    let len;
    let data = new ControllerData(null);
    // Deserialize message field [StatusControl]
    data.StatusControl = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [data]
    data.data = _arrayDeserializer.int16(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 2 * object.data.length;
    return length + 6;
  }

  static datatype() {
    // Returns string type for a message object
    return 'main_controller/ControllerData';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '244e1cda6d3e37f787f007f1bc662fea';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int16 StatusControl
    int16[] data
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ControllerData(null);
    if (msg.StatusControl !== undefined) {
      resolved.StatusControl = msg.StatusControl;
    }
    else {
      resolved.StatusControl = 0
    }

    if (msg.data !== undefined) {
      resolved.data = msg.data;
    }
    else {
      resolved.data = []
    }

    return resolved;
    }
};

module.exports = ControllerData;
