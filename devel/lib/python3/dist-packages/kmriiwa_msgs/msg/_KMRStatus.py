# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from kmriiwa_msgs/KMRStatus.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import std_msgs.msg

class KMRStatus(genpy.Message):
  _md5sum = "4a74515beaa6408bf61b8361cab81069"
  _type = "kmriiwa_msgs/KMRStatus"
  _has_header = True  # flag to mark the presence of a Header object
  _full_text = """Header header
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
"""
  __slots__ = ['header','charge_state_percentage','motion_enabled','warning_field_clear','safety_field_clear','safety_state_enabled']
  _slot_types = ['std_msgs/Header','int32','bool','bool','bool','bool']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,charge_state_percentage,motion_enabled,warning_field_clear,safety_field_clear,safety_state_enabled

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(KMRStatus, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.charge_state_percentage is None:
        self.charge_state_percentage = 0
      if self.motion_enabled is None:
        self.motion_enabled = False
      if self.warning_field_clear is None:
        self.warning_field_clear = False
      if self.safety_field_clear is None:
        self.safety_field_clear = False
      if self.safety_state_enabled is None:
        self.safety_state_enabled = False
    else:
      self.header = std_msgs.msg.Header()
      self.charge_state_percentage = 0
      self.motion_enabled = False
      self.warning_field_clear = False
      self.safety_field_clear = False
      self.safety_state_enabled = False

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_i4B().pack(_x.charge_state_percentage, _x.motion_enabled, _x.warning_field_clear, _x.safety_field_clear, _x.safety_state_enabled))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 8
      (_x.charge_state_percentage, _x.motion_enabled, _x.warning_field_clear, _x.safety_field_clear, _x.safety_state_enabled,) = _get_struct_i4B().unpack(str[start:end])
      self.motion_enabled = bool(self.motion_enabled)
      self.warning_field_clear = bool(self.warning_field_clear)
      self.safety_field_clear = bool(self.safety_field_clear)
      self.safety_state_enabled = bool(self.safety_state_enabled)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_i4B().pack(_x.charge_state_percentage, _x.motion_enabled, _x.warning_field_clear, _x.safety_field_clear, _x.safety_state_enabled))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 8
      (_x.charge_state_percentage, _x.motion_enabled, _x.warning_field_clear, _x.safety_field_clear, _x.safety_state_enabled,) = _get_struct_i4B().unpack(str[start:end])
      self.motion_enabled = bool(self.motion_enabled)
      self.warning_field_clear = bool(self.warning_field_clear)
      self.safety_field_clear = bool(self.safety_field_clear)
      self.safety_state_enabled = bool(self.safety_state_enabled)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_3I = None
def _get_struct_3I():
    global _struct_3I
    if _struct_3I is None:
        _struct_3I = struct.Struct("<3I")
    return _struct_3I
_struct_i4B = None
def _get_struct_i4B():
    global _struct_i4B
    if _struct_i4B is None:
        _struct_i4B = struct.Struct("<i4B")
    return _struct_i4B
