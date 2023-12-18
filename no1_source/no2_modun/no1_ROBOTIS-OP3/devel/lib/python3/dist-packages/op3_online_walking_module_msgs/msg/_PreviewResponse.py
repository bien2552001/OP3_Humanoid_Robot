# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from op3_online_walking_module_msgs/PreviewResponse.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class PreviewResponse(genpy.Message):
  _md5sum = "485c4c72e8d24c7f7a770f8a88709eb6"
  _type = "op3_online_walking_module_msgs/PreviewResponse"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """int32 K_row
int32 K_col
float64[] K
int32 P_row
int32 P_col
float64[] P
"""
  __slots__ = ['K_row','K_col','K','P_row','P_col','P']
  _slot_types = ['int32','int32','float64[]','int32','int32','float64[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       K_row,K_col,K,P_row,P_col,P

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(PreviewResponse, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.K_row is None:
        self.K_row = 0
      if self.K_col is None:
        self.K_col = 0
      if self.K is None:
        self.K = []
      if self.P_row is None:
        self.P_row = 0
      if self.P_col is None:
        self.P_col = 0
      if self.P is None:
        self.P = []
    else:
      self.K_row = 0
      self.K_col = 0
      self.K = []
      self.P_row = 0
      self.P_col = 0
      self.P = []

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
      buff.write(_get_struct_2i().pack(_x.K_row, _x.K_col))
      length = len(self.K)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.Struct(pattern).pack(*self.K))
      _x = self
      buff.write(_get_struct_2i().pack(_x.P_row, _x.P_col))
      length = len(self.P)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.Struct(pattern).pack(*self.P))
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
      end = 0
      _x = self
      start = end
      end += 8
      (_x.K_row, _x.K_col,) = _get_struct_2i().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.K = s.unpack(str[start:end])
      _x = self
      start = end
      end += 8
      (_x.P_row, _x.P_col,) = _get_struct_2i().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.P = s.unpack(str[start:end])
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
      buff.write(_get_struct_2i().pack(_x.K_row, _x.K_col))
      length = len(self.K)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.K.tostring())
      _x = self
      buff.write(_get_struct_2i().pack(_x.P_row, _x.P_col))
      length = len(self.P)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.P.tostring())
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
      end = 0
      _x = self
      start = end
      end += 8
      (_x.K_row, _x.K_col,) = _get_struct_2i().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.K = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      _x = self
      start = end
      end += 8
      (_x.P_row, _x.P_col,) = _get_struct_2i().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.P = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_2i = None
def _get_struct_2i():
    global _struct_2i
    if _struct_2i is None:
        _struct_2i = struct.Struct("<2i")
    return _struct_2i