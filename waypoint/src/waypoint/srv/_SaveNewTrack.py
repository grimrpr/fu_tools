"""autogenerated by genmsg_py from SaveNewTrackRequest.msg. Do not edit."""
import roslib.message
import struct

import geometry_msgs.msg
import nav_msgs.msg
import waypoint.msg
import std_msgs.msg

class SaveNewTrackRequest(roslib.message.Message):
  _md5sum = "ad975203ca82e8bdcc9be50e241c3487"
  _type = "waypoint/SaveNewTrackRequest"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """Track track

================================================================================
MSG: waypoint/Track
string name
string time
nav_msgs/Path path

================================================================================
MSG: nav_msgs/Path
#An array of poses that represents a Path for a robot to follow
Header header
geometry_msgs/PoseStamped[] poses

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of postion and orientation. 
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

"""
  __slots__ = ['track']
  _slot_types = ['waypoint/Track']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       track
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(SaveNewTrackRequest, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.track is None:
        self.track = waypoint.msg.Track()
    else:
      self.track = waypoint.msg.Track()

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    @param buff: buffer
    @type  buff: StringIO
    """
    try:
      _x = self.track.name
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.track.time
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_3I.pack(_x.track.path.header.seq, _x.track.path.header.stamp.secs, _x.track.path.header.stamp.nsecs))
      _x = self.track.path.header.frame_id
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      length = len(self.track.path.poses)
      buff.write(_struct_I.pack(length))
      for val1 in self.track.path.poses:
        _v1 = val1.header
        buff.write(_struct_I.pack(_v1.seq))
        _v2 = _v1.stamp
        _x = _v2
        buff.write(_struct_2I.pack(_x.secs, _x.nsecs))
        _x = _v1.frame_id
        length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _v3 = val1.pose
        _v4 = _v3.position
        _x = _v4
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        _v5 = _v3.orientation
        _x = _v5
        buff.write(_struct_4d.pack(_x.x, _x.y, _x.z, _x.w))
    except struct.error, se: self._check_types(se)
    except TypeError, te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    @param str: byte array of serialized message
    @type  str: str
    """
    try:
      if self.track is None:
        self.track = waypoint.msg.Track()
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.track.name = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.track.time = str[start:end]
      _x = self
      start = end
      end += 12
      (_x.track.path.header.seq, _x.track.path.header.stamp.secs, _x.track.path.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.track.path.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.track.path.poses = []
      for i in xrange(0, length):
        val1 = geometry_msgs.msg.PoseStamped()
        _v6 = val1.header
        start = end
        end += 4
        (_v6.seq,) = _struct_I.unpack(str[start:end])
        _v7 = _v6.stamp
        _x = _v7
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _struct_2I.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        _v6.frame_id = str[start:end]
        _v8 = val1.pose
        _v9 = _v8.position
        _x = _v9
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        _v10 = _v8.orientation
        _x = _v10
        start = end
        end += 32
        (_x.x, _x.y, _x.z, _x.w,) = _struct_4d.unpack(str[start:end])
        self.track.path.poses.append(val1)
      return self
    except struct.error, e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    @param buff: buffer
    @type  buff: StringIO
    @param numpy: numpy python module
    @type  numpy module
    """
    try:
      _x = self.track.name
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.track.time
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_3I.pack(_x.track.path.header.seq, _x.track.path.header.stamp.secs, _x.track.path.header.stamp.nsecs))
      _x = self.track.path.header.frame_id
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      length = len(self.track.path.poses)
      buff.write(_struct_I.pack(length))
      for val1 in self.track.path.poses:
        _v11 = val1.header
        buff.write(_struct_I.pack(_v11.seq))
        _v12 = _v11.stamp
        _x = _v12
        buff.write(_struct_2I.pack(_x.secs, _x.nsecs))
        _x = _v11.frame_id
        length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _v13 = val1.pose
        _v14 = _v13.position
        _x = _v14
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        _v15 = _v13.orientation
        _x = _v15
        buff.write(_struct_4d.pack(_x.x, _x.y, _x.z, _x.w))
    except struct.error, se: self._check_types(se)
    except TypeError, te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    @param str: byte array of serialized message
    @type  str: str
    @param numpy: numpy python module
    @type  numpy: module
    """
    try:
      if self.track is None:
        self.track = waypoint.msg.Track()
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.track.name = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.track.time = str[start:end]
      _x = self
      start = end
      end += 12
      (_x.track.path.header.seq, _x.track.path.header.stamp.secs, _x.track.path.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.track.path.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.track.path.poses = []
      for i in xrange(0, length):
        val1 = geometry_msgs.msg.PoseStamped()
        _v16 = val1.header
        start = end
        end += 4
        (_v16.seq,) = _struct_I.unpack(str[start:end])
        _v17 = _v16.stamp
        _x = _v17
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _struct_2I.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        _v16.frame_id = str[start:end]
        _v18 = val1.pose
        _v19 = _v18.position
        _x = _v19
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        _v20 = _v18.orientation
        _x = _v20
        start = end
        end += 32
        (_x.x, _x.y, _x.z, _x.w,) = _struct_4d.unpack(str[start:end])
        self.track.path.poses.append(val1)
      return self
    except struct.error, e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I
_struct_4d = struct.Struct("<4d")
_struct_3I = struct.Struct("<3I")
_struct_2I = struct.Struct("<2I")
_struct_3d = struct.Struct("<3d")
"""autogenerated by genmsg_py from SaveNewTrackResponse.msg. Do not edit."""
import roslib.message
import struct


class SaveNewTrackResponse(roslib.message.Message):
  _md5sum = "6ccdb752b565be6d250259ff3d0c7ea1"
  _type = "waypoint/SaveNewTrackResponse"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """bool successful


"""
  __slots__ = ['successful']
  _slot_types = ['bool']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       successful
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(SaveNewTrackResponse, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.successful is None:
        self.successful = False
    else:
      self.successful = False

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    @param buff: buffer
    @type  buff: StringIO
    """
    try:
      buff.write(_struct_B.pack(self.successful))
    except struct.error, se: self._check_types(se)
    except TypeError, te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    @param str: byte array of serialized message
    @type  str: str
    """
    try:
      end = 0
      start = end
      end += 1
      (self.successful,) = _struct_B.unpack(str[start:end])
      self.successful = bool(self.successful)
      return self
    except struct.error, e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    @param buff: buffer
    @type  buff: StringIO
    @param numpy: numpy python module
    @type  numpy module
    """
    try:
      buff.write(_struct_B.pack(self.successful))
    except struct.error, se: self._check_types(se)
    except TypeError, te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    @param str: byte array of serialized message
    @type  str: str
    @param numpy: numpy python module
    @type  numpy: module
    """
    try:
      end = 0
      start = end
      end += 1
      (self.successful,) = _struct_B.unpack(str[start:end])
      self.successful = bool(self.successful)
      return self
    except struct.error, e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I
_struct_B = struct.Struct("<B")
class SaveNewTrack(roslib.message.ServiceDefinition):
  _type          = 'waypoint/SaveNewTrack'
  _md5sum = '73fcb0a6fcef35d4fd64468bbfacbc7a'
  _request_class  = SaveNewTrackRequest
  _response_class = SaveNewTrackResponse