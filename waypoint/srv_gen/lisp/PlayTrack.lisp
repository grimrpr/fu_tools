; Auto-generated. Do not edit!


(cl:in-package waypoint-srv)


;//! \htmlinclude PlayTrack-request.msg.html

(cl:defclass <PlayTrack-request> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform ""))
)

(cl:defclass PlayTrack-request (<PlayTrack-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PlayTrack-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PlayTrack-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name waypoint-srv:<PlayTrack-request> is deprecated: use waypoint-srv:PlayTrack-request instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <PlayTrack-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader waypoint-srv:name-val is deprecated.  Use waypoint-srv:name instead.")
  (name m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PlayTrack-request>) ostream)
  "Serializes a message object of type '<PlayTrack-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PlayTrack-request>) istream)
  "Deserializes a message object of type '<PlayTrack-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PlayTrack-request>)))
  "Returns string type for a service object of type '<PlayTrack-request>"
  "waypoint/PlayTrackRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlayTrack-request)))
  "Returns string type for a service object of type 'PlayTrack-request"
  "waypoint/PlayTrackRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PlayTrack-request>)))
  "Returns md5sum for a message object of type '<PlayTrack-request>"
  "724f1e84629fbfec0fd754ee0937a6c2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PlayTrack-request)))
  "Returns md5sum for a message object of type 'PlayTrack-request"
  "724f1e84629fbfec0fd754ee0937a6c2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PlayTrack-request>)))
  "Returns full string definition for message of type '<PlayTrack-request>"
  (cl:format cl:nil "string name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PlayTrack-request)))
  "Returns full string definition for message of type 'PlayTrack-request"
  (cl:format cl:nil "string name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PlayTrack-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PlayTrack-request>))
  "Converts a ROS message object to a list"
  (cl:list 'PlayTrack-request
    (cl:cons ':name (name msg))
))
;//! \htmlinclude PlayTrack-response.msg.html

(cl:defclass <PlayTrack-response> (roslisp-msg-protocol:ros-message)
  ((nameWasValid
    :reader nameWasValid
    :initarg :nameWasValid
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass PlayTrack-response (<PlayTrack-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PlayTrack-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PlayTrack-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name waypoint-srv:<PlayTrack-response> is deprecated: use waypoint-srv:PlayTrack-response instead.")))

(cl:ensure-generic-function 'nameWasValid-val :lambda-list '(m))
(cl:defmethod nameWasValid-val ((m <PlayTrack-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader waypoint-srv:nameWasValid-val is deprecated.  Use waypoint-srv:nameWasValid instead.")
  (nameWasValid m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PlayTrack-response>) ostream)
  "Serializes a message object of type '<PlayTrack-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'nameWasValid) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PlayTrack-response>) istream)
  "Deserializes a message object of type '<PlayTrack-response>"
    (cl:setf (cl:slot-value msg 'nameWasValid) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PlayTrack-response>)))
  "Returns string type for a service object of type '<PlayTrack-response>"
  "waypoint/PlayTrackResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlayTrack-response)))
  "Returns string type for a service object of type 'PlayTrack-response"
  "waypoint/PlayTrackResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PlayTrack-response>)))
  "Returns md5sum for a message object of type '<PlayTrack-response>"
  "724f1e84629fbfec0fd754ee0937a6c2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PlayTrack-response)))
  "Returns md5sum for a message object of type 'PlayTrack-response"
  "724f1e84629fbfec0fd754ee0937a6c2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PlayTrack-response>)))
  "Returns full string definition for message of type '<PlayTrack-response>"
  (cl:format cl:nil "bool nameWasValid~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PlayTrack-response)))
  "Returns full string definition for message of type 'PlayTrack-response"
  (cl:format cl:nil "bool nameWasValid~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PlayTrack-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PlayTrack-response>))
  "Converts a ROS message object to a list"
  (cl:list 'PlayTrack-response
    (cl:cons ':nameWasValid (nameWasValid msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'PlayTrack)))
  'PlayTrack-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'PlayTrack)))
  'PlayTrack-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlayTrack)))
  "Returns string type for a service object of type '<PlayTrack>"
  "waypoint/PlayTrack")
