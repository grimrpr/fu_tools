; Auto-generated. Do not edit!


(cl:in-package waypoint-srv)


;//! \htmlinclude GetTracks-request.msg.html

(cl:defclass <GetTracks-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetTracks-request (<GetTracks-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetTracks-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetTracks-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name waypoint-srv:<GetTracks-request> is deprecated: use waypoint-srv:GetTracks-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetTracks-request>) ostream)
  "Serializes a message object of type '<GetTracks-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetTracks-request>) istream)
  "Deserializes a message object of type '<GetTracks-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetTracks-request>)))
  "Returns string type for a service object of type '<GetTracks-request>"
  "waypoint/GetTracksRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetTracks-request)))
  "Returns string type for a service object of type 'GetTracks-request"
  "waypoint/GetTracksRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetTracks-request>)))
  "Returns md5sum for a message object of type '<GetTracks-request>"
  "c1f3d28f1b044c871e6eff2e9fc3c667")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetTracks-request)))
  "Returns md5sum for a message object of type 'GetTracks-request"
  "c1f3d28f1b044c871e6eff2e9fc3c667")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetTracks-request>)))
  "Returns full string definition for message of type '<GetTracks-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetTracks-request)))
  "Returns full string definition for message of type 'GetTracks-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetTracks-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetTracks-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetTracks-request
))
;//! \htmlinclude GetTracks-response.msg.html

(cl:defclass <GetTracks-response> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform ""))
)

(cl:defclass GetTracks-response (<GetTracks-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetTracks-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetTracks-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name waypoint-srv:<GetTracks-response> is deprecated: use waypoint-srv:GetTracks-response instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <GetTracks-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader waypoint-srv:name-val is deprecated.  Use waypoint-srv:name instead.")
  (name m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetTracks-response>) ostream)
  "Serializes a message object of type '<GetTracks-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetTracks-response>) istream)
  "Deserializes a message object of type '<GetTracks-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetTracks-response>)))
  "Returns string type for a service object of type '<GetTracks-response>"
  "waypoint/GetTracksResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetTracks-response)))
  "Returns string type for a service object of type 'GetTracks-response"
  "waypoint/GetTracksResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetTracks-response>)))
  "Returns md5sum for a message object of type '<GetTracks-response>"
  "c1f3d28f1b044c871e6eff2e9fc3c667")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetTracks-response)))
  "Returns md5sum for a message object of type 'GetTracks-response"
  "c1f3d28f1b044c871e6eff2e9fc3c667")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetTracks-response>)))
  "Returns full string definition for message of type '<GetTracks-response>"
  (cl:format cl:nil "string name~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetTracks-response)))
  "Returns full string definition for message of type 'GetTracks-response"
  (cl:format cl:nil "string name~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetTracks-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetTracks-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetTracks-response
    (cl:cons ':name (name msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetTracks)))
  'GetTracks-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetTracks)))
  'GetTracks-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetTracks)))
  "Returns string type for a service object of type '<GetTracks>"
  "waypoint/GetTracks")
