; Auto-generated. Do not edit!


(cl:in-package waypoint-srv)


;//! \htmlinclude DeleteTrack-request.msg.html

(cl:defclass <DeleteTrack-request> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform ""))
)

(cl:defclass DeleteTrack-request (<DeleteTrack-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DeleteTrack-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DeleteTrack-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name waypoint-srv:<DeleteTrack-request> is deprecated: use waypoint-srv:DeleteTrack-request instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <DeleteTrack-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader waypoint-srv:name-val is deprecated.  Use waypoint-srv:name instead.")
  (name m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DeleteTrack-request>) ostream)
  "Serializes a message object of type '<DeleteTrack-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DeleteTrack-request>) istream)
  "Deserializes a message object of type '<DeleteTrack-request>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DeleteTrack-request>)))
  "Returns string type for a service object of type '<DeleteTrack-request>"
  "waypoint/DeleteTrackRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DeleteTrack-request)))
  "Returns string type for a service object of type 'DeleteTrack-request"
  "waypoint/DeleteTrackRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DeleteTrack-request>)))
  "Returns md5sum for a message object of type '<DeleteTrack-request>"
  "724f1e84629fbfec0fd754ee0937a6c2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DeleteTrack-request)))
  "Returns md5sum for a message object of type 'DeleteTrack-request"
  "724f1e84629fbfec0fd754ee0937a6c2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DeleteTrack-request>)))
  "Returns full string definition for message of type '<DeleteTrack-request>"
  (cl:format cl:nil "string name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DeleteTrack-request)))
  "Returns full string definition for message of type 'DeleteTrack-request"
  (cl:format cl:nil "string name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DeleteTrack-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DeleteTrack-request>))
  "Converts a ROS message object to a list"
  (cl:list 'DeleteTrack-request
    (cl:cons ':name (name msg))
))
;//! \htmlinclude DeleteTrack-response.msg.html

(cl:defclass <DeleteTrack-response> (roslisp-msg-protocol:ros-message)
  ((nameWasValid
    :reader nameWasValid
    :initarg :nameWasValid
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass DeleteTrack-response (<DeleteTrack-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DeleteTrack-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DeleteTrack-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name waypoint-srv:<DeleteTrack-response> is deprecated: use waypoint-srv:DeleteTrack-response instead.")))

(cl:ensure-generic-function 'nameWasValid-val :lambda-list '(m))
(cl:defmethod nameWasValid-val ((m <DeleteTrack-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader waypoint-srv:nameWasValid-val is deprecated.  Use waypoint-srv:nameWasValid instead.")
  (nameWasValid m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DeleteTrack-response>) ostream)
  "Serializes a message object of type '<DeleteTrack-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'nameWasValid) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DeleteTrack-response>) istream)
  "Deserializes a message object of type '<DeleteTrack-response>"
    (cl:setf (cl:slot-value msg 'nameWasValid) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DeleteTrack-response>)))
  "Returns string type for a service object of type '<DeleteTrack-response>"
  "waypoint/DeleteTrackResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DeleteTrack-response)))
  "Returns string type for a service object of type 'DeleteTrack-response"
  "waypoint/DeleteTrackResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DeleteTrack-response>)))
  "Returns md5sum for a message object of type '<DeleteTrack-response>"
  "724f1e84629fbfec0fd754ee0937a6c2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DeleteTrack-response)))
  "Returns md5sum for a message object of type 'DeleteTrack-response"
  "724f1e84629fbfec0fd754ee0937a6c2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DeleteTrack-response>)))
  "Returns full string definition for message of type '<DeleteTrack-response>"
  (cl:format cl:nil "bool nameWasValid~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DeleteTrack-response)))
  "Returns full string definition for message of type 'DeleteTrack-response"
  (cl:format cl:nil "bool nameWasValid~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DeleteTrack-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DeleteTrack-response>))
  "Converts a ROS message object to a list"
  (cl:list 'DeleteTrack-response
    (cl:cons ':nameWasValid (nameWasValid msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'DeleteTrack)))
  'DeleteTrack-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'DeleteTrack)))
  'DeleteTrack-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DeleteTrack)))
  "Returns string type for a service object of type '<DeleteTrack>"
  "waypoint/DeleteTrack")
