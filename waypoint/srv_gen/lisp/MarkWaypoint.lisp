; Auto-generated. Do not edit!


(cl:in-package waypoint-srv)


;//! \htmlinclude MarkWaypoint-request.msg.html

(cl:defclass <MarkWaypoint-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass MarkWaypoint-request (<MarkWaypoint-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MarkWaypoint-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MarkWaypoint-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name waypoint-srv:<MarkWaypoint-request> is deprecated: use waypoint-srv:MarkWaypoint-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MarkWaypoint-request>) ostream)
  "Serializes a message object of type '<MarkWaypoint-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MarkWaypoint-request>) istream)
  "Deserializes a message object of type '<MarkWaypoint-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MarkWaypoint-request>)))
  "Returns string type for a service object of type '<MarkWaypoint-request>"
  "waypoint/MarkWaypointRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MarkWaypoint-request)))
  "Returns string type for a service object of type 'MarkWaypoint-request"
  "waypoint/MarkWaypointRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MarkWaypoint-request>)))
  "Returns md5sum for a message object of type '<MarkWaypoint-request>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MarkWaypoint-request)))
  "Returns md5sum for a message object of type 'MarkWaypoint-request"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MarkWaypoint-request>)))
  "Returns full string definition for message of type '<MarkWaypoint-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MarkWaypoint-request)))
  "Returns full string definition for message of type 'MarkWaypoint-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MarkWaypoint-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MarkWaypoint-request>))
  "Converts a ROS message object to a list"
  (cl:list 'MarkWaypoint-request
))
;//! \htmlinclude MarkWaypoint-response.msg.html

(cl:defclass <MarkWaypoint-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass MarkWaypoint-response (<MarkWaypoint-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MarkWaypoint-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MarkWaypoint-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name waypoint-srv:<MarkWaypoint-response> is deprecated: use waypoint-srv:MarkWaypoint-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MarkWaypoint-response>) ostream)
  "Serializes a message object of type '<MarkWaypoint-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MarkWaypoint-response>) istream)
  "Deserializes a message object of type '<MarkWaypoint-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MarkWaypoint-response>)))
  "Returns string type for a service object of type '<MarkWaypoint-response>"
  "waypoint/MarkWaypointResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MarkWaypoint-response)))
  "Returns string type for a service object of type 'MarkWaypoint-response"
  "waypoint/MarkWaypointResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MarkWaypoint-response>)))
  "Returns md5sum for a message object of type '<MarkWaypoint-response>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MarkWaypoint-response)))
  "Returns md5sum for a message object of type 'MarkWaypoint-response"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MarkWaypoint-response>)))
  "Returns full string definition for message of type '<MarkWaypoint-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MarkWaypoint-response)))
  "Returns full string definition for message of type 'MarkWaypoint-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MarkWaypoint-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MarkWaypoint-response>))
  "Converts a ROS message object to a list"
  (cl:list 'MarkWaypoint-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'MarkWaypoint)))
  'MarkWaypoint-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'MarkWaypoint)))
  'MarkWaypoint-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MarkWaypoint)))
  "Returns string type for a service object of type '<MarkWaypoint>"
  "waypoint/MarkWaypoint")
