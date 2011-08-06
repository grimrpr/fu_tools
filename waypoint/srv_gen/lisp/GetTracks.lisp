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
  "2495adb8775bc7b61be81a65755a1d7e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetTracks-request)))
  "Returns md5sum for a message object of type 'GetTracks-request"
  "2495adb8775bc7b61be81a65755a1d7e")
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
  ((tracks
    :reader tracks
    :initarg :tracks
    :type (cl:vector waypoint-msg:Track)
   :initform (cl:make-array 0 :element-type 'waypoint-msg:Track :initial-element (cl:make-instance 'waypoint-msg:Track))))
)

(cl:defclass GetTracks-response (<GetTracks-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetTracks-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetTracks-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name waypoint-srv:<GetTracks-response> is deprecated: use waypoint-srv:GetTracks-response instead.")))

(cl:ensure-generic-function 'tracks-val :lambda-list '(m))
(cl:defmethod tracks-val ((m <GetTracks-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader waypoint-srv:tracks-val is deprecated.  Use waypoint-srv:tracks instead.")
  (tracks m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetTracks-response>) ostream)
  "Serializes a message object of type '<GetTracks-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'tracks))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'tracks))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetTracks-response>) istream)
  "Deserializes a message object of type '<GetTracks-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'tracks) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'tracks)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'waypoint-msg:Track))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
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
  "2495adb8775bc7b61be81a65755a1d7e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetTracks-response)))
  "Returns md5sum for a message object of type 'GetTracks-response"
  "2495adb8775bc7b61be81a65755a1d7e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetTracks-response>)))
  "Returns full string definition for message of type '<GetTracks-response>"
  (cl:format cl:nil "Track[] tracks~%~%~%================================================================================~%MSG: waypoint/Track~%string name~%int64 routeCount~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetTracks-response)))
  "Returns full string definition for message of type 'GetTracks-response"
  (cl:format cl:nil "Track[] tracks~%~%~%================================================================================~%MSG: waypoint/Track~%string name~%int64 routeCount~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetTracks-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'tracks) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetTracks-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetTracks-response
    (cl:cons ':tracks (tracks msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetTracks)))
  'GetTracks-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetTracks)))
  'GetTracks-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetTracks)))
  "Returns string type for a service object of type '<GetTracks>"
  "waypoint/GetTracks")
