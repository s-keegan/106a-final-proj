; Auto-generated. Do not edit!


(cl:in-package lab4_cam-srv)


;//! \htmlinclude CentroidSrv-request.msg.html

(cl:defclass <CentroidSrv-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass CentroidSrv-request (<CentroidSrv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CentroidSrv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CentroidSrv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name lab4_cam-srv:<CentroidSrv-request> is deprecated: use lab4_cam-srv:CentroidSrv-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CentroidSrv-request>) ostream)
  "Serializes a message object of type '<CentroidSrv-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CentroidSrv-request>) istream)
  "Deserializes a message object of type '<CentroidSrv-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CentroidSrv-request>)))
  "Returns string type for a service object of type '<CentroidSrv-request>"
  "lab4_cam/CentroidSrvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CentroidSrv-request)))
  "Returns string type for a service object of type 'CentroidSrv-request"
  "lab4_cam/CentroidSrvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CentroidSrv-request>)))
  "Returns md5sum for a message object of type '<CentroidSrv-request>"
  "9eb286b1528e6357255e84ebb1efd9d1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CentroidSrv-request)))
  "Returns md5sum for a message object of type 'CentroidSrv-request"
  "9eb286b1528e6357255e84ebb1efd9d1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CentroidSrv-request>)))
  "Returns full string definition for message of type '<CentroidSrv-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CentroidSrv-request)))
  "Returns full string definition for message of type 'CentroidSrv-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CentroidSrv-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CentroidSrv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'CentroidSrv-request
))
;//! \htmlinclude CentroidSrv-response.msg.html

(cl:defclass <CentroidSrv-response> (roslisp-msg-protocol:ros-message)
  ((centroid
    :reader centroid
    :initarg :centroid
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point)))
)

(cl:defclass CentroidSrv-response (<CentroidSrv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CentroidSrv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CentroidSrv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name lab4_cam-srv:<CentroidSrv-response> is deprecated: use lab4_cam-srv:CentroidSrv-response instead.")))

(cl:ensure-generic-function 'centroid-val :lambda-list '(m))
(cl:defmethod centroid-val ((m <CentroidSrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lab4_cam-srv:centroid-val is deprecated.  Use lab4_cam-srv:centroid instead.")
  (centroid m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CentroidSrv-response>) ostream)
  "Serializes a message object of type '<CentroidSrv-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'centroid) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CentroidSrv-response>) istream)
  "Deserializes a message object of type '<CentroidSrv-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'centroid) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CentroidSrv-response>)))
  "Returns string type for a service object of type '<CentroidSrv-response>"
  "lab4_cam/CentroidSrvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CentroidSrv-response)))
  "Returns string type for a service object of type 'CentroidSrv-response"
  "lab4_cam/CentroidSrvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CentroidSrv-response>)))
  "Returns md5sum for a message object of type '<CentroidSrv-response>"
  "9eb286b1528e6357255e84ebb1efd9d1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CentroidSrv-response)))
  "Returns md5sum for a message object of type 'CentroidSrv-response"
  "9eb286b1528e6357255e84ebb1efd9d1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CentroidSrv-response>)))
  "Returns full string definition for message of type '<CentroidSrv-response>"
  (cl:format cl:nil "geometry_msgs/Point centroid~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CentroidSrv-response)))
  "Returns full string definition for message of type 'CentroidSrv-response"
  (cl:format cl:nil "geometry_msgs/Point centroid~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CentroidSrv-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'centroid))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CentroidSrv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'CentroidSrv-response
    (cl:cons ':centroid (centroid msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'CentroidSrv)))
  'CentroidSrv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'CentroidSrv)))
  'CentroidSrv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CentroidSrv)))
  "Returns string type for a service object of type '<CentroidSrv>"
  "lab4_cam/CentroidSrv")