
(cl:in-package :asdf)

(defsystem "lab4_cam-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :sensor_msgs-msg
)
  :components ((:file "_package")
    (:file "ImageSrv" :depends-on ("_package_ImageSrv"))
    (:file "_package_ImageSrv" :depends-on ("_package"))
    (:file "CentroidSrv" :depends-on ("_package_CentroidSrv"))
    (:file "_package_CentroidSrv" :depends-on ("_package"))
    (:file "CamInfoSrv" :depends-on ("_package_CamInfoSrv"))
    (:file "_package_CamInfoSrv" :depends-on ("_package"))
  ))