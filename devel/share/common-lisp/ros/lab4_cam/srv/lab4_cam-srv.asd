
(cl:in-package :asdf)

(defsystem "lab4_cam-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :sensor_msgs-msg
)
  :components ((:file "_package")
    (:file "ImageSrv" :depends-on ("_package_ImageSrv"))
    (:file "_package_ImageSrv" :depends-on ("_package"))
    (:file "CamInfoSrv" :depends-on ("_package_CamInfoSrv"))
    (:file "_package_CamInfoSrv" :depends-on ("_package"))
  ))