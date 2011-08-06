
(cl:in-package :asdf)

(defsystem "waypoint-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "GetTracks" :depends-on ("_package_GetTracks"))
    (:file "_package_GetTracks" :depends-on ("_package"))
  ))