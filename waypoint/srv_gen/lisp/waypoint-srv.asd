
(cl:in-package :asdf)

(defsystem "waypoint-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :waypoint-msg
)
  :components ((:file "_package")
    (:file "DeleteTrack" :depends-on ("_package_DeleteTrack"))
    (:file "_package_DeleteTrack" :depends-on ("_package"))
    (:file "GetTracks" :depends-on ("_package_GetTracks"))
    (:file "_package_GetTracks" :depends-on ("_package"))
    (:file "MarkWaypoint" :depends-on ("_package_MarkWaypoint"))
    (:file "_package_MarkWaypoint" :depends-on ("_package"))
    (:file "PlayTrack" :depends-on ("_package_PlayTrack"))
    (:file "_package_PlayTrack" :depends-on ("_package"))
    (:file "RecordTrack" :depends-on ("_package_RecordTrack"))
    (:file "_package_RecordTrack" :depends-on ("_package"))
  ))