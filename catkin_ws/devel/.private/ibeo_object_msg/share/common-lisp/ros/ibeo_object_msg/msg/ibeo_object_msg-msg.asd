
(cl:in-package :asdf)

(defsystem "ibeo_object_msg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :shape_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "IbeoObject" :depends-on ("_package_IbeoObject"))
    (:file "_package_IbeoObject" :depends-on ("_package"))
  ))