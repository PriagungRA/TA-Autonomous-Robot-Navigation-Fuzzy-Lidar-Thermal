
(cl:in-package :asdf)

(defsystem "main_controller-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ControllerData" :depends-on ("_package_ControllerData"))
    (:file "_package_ControllerData" :depends-on ("_package"))
  ))