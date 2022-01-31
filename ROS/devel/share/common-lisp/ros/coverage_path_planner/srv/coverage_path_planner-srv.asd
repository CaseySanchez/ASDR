
(cl:in-package :asdf)

(defsystem "coverage_path_planner-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :nav_msgs-msg
)
  :components ((:file "_package")
    (:file "make_plan" :depends-on ("_package_make_plan"))
    (:file "_package_make_plan" :depends-on ("_package"))
  ))