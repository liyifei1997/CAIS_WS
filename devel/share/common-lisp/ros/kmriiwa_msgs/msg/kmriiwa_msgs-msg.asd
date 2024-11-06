
(cl:in-package :asdf)

(defsystem "kmriiwa_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "JointPosition" :depends-on ("_package_JointPosition"))
    (:file "_package_JointPosition" :depends-on ("_package"))
    (:file "KMRStatus" :depends-on ("_package_KMRStatus"))
    (:file "_package_KMRStatus" :depends-on ("_package"))
    (:file "LBRStatus" :depends-on ("_package_LBRStatus"))
    (:file "_package_LBRStatus" :depends-on ("_package"))
  ))