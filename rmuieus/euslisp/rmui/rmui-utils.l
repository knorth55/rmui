
(defun set-rmui-parameters (rmui-names)
  (defvar *device-namespaces*
    (mapcar #'(lambda (rmui-name)
                (format nil "/~A/rmui_node" rmui-name))
            rmui-names))
  (defvar *device-frame-ids*
    (mapcar #'(lambda (rmui-name)
                (format nil "~A_link" rmui-name))
            rmui-names))
  (defvar *device-groupnames*
    (mapcar #'(lambda (rmui-name)
                (format nil "~A_device" rmui-name))
            rmui-names))
  (defvar *device-world-frame-ids*
    (mapcar #'(lambda (rmui-name)
                (format nil "~A_world" rmui-name))
            rmui-names)))


(provide :rmui-utils "rmui-utils.l")
