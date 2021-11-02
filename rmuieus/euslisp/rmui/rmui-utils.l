
(defun set-rmui-parameters (rmui-name)
  (defvar *namespace* (format nil "/~A/rmui_node" rmui-name))
  (defvar *device-frame-id* (format nil "~A_link" rmui-name))
  (defvar *device-groupname* (format nil "~A_device" rmui-name)))


(provide :rmui-utils "rmui-utils.l")
