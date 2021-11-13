(require :common-rmui-server "package://rmuieus/euslisp/common/common-rmui-server.l")
(require :rmui-utils "package://rmuieus/euslisp/rmui/rmui-utils.l")


(defun rmui-planning-main (&key (rmui-names (list "rmui0")))
  (if (not (listp rmui-names))
    (setq rmui-names (list rmui-names)))
  (set-rmui-parameters rmui-names)
  (scene-init :box-pos-y 0)
  (motion-init :move-robot t
               :robot *robot* :ri *ri* :cube *box* :vw *irtviewer*)
  (pilot-scene-init)
  (motion-init :move-robot nil
               :robot *pilot* :ri nil :cube *pilot-device* :vw *pilotviewer*)
  (ros::ros-info "motion-init finished")

  ;; TODO (knorth55): support multiple devices
  (setq *server*
        (instance rmui-server :init
                  :namespace (car *device-namespaces*)
                  :prx-threshold *prx-threshold*
                  :prx-position-threshold *prx-position-threshold*
                  :device-frame-id (car *device-frame-ids*)
                  :device-world-frame-id (car *device-world-frame-ids*)
                  :motion-symbol-length *motion-symbol-length*
                  :use-torso *use-torso*))
  (ros::rate 10)
  (ros::ros-info "initialization finished, starting")
  (while (ros::ok) (send *server* :spin-once)))
