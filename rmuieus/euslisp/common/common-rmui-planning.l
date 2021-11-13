(require :common-rmui-server "package://rmuieus/euslisp/common/common-rmui-server.l")
(require :rmui-utils "package://rmuieus/euslisp/rmui/rmui-utils.l")


(defun rmui-planning-main (&key (rmui-name "rmui0"))
  (set-rmui-parameters rmui-name)
  (scene-init :box-pos-y 0)
  (motion-init :move-robot t
               :robot *robot* :ri *ri* :cube *box* :vw *irtviewer*)
  (pilot-scene-init)
  (motion-init :move-robot nil
               :robot *pilot* :ri nil :cube *pilot-device* :vw *pilotviewer*)
  (ros::ros-info "motion-init finished")
  (setq *server*
        (instance rmui-server :init
                  :namespace *device-namespace*
                  :prx-threshold *prx-threshold*
                  :prx-position-threshold *prx-position-threshold*
                  :device-frame-id *device-frame-id*
                  :world-frame-id *world-frame-id*
                  :motion-symbol-length *motion-symbol-length*
                  :use-torso *use-torso*))
  (ros::rate 10)
  (ros::ros-info "initialization finished, starting")
  (while (ros::ok) (send *server* :spin-once)))
