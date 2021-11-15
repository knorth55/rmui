(require :common-rmui-server "package://rmuieus/euslisp/common/common-rmui-server.l")
(require :rmui-utils "package://rmuieus/euslisp/rmui/rmui-utils.l")


(defun rmui-planning-main (&key (rmui-names (list "rmui0")))
  (if (not (listp rmui-names))
    (setq rmui-names (list rmui-names)))
  (set-rmui-parameters rmui-names)
  (scene-init :table-pos-y 0
              :n-box (length rmui-names))
  (motion-init :move-robot t
               :robot *robot*
               :ri *ri*
               :cubes *boxes*
               :vw *irtviewer*)
  (pilot-scene-init :table-pos-y 0
                    :n-device (length rmui-names))
  (motion-init :move-robot nil
               :robot *pilot*
               :ri nil
               :cubes *pilot-devices*
               :vw *pilotviewer*)
  (ros::ros-info "motion-init finished")

  (setq *server*
        (instance rmui-server :init
                  :namespaces *device-namespaces*
                  :prx-threshold *prx-threshold*
                  :prx-position-threshold *prx-position-threshold*
                  :device-frame-ids *device-frame-ids*
                  :device-world-frame-ids *device-world-frame-ids*
                  :motion-symbol-length *motion-symbol-length*
                  :robot *robot*
                  :ri *ri*
                  :box *box0*
                  :robotviewer *irtviewer*
                  :table-z *table-z*
                  :table-height *table-height*
                  :pilot *pilot*
                  :pilot-device *pilot-device0*
                  :pilotviewer *pilotviewer*
                  :pilot-table-z *pilot-table-z*
                  :pilot-table-height *pilot-table-height*
                  :use-torso *use-torso*
                  :move-robot t))
  (ros::rate 10)
  (ros::ros-info "initialization finished, starting")
  (while (ros::ok) (send *server* :spin-once)))
