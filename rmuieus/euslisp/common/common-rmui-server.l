(require :pddl-result-graph "package://pddl_planner/src/pddl-result-graph.l")
(require :eus-pddl-client "package://pddl_planner/src/eus-pddl-client.l")

(require :rmui-device "package://rmuieus/euslisp/rmui/rmui-device.l")
(require :statenet-graph "package://rmuieus/euslisp/statenet/statenet-graph.l")
(require :common-io "package://rmuieus/euslisp/common/common-io.l")
(require :common-graph "package://rmuieus/euslisp/common/common-graph.l")
(require :common-motions "package://rmuieus/euslisp/common/common-motions.l")
(require :common-actions "package://rmuieus/euslisp/common/common-actions.l")
(require :common-conditions "package://rmuieus/euslisp/common/common-conditions.l")
(require :common-rmui-executor "package://rmuieus/euslisp/common/common-rmui-executor.l")
(require :common-rmui-planner "package://rmuieus/euslisp/common/common-rmui-planner.l")
(require :common-rmui-updater "package://rmuieus/euslisp/common/common-rmui-updater.l")


(ros::roseus-add-msgs "std_msgs")


(defclass rmui-server
  :super propertied-object
  :slots (namespaces
          groupname
          devices
          planner
          executor
          updater
          use-torso
          move-robot
          ri
          robotviewer
          pilotviewer
          ))


(defmethod rmui-server
  (:init (&key ((:namespaces nss))
               ((:groupname gpn))
               (prx-threshold) (prx-position-threshold)
               (device-frame-ids)
               (device-world-frame-ids)
               ((:motion-symbol-length msym-length) 5)
               ((:robot rbt) *robot*)
               ((:ri rbti) *ri*)
               ((:box bx) *box*)
               ((:robotviewer rbtvw) *irtviewer*)
               ((:table-z tblz) *table-z*)
               ((:table-height tblh) *table-height*)
               ((:pilot plt) *pilot*)
               ((:pilot-device pltdev) *pilot-device*)
               ((:pilotviewer pltvw) *pilotviewer*)
               ((:pilot-table-z plttblz) *pilot-table-z*)
               ((:pilot-table-height plttblh) *pilot-table-height*)
               ((:use-torso utrso) 0.001)
               ((:move-robot mvrbt) t))
    (let ((topic-namespace))
      (setq namespaces (if nss nss (list "")))
      (setq ri rbti)
      (setq robotviewer rbtvw)
      (setq pilot-device pltdev)
      (setq pilotviewer pltvw)
      (setq topic-namespace
            (if (> (length namespaces) 1)
              "multi_rmui" (car namespaces)))
      ;; rmui insntaces
      (setq devices nil)
      (dotimes (i (length namespaces))
        (let ((namespace (elt namespaces i))
              (device-frame-id (elt device-frame-ids i))
              (device-world-frame-id (elt device-world-frame-ids i)))
          (setq devices
                (append devices
                        (list (instance rmui-device :init
                                        :namespace namespace
                                        :prx-threshold prx-threshold
                                        :prx-position-threshold prx-position-threshold
                                        :device-frame-id device-frame-id
                                        :device-world-frame-id device-world-frame-id))))))
      (setq planner (instance rmui-planner :init
                              :namespace topic-namespace))
      (setq executor (instance rmui-executor :init
                               :namespace topic-namespace
                               :motion-symbol-length msym-length
                               :robot rbt
                               :ri rbti
                               :box bx
                               :robotviewer rbtvw
                               :table-z tblz
                               :table-height tblh
                               :pilot plt
                               :pilot-device pltdev
                               :pilotviewer pltvw
                               :pilot-table-z plttblz
                               :pilot-table-height plttblh
                               ))
      (setq updater (instance rmui-updater :init
                              :namespace topic-namespace))

      (setq groupname (if gpn gpn))
      (if groupname (ros::create-nodehandle groupname))
      (setq use-torso utrso)
      (setq move-robot mvrbt)
      (send self :init-timer))
    self)
  (:init-timer ()
    (if groupname
      (ros::create-timer 0.1 #'send self :server-cb :groupname groupname)
      (ros::create-timer 0.1 #'send self :server-cb)))
  (:server-cb (&rest args)
    (let* ((update-p
             (eval (append (list 'or)
                           (send-all devices :update-device-orientation))))
           (contact-positions-list
             (if update-p (send-all devices :get-contact-positions)))
           (motion-symbols
             (send planner :get-motion-symbols
                   contact-positions-list
                   :scene-state (send updater :last-scene-state)))
           (execute-symbols-and-actions
             (send executor :execute-motion-symbols motion-symbols
                   :scene-state (send updater :last-scene-state)
                   :move-robot move-robot :use-torso use-torso))
           (update-scene-p (send updater :update-scene-state execute-symbols-and-actions))
           (dump-p (if update-scene-p (send updater :dump-files)))
           (execute-symbols (car execute-symbols-and-actions))
           (last-scene-state (send updater :last-scene-state))
           (executed-actions (send updater :executed-actions)))
      (if robotviewer (send robotviewer :draw-objects))
      (if pilotviewer (send pilotviewer :draw-objects))
      (if ri (send ri :draw-objects))
      (ros::ros-info (format nil "motion-symbols        : ~A" motion-symbols))
      (ros::ros-info (format nil "execute-symbols       : ~A" execute-symbols))
      (ros::ros-info (format nil "larm-contact-state    : ~A"
                             (cadr (assoc :larm-contact-state last-scene-state))))
      (ros::ros-info (format nil "rarm-contact-state    : ~A"
                             (cadr (assoc :rarm-contact-state last-scene-state))))
      (ros::ros-info (format nil "floor-contact-state   : ~A"
                             (cadr (assoc :floor-contact-state last-scene-state))))
      (ros::ros-info (format nil "object-location-state : ~A"
                             (cadr (assoc :object-location-state last-scene-state))))
      (ros::ros-info (format nil "object-rotation-state : ~A"
                             (cadr (assoc :object-rotation-state last-scene-state))))
      (ros::ros-info (format nil "executed-actions      : ~A"
                             (mapcar #'car executed-actions)))))
  (:spin-once ()
    (if groupname
      (ros::spin-once groupname)
      (ros::spin-once)))
  )


(provide :common-rmui-server "common-rmui-server.l")
