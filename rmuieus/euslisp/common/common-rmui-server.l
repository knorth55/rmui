(require :pddl-result-graph "package://pddl_planner/src/pddl-result-graph.l")
(require :eus-pddl-client "package://pddl_planner/src/eus-pddl-client.l")

(require :rmui-device "package://rmuieus/euslisp/rmui/rmui-device.l")
(require :rmui-utils "package://rmuieus/euslisp/rmui/rmui-utils.l")
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
               (prx-threshold)
               (prx-position-threshold)
               (device-frame-ids)
               (device-world-frame-ids)
               ((:motion-symbol-length msym-length) 5)
               ((:robot rbt) *robot*)
               ((:ri rbti) *ri*)
               ((:boxes bxs) *boxes*)
               ((:robotviewer rbtvw) *irtviewer*)
               ((:table-z tblz) *table-z*)
               ((:table-height tblh) *table-height*)
               ((:pilot plt) *pilot*)
               ((:pilot-devices pltdevs) *pilot-devices*)
               ((:pilotviewer pltvw) *pilotviewer*)
               ((:pilot-table-z plttblz) *pilot-table-z*)
               ((:pilot-table-height plttblh) *pilot-table-height*)
               ((:use-torso utrso) 0.001)
               ((:move-robot mvrbt) t))
    (let ((topic-namespace))
      (setq namespaces (if nss nss (list "")))
      (setq ri rbti)
      (setq robotviewer rbtvw)
      (setq pilotviewer pltvw)
      (setq topic-namespace
            (if (> (length namespaces) 1)
              "multi_rmui" (car namespaces)))
      ;; rmui instances
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
                               :boxes bxs
                               :robotviewer rbtvw
                               :table-z tblz
                               :table-height tblh
                               :pilot plt
                               :pilot-devices pltdevs
                               :pilotviewer pltvw
                               :pilot-table-z plttblz
                               :pilot-table-height plttblh
                               ))
      (setq updater (instance rmui-updater :init
                              :namespace topic-namespace
                              :boxes bxs))

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
    (let* ((prev-scene-states (send updater :last-scene-states))
           (update-p
             (eval (append '(or)
                            (send-all devices :update-device-orientation))))
           (device-states
             (if update-p (send-all devices :get-device-state)))
           (contact-positions
             (mapcar
               #'(lambda (device-state)
                   (cadr (assoc :contact-positions device-state)))
               device-states))
           (motion-symbols
             (send planner :get-motion-symbols
                   contact-positions
                   :scene-states prev-scene-states))
           (execute-symbols-and-actions
             (send executor :execute-motion-symbols
                   motion-symbols
                   :scene-states prev-scene-states
                   :move-robot move-robot :use-torso use-torso))
           (update-scene-p
             (send updater :update-scene-states execute-symbols-and-actions))
           (dump-p (if update-scene-p (send updater :dump-files)))
           (execute-symbol update-scene-p)
           (scene-states (send updater :last-scene-states))
           (executed-actions (send updater :executed-actions)))
      (if robotviewer (send robotviewer :draw-objects))
      (if pilotviewer (send pilotviewer :draw-objects))
      (if ri (send ri :draw-objects))
      (ros::ros-info
        (format nil "motion-symbols        : ~A" motion-symbols))
      (ros::ros-info
        (format nil "execute-symbol        : ~A" execute-symbol))
      (ros::ros-info
        (format nil "executed-actions      : ~A" (mapcar #'car executed-actions)))
      (ros::ros-info
        (format nil "larm-contact-state    : ~A"
                (mapcar #'(lambda (scene-state)
                            (cadr (assoc :larm-contact-state scene-state)))
                        scene-states)))
      (ros::ros-info
        (format nil "rarm-contact-state    : ~A"
                (mapcar #'(lambda (scene-state)
                            (cadr (assoc :rarm-contact-state scene-state)))
                        scene-states)))
      (ros::ros-info
        (format nil "floor-contact-state   : ~A"
                (mapcar #'(lambda (scene-state)
                            (cadr (assoc :floor-contact-state scene-state)))
                        scene-states)))
      (ros::ros-info
        (format nil "object-location-state : ~A"
                (mapcar #'(lambda (scene-state)
                            (cadr (assoc :object-location-state scene-state)))
                        scene-states)))
      (ros::ros-info
        (format nil "object-rotation-state : ~A"
                (mapcar #'(lambda (scene-state)
                            (cadr (assoc :object-rotation-state scene-state)))
                        scene-states)))
      (ros::ros-info
        (format nil "cube-contact-state    : ~A"
                (mapcar #'(lambda (scene-state)
                            (cadr (assoc :cube-contact-state scene-state)))
                        scene-states)))
      ))
  (:spin-once ()
    (if groupname
      (ros::spin-once groupname)
      (ros::spin-once)))
  )


(defun rmui-planning-main (&key (rmui-names (list "rmui0"))
                                (box-types :ctb))
  (if (not (listp rmui-names))
    (setq rmui-names (list rmui-names)))
  (set-rmui-parameters rmui-names)
  (scene-init :table-pos-y 0
              :n-box (length rmui-names)
              :box-types box-types)
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
                  :boxes *boxes*
                  :robotviewer *irtviewer*
                  :table-z *table-z*
                  :table-height *table-height*
                  :pilot *pilot*
                  :pilot-devices *pilot-devices*
                  :pilotviewer *pilotviewer*
                  :pilot-table-z *pilot-table-z*
                  :pilot-table-height *pilot-table-height*
                  :use-torso *use-torso*
                  :move-robot t))
  (ros::rate 10)
  (ros::ros-info "initialization finished, starting")
  (while (ros::ok) (send *server* :spin-once)))


(provide :common-rmui-server "common-rmui-server.l")
