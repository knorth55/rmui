(require :pddl-result-graph "package://pddl_planner/src/pddl-result-graph.l")
(require :eus-pddl-client "package://pddl_planner/src/eus-pddl-client.l")

(require :rmui-device "package://rmuieus/euslisp/rmui/rmui-device.l")
(require :statenet-graph "package://rmuieus/euslisp/statenet/statenet-graph.l")
(require :common-io "package://rmuieus/euslisp/common/common-io.l")
(require :common-graph "package://rmuieus/euslisp/common/common-graph.l")
(require :common-motions "package://rmuieus/euslisp/common/common-motions.l")
(require :common-actions "package://rmuieus/euslisp/common/common-actions.l")


(ros::roseus-add-msgs "std_msgs")


(defclass rmui-executor
  :super propertied-object
  :slots (namespace
          waiting-visualize-topic-name
          moving-visualize-topic-name
          motion-symbol-queues
          motion-symbol-length
          robot
          ri
          boxes
          robotviewer
          table-z
          table-height
          pilot
          pilot-devices
          pilotviewer
          pilot-table-z
          pilot-table-height
          use-torso
          move-robot
          )
  )


(defmethod rmui-executor
  (:init (&key ((:namespace ns))
               ((:motion-symbol-length msym-length) 5)
               ((:robot rbt))
               ((:ri rbti))
               ((:boxes bxs))
               ((:robotviewer rbtvw))
               ((:table-z tblz))
               ((:table-height tblh))
               ((:pilot plt))
               ((:pilot-devices pltdevs))
               ((:pilotviewer pltvw))
               ((:pilot-table-z plttblz))
               ((:pilot-table-height plttblh))
               ((:use-torso utrso) 0.001)
               ((:move-robot mvrbt) t))
    (setq namespace (if ns ns ""))
    (setq waiting-visualize-topic-name
          (format nil "~A/output/status/waiting" namespace))
    (setq moving-visualize-topic-name
          (format nil "~A/output/status/moving" namespace))
    (ros::advertise waiting-visualize-topic-name std_msgs::Float32 1)
    (ros::advertise moving-visualize-topic-name std_msgs::Float32 1)
    (setq motion-symbol-length msym-length)

    (setq motion-symbol-queues nil)
    (dotimes (i (length bxs))
      (setq motion-symbol-queues
            (append motion-symbol-queues (list (list nil nil)))))

    (setq robot rbt)
    (setq ri rbti)
    (setq boxes bxs)
    (setq robotviewer rbtvw)
    (setq table-z tblz)
    (setq table-height tblh)
    (setq pilot plt)
    (setq pilot-devices pltdevs)
    (setq pilotviewer pltvw)
    (setq pilot-table-z plttblz)
    (setq pilot-table-height plttblh)
    (setq use-torso utrso)
    (setq move-robot mvrbt)
    self)
  (:append-motion-symbol-queue (motion-symbol-queue motion-symbol)
    (append
      (if (>= (length motion-symbol-queue) motion-symbol-length)
        (cdr motion-symbol-queue) motion-symbol-queue)
      (list motion-symbol)))
  (:check-every-motion-queue (motion-symbol-queue motion-symbol)
    (and (>= (length motion-symbol-queue) motion-symbol-length)
         (every #'(lambda (x) (equal x motion-symbol)) motion-symbol-queue)))
  (:get-action-from-motion-symbol (larm-motion-symbol rarm-motion-symbol)
    (let ((larm-action nil)
          (larm-action-args nil)
          (rarm-action nil)
          (rarm-action-args nil))
      (if (and larm-motion-symbol (listp larm-motion-symbol))
        (progn
          (setq larm-action (car larm-motion-symbol))
          (setq larm-action-args (cdr larm-motion-symbol)))
        (setq larm-action larm-motion-symbol))
      (if (and rarm-motion-symbol (listp rarm-motion-symbol))
        (progn
          (setq rarm-action (car rarm-motion-symbol))
          (setq rarm-action-args (cdr rarm-motion-symbol)))
        (setq rarm-action rarm-motion-symbol))
      (cond
        ((and larm-action rarm-action
              (equal larm-action rarm-action))
         (let* ((action-name
                  (format nil "dualarm-~A-action"
                          (string-downcase larm-action)))
                (action (append (list (read-from-string action-name))
                                larm-action-args)))
           (if (not (equal larm-action-args rarm-action-args))
             (ros::ros-error
               (format nil "action args are not same: ~A != ~A"
                       larm-action-args rarm-action-args)))
           (list action-name action)))
        (larm-action
         (let* ((action-name
                  (format nil "larm-~A-action"
                               (string-downcase larm-action)))
                (action (append (list (read-from-string action-name))
                                larm-action-args)))
           (list action-name action)))
        (rarm-action
         (let* ((action-name
                  (format nil "rarm-~A-action"
                              (string-downcase rarm-action)))
                (action (append (list (read-from-string action-name))
                                rarm-action-args)))
           (list action-name action)))
        (t nil))))
  (:execute-motion-symbols
    (motion-symbols &key (scene-states))
    (let ((motion-symbols-and-actions
            (make-list (length motion-symbols) :initial-element nil)))
      ;; update motion symbol queue
      (dotimes (i (length motion-symbols))
        (let ((motion-symbol-queue
                (send self :update-motion-symbol-queue
                      (elt motion-symbols i)
                      (elt motion-symbol-queues i)
                      :scene-state (elt scene-states i))))
          (setf (elt motion-symbol-queues i) motion-symbol-queue)))
      ;; execute motion symbols
      ;; execute only one symbol
      (dotimes (i (length motion-symbols))
        (let ((motion-symbol-and-action
                (send self :execute-motion-symbol
                      (elt motion-symbols i)
                      (elt motion-symbol-queues i)
                      :scene-state (elt scene-states i)
                      :pilot-device (elt pilot-devices i)
                      :box (elt boxes i))))
          (if motion-symbol-and-action
            (progn
              (setf (elt motion-symbols-and-actions i) motion-symbol-and-action)
              (return-from :execute-motion-symbols motion-symbols-and-actions)))))
      motion-symbols-and-actions))
  (:update-motion-symbol-queue
    (motion-symbol motion-symbol-queue &key (scene-state))
    (let* ((larm-motion-symbol-queue (car motion-symbol-queue))
           (rarm-motion-symbol-queue (cadr motion-symbol-queue))
           (larm-motion-symbol (car motion-symbol))
           (rarm-motion-symbol (cadr motion-symbol)))
      (setq larm-motion-symbol-queue
            (send self :append-motion-symbol-queue
                  larm-motion-symbol-queue larm-motion-symbol))
      (setq rarm-motion-symbol-queue
            (send self :append-motion-symbol-queue
                  rarm-motion-symbol-queue rarm-motion-symbol))
      (list larm-motion-symbol-queue rarm-motion-symbol-queue)))
  (:execute-motion-symbol
    (motion-symbol motion-symbol-queue &key (scene-state) (pilot-device) (box))
    (let* ((larm-contact-state
             (cadr (assoc :larm-contact-state scene-state)))
           (rarm-contact-state
             (cadr (assoc :rarm-contact-state scene-state)))
           (floor-contact-state
             (cadr (assoc :floor-contact-state scene-state)))
           (object-location-state
             (cadr (assoc :object-location-state scene-state)))
           (object-rotation-state
             (cadr (assoc :object-rotation-state scene-state)))
           (larm-motion-symbol-queue (car motion-symbol-queue))
           (rarm-motion-symbol-queue (cadr motion-symbol-queue))
           (larm-motion-symbol (car motion-symbol))
           (rarm-motion-symbol (cadr motion-symbol))
           (executed nil)
           (execute-symbol nil)
           larm-execute-p rarm-execute-p)
      ;; check motion symbol queue
      (if (or larm-motion-symbol rarm-motion-symbol)
        (ros::publish waiting-visualize-topic-name
                      (instance std_msgs::Float32 :init :data 1.0)))
      (setq larm-execute-p
            (send self :check-every-motion-queue larm-motion-symbol-queue larm-motion-symbol))
      (setq rarm-execute-p
            (send self :check-every-motion-queue rarm-motion-symbol-queue rarm-motion-symbol))
      (setq larm-motion-symbol (if larm-execute-p larm-motion-symbol))
      (setq rarm-motion-symbol (if rarm-execute-p rarm-motion-symbol))
      (setq motion-symbol-queue (list larm-motion-symbol-queue rarm-motion-symbol-queue))

      ;; execute motion symbols
      (if (or larm-motion-symbol rarm-motion-symbol)
        (progn
          (if move-robot
            (progn
              (ros::publish waiting-visualize-topic-name
                            (instance std_msgs::Float32 :init :data 1.0))
              (ros::publish moving-visualize-topic-name
                            (instance std_msgs::Float32 :init :data 1.0))))
          (let* ((action-name-and-action
                   (send self :get-action-from-motion-symbol
                     larm-motion-symbol rarm-motion-symbol))
                 (action-name (if action-name-and-action (car action-name-and-action)))
                 (action (if action-name-and-action (cadr action-name-and-action))))
            ;; pilot viewer
            (eval (append
                    action
                    '(:move-robot nil
                      :use-torso t
                      :robot pilot
                      :ri nil
                      :cube pilot-device
                      :cubes pilot-devices
                      :vw pilotviewer
                      :table-z pilot-table-z
                      :table-height pilot-table-height)))
            (if pilotviewer (send pilotviewer :draw-objects))
            ;; robot execution
            (setq executed
                  (eval (append
                          action
                          '(:move-robot move-robot
                            :use-torso use-torso
                            :robot robot
                            :ri ri
                            :cube box
                            :cubes boxes
                            :vw robotviewer
                            :table-z table-z
                            :table-height table-height))))
            (if robotviewer (send robotviewer :draw-objects))
            (if executed
              (progn
                (setq execute-symbol
                      (cond
                        ((and larm-motion-symbol rarm-motion-symbol
                              (equal larm-motion-symbol rarm-motion-symbol))
                         (list larm-motion-symbol rarm-motion-symbol))
                        (larm-motion-symbol (list larm-motion-symbol nil))
                        (rarm-motion-symbol (list nil rarm-motion-symbol))
                        (t nil)))
                (list execute-symbol action-name action)
                )))))))
  )


(provide :common-rmui-executor "common-rmui-executor.l")
