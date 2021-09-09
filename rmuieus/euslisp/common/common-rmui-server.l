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


(ros::roseus-add-msgs "std_msgs")


;; initial conditions

(defclass rmui-server
  :super propertied-object
  :slots (object-contact-states
          object-location-state
          object-rotation-state
          initial-state
          executed-actions
          scene-states
          pddl-gr
          statenet-gr
          dump-dir
          larm-contact-state-conditions
          rarm-contact-state-conditions
          floor-contact-state-conditions
          object-location-state-conditions
          object-rotation-state-conditions
          ))


(defmethod rmui-server
  (:init (&key ((:namespace ns))
               (prx-threshold) (prx-position-threshold)
               (device-frame-id) (world-frame-id)
               ((:motion-symbol-length msym-length) 5))
    (setq namespace (if ns ns ""))
    ;; rmui insntaces
    (setq rmui-device
          (instance rmui-device :init
                    :namespace namespace
                    :prx-threshold prx-threshold
                    :prx-position-threshold prx-position-threshold
                    :device-frame-id device-frame-id
                    :world-frame-id world-frame-id))
    (setq planner (instance rmui-planner :init :namespace namespace))
    (setq executor
          (instance rmui-executor :init
                    :namespace namespace
                    :motion-symbol-length msym-length))

    ;; initialize
    (setq executed-actions nil)
    (setq scene-states nil)
    (setq pddl-gr (instance pddl-graph :init))
    (setq statenet-gr (instance statenet-graph :init))
    (setq dump-dir nil)

    ;; initial conditions
    (setq object-contact-states (list :released :released :bottom-contact))
    (setq object-location-state :center)
    (setq object-rotation-state (list :x0 :y0 :z0))
    (setq initial-state
          (list (list :larm-contact-state (car object-contact-states))
                (list :rarm-contact-state (cadr object-contact-states))
                (list :floor-contact-state (caddr object-contact-states))
                (list :object-location-state object-location-state)
                (list :object-rotation-state object-rotation-state)))

    ;; update conditions
    (setq larm-contact-state-conditions (load-larm-contact-state-conditions))
    (setq rarm-contact-state-conditions (load-rarm-contact-state-conditions))
    (setq floor-contact-state-conditions (load-floor-contact-state-conditions))
    (setq object-location-state-conditions (load-object-location-state-conditions))
    (setq object-rotation-state-conditions (load-object-rotation-state-conditions))
    self)
  (:append-executed-actions (action-name action)
    (let ((executed-action
            (list (read-from-string (format nil ":~A" action-name)) action)))
      (setq executed-actions (append executed-actions (list executed-action)))))
  (:append-scene-states (scene-state)
    (setq scene-states (append scene-states (list scene-state))))
  (:add-action-state-in-pddl-graph (action prev-scene-state scene-state)
    (add-action-state-in-graph pddl-gr action prev-scene-state scene-state))
  (:add-action-state-in-statenet-graph (action prev-scene-state scene-state)
    (add-action-state-in-graph statenet-gr action prev-scene-state scene-state))
  (:update-scene-state (motion-symbols-and-actions)
    (let* ((prev-larm-contact-state (car object-contact-states))
           (prev-rarm-contact-state (cadr object-contact-states))
           (prev-floor-contact-state (caddr object-contact-states))
           (prev-object-location-state object-location-state)
           (prev-object-rotation-state object-rotation-state)
           (prev-object-x-rotation-symbol (car object-rotation-state))
           (prev-object-y-rotation-symbol (cadr object-rotation-state))
           (prev-object-z-rotation-symbol (caddr object-rotation-state))
           (prev-scene-state (list (list :larm-contact-state prev-larm-contact-state)
                                   (list :rarm-contact-state prev-rarm-contact-state)
                                   (list :floor-contact-state prev-floor-contact-state)
                                   (list :object-location-state prev-object-location-state)
                                   (list :object-rotation-state prev-object-rotation-state))))
      (if (null scene-states) (send self :append-scene-states prev-scene-state))
      (unless motion-symbols-and-actions (return-from :update-scene-state nil))
      (let* ((motion-symbols (car motion-symbols-and-actions))
             (action-name (cadr motion-symbols-and-actions))
             (action (caddr motion-symbols-and-actions))
             (larm-motion-symbol (car motion-symbols))
             (rarm-motion-symbol (cadr motion-symbols))
             (larm-conditions larm-contact-state-conditions)
             (rarm-conditions rarm-contact-state-conditions)
             (floor-conditions floor-contact-state-conditions)
             (object-location-conditions object-location-state-conditions)
             (object-rotation-conditions object-rotation-state-conditions)
             (larm-contact-state
               (eval (append '(cond) larm-conditions '((t prev-larm-contact-state)))))
             (rarm-contact-state
               (eval (append '(cond) rarm-conditions '((t prev-rarm-contact-state)))))
             (floor-contact-state
               (eval (append '(cond) floor-conditions '((t prev-floor-contact-state)))))
             (object-location-state
               (eval (append '(cond) object-location-conditions
                             '((t prev-object-location-state)))))
             (object-rotation-state
               (eval (append '(cond) object-rotation-conditions
                             '((t prev-object-rotation-state)))))
             (scene-state (list (list :larm-contact-state larm-contact-state)
                                (list :rarm-contact-state rarm-contact-state)
                                (list :floor-contact-state floor-contact-state)
                                (list :object-location-state object-location-state)
                                (list :object-rotation-state object-rotation-state))))
        (setq object-contact-states
              (list larm-contact-state rarm-contact-state floor-contact-state))
        (setq object-location-state object-location-state)
        (setq object-rotation-state object-rotation-state)
        (send self :append-scene-states scene-state)
        (send self :append-executed-actions action-name action)
        (send self :add-action-state-in-pddl-graph action prev-scene-state scene-state)
        (send self :add-action-state-in-statenet-graph action prev-scene-state scene-state)
        (if (null dump-dir) (setq dump-dir (create-dump-dir)))
        (dump-scene-states scene-states)
        (dump-executed-actions executed-actions)
        (dump-pddl-graph pddl-gr)
        (dump-statenet-graph statenet-gr))))
  (:server-cb (&rest args)
    (let* ((update-p (send rmui-device :update-device-orientation))
           (contact-positions
             (if update-p (send rmui-device :get-contact-positions)))
           (motion-symbols
             (send planner :get-motion-symbols contact-positions
                   :object-contact-states object-contact-states
                   :object-location-state object-location-state
                   :object-rotation-state object-rotation-state
                   ))
           (execute-symbols-and-actions
             (send executor :execute-motion-symbols motion-symbols
                   :object-contact-states object-contact-states
                   :object-location-state object-location-state
                   :object-rotation-state object-rotation-state
                   ))
           (update-scene-p (send self :update-scene-state execute-symbols-and-actions))
           (execute-symbols (car execute-symbols-and-actions)))
      (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
      (if (boundp '*ri*) (send *ri* :draw-objects))
      (ros::ros-info (format nil "motion-symbols        : ~A" motion-symbols))
      (ros::ros-info (format nil "execute-symbols       : ~A" execute-symbols))
      ; (ros::ros-info (format nil "contact-positions     : ~A" contact-positions))
      (ros::ros-info (format nil "object-contact-states : ~A" object-contact-states))
      (ros::ros-info (format nil "object-location-state : ~A" object-location-state))
      (ros::ros-info (format nil "object-rotation-state : ~A" object-rotation-state))
      (ros::ros-info (format nil "executed-actions: ~A" (mapcar #'car executed-actions)))))
  )


(provide :common-rmui-server "common-rmui-server.l")