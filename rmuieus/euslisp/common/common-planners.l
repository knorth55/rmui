(require :pddl-result-graph "package://pddl_planner/src/pddl-result-graph.l")
(require :eus-pddl-client "package://pddl_planner/src/eus-pddl-client.l")

(require :rmui-device "package://rmuieus/euslisp/rmui/rmui-device.l")
(require :statenet-graph "package://rmuieus/euslisp/statenet/statenet-graph.l")
(require :common-io "package://rmuieus/euslisp/common/common-io.l")
(require :common-graph "package://rmuieus/euslisp/common/common-graph.l")
(require :common-motions "package://rmuieus/euslisp/common/common-motions.l")
(require :common-actions "package://rmuieus/euslisp/common/common-actions.l")
(require :common-conditions "package://rmuieus/euslisp/common/common-conditions.l")


(ros::roseus-add-msgs "std_msgs")


;; initial conditions

(defclass rmui-planner
  :super propertied-object
  :slots (object-contact-states
          object-location-state
          object-rotation-state
          initial-state
          motion-symbol-queue
          executed-actions
          scene-states
          pddl-gr
          statenet-gr
          dump-dir
          waiting-visualize-topic-name
          moving-visualize-topic-name
          larm-get-motion-symbols-conditions
          rarm-get-motion-symbols-conditions
          dualarm-get-motion-symbols-conditions
          larm-contact-state-conditions
          rarm-contact-state-conditions
          floor-contact-state-conditions
          object-location-state-conditions
          object-rotation-state-conditions
          ))


(defmethod rmui-planner
  (:init (&key (namespace)
               (prx-threshold) (prx-position-threshold)
               (device-frame-id) (world-frame-id)
               ((:motion-symbol-length msym-length) 5))
    (setq waiting-visualize-topic-name
          (format nil "~A/output/status/waiting" namespace))
    (setq moving-visualize-topic-name
          (format nil "~A/output/status/moving" namespace))
    (ros::advertise waiting-visualize-topic-name std_msgs::Float32 1)
    (ros::advertise moving-visualize-topic-name std_msgs::Float32 1)
    (setq rmui-device
          (instance rmui-device :init
                    :namespace namespace
                    :prx-threshold prx-threshold
                    :prx-position-threshold prx-position-threshold
                    :device-frame-id device-frame-id
                    :world-frame-id world-frame-id))
    (setq larm-get-motion-symbols-conditions
          (load-larm-get-motion-symbols-conditions))
    (setq rarm-get-motion-symbols-conditions
          (load-rarm-get-motion-symbols-conditions))
    (setq dualarm-get-motion-symbols-conditions
          (load-dualarm-get-motion-symbols-conditions))
    (setq larm-contact-state-conditions (load-larm-contact-state-conditions))
    (setq rarm-contact-state-conditions (load-rarm-contact-state-conditions))
    (setq floor-contact-state-conditions (load-floor-contact-state-conditions))
    (setq object-location-state-conditions (load-object-location-state-conditions))
    (setq object-rotation-state-conditions (load-object-rotation-state-conditions))
    (setq object-contact-states (list :released :released :bottom-contact))
    (setq object-location-state :center)
    (setq object-rotation-state (list :x0 :y0 :z0))
    (setq initial-state
          (list (list :larm-contact-state (car object-contact-states))
                (list :rarm-contact-state (cadr object-contact-states))
                (list :floor-contact-state (caddr object-contact-states))
                (list :object-location-state object-location-state)
                (list :object-rotation-state object-rotation-state)))
    (setq motion-symbol-length msym-length)
    (setq motion-symbol-queue (list nil nil))
    (setq executed-actions nil)
    (setq scene-states nil)
    (setq pddl-gr (instance pddl-graph :init))
    (setq statenet-gr (instance statenet-graph :init))
    (setq dump-dir nil)
    self)
  (:get-motion-symbol (arm contact-positions)
    (let ((contact-surfaces (mapcar #'car contact-positions))
          (bottom-contact-positions
            (remove-if #'null
                       (mapcar #'(lambda (x) (if (equal (car x) :bottom) (cadr x) nil))
                               contact-positions)))
          (larm-contact-state (car object-contact-states))
          (rarm-contact-state (cadr object-contact-states))
          (floor-contact-state (caddr object-contact-states))
          (object-location-state object-location-state)
          (arm-conditions (cond ((equal arm :larm) larm-get-motion-symbols-conditions)
                                ((equal arm :rarm) rarm-get-motion-symbols-conditions)
                                (t nil)))
          (dualarm-conditions dualarm-get-motion-symbols-conditions))
      (ros::publish waiting-visualize-topic-name
                    (instance std_msgs::Float32 :init :data 0.0))
      (ros::publish moving-visualize-topic-name
                    (instance std_msgs::Float32 :init :data 0.0))
      (eval (append '(cond) arm-conditions dualarm-conditions
                    '(((null contact-positions) nil) (t nil))))))
  (:append-motion-symbol-queue (motion-symbol-queue motion-symbol)
    (append
      (if (>= (length motion-symbol-queue) motion-symbol-length)
        (cdr motion-symbol-queue) motion-symbol-queue)
      (list motion-symbol)))
  (:check-every-motion-queue (motion-symbol-queue motion-symbol)
    (and (>= (length motion-symbol-queue) motion-symbol-length)
         (every #'(lambda (x) (equal x motion-symbol)) motion-symbol-queue)))
  (:get-action-from-motion-symbol (larm-motion-symbol rarm-motion-symbol
                                        &key (move-robot t) (use-torso t))
    (cond
      ((and larm-motion-symbol rarm-motion-symbol
            (equal larm-motion-symbol rarm-motion-symbol))
       (let* ((action-name
                (format nil "dualarm-~A-action"
                        (string-downcase larm-motion-symbol)))
              (action
                (list (read-from-string action-name)
                     :move-robot move-robot :use-torso use-torso)))
         (list action-name action)))
      (larm-motion-symbol
       (let* ((action-name
                (format nil "larm-~A-action"
                             (string-downcase larm-motion-symbol)))
              (action
                (list (read-from-string action-name)
                   :move-robot move-robot :use-torso use-torso)))
         (list action-name action)))
      (rarm-motion-symbol
       (let* ((action-name
                (format nil "rarm-~A-action"
                            (string-downcase rarm-motion-symbol)))
              (action
                (list (read-from-string action-name)
                  :move-robot move-robot :use-torso use-torso)))
         (list action-name action)))
      (t nil)))
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
  (:get-motion-symbols (contact-positions)
    (let* ((larm-motion-symbols
             (send self :get-motion-symbol :larm contact-positions))
           (rarm-motion-symbols
             (send self :get-motion-symbol :rarm contact-positions)))
      (list larm-motion-symbols rarm-motion-symbols)))
  (:execute-motion-symbols (motion-symbols &key (move-robot t) (use-torso 0.001))
    (let ((larm-motion-symbol-queue (car motion-symbol-queue))
          (rarm-motion-symbol-queue (cadr motion-symbol-queue))
          (larm-motion-symbol (car motion-symbols))
          (rarm-motion-symbol (cadr motion-symbols))
          (larm-contact-state (car object-contact-states))
          (rarm-contact-state (cadr object-contact-states))
          (floor-contact-state (caddr object-contact-states))
          (execute-symbols nil)
          larm-execute-p rarm-execute-p)
      ;; check motion symbol queue
      (if (or larm-motion-symbol rarm-motion-symbol)
        (ros::publish waiting-visualize-topic-name
                      (instance std_msgs::Float32 :init :data 1.0)))
      (setq larm-motion-symbol-queue
            (send self :append-motion-symbol-queue larm-motion-symbol-queue larm-motion-symbol))
      (setq rarm-motion-symbol-queue
            (send self :append-motion-symbol-queue rarm-motion-symbol-queue rarm-motion-symbol))
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
                     larm-motion-symbol rarm-motion-symbol
                     :move-robot move-robot :use-torso use-torso))
                 (action-name (if action-name-and-action (car action-name-and-action)))
                 (action (if action-name-and-action (cadr action-name-and-action))))
            (eval action)
            (setq execute-symbols
                  (cond
                    ((and larm-motion-symbol rarm-motion-symbol
                          (equal larm-motion-symbol rarm-motion-symbol))
                     (list larm-motion-symbol rarm-motion-symbol))
                    (larm-motion-symbol (list larm-motion-symbol nil))
                    (rarm-motion-symbol (list nil rarm-motion-symbol))
                    (t nil)))
            (list execute-symbols action-name action))))))
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
  (:planner-cb (&rest args)
    (let* ((update-p (send rmui-device :update-device-orientation))
           (contact-positions
             (if update-p (send rmui-device :get-contact-positions)))
           (motion-symbols (send self :get-motion-symbols contact-positions))
           (execute-symbols-and-actions (send self :execute-motion-symbols motion-symbols))
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


(provide :common-planners "common-planners.l")
