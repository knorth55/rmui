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

(defclass rmui-updater
  :super propertied-object
  :slots (namespace
          initial-state
          scene-states
          executed-actions
          pddl-gr
          statenet-gr
          dump-dir
          larm-contact-state-conditions
          rarm-contact-state-conditions
          floor-contact-state-conditions
          object-location-state-conditions
          object-rotation-state-conditions
          ))


(defmethod rmui-updater
  (:init (&key ((:namespace ns)))
    (setq namespace (if ns ns ""))

    ;; initialize
    (setq executed-actions nil)
    (setq pddl-gr (instance pddl-graph :init))
    (setq statenet-gr (instance statenet-graph :init))
    (setq dump-dir nil)

    ;; initial conditions
    (setq scene-states
          (list
            (list
              (list :larm-contact-state :released)
              (list :rarm-contact-state :released)
              (list :floor-contact-state :bottom-contact)
              (list :object-location-state :center)
              (list :object-rotation-state (list :x0 :y0 :z0)))))
    (setq initial-scene-state (mapcar #'copy-seq (car scene-states)))

    ;; update conditions
    (setq larm-contact-state-conditions (load-larm-contact-state-conditions))
    (setq rarm-contact-state-conditions (load-rarm-contact-state-conditions))
    (setq floor-contact-state-conditions (load-floor-contact-state-conditions))
    (setq object-location-state-conditions (load-object-location-state-conditions))
    (setq object-rotation-state-conditions (load-object-rotation-state-conditions))
    self)
  (:initial-scene-state () (mapcar #'copy-seq initial-scene-state))
  (:last-scene-state () (mapcar #'copy-seq (car (last scene-states))))
  (:executed-actions () (mapcar #'copy-seq executed-actions))
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
    (let* ((prev-scene-state (mapcar #'copy-seq (car (last scene-states))))
           (prev-larm-contact-state
             (cadr (assoc :larm-contact-state prev-scene-state)))
           (prev-rarm-contact-state
             (cadr (assoc :rarm-contact-state prev-scene-state)))
           (prev-floor-contact-state
             (cadr (assoc :floor-contact-state prev-scene-state)))
           (prev-object-location-state
             (cadr (assoc :object-location-state prev-scene-state)))
           (prev-object-rotation-state
             (cadr (assoc :object-rotation-state prev-scene-state)))
           (prev-object-x-rotation-symbol (car prev-object-rotation-state))
           (prev-object-y-rotation-symbol (cadr prev-object-rotation-state))
           (prev-object-z-rotation-symbol (caddr prev-object-rotation-state)))
      (if (null scene-states) (send self :append-scene-states prev-scene-state))
      (if (null motion-symbols-and-actions) (return-from :update-scene-state nil))
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
             (scene-state
               (list (list :larm-contact-state larm-contact-state)
                     (list :rarm-contact-state rarm-contact-state)
                     (list :floor-contact-state floor-contact-state)
                     (list :object-location-state object-location-state)
                     (list :object-rotation-state object-rotation-state))))
        (send self :append-scene-states scene-state)
        (send self :append-executed-actions action-name action)
        (send self :add-action-state-in-pddl-graph action prev-scene-state scene-state)
        (send self :add-action-state-in-statenet-graph action prev-scene-state scene-state)
        t)))
  (:dump-files ()
    (if (null dump-dir) (setq dump-dir (create-dump-dir)))
    (dump-scene-states scene-states)
    (dump-executed-actions executed-actions)
    (dump-pddl-graph pddl-gr)
    (dump-statenet-graph statenet-gr)))


(provide :common-rmui-updater "common-rmui-updater.l")
