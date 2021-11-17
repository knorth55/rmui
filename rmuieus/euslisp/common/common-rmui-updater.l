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
          scene-states-list
          executed-actions
          pddl-gr
          statenet-gr
          data-dir
          dump-dir
          larm-contact-state-conditions
          rarm-contact-state-conditions
          floor-contact-state-conditions
          object-location-state-conditions
          object-rotation-state-conditions
          ))


(defmethod rmui-updater
  (:init (&key ((:namespace ns))
               ((:data-dir ddir)
                *training-object-statenet-path*)
               ((:boxes bxs)))
    (setq namespace (if ns ns ""))

    ;; initialize
    (setq executed-actions nil)
    (setq pddl-gr (instance pddl-graph :init))
    (setq statenet-gr (instance statenet-graph :init))
    (setq data-dir ddir)
    (setq dump-dir nil)

    ;; TODO (knorth55): change initial state from argument
    ;; initial conditions
    (let ((scene-states nil))
      (dotimes (i (length bxs))
        (setq scene-states
              (append scene-states
                      (list
                        (list
                          (list :larm-contact-state :released)
                          (list :rarm-contact-state :released)
                          (list :floor-contact-state :bottom-contact)
                          (list :object-location-state :center)
                          (list :object-rotation-state (list :x0 :y0 :z0)))))))
      (setq scene-states-list (list scene-states)))

    ;; update conditions
    (setq larm-contact-state-conditions (load-larm-contact-state-conditions))
    (setq rarm-contact-state-conditions (load-rarm-contact-state-conditions))
    (setq floor-contact-state-conditions (load-floor-contact-state-conditions))
    (setq object-location-state-conditions (load-object-location-state-conditions))
    (setq object-rotation-state-conditions (load-object-rotation-state-conditions))
    self)
  (:initial-scene-states ()
    (mapcar #'(lambda (x) (mapcar #'copy-seq x))
            (car scene-states-list)))
  (:last-scene-states ()
    (mapcar #'(lambda (x) (mapcar #'copy-seq x))
            (car (last scene-states-list))))
  (:executed-actions () executed-actions)
  (:append-executed-actions (action-name action)
    (let ((executed-action
            (list (read-from-string (format nil ":~A" action-name)) action)))
      (setq executed-actions (append executed-actions (list executed-action)))))
  (:append-scene-states-list (scene-states)
    (setq scene-states-list (append scene-states-list (list scene-states))))
  (:add-action-state-in-pddl-graph (action prev-scene-states scene-states)
    (add-action-state-in-graph pddl-gr action prev-scene-states scene-states))
  (:add-action-state-in-statenet-graph (action prev-scene-states scene-states)
    (add-action-state-in-graph statenet-gr action prev-scene-states scene-states))
  ;; TODO (knorth55): condition for multiple device combination
  (:update-scene-states (motion-symbols-and-actions)
    (if (null motion-symbols-and-actions) (return-from :update-scene-states nil))
    (let* ((prev-scene-states (send self :last-scene-states))
           (scene-states
             (mapcar #'(lambda (x) (mapcar #'copy-seq x)) prev-scene-states))
           (execute-motion-symbol nil)
           (execute-action-name nil)
           (execute-action nil))
      (dotimes (i (length motion-symbols-and-actions))
        (if (elt motion-symbols-and-actions i)
          (let* ((prev-scene-state
                   (elt prev-scene-states i))
                 (motion-symbol-and-action
                   (elt motion-symbols-and-actions i))
                 (motion-symbol (car motion-symbol-and-action))
                 (action-name (cadr motion-symbol-and-action))
                 (action (caddr motion-symbol-and-action)))
            (if motion-symbol-and-action
              (progn
                (setf (elt scene-states i)
                      (send self :update-scene-state
                            motion-symbol-and-action prev-scene-state))
                (setq execute-action-name action-name)
                (setq execute-action action)
                (setq execute-motion-symbol motion-symbol)
                )))))
      (let* ((larm-used-or-released
               (mapcar
                 #'(lambda (scene-state)
                     (let ((larm-contact-state
                             (cadr (assoc :larm-contact-state scene-state))))
                       (or (equal larm-contact-state :released)
                           (equal larm-contact-state :used))))
                 scene-states))
             (rarm-used-or-released
               (mapcar
                 #'(lambda (scene-state)
                     (let ((rarm-contact-state
                             (cadr (assoc :rarm-contact-state scene-state))))
                       (or (equal rarm-contact-state :released)
                           (equal rarm-contact-state :used))))
                 scene-states))
             (larm-released (eval (append '(and) larm-used-or-released)))
             (rarm-released (eval (append '(and) rarm-used-or-released)))
             (larm-contact-index (position nil larm-used-or-released))
             (rarm-contact-index (position nil rarm-used-or-released)))
        (if larm-released
          (dolist (scene-state scene-states)
            (setf (cadr (assoc :larm-contact-state scene-state)) :released))
          (dotimes (i (length scene-states))
            (let ((scene-state (elt scene-states i)))
              (if (not (equal i larm-contact-index))
                (setf (cadr (assoc :larm-contact-state scene-state)) :used)))))
        (if rarm-released
          (dolist (scene-state scene-states)
            (setf (cadr (assoc :rarm-contact-state scene-state)) :released))
          (dotimes (i (length scene-states))
            (let ((scene-state (elt scene-states i)))
              (if (not (equal i rarm-contact-index))
                (setf (cadr (assoc :rarm-contact-state scene-state)) :used))))))
      (if (and execute-action-name execute-action execute-motion-symbol)
        (progn
          (send self :append-scene-states-list scene-states)
          (send self :append-executed-actions
                execute-action-name execute-action)
          (send self :add-action-state-in-pddl-graph
                execute-action prev-scene-states scene-states)
          (send self :add-action-state-in-statenet-graph
                execute-action prev-scene-states scene-states)))
      (return-from :update-scene-states execute-motion-symbol)))
  (:update-scene-state (motion-symbol-and-action prev-scene-state)
    (if (null motion-symbol-and-action) (return-from :update-scene-state nil))
    (let* ((prev-larm-contact-state
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
           (prev-object-z-rotation-symbol (caddr prev-object-rotation-state))
           (motion-symbol (car motion-symbol-and-action))
           (action-name (cadr motion-symbol-and-action))
           (action (caddr motion-symbol-and-action))
           (larm-motion-symbol (car motion-symbol))
           (rarm-motion-symbol (cadr motion-symbol))
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
        scene-state))
  (:dump-files ()
    (if (null dump-dir) (setq dump-dir (create-dump-dir data-dir)))
    (dump-scene-states scene-states-list :data-dir data-dir)
    (dump-executed-actions executed-actions :data-dir data-dir)
    (dump-pddl-graph pddl-gr :data-dir data-dir)
    (dump-statenet-graph statenet-gr :data-dir data-dir)))


(provide :common-rmui-updater "common-rmui-updater.l")
