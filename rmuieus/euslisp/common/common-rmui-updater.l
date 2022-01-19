(require :pddl-result-graph "package://pddl_planner/src/pddl-result-graph.l")
(require :eus-pddl-client "package://pddl_planner/src/eus-pddl-client.l")

(require :rmui-device "package://rmuieus/euslisp/rmui/rmui-device.l")
(require :statenet-graph "package://rmuieus/euslisp/statenet/statenet-graph.l")
(require :common-io "package://rmuieus/euslisp/common/common-io.l")
(require :common-graph "package://rmuieus/euslisp/common/common-graph.l")
(require :common-motions "package://rmuieus/euslisp/common/common-motions.l")
(require :common-actions "package://rmuieus/euslisp/common/common-actions.l")
(require :common-conditions "package://rmuieus/euslisp/common/common-conditions.l")
(require :common-scenes "package://rmuieus/euslisp/common/common-scenes.l")

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
          boxes
          box-types
          larm-contact-state-conditions
          rarm-contact-state-conditions
          floor-contact-state-conditions
          object-location-state-conditions
          object-rotation-state-conditions
          reset-state
          ))


(defmethod rmui-updater
  (:init (&key ((:namespace ns))
               ((:data-dir ddir) *training-object-statenet-path*)
               ((:boxes bxs))
               ((:reset-state rs) *reset-state*))
    (setq namespace (if ns ns ""))
    (setq data-dir ddir)
    (setq boxes bxs)
    (setq box-types (mapcar #'(lambda (bx) (send bx :get :box-type)) boxes))
    (setq reset-state rs)

    ;; update conditions
    (setq larm-contact-state-conditions (load-larm-contact-state-conditions))
    (setq rarm-contact-state-conditions (load-rarm-contact-state-conditions))
    (setq floor-contact-state-conditions (load-floor-contact-state-conditions))
    (setq object-location-state-conditions (load-object-location-state-conditions))
    (setq object-rotation-state-conditions (load-object-rotation-state-conditions))

    ;; initialize
    ;; TODO (knorth55): change initial state from argument
    (send self :reset-updater :initial-scene-states (copy-tree reset-state))
    self)
  (:reset-updater
    (&key (initial-scene-states))
    (if (null initial-scene-states)
      (setq initial-scene-states (copy-tree reset-state)))
    (setq executed-actions nil)
    (setq pddl-gr (instance pddl-graph :init))
    (setq statenet-gr (instance statenet-graph :init))
    (send statenet-gr :put :box-types (copy-treee box-types))
    (setq dump-dir nil)
    (setq scene-states-list (list initial-scene-states)))
  (:initial-scene-states ()
    (copy-tree (car scene-states-list)))
  (:last-scene-states ()
    (copy-tree (car (last scene-states-list))))
  (:last-executed-action-name ()
    (car (car (last executed-actions))))
  (:last-executed-action ()
    (copy-tree (cadr (car (last executed-actions)))))
  (:statenet-graph () statenet-gr)
  (:pddl-graph () pddl-gr)
  (:executed-actions () executed-actions)
  (:scene-states-list () scene-states-list)
  (:append-executed-actions (action-name action)
    (let ((executed-action (list action-name (copy-tree action))))
      (setq executed-actions
            (append executed-actions (list executed-action)))))
  (:append-scene-states-list (scene-states)
    (setq scene-states-list
          (append scene-states-list (list (copy-tree scene-states)))))
  (:add-action-state-in-pddl-graph (action prev-scene-states scene-states)
    (add-action-state-in-graph
      pddl-gr (copy-tree action)
      (copy-tree prev-scene-states) (copy-tree scene-states)))
  (:add-action-state-in-statenet-graph (action prev-scene-states scene-states)
    (add-action-state-in-graph
      statenet-gr (copy-tree action)
      (copy-tree prev-scene-states) (copy-tree scene-states)))
  (:update-scene-states (motion-symbols-and-actions)
    (if (null motion-symbols-and-actions) (return-from :update-scene-states nil))
    (let* ((prev-scene-states (send self :last-scene-states))
           (scene-states (copy-tree prev-scene-states))
           (execute-action-names
             (make-list (length motion-symbols-and-actions) :initial-element nil))
           (execute-actions
             (make-list (length motion-symbols-and-actions) :initial-element nil))
           (execute-cube-index nil))
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
                            motion-symbol prev-scene-state))
                (setq execute-cube-index i))))))
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
      (if execute-cube-index
        (let* ((execute-motion-symbol-and-action
                 (elt motion-symbols-and-actions execute-cube-index))
               (execute-motion-symbol (car execute-motion-symbol-and-action))
               (execute-action-name (cadr execute-motion-symbol-and-action))
               (execute-action (caddr execute-motion-symbol-and-action))
               (larm-execute-motion-symbol (car execute-motion-symbol))
               (rarm-execute-motion-symbol (cadr execute-motion-symbol))
               (prev-scene-state (elt prev-scene-states execute-cube-index))
               (prev-cube-contact-state
                 (copy-tree (cadr (assoc :cube-contact-state prev-scene-state)))))
          (setf (elt execute-action-names execute-cube-index) execute-action-name)
          (setf (elt execute-actions execute-cube-index) (copy-tree execute-action))
          (cond
            ;; hold pile-up
            ((and (not (find :bottom-contact prev-cube-contact-state))
                  (listp larm-execute-motion-symbol)
                  (equal (car larm-execute-motion-symbol) :push-hold-pile-up)
                  (listp rarm-execute-motion-symbol)
                  (equal (car rarm-execute-motion-symbol) :push-hold-pile-up))
             (let ((cube-index
                     (elt larm-execute-motion-symbol
                          (+ 1 (position :cube-index larm-execute-motion-symbol)))))
               (setf (elt (cadr (assoc :cube-contact-state (elt scene-states cube-index)))
                          execute-cube-index)
                     :top-contact)))
            ;; grasp pile-up
            ((and (not (find :bottom-contact prev-cube-contact-state))
                  (listp larm-execute-motion-symbol)
                  (equal (car larm-execute-motion-symbol) :grasp-pile-up))
             (let ((cube-index
                     (elt larm-execute-motion-symbol
                          (+ 1 (position :cube-index larm-execute-motion-symbol)))))
               (setf (elt (cadr (assoc :cube-contact-state (elt scene-states cube-index)))
                          execute-cube-index)
                     :top-contact)))
            ((and (not (find :bottom-contact prev-cube-contact-state))
                  (listp rarm-execute-motion-symbol)
                  (equal (car rarm-execute-motion-symbol) :grasp-pile-up))
             (let ((cube-index
                     (elt rarm-execute-motion-symbol
                          (+ 1 (position :cube-index rarm-execute-motion-symbol)))))
               (setf (elt (cadr (assoc :cube-contact-state (elt scene-states cube-index)))
                          execute-cube-index)
                     :top-contact)))
            ;; lift-up from hold pile-up
            ((and (find :bottom-contact prev-cube-contact-state)
                  (equal larm-execute-motion-symbol :push-hold-up)
                  (equal rarm-execute-motion-symbol :push-hold-up))
             (let ((cube-index
                     (position :bottom-contact prev-cube-contact-state)))
               (setf (elt (cadr (assoc :cube-contact-state (elt scene-states cube-index)))
                          execute-cube-index)
                     :released)))
            ;; lift-up from grasp pile-up
            ((and (find :bottom-contact prev-cube-contact-state)
                  (or (equal larm-execute-motion-symbol :grasp-up)
                      (equal rarm-execute-motion-symbol :grasp-up)))
             (let ((cube-index
                     (position :bottom-contact prev-cube-contact-state)))
               (setf (elt (cadr (assoc :cube-contact-state (elt scene-states cube-index)))
                          execute-cube-index)
                     :released)))
            ;; next-to
            ((and (not (find :left-contact prev-cube-contact-state))
                  (listp rarm-execute-motion-symbol)
                  (equal (car rarm-execute-motion-symbol) :left-push-next-to))
             (let ((cube-index
                     (elt rarm-execute-motion-symbol
                          (+ 1 (position :cube-index rarm-execute-motion-symbol)))))
               (setf (elt (cadr (assoc :cube-contact-state (elt scene-states cube-index)))
                          execute-cube-index)
                     :right-contact)))
            ((and (not (find :back-contact prev-cube-contact-state))
                  (listp rarm-execute-motion-symbol)
                  (equal (car rarm-execute-motion-symbol) :back-push-next-to))
             (let ((cube-index
                     (elt rarm-execute-motion-symbol
                          (+ 1 (position :cube-index rarm-execute-motion-symbol)))))
               (setf (elt (cadr (assoc :cube-contact-state (elt scene-states cube-index)))
                          execute-cube-index)
                     :front-contact)))
            ((and (not (find :right-contact prev-cube-contact-state))
                  (listp larm-execute-motion-symbol)
                  (equal (car larm-execute-motion-symbol) :right-push-next-to))
             (let ((cube-index
                     (elt larm-execute-motion-symbol
                          (+ 1 (position :cube-index larm-execute-motion-symbol)))))
               (setf (elt (cadr (assoc :cube-contact-state (elt scene-states cube-index)))
                          execute-cube-index)
                     :left-contact)))
            ((and (not (find :front-contact prev-cube-contact-state))
                  (listp larm-execute-motion-symbol)
                  (equal (car larm-execute-motion-symbol) :front-push-next-to))
             (let ((cube-index
                     (elt larm-execute-motion-symbol
                          (+ 1 (position :cube-index larm-execute-motion-symbol)))))
               (setf (elt (cadr (assoc :cube-contact-state (elt scene-states cube-index)))
                          execute-cube-index)
                     :back-contact)))
            (t nil))
          (send self :append-scene-states-list scene-states)
          (send self :append-executed-actions
                execute-action-names execute-actions)
          (send self :add-action-state-in-pddl-graph
                execute-actions prev-scene-states scene-states)
          (send self :add-action-state-in-statenet-graph
                execute-actions prev-scene-states scene-states)
          (return-from :update-scene-states execute-motion-symbol)))))
  (:update-scene-state (motion-symbol prev-scene-state)
    (if (null motion-symbol) (return-from :update-scene-state nil))
    (let* ((prev-larm-contact-state
             (cadr (assoc :larm-contact-state prev-scene-state)))
           (prev-rarm-contact-state
             (cadr (assoc :rarm-contact-state prev-scene-state)))
           (prev-floor-contact-state
             (cadr (assoc :floor-contact-state prev-scene-state)))
           (prev-object-location-state
             (cadr (assoc :object-location-state prev-scene-state)))
           (prev-object-rotation-state
             (copy-tree (cadr (assoc :object-rotation-state prev-scene-state))))
           (prev-cube-contact-state
             (copy-tree (cadr (assoc :cube-contact-state prev-scene-state))))
           (prev-left-wall-contact-state
             (cadr (assoc :left-wall-contact-state prev-scene-state)))
           (prev-front-wall-contact-state
             (cadr (assoc :front-wall-contact-state prev-scene-state)))
           (prev-object-x-rotation-symbol (car prev-object-rotation-state))
           (prev-object-y-rotation-symbol (cadr prev-object-rotation-state))
           (prev-object-z-rotation-symbol (caddr prev-object-rotation-state))
           (larm-motion-symbol (car motion-symbol))
           (rarm-motion-symbol (cadr motion-symbol))
           (larm-contact-state
             (eval (append '(cond) larm-contact-state-conditions
                           '((t prev-larm-contact-state)))))
           (rarm-contact-state
             (eval (append '(cond) rarm-contact-state-conditions
                           '((t prev-rarm-contact-state)))))
           (floor-contact-state
             (eval (append '(cond) floor-contact-state-conditions
                           '((t prev-floor-contact-state)))))
           (object-location-state
             (eval (append '(cond) object-location-state-conditions
                           '((t prev-object-location-state)))))
           (object-rotation-state
             (eval (append '(cond) object-rotation-state-conditions
                           '((t prev-object-rotation-state)))))
           (cube-contact-state prev-cube-contact-state)
           (left-wall-contact-state
             (cond
               ((and (equal prev-larm-contact-state :released)
                     (equal prev-rarm-contact-state :right-contact)
                     (equal prev-left-wall-contact-state :released)
                     (equal prev-front-wall-contact-state :front-contact)
                     (equal rarm-motion-symbol :left-wall-push-next-to))
                :left-contact)
               (t prev-left-wall-contact-state)))
           (front-wall-contact-state
             (cond
               ((and (equal prev-larm-contact-state :left-contact)
                     (equal prev-rarm-contact-state :right-contact)
                     (equal prev-front-wall-contact-state :released)
                     (equal prev-left-wall-contact-state :released)
                     (equal larm-motion-symbol :front-wall-push-next-to)
                     (equal rarm-motion-symbol :front-wall-push-next-to))
                :front-contact)
               ((and (equal prev-larm-contact-state :left-contact)
                     (equal prev-rarm-contact-state :right-contact)
                     (equal prev-front-wall-contact-state :front-contact)
                     (equal prev-left-wall-contact-state :released)
                     (equal larm-motion-symbol :front-wall-push-release)
                     (equal rarm-motion-symbol :front-wall-push-release))
                :released)
               (t prev-front-wall-contact-state)))
           )
      (cond
        ;; hold pile-up
        ((and (not (find :bottom-contact prev-cube-contact-state))
              (listp larm-motion-symbol)
              (equal (car larm-motion-symbol) :push-hold-pile-up)
              (listp rarm-motion-symbol)
              (equal (car rarm-motion-symbol) :push-hold-pile-up))
         (setf (elt cube-contact-state
                    (elt larm-motion-symbol
                         (+ 1 (position :cube-index larm-motion-symbol))))
               :bottom-contact))
        ;; grasp pile-up
        ((and (not (find :bottom-contact prev-cube-contact-state))
              (listp larm-motion-symbol)
              (equal (car larm-motion-symbol) :grasp-pile-up)
              )
         (setf (elt cube-contact-state
                    (elt larm-motion-symbol
                         (+ 1 (position :cube-index larm-motion-symbol))))
               :bottom-contact))
        ((and (not (find :bottom-contact prev-cube-contact-state))
              (listp rarm-motion-symbol)
              (equal (car rarm-motion-symbol) :grasp-pile-up)
              )
         (setf (elt cube-contact-state
                    (elt rarm-motion-symbol
                         (+ 1 (position :cube-index rarm-motion-symbol))))
               :bottom-contact))
        ;; lift up from hold pile-up
        ((and (find :bottom-contact prev-cube-contact-state)
              (equal larm-motion-symbol :push-hold-up)
              (equal rarm-motion-symbol :push-hold-up))
         (setf (elt cube-contact-state
                    (position :bottom-contact cube-contact-state))
               :released))
        ;; lift up from grasp pile-up
        ((and (find :bottom-contact prev-cube-contact-state)
              (or (equal larm-motion-symbol :grasp-up)
                  (equal rarm-motion-symbol :grasp-up)))
         (setf (elt cube-contact-state
                    (position :bottom-contact cube-contact-state))
               :released))
        ;; next-to
        ((and (not (find :left-contact prev-cube-contact-state))
              (listp rarm-motion-symbol)
              (equal (car rarm-motion-symbol) :left-push-next-to))
         (setf (elt cube-contact-state
                    (elt rarm-motion-symbol
                         (+ 1 (position :cube-index rarm-motion-symbol))))
               :left-contact))
        ((and (not (find :back-contact prev-cube-contact-state))
              (listp rarm-motion-symbol)
              (equal (car rarm-motion-symbol) :back-push-next-to))
         (setf (elt cube-contact-state
                    (elt rarm-motion-symbol
                         (+ 1 (position :cube-index rarm-motion-symbol))))
               :back-contact))
        ((and (not (find :right-contact prev-cube-contact-state))
              (listp larm-motion-symbol)
              (equal (car larm-motion-symbol) :right-push-next-to))
         (setf (elt cube-contact-state
                    (elt larm-motion-symbol
                         (+ 1 (position :cube-index larm-motion-symbol))))
               :right-contact))
        ((and (not (find :front-contact prev-cube-contact-state))
              (listp larm-motion-symbol)
              (equal (car larm-motion-symbol) :front-push-next-to))
         (setf (elt cube-contact-state
                    (elt larm-motion-symbol
                         (+ 1 (position :cube-index larm-motion-symbol))))
               :front-contact))
        (t nil))
      (list (list :larm-contact-state larm-contact-state)
            (list :rarm-contact-state rarm-contact-state)
            (list :floor-contact-state floor-contact-state)
            (list :object-location-state object-location-state)
            (list :object-rotation-state object-rotation-state)
            (list :cube-contact-state cube-contact-state)
            (list :left-wall-contact-state left-wall-contact-state)
            (list :front-wall-contact-state front-wall-contact-state))))
  (:dump-files ()
    (if (null dump-dir) (setq dump-dir (create-dump-dir data-dir)))
    (dump-scene-states scene-states-list :data-dir data-dir)
    (dump-executed-actions executed-actions :data-dir data-dir)
    (dump-pddl-graph pddl-gr :data-dir data-dir)
    (dump-statenet-graph statenet-gr :data-dir data-dir)))


(provide :common-rmui-updater "common-rmui-updater.l")
