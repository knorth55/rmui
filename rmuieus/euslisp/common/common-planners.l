(require :pddl-result-graph "package://pddl_planner/src/pddl-result-graph.l")
(require :eus-pddl-client "package://pddl_planner/src/eus-pddl-client.l")

(require :common-motions "package://rmuieus/euslisp/common/common-motions.l")
(require :common-actions "package://rmuieus/euslisp/common/common-actions.l")
(require :common-conditions "package://rmuieus/euslisp/common/common-conditions.l")
(require :prx-utils "package://rmuieus/euslisp/rmui/prx-utils.l")

(ros::load-ros-manifest "std_msgs")


;; parameters

(defvar *device* (make-cube 120 120 120))
(defvar *device-frame-id* "rmui_link")
;; (defvar *motion-symbol-length* 10)
(defvar *motion-symbol-length* 5)
(defvar *world-frame-id* "world")
(defvar *waiting-visualize-topic-name* "/rmui0/rmui_node/output/status/waiting")
(defvar *moving-visualize-topic-name* "/rmui0/rmui_node/output/status/moving")
(defvar *training-data-path* (ros::resolve-ros-path "package://rmuieus/training_data"))


;; initial conditions

(setq *object-contact-states* (list :released :released :bottom-contact))
(setq *object-location-state* :center)
(setq *motion-symbol-queue* (list nil nil))
(setq *executed-actions* nil)
(setq *scene-states* nil)
(setq *pddl-graph* (instance pddl-graph :init))
(setq *dump-dir* nil)


;; functions

(defun get-iso-time ()
  (let* ((localtime (unix::localtime))
         (iso-time (format nil "~A-~0,2d-~0,2dT~0,2d:~0,2d:~0,2d"
                       (+ (elt localtime 5) 1900)
                       (+ (elt localtime 4) 1)
                       (elt localtime 3)
                       (elt localtime 2)
                       (elt localtime 1)
                       (elt localtime 0))))
    iso-time))


(defun get-motion-symbols (contact-positions)
  (let* ((larm-motion-symbols
           (get-motion-symbol :larm contact-positions))
         (rarm-motion-symbols
           (get-motion-symbol :rarm contact-positions)))
    (list larm-motion-symbols rarm-motion-symbols)))


(defun get-motion-symbol (arm contact-positions)
  (let ((contact-surfaces (mapcar #'car contact-positions))
        (bottom-contact-positions
          (remove-if #'null
                     (mapcar #'(lambda (x) (if (equal (car x) :bottom) (cadr x) nil))
                             contact-positions)))
        (larm-contact-state (car *object-contact-states*))
        (rarm-contact-state (cadr *object-contact-states*))
        (floor-contact-state (caddr *object-contact-states*))
        (object-location-state *object-location-state*)
        (arm-conditions (cond ((equal arm :larm) *larm-get-motion-symbols-conditions*)
                              ((equal arm :rarm) *rarm-get-motion-symbols-conditions*)
                              (t nil)))
        (dualarm-conditions *dualarm-get-motion-symbols-conditions*))
    (ros::publish *waiting-visualize-topic-name*
                  (instance std_msgs::Float32 :init :data 0.0))
    (ros::publish *moving-visualize-topic-name*
                  (instance std_msgs::Float32 :init :data 0.0))
    (eval (append '(cond) arm-conditions dualarm-conditions
                  '(((null contact-positions) nil) (t nil))))))


(defun append-motion-symbol-queue (motion-symbol-queue motion-symbol)
  (append
    (if (>= (length motion-symbol-queue) *motion-symbol-length*)
      (cdr motion-symbol-queue) motion-symbol-queue)
    (list motion-symbol)))


(defun check-every-motion-queue (motion-symbol-queue motion-symbol)
  (and (>= (length motion-symbol-queue) *motion-symbol-length*)
       (every #'(lambda (x) (equal x motion-symbol)) motion-symbol-queue)))


(defun get-action-from-motion-symbol (larm-motion-symbol rarm-motion-symbol
                                      &key (move-robot t) (use-torso t))
  (cond
    ((and larm-motion-symbol rarm-motion-symbol
          (equal larm-motion-symbol rarm-motion-symbol))
     (let* ((action-name
              (format nil "dualarm-~A-action" (string-downcase larm-motion-symbol)))
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


(defun execute-motion-symbols (motion-symbols &key (move-robot t) (use-torso 0.001))
  (let ((larm-motion-symbol-queue (car *motion-symbol-queue*))
        (rarm-motion-symbol-queue (cadr *motion-symbol-queue*))
        (larm-motion-symbol (car motion-symbols))
        (rarm-motion-symbol (cadr motion-symbols))
        (larm-contact-state (car *object-contact-states*))
        (rarm-contact-state (cadr *object-contact-states*))
        (floor-contact-state (caddr *object-contact-states*))
        (execute-symbols nil)
        larm-execute-p rarm-execute-p)
    ;; check motion symbol queue
    (if (or larm-motion-symbol rarm-motion-symbol)
      (ros::publish *waiting-visualize-topic-name*
                    (instance std_msgs::Float32 :init :data 1.0)))
    (setq larm-motion-symbol-queue
          (append-motion-symbol-queue larm-motion-symbol-queue larm-motion-symbol))
    (setq rarm-motion-symbol-queue
          (append-motion-symbol-queue rarm-motion-symbol-queue rarm-motion-symbol))
    (setq larm-execute-p
          (check-every-motion-queue larm-motion-symbol-queue larm-motion-symbol))
    (setq rarm-execute-p
          (check-every-motion-queue rarm-motion-symbol-queue rarm-motion-symbol))
    (setq larm-motion-symbol (if larm-execute-p larm-motion-symbol))
    (setq rarm-motion-symbol (if rarm-execute-p rarm-motion-symbol))
    (setq *motion-symbol-queue* (list larm-motion-symbol-queue rarm-motion-symbol-queue))

    ;; execute motion symbols
    (if (or larm-motion-symbol rarm-motion-symbol)
      (progn
        (if move-robot
          (progn
            (ros::publish *waiting-visualize-topic-name*
                          (instance std_msgs::Float32 :init :data 1.0))
            (ros::publish *moving-visualize-topic-name*
                          (instance std_msgs::Float32 :init :data 1.0))))
        (let* ((action-name-and-action
                 (get-action-from-motion-symbol
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


(defun create-dump-dir (&key (stamp (get-iso-time)))
  (let ((dump-dir (format nil "~A/~A" *training-data-path* stamp))
        (latest-dir (format nil "~A/latest" *training-data-path*)))
    (unix::system (format nil "mkdir -p ~A" dump-dir))
    (unix::system (format nil "ln -sf ~A ~A" dump-dir latest-dir))
    dump-dir))


(defun dump-executed-actions (&key (stamp "latest") (filename "executed-actions.l"))
  (let ((filepath (format nil "~A/~A/~A" *training-data-path* stamp filename)))
    (dump-structure filepath *executed-actions*)))


(defun load-executed-actions (&key (stamp "latest") (filename "executed-actions.l"))
  (let ((filepath (format nil "~A/~A/~A" *training-data-path* stamp filename)))
    (with-open-file (executed-actions-f filepath)
      (setq *executed-actions* (read executed-actions-f)))))


(defun dump-scene-states (&key (stamp "latest") (filename "scene-states.l"))
  (let ((filepath (format nil "~A/~A/~A" *training-data-path* stamp filename)))
    (dump-structure filepath *scene-states*)))


(defun load-scene-states (&key (stamp "latest") (filename "scene-states.l"))
  (let ((filepath (format nil "~A/~A/~A" *training-data-path* stamp filename)))
    (with-open-file (scene-states-f filepath)
      (setq *scene-states* (read scene-states-f)))))


(defun dump-pddl-graph (&key (stamp "latest") (filename "pddl-graph.l"))
  (let ((filepath (format nil "~A/~A/~A" *training-data-path* stamp filename)))
    (dump-structure filepath *pddl-graph*)))


(defun load-pddl-graph (&key (stamp "latest") (filename "pddl-graph.l"))
  (let ((filepath (format nil "~A/~A/~A" *training-data-path* stamp filename)))
    (with-open-file (pddl-graph-f filepath)
      (setq *pddl-graph* (read pddl-graph-f)))))


(defun append-executed-actions (action-name action)
  (let ((executed-action
          (list (read-from-string (format nil ":~A" action-name)) action)))
    (setq *executed-actions* (append *executed-actions* (list executed-action)))))


(defun append-scene-states (scene-state)
  (setq *scene-states* (append *scene-states* (list scene-state))))


(defun add-action-state-in-graph (action prev-scene-state scene-state)
  (let ((prev-node (send *pddl-graph* :search-node-from-pddl-state prev-scene-state))
        (node (send *pddl-graph* :search-node-from-pddl-state scene-state)))
    (if (null prev-node)
      (setq prev-node (send *pddl-graph* :add-pddl-state-node prev-scene-state :node-name :pprint)))
    (if (null node)
      (setq node (send *pddl-graph* :add-pddl-state-node scene-state :node-name :pprint)))
    (send *pddl-graph* :add-arc-from-to prev-node node (format nil "~A" action))))


(defun update-scene-state (motion-symbols-and-actions)
  (let* ((prev-larm-contact-state (car *object-contact-states*))
         (prev-rarm-contact-state (cadr *object-contact-states*))
         (prev-floor-contact-state (caddr *object-contact-states*))
         (prev-object-location-state *object-location-state*)
         (prev-scene-state (list (list :larm-contact-state prev-larm-contact-state)
                                 (list :rarm-contact-state prev-rarm-contact-state)
                                 (list :floor-contact-state prev-floor-contact-state)
                                 (list :object-location-state prev-object-location-state))))
    (if (null *scene-states*) (append-scene-states prev-scene-state))
    (unless motion-symbols-and-actions (return-from update-scene-state nil))
    (let* ((motion-symbols (car motion-symbols-and-actions))
           (action-name (cadr motion-symbols-and-actions))
           (action (caddr motion-symbols-and-actions))
           (larm-motion-symbol (car motion-symbols))
           (rarm-motion-symbol (cadr motion-symbols))
           (larm-conditions *larm-contact-state-conditions*)
           (rarm-conditions *rarm-contact-state-conditions*)
           (floor-conditions *floor-contact-state-conditions*)
           (object-conditions *object-location-state-conditions*)
           (larm-contact-state
             (eval (append '(cond) larm-conditions '((t prev-larm-contact-state)))))
           (rarm-contact-state
             (eval (append '(cond) rarm-conditions '((t prev-rarm-contact-state)))))
           (floor-contact-state
             (eval (append '(cond) floor-conditions '((t prev-floor-contact-state)))))
           (object-location-state
             (eval (append '(cond) object-conditions '((t prev-object-location-state)))))
           (scene-state (list (list :larm-contact-state larm-contact-state)
                              (list :rarm-contact-state rarm-contact-state)
                              (list :floor-contact-state floor-contact-state)
                              (list :object-location-state object-location-state))))
      (setq *object-contact-states*
            (list larm-contact-state rarm-contact-state floor-contact-state))
      (setq *object-location-state* object-location-state)
      (append-scene-states scene-state)
      (append-executed-actions action-name action)
      (add-action-state-in-graph action prev-scene-state scene-state)
      (if (null *dump-dir*) (setq *dump-dir* (create-dump-dir)))
      (dump-scene-states)
      (dump-executed-actions)
      (dump-pddl-graph))))


(provide :common-planners "common-planners.l")
