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


;; functions

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
          (update-executed-actions action-name action))))
    (if execute-symbols (dump-executed-actions))
    execute-symbols))


(defun dump-executed-actions (&key (filename "executed-actions.l"))
  (dump-structure (format nil "~A/~A" *training-data-path* filename) *executed-actions*))


(defun load-executed-actions (&key (filename "executed-actions.l"))
  (let ((filepath (format nil "~A/~A" *training-data-path* filename)))
    (with-open-file (executed-actions-f filepath)
      (setq *executed-actions* (read executed-actions-f)))))


(defun dump-scene-states (&key (filename "scene-states.l"))
  (dump-structure (format nil "~A/~A" *training-data-path* filename) *scene-states*))


(defun load-scene-states (&key (filename "scene-states.l"))
  (let ((filepath (format nil "~A/~A" *training-data-path* filename)))
    (with-open-file (scene-states-f filepath)
      (setq *scene-states* (read scene-states-f)))))


(defun update-executed-actions (action-name action)
  (setq *executed-actions*
        (append *executed-actions*
                (list (list (read-from-string (format nil ":~A" action-name)) action)))))


(defun update-scene-states ()
  (let ((larm-contact-state (car *object-contact-states*))
        (rarm-contact-state (cadr *object-contact-states*))
        (floor-contact-state (caddr *object-contact-states*))
        (object-location-state *object-location-state*))
    (setq *scene-states*
          (append *scene-states*
                  (list (list (list :larm-contact-state larm-contact-state)
                              (list :rarm-contact-state rarm-contact-state)
                              (list :floor-contact-state floor-contact-state)
                              (list :object-location-state object-location-state)))))))


(defun update-scene-state (motion-symbols)
  (if (null *scene-states*) (update-scene-states))
  (unless motion-symbols (return-from update-scene-state nil))
  (let ((larm-motion-symbol (car motion-symbols))
        (rarm-motion-symbol (cadr motion-symbols))
        (larm-contact-state (car *object-contact-states*))
        (rarm-contact-state (cadr *object-contact-states*))
        (floor-contact-state (caddr *object-contact-states*))
        (object-location-state *object-location-state*)
        (larm-conditions *larm-contact-state-conditions*)
        (rarm-conditions *rarm-contact-state-conditions*)
        (floor-conditions *floor-contact-state-conditions*)
        (object-conditions *object-location-state-conditions*))
    (setq larm-contact-state
          (eval (append '(cond) larm-conditions '((t larm-contact-state)))))
    (setq rarm-contact-state
          (eval (append '(cond) rarm-conditions '((t rarm-contact-state)))))
    (setq floor-contact-state
          (eval (append '(cond) floor-conditions '((t floor-contact-state)))))
    (setq object-location-state
          (eval (append '(cond) object-conditions '((t object-location-state)))))
    (setq *object-contact-states*
          (list larm-contact-state rarm-contact-state floor-contact-state))
    (setq *object-location-state* object-location-state)
    (update-scene-states)
    (dump-scene-states)))


(provide :common-planners "common-planners.l")
