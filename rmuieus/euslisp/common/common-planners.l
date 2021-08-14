(require :common-conditions "package://rmuieus/euslisp/common/common-conditions.l")
(require :prx-utils "package://rmuieus/euslisp/rmui/prx-utils.l")

(ros::load-ros-manifest "std_msgs")


;; parameters

(defvar *device* (make-cube 120 120 120))
(defvar *device-frame-id* "rmui_link")
(defvar *prx-threshold* 800)
(defvar *prx-position-threshold* 0.5)
;; (defvar *motion-symbol-length* 10)
(defvar *motion-symbol-length* 5)
(defvar *world-frame-id* "world")
(defvar *waiting-visualize-topic-name* "/rmui0/rmui_node/output/status/waiting")
(defvar *moving-visualize-topic-name* "/rmui0/rmui_node/output/status/moving")


;; initial conditions

(setq *object-contact-states* (list :released :released :bottom-contact))
(setq *object-location-state* :center)
(setq *motion-symbol-queue* (list nil nil))


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


(defun execute-motion-symbols (motion-symbols &key (move-robot t) (use-torso 0.001))
  (let ((larm-motion-symbol-queue (car *motion-symbol-queue*))
        (rarm-motion-symbol-queue (cadr *motion-symbol-queue*))
        (larm-motion-symbol (car motion-symbols))
        (rarm-motion-symbol (cadr motion-symbols))
        (larm-contact-state (car *object-contact-states*))
        (rarm-contact-state (cadr *object-contact-states*))
        (floor-contact-state (caddr *object-contact-states*))
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
    (let ((larm-conditions *larm-execute-motion-symbols-conditions*)
          (rarm-conditions *rarm-execute-motion-symbols-conditions*)
          (dualarm-conditions *dualarm-execute-motion-symbols-conditions*))
      (if (or larm-motion-symbol rarm-motion-symbol)
        (progn
          (if move-robot
            (progn
              (ros::publish *waiting-visualize-topic-name*
                            (instance std_msgs::Float32 :init :data 1.0))
              (ros::publish *moving-visualize-topic-name*
                            (instance std_msgs::Float32 :init :data 1.0))))
          (if (equal larm-motion-symbol rarm-motion-symbol)
            (eval (append '(cond) dualarm-conditions '((t nil)))))
          (if larm-motion-symbol
            (eval (append '(cond) larm-conditions '((t nil)))))
          (if rarm-motion-symbol
            (eval (append '(cond) rarm-conditions '((t nil))))))))))


(defun update-scene-state (motion-symbols)
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
    (setq *object-location-state* object-location-state)))


(provide :common-planners "common-planners.l")
