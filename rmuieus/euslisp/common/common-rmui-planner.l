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


(defclass rmui-planner
  :super propertied-object
  :slots (namespace
          waiting-visualize-topic-name
          moving-visualize-topic-name
          larm-get-motion-symbol-conditions
          rarm-get-motion-symbol-conditions
          dualarm-get-motion-symbol-conditions
          ))


(defmethod rmui-planner
  (:init (&key ((:namespace ns)))
    (setq namespace (if ns ns ""))
    (setq waiting-visualize-topic-name
          (format nil "~A/output/status/waiting" namespace))
    (setq moving-visualize-topic-name
          (format nil "~A/output/status/moving" namespace))
    (ros::advertise waiting-visualize-topic-name std_msgs::Float32 1)
    (ros::advertise moving-visualize-topic-name std_msgs::Float32 1)
    (setq larm-get-motion-symbol-conditions
          (load-larm-get-motion-symbol-conditions))
    (setq rarm-get-motion-symbol-conditions
          (load-rarm-get-motion-symbol-conditions))
    (setq dualarm-get-motion-symbol-conditions
          (load-dualarm-get-motion-symbol-conditions))
    self)
  (:get-motion-symbols
    (contact-positions &key (scene-states))
    (let ((motion-symbols nil)
          (push-hold-down-index nil)
          (push-hold-up-index nil)
          (left-push-index nil)
          (right-push-index nil)
          (front-push-index nil)
          (back-push-index nil))
      (dotimes (i (length contact-positions))
        (let* ((contact-position (elt contact-positions i))
               (scene-state (elt scene-states i))
               (larm-motion-symbol
                 (send self :get-motion-symbol :larm
                       contact-position
                       :scene-state scene-state))
               (rarm-motion-symbol
                 (send self :get-motion-symbol :rarm
                       contact-position
                       :scene-state scene-state))
               (larm-contact-state
                 (cadr (assoc :larm-contact-state scene-state)))
               (rarm-contact-state
                 (cadr (assoc :rarm-contact-state scene-state)))
               (floor-contact-state
                 (cadr (assoc :floor-contact-state scene-state)))
               (cube-contact-state
                 (cadr (assoc :cube-contact-state scene-state))))
          ;; pile up
          (if (and (or (and (equal larm-contact-state :front-contact)
                            (equal rarm-contact-state :back-contact))
                       (and (equal larm-contact-state :back-contact)
                            (equal rarm-contact-state :front-contact))
                       (and (equal larm-contact-state :left-contact)
                            (equal rarm-contact-state :right-contact)))
                   (equal larm-motion-symbol :push-hold-down)
                   (equal rarm-motion-symbol :push-hold-down)
                   (not (find :bottom-contact cube-contact-state)))
            (setq push-hold-down-index i))
          ;; lift up from pile up
          (if (and (or (and (equal larm-contact-state :front-contact)
                            (equal rarm-contact-state :back-contact))
                       (and (equal larm-contact-state :back-contact)
                            (equal rarm-contact-state :front-contact))
                       (and (equal larm-contact-state :left-contact)
                            (equal rarm-contact-state :right-contact)))
                   (equal larm-motion-symbol :push-hold-up)
                   (equal rarm-motion-symbol :push-hold-up)
                   (find :bottom-contact cube-contact-state))
            (setq push-hold-up-index i))
          ;; next to
          (if (and (equal rarm-contact-state :right-contact)
                   (equal larm-motion-symbol :left-push))
            (setq left-push-index i))
          (if (and (equal rarm-contact-state :front-contact)
                   (equal larm-motion-symbol :back-push))
            (setq back-push-index i))
          (if (and (equal larm-contact-state :left-contact)
                   (equal rarm-motion-symbol :right-push))
            (setq right-push-index i))
          (if (and (equal larm-contact-state :back-contact)
                   (equal rarm-motion-symbol :front-push))
            (setq front-push-index i))
          ;; push hold
          (if (and (equal larm-motion-symbol :left-push)
                   (equal rarm-motion-symbol :right-push))
            (progn
              (setq larm-motion-symbol :left-right-push-hold)
              (setq rarm-motion-symbol :left-right-push-hold)))
          (if (and (equal larm-motion-symbol :front-left-push)
                   (equal rarm-motion-symbol :back-right-push))
            (progn
              (setq larm-motion-symbol :front-left-back-right-push-hold)
              (setq rarm-motion-symbol :front-left-back-right-push-hold)))
          (if (and (equal larm-motion-symbol :back-left-push)
                   (equal rarm-motion-symbol :front-right-push))
            (progn
              (setq larm-motion-symbol :back-left-front-right-push-hold)
              (setq rarm-motion-symbol :back-left-front-right-push-hold)))
          ;; hold release
          (if (and (equal larm-motion-symbol :push-release)
                   (equal rarm-motion-symbol :push-release))
            (progn
              (setq larm-motion-symbol :hold-release)
              (setq rarm-motion-symbol :hold-release)))
          ;; wall push
          ;; TODO: priority ?
          (if (and (equal larm-motion-symbol :left-push)
                   (equal rarm-motion-symbol :left-wall-push-next-to))
            (setq larm-motion-symbol nil))
          (setq motion-symbols
                (append motion-symbols
                        (list (list larm-motion-symbol rarm-motion-symbol))))))
      (if (or push-hold-down-index
              push-hold-up-index
              left-push-index
              right-push-index
              front-push-index
              back-push-index)
        (dotimes (i (length motion-symbols))
          (let* ((motion-symbol (elt motion-symbols i))
                 (contact-position (elt contact-positions i))
                 (contact-surface (mapcar #'car contact-position))
                 (scene-state (elt scene-states i))
                 (larm-contact-state
                   (cadr (assoc :larm-contact-state scene-state)))
                 (rarm-contact-state
                   (cadr (assoc :rarm-contact-state scene-state)))
                 (floor-contact-state
                   (cadr (assoc :floor-contact-state scene-state)))
                 (cube-contact-state
                   (cadr (assoc :cube-contact-state scene-state)))
                 (left-wall-contact-state
                   (cadr (assoc :left-wall-contact-state scene-state)))
                 (front-wall-contact-state
                   (cadr (assoc :front-wall-contact-state scene-state))))
            (cond
              ;; pile-up
              ((and push-hold-down-index
                    (not (equal push-hold-down-index i))
                    (equal (elt cube-contact-state push-hold-down-index) :released)
                    (not (find :top-contact cube-contact-state))
                    ;; top-push
                    (or (equal floor-contact-state :bottom-contact)
                        (equal floor-contact-state :bottom-left-side-contact)
                        (equal floor-contact-state :bottom-right-side-contact)
                        (find :bottom-contact cube-contact-state))
                    (find :top contact-surface)
                    (find :bottom contact-surface))
               (setf (car (elt motion-symbols push-hold-down-index))
                     (list :push-hold-pile-up :cube-index i))
               (setf (cadr (elt motion-symbols push-hold-down-index))
                     (list :push-hold-pile-up :cube-index i)))
              ;; lift up from pile up
              ((and push-hold-up-index
                    (not (equal push-hold-up-index i))
                    ;; top-push
                    (equal (elt cube-contact-state push-hold-up-index) :top-contact)
                    (or (equal floor-contact-state :bottom-contact)
                        (equal floor-contact-state :bottom-left-side-contact)
                        (equal floor-contact-state :bottom-right-side-contact)
                        (find :bottom-contact cube-contact-state))
                    (find :top contact-surface)
                    (find :bottom contact-surface))
               (setf (car (elt motion-symbols push-hold-up-index)) nil)
               (setf (cadr (elt motion-symbols push-hold-up-index)) nil))
              ;; next to
              ((and left-push-index
                    (not (equal left-push-index i))
                    (equal (elt cube-contact-state left-push-index) :released)
                    ;; right-push
                    (not (find :right-contact cube-contact-state))
                    (or (equal floor-contact-state :bottom-contact)
                        (equal floor-contact-state :bottom-left-side-contact)
                        (equal floor-contact-state :bottom-right-side-contact))
                    ;; (equal right-wall-contact-state :released)
                    (find :right contact-surface)
                    (find :bottom contact-surface))
               (setf (car (elt motion-symbols left-push-index)) nil)
               (setf (cadr (elt motion-symbols left-push-index))
                     (list :left-push-next-to :cube-index i)))
              ((and back-push-index
                    (not (equal back-push-index i))
                    (equal (elt cube-contact-state back-push-index) :released)
                    ;; front-push
                    (not (find :front-contact cube-contact-state))
                    (or (equal floor-contact-state :bottom-contact)
                        (equal floor-contact-state :bottom-left-side-contact)
                        (equal floor-contact-state :bottom-right-side-contact))
                    (equal front-wall-contact-state :released)
                    (find :front contact-surface)
                    (find :bottom contact-surface))
               (setf (car (elt motion-symbols back-push-index)) nil)
               (setf (cadr (elt motion-symbols back-push-index))
                     (list :back-push-next-to :cube-index)))
              ((and right-push-index
                    (not (equal right-push-index i))
                    (equal (elt cube-contact-state right-push-index) :released)
                    ;; left-push
                    (not (find :left-contact cube-contact-state))
                    (or (equal floor-contact-state :bottom-contact)
                        (equal floor-contact-state :bottom-left-side-contact)
                        (equal floor-contact-state :bottom-right-side-contact))
                    (equal left-wall-contact-state :released)
                    (find :left contact-surface)
                    (find :bottom contact-surface))
               (setf (car (elt motion-symbols right-push-index))
                     (list :right-push-next-to :cube-index i))
               (setf (cadr (elt motion-symbols right-push-index)) nil))
              ((and front-push-index
                    (not (equal front-push-index i))
                    (equal (elt cube-contact-state front-push-index) :released)
                    ;; back-push
                    (not (find :back-contact cube-contact-state))
                    (or (equal floor-contact-state :bottom-contact)
                        (equal floor-contact-state :bottom-left-side-contact)
                        (equal floor-contact-state :bottom-right-side-contact))
                    ;; (equal back-wall-contact-state :released)
                    (find :back contact-surface)
                    (find :bottom contact-surface))
               (setf (car (elt motion-symbols front-push-index))
                     (list :front-push-next-to :cube-index i))
               (setf (cadr (elt motion-symbols front-push-index)) nil))
              (t nil))
            )))
      motion-symbols))
  (:get-motion-symbol
    (arm contact-position &key (scene-state))
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
           (cube-contact-state
             (cadr (assoc :cube-contact-state scene-state)))
           (left-wall-contact-state
             (cadr (assoc :left-wall-contact-state scene-state)))
           (front-wall-contact-state
             (cadr (assoc :front-wall-contact-state scene-state)))
           (contact-surface (mapcar #'car contact-position))
           (bottom-contact-position
             (remove-if #'null
                        (mapcar #'(lambda (x) (if (equal (car x) :bottom) (cadr x) nil))
                                contact-position)))
           (arm-conditions (cond ((equal arm :larm) larm-get-motion-symbol-conditions)
                                 ((equal arm :rarm) rarm-get-motion-symbol-conditions)
                                 (t nil)))
           (dualarm-conditions dualarm-get-motion-symbol-conditions))
      (ros::publish waiting-visualize-topic-name
                    (instance std_msgs::Float32 :init :data 0.0))
      (ros::publish moving-visualize-topic-name
                    (instance std_msgs::Float32 :init :data 0.0))
      (eval (append '(cond) arm-conditions dualarm-conditions
                    '(((null contact-position) nil) (t nil))))))
  )


(provide :common-rmui-planner "common-rmui-planner.l")
