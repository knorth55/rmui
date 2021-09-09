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
          larm-get-motion-symbols-conditions
          rarm-get-motion-symbols-conditions
          dualarm-get-motion-symbols-conditions
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
    (setq larm-get-motion-symbols-conditions
          (load-larm-get-motion-symbols-conditions))
    (setq rarm-get-motion-symbols-conditions
          (load-rarm-get-motion-symbols-conditions))
    (setq dualarm-get-motion-symbols-conditions
          (load-dualarm-get-motion-symbols-conditions))
    self)
  (:get-motion-symbols
    (contact-positions
         &key (object-contact-states) (object-location-state) (object-rotation-state))
    (let* ((larm-motion-symbols
             (send self :get-motion-symbol :larm contact-positions
                   :object-contact-states object-contact-states
                   :object-location-state object-location-state
                   :object-rotation-state object-rotation-state))
           (rarm-motion-symbols
             (send self :get-motion-symbol :rarm contact-positions
                   :object-contact-states object-contact-states
                   :object-location-state object-location-state
                   :object-rotation-state object-rotation-state)))
      (list larm-motion-symbols rarm-motion-symbols)))
  (:get-motion-symbol
    (arm contact-positions
         &key (object-contact-states) (object-location-state) (object-rotation-state))
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
  )


(provide :common-rmui-planner "common-rmui-planner.l")
