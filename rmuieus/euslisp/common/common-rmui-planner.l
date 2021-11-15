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
  ;; TODO (knorth55): support multiple devices
  (:get-motion-symbols
    (contact-positions-list &key (scene-state))
    (let* ((larm-motion-symbols
             (send self :get-motion-symbol :larm
                   (car contact-positions-list)
                   :scene-state scene-state))
           (rarm-motion-symbols
             (send self :get-motion-symbol :rarm
                   (car contact-positions-list)
                   :scene-state scene-state)))
      (list larm-motion-symbols rarm-motion-symbols)))
  (:get-motion-symbol
    (arm contact-positions &key (scene-state))
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
           (contact-surfaces (mapcar #'car contact-positions))
           (bottom-contact-positions
             (remove-if #'null
                        (mapcar #'(lambda (x) (if (equal (car x) :bottom) (cadr x) nil))
                                contact-positions)))
           (arm-conditions (cond ((equal arm :larm) larm-get-motion-symbol-conditions)
                                 ((equal arm :rarm) rarm-get-motion-symbol-conditions)
                                 (t nil)))
           (dualarm-conditions dualarm-get-motion-symbol-conditions))
      (ros::publish waiting-visualize-topic-name
                    (instance std_msgs::Float32 :init :data 0.0))
      (ros::publish moving-visualize-topic-name
                    (instance std_msgs::Float32 :init :data 0.0))
      (eval (append '(cond) arm-conditions dualarm-conditions
                    '(((null contact-positions) nil) (t nil))))))
  )


(provide :common-rmui-planner "common-rmui-planner.l")
