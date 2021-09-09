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


(defclass rmui-executor
  :super propertied-object
  :slots (namespace
          waiting-visualize-topic-name
          moving-visualize-topic-name
          motion-symbol-queue
          motion-symbol-length)
  )


(defmethod rmui-executor
  (:init (&key ((:namespace ns))
               ((:motion-symbol-length msym-length) 5))
    (setq namespace (if ns ns ""))
    (setq waiting-visualize-topic-name
          (format nil "~A/output/status/waiting" namespace))
    (setq moving-visualize-topic-name
          (format nil "~A/output/status/moving" namespace))
    (ros::advertise waiting-visualize-topic-name std_msgs::Float32 1)
    (ros::advertise moving-visualize-topic-name std_msgs::Float32 1)
    (setq motion-symbol-length msym-length)
    (setq motion-symbol-queue (list nil nil))
    self)
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
  (:execute-motion-symbols
    (motion-symbols
      &key (object-contact-states) (object-location-state) (object-rotation-state)
      (move-robot t) (use-torso 0.001))
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
  )


(provide :common-rmui-executor "common-rmui-executor.l")