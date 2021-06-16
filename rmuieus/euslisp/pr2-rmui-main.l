#!/usr/bin/env roseus

(require :imu-utils "package://rmuieus/euslisp/rmui/imu-utils.l")
(require :prx-utils "package://rmuieus/euslisp/rmui/prx-utils.l")
(require :pr2-motions "package://rmuieus/euslisp/pr2/pr2-motions.l")
(require :pr2-planners "package://rmuieus/euslisp/pr2/pr2-planners.l")
(require :rmui-planners "package://rmuieus/euslisp/rmui/rmui-planners.l")

(ros::load-ros-manifest "force_proximity_ros")


;; functions

(defun proximity-cb (msg)
  (let* ((proximity (mapcar #'(lambda (x) (send x :average)) (send msg :proximities)))
         (update-p (update-device-orientation))
         (contact-positions
           (if update-p
             (get-contact-positions proximity *prx-threshold* *prx-position-threshold*)))
         (motion-symbols (get-motion-symbols contact-positions))
         (execute-symbols (execute-motion-symbols motion-symbols))
         (update-object-p (update-object-contact-state execute-symbols)))
    (send *irtviewer* :draw-objects)
    (send *ri* :draw-objects)
    (ros::ros-info (format nil "motion-symbols        : ~A" motion-symbols))
    (ros::ros-info (format nil "execute-symbols       : ~A" execute-symbols))
    ; (ros::ros-info (format nil "contact-positions     : ~A" contact-positions))
    (ros::ros-info (format nil "object-contact-states : ~A" *object-contact-states*))
    (ros::ros-info (format nil "object-location-state : ~A" *object-location-state*))))


(defun main ()
  (ros::roseus "rmuieus")
  (planner-init)
  (setq *tfl* (instance ros::transform-listener :init))
  (ros::rate 10)
  (ros::subscribe "/rmui0/rmui_node/output/proximities" force_proximity_ros::ProximityArray
                  #'proximity-cb)
  (ros::spin))
