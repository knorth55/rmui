#!/usr/bin/env roseus

(require :imu-utils "package://rmuieus/euslisp/imu-utils.l")
(require :prx-utils "package://rmuieus/euslisp/prx-utils.l")
(require :pr2-motions "package://rmuieus/euslisp/pr2-motions.l")
(require :pr2-planners "package://rmuieus/euslisp/pr2-planners.l")
(require :rmui-planners "package://rmuieus/euslisp/rmui-planners.l")

(ros::load-ros-manifest "force_proximity_ros")


;; functions

(defun proximity-cb (msg)
  (let* ((proximity (mapcar #'(lambda (x) (send x :average)) (send msg :proximities)))
         (update-p (update-device-orientation))
         (contact-positions (if update-p (get-contact-positions proximity)))
         (motion-symbol (get-motion-symbol contact-positions))
         (execute-p (execute-motion-symbol motion-symbol))
         (update-device-p (if execute-p (update-device-state motion-symbol))))
    (ros::ros-info (format nil "motion-symbol: ~A" motion-symbol))
    (ros::ros-info (format nil "execute-p    : ~A" execute-p))
    (ros::ros-info (format nil "device-state : ~A" *device-state*))))


(defun main ()
  (ros::roseus "rmuieus")
  (planner-init)
  (setq *tfl* (instance ros::transform-listener :init))
  (ros::rate 10)
  (ros::subscribe "/rmui0/rmui_node/output/proximities" force_proximity_ros::ProximityArray
                  #'proximity-cb)
  (ros::spin))
