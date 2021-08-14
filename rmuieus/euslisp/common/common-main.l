(require :common-planners "package://rmuieus/euslisp/common/common-planners.l")

(ros::load-ros-manifest "force_proximity_ros")
(ros::load-ros-manifest "std_msgs")


;; functions

(defun proximity-cb (msg)
  (let* ((proximity (mapcar #'(lambda (x) (send x :average)) (send msg :proximities)))
         (update-p (update-device-orientation))
         (contact-positions
           (if update-p
             (get-contact-positions proximity *prx-threshold* *prx-position-threshold*)))
         (motion-symbols (get-motion-symbols contact-positions))
         (execute-symbols (execute-motion-symbols motion-symbols))
         (update-scene-p (update-scene-state execute-symbols)))
    (send *irtviewer* :draw-objects)
    (send *ri* :draw-objects)
    (ros::ros-info (format nil "motion-symbols        : ~A" motion-symbols))
    (ros::ros-info (format nil "execute-symbols       : ~A" execute-symbols))
    ; (ros::ros-info (format nil "contact-positions     : ~A" contact-positions))
    (ros::ros-info (format nil "object-contact-states : ~A" *object-contact-states*))
    (ros::ros-info (format nil "object-location-state : ~A" *object-location-state*))))


(defun main ()
  (ros::roseus "rmuieus")
  (motion-init)
  (ros::ros-info "motion-init finished")
  (setq *tfl* (instance ros::transform-listener :init))
  (ros::rate 10)
  (ros::advertise *waiting-visualize-topic-name* std_msgs::Float32 1)
  (ros::advertise *moving-visualize-topic-name* std_msgs::Float32 1)
  (ros::subscribe "/rmui0/rmui_node/output/proximities" force_proximity_ros::ProximityArray
                  #'proximity-cb)
  (ros::ros-info "initialization finished, starting")
  (ros::spin))

(provide :common-main "common-main.l")
