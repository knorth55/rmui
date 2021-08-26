(require :common-planners "package://rmuieus/euslisp/common/common-planners.l")

(ros::roseus-add-msgs "force_proximity_ros")
(ros::roseus-add-msgs "std_msgs")


;; parameters

(defvar *prx-threshold* 800)
(defvar *prx-position-threshold* 0.5)


;; functions

(defun proximity-cb (msg)
  (let* ((proximity (mapcar #'(lambda (x) (send x :average)) (send msg :proximities)))
         (update-p (update-device-orientation))
         (contact-positions
           (if update-p
             (get-contact-positions proximity *prx-threshold* *prx-position-threshold*)))
         (motion-symbols (get-motion-symbols contact-positions))
         (execute-symbols-and-actions (execute-motion-symbols motion-symbols))
         (update-scene-p (update-scene-state execute-symbols-and-actions))
         (execute-symbols (car execute-symbols-and-actions)))
    (send *irtviewer* :draw-objects)
    (send *ri* :draw-objects)
    (ros::ros-info (format nil "motion-symbols        : ~A" motion-symbols))
    (ros::ros-info (format nil "execute-symbols       : ~A" execute-symbols))
    ; (ros::ros-info (format nil "contact-positions     : ~A" contact-positions))
    (ros::ros-info (format nil "object-contact-states : ~A" *object-contact-states*))
    (ros::ros-info (format nil "object-location-state : ~A" *object-location-state*))
    (ros::ros-info (format nil "object-rotation-state : ~A" *object-rotation-state*))
    (ros::ros-info (format nil "executed-actions: ~A" (mapcar #'car *executed-actions*)))
    ))


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
