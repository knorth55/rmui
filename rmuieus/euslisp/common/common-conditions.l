;; conditions

(with-open-file
  (larm-get-motion-symbols-conditions-f
    (ros::resolve-ros-path
      "package://rmuieus/euslisp/conditions/get_motion_symbols/larm-get-motion-symbols-conditions.l"))
  (setq *larm-get-motion-symbols-conditions* (read larm-get-motion-symbols-conditions-f)))
(with-open-file
  (rarm-get-motion-symbols-conditions-f
    (ros::resolve-ros-path
      "package://rmuieus/euslisp/conditions/get_motion_symbols/rarm-get-motion-symbols-conditions.l"))
  (setq *rarm-get-motion-symbols-conditions* (read rarm-get-motion-symbols-conditions-f)))
(with-open-file
  (dualarm-get-motion-symbols-conditions-f
    (ros::resolve-ros-path
      "package://rmuieus/euslisp/conditions/get_motion_symbols/dualarm-get-motion-symbols-conditions.l"))
  (setq *dualarm-get-motion-symbols-conditions* (read dualarm-get-motion-symbols-conditions-f)))

(with-open-file
  (larm-execute-motion-symbols-conditions-f
    (ros::resolve-ros-path
      "package://rmuieus/euslisp/conditions/execute_motion_symbols/larm-execute-motion-symbols-conditions.l"))
  (setq *larm-execute-motion-symbols-conditions* (read larm-execute-motion-symbols-conditions-f)))
(with-open-file
  (rarm-execute-motion-symbols-conditions-f
    (ros::resolve-ros-path
      "package://rmuieus/euslisp/conditions/execute_motion_symbols/rarm-execute-motion-symbols-conditions.l"))
  (setq *rarm-execute-motion-symbols-conditions* (read rarm-execute-motion-symbols-conditions-f)))
(with-open-file
  (dualarm-execute-motion-symbols-conditions-f
    (ros::resolve-ros-path
      "package://rmuieus/euslisp/conditions/execute_motion_symbols/dualarm-execute-motion-symbols-conditions.l"))
  (setq *dualarm-execute-motion-symbols-conditions* (read dualarm-execute-motion-symbols-conditions-f)))

(with-open-file
  (larm-contact-state-conditions-f
    (ros::resolve-ros-path
      "package://rmuieus/euslisp/conditions/update_scene_state/larm-contact-state-conditions.l"))
  (setq *larm-contact-state-conditions* (read larm-contact-state-conditions-f)))
(with-open-file
  (rarm-contact-state-conditions-f
    (ros::resolve-ros-path
      "package://rmuieus/euslisp/conditions/update_scene_state/rarm-contact-state-conditions.l"))
  (setq *rarm-contact-state-conditions* (read rarm-contact-state-conditions-f)))
(with-open-file
  (floor-contact-state-conditions-f
    (ros::resolve-ros-path
      "package://rmuieus/euslisp/conditions/update_scene_state/floor-contact-state-conditions.l"))
  (setq *floor-contact-state-conditions* (read floor-contact-state-conditions-f)))
(with-open-file
  (object-location-state-conditions-f
    (ros::resolve-ros-path
      "package://rmuieus/euslisp/conditions/update_scene_state/object-location-state-conditions.l"))
  (setq *object-location-state-conditions* (read object-location-state-conditions-f)))


(provide :common-conditions "common-conditions.l")
