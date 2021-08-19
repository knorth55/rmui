;; conditions

;; get-motion-symbols

(setq *conditions-path* (ros::resolve-ros-path "package://rmuieus/euslisp/common/conditions"))

(with-open-file
  (larm-get-motion-symbols-conditions-f
    (format nil "~A/get_motion_symbols/larm-get-motion-symbols-conditions.l" *conditions-path*))
  (setq *larm-get-motion-symbols-conditions* (read larm-get-motion-symbols-conditions-f)))

(with-open-file
  (rarm-get-motion-symbols-conditions-f
    (format nil "~A/get_motion_symbols/rarm-get-motion-symbols-conditions.l" *conditions-path*))
  (setq *rarm-get-motion-symbols-conditions* (read rarm-get-motion-symbols-conditions-f)))

(with-open-file
  (dualarm-get-motion-symbols-conditions-f
    (format nil "~A/get_motion_symbols/dualarm-get-motion-symbols-conditions.l" *conditions-path*))
  (setq *dualarm-get-motion-symbols-conditions* (read dualarm-get-motion-symbols-conditions-f)))

;; update-scene-state

(with-open-file
  (larm-contact-state-conditions-f
    (format nil "~A/update_scene_state/larm-contact-state-conditions.l" *conditions-path*))
  (setq *larm-contact-state-conditions* (read larm-contact-state-conditions-f)))

(with-open-file
  (rarm-contact-state-conditions-f
    (format nil "~A/update_scene_state/rarm-contact-state-conditions.l" *conditions-path*))
  (setq *rarm-contact-state-conditions* (read rarm-contact-state-conditions-f)))

(with-open-file
  (floor-contact-state-conditions-f
    (format nil "~A/update_scene_state/floor-contact-state-conditions.l" *conditions-path*))
  (setq *floor-contact-state-conditions* (read floor-contact-state-conditions-f)))

(with-open-file
  (object-location-state-conditions-f
    (format nil "~A/update_scene_state/object-location-state-conditions.l" *conditions-path*))
  (setq *object-location-state-conditions* (read object-location-state-conditions-f)))

(with-open-file
  (object-rotation-state-conditions-f
    (format nil "~A/update_scene_state/object-rotation-state-conditions.l" *conditions-path*))
  (setq *object-rotation-state-conditions* (read object-rotation-state-conditions-f)))


(provide :common-conditions "common-conditions.l")
