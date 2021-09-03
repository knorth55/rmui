;; conditions

(defvar *conditions-path*
  (ros::resolve-ros-path "package://rmuieus/euslisp/common/conditions"))


;; functions

(defun rotate-rotation-symbol (rotation-symbol angle)
  (let* ((rotation-symbol-string (string-downcase rotation-symbol))
         (rotation-angle
           (+ (read-from-string (subseq rotation-symbol-string 1)) angle))
         (rotated-rotation-symbol
           (read-from-string
             (format nil ":~A~A"
                     (subseq rotation-symbol-string 0 1)
                     (cond ((> rotation-angle 180) (- rotation-angle 360))
                           ((<= rotation-angle -180) (+ rotation-angle 360))
                           (t rotation-angle))))))
    rotated-rotation-symbol))


;; get-motion-symbols

(defun load-larm-get-motion-symbols-conditions ()
  (with-open-file
    (larm-get-motion-symbols-conditions-f
      (format nil "~A/get_motion_symbols/larm-get-motion-symbols-conditions.l"
              *conditions-path*))
    (read larm-get-motion-symbols-conditions-f)))


(defun load-rarm-get-motion-symbols-conditions ()
  (with-open-file
    (rarm-get-motion-symbols-conditions-f
      (format nil "~A/get_motion_symbols/rarm-get-motion-symbols-conditions.l"
              *conditions-path*))
    (read rarm-get-motion-symbols-conditions-f)))


(defun load-dualarm-get-motion-symbols-conditions ()
  (with-open-file
    (dualarm-get-motion-symbols-conditions-f
      (format nil "~A/get_motion_symbols/dualarm-get-motion-symbols-conditions.l"
              *conditions-path*))
    (read dualarm-get-motion-symbols-conditions-f)))


;; update-scene-state

(defun load-larm-contact-state-conditions ()
  (with-open-file
    (larm-contact-state-conditions-f
      (format nil "~A/update_scene_state/larm-contact-state-conditions.l"
              *conditions-path*))
    (read larm-contact-state-conditions-f)))


(defun load-rarm-contact-state-conditions ()
  (with-open-file
    (rarm-contact-state-conditions-f
      (format nil "~A/update_scene_state/rarm-contact-state-conditions.l"
              *conditions-path*))
    (read rarm-contact-state-conditions-f)))


(defun load-floor-contact-state-conditions ()
  (with-open-file
    (floor-contact-state-conditions-f
      (format nil "~A/update_scene_state/floor-contact-state-conditions.l"
              *conditions-path*))
    (read floor-contact-state-conditions-f)))


(defun load-object-location-state-conditions ()
  (with-open-file
    (object-location-state-conditions-f
      (format nil "~A/update_scene_state/object-location-state-conditions.l"
              *conditions-path*))
    (read object-location-state-conditions-f)))


(defun load-object-rotation-state-conditions ()
  (with-open-file
    (object-rotation-state-conditions-f
      (format nil "~A/update_scene_state/object-rotation-state-conditions.l"
              *conditions-path*))
    (read object-rotation-state-conditions-f)))


(provide :common-conditions "common-conditions.l")
