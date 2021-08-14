(((or (and (equal larm-motion-symbol :push-hold-up)
           (equal rarm-motion-symbol :push-hold-up))
      (and (equal larm-motion-symbol :support-hold-up)
           (equal rarm-motion-symbol :support-hold-up)))
  (assoc-cube :arms)
  (hold-lift-up-motion :lift-height 200 :move-robot move-robot :use-torso use-torso)
  (dissoc-cube :arms)
  (return-from execute-motion-symbols (list larm-motion-symbol rarm-motion-symbol)))
 ((or (and (equal larm-motion-symbol :push-hold-down)
           (equal rarm-motion-symbol :push-hold-down))
      (and (equal larm-motion-symbol :support-hold-down)
           (equal rarm-motion-symbol :support-hold-down)))
  (assoc-cube :arms)
  (hold-lift-down-motion :move-robot move-robot :use-torso use-torso)
  (dissoc-cube :arms)
  (return-from execute-motion-symbols (list larm-motion-symbol rarm-motion-symbol)))
 ;; push rotate ccw x45
 ((and (equal larm-motion-symbol :push-rotate-ccw-x45)
       (equal rarm-motion-symbol :push-rotate-ccw-x45))
  (assoc-cube :arms)
  (let* ((initial-rotate-angle
           (if (and (equal larm-contact-state :bottom-left-contact)
                    (equal rarm-contact-state :top-right-contact))
             (/ -pi 4.0) 0))
         (cube-rpy (float-vector 0 0 initial-rotate-angle))
         (cube-height (get-cube-height :cube-rpy cube-rpy))
         (cube-width (get-cube-width :cube-rpy cube-rpy))
         (cube-depth (get-cube-depth :cube-rpy cube-rpy)))
    (push-rotate-motion :rotate-angle (/ pi 4.0) :rotate-axis :x
                        :initial-rotate-angle initial-rotate-angle
                        :cube-height cube-height
                        :cube-width cube-width
                        :cube-depth cube-depth
                        :move-robot move-robot
                        :use-torso use-torso))
  (dissoc-cube :arms)
  (return-from execute-motion-symbols (list larm-motion-symbol rarm-motion-symbol)))
 ;; push rotate cw x45
 ((and (equal larm-motion-symbol :push-rotate-cw-x45)
       (equal rarm-motion-symbol :push-rotate-cw-x45))
  (assoc-cube :arms)
  (let* ((initial-rotate-angle
           (if (and (equal larm-contact-state :top-left-contact)
                    (equal rarm-contact-state :bottom-right-contact))
             (/ pi 4.0) 0))
         (cube-rpy (float-vector 0 0 initial-rotate-angle))
         (cube-height (get-cube-height :cube-rpy cube-rpy))
         (cube-width (get-cube-width :cube-rpy cube-rpy))
         (cube-depth (get-cube-depth :cube-rpy cube-rpy)))
    (push-rotate-motion :rotate-angle (/ -pi 4.0) :rotate-axis :x
                        :initial-rotate-angle initial-rotate-angle
                        :cube-height cube-height
                        :cube-width cube-width
                        :cube-depth cube-depth
                        :move-robot move-robot
                        :use-torso use-torso))
  (dissoc-cube :arms)
  (return-from execute-motion-symbols (list larm-motion-symbol rarm-motion-symbol)))
 ;; lift push rotate ccw x90
 ((or (and (equal larm-motion-symbol :lift-push-rotate-ccw-x90)
           (equal rarm-motion-symbol :lift-push-rotate-ccw-x90))
      (and (equal larm-motion-symbol :side-push-rotate-ccw-x90)
           (equal rarm-motion-symbol :side-push-rotate-ccw-x90)))
  (assoc-cube :arms)
  (let* ((initial-rotate-angle 0)
         (cube-rpy (float-vector 0 0 initial-rotate-angle))
         (cube-height (get-cube-height :cube-rpy cube-rpy))
         (cube-width (get-cube-width :cube-rpy cube-rpy))
         (cube-depth (get-cube-depth :cube-rpy cube-rpy)))
    (push-rotate-motion :rotate-angle (/ pi 2.0) :rotate-axis :x
                        :initial-rotate-angle initial-rotate-angle
                        :cube-height cube-height
                        :cube-width cube-width
                        :cube-depth cube-depth
                        :move-robot move-robot
                        :use-torso use-torso))
  (dissoc-cube :arms)
  (return-from execute-motion-symbols (list larm-motion-symbol rarm-motion-symbol)))
 ;; lift push rotate cw x90
 ((or (and (equal larm-motion-symbol :lift-push-rotate-cw-x90)
           (equal rarm-motion-symbol :lift-push-rotate-cw-x90))
      (and (equal larm-motion-symbol :side-push-rotate-cw-x90)
           (equal rarm-motion-symbol :side-push-rotate-cw-x90)))
  (assoc-cube :arms)
  (let* ((initial-rotate-angle 0)
         (cube-rpy (float-vector 0 0 initial-rotate-angle))
         (cube-height (get-cube-height :cube-rpy cube-rpy))
         (cube-width (get-cube-width :cube-rpy cube-rpy))
         (cube-depth (get-cube-depth :cube-rpy cube-rpy)))
    (push-rotate-motion :rotate-angle (/ -pi 2.0) :rotate-axis :x
                        :initial-rotate-angle initial-rotate-angle
                        :cube-height cube-height
                        :cube-width cube-width
                        :cube-depth cube-depth
                        :move-robot move-robot
                        :use-torso use-torso))
  (dissoc-cube :arms)
  (return-from execute-motion-symbols (list larm-motion-symbol rarm-motion-symbol)))
 ;; push rotate ccw z45
 ((and (equal larm-motion-symbol :push-rotate-ccw-z45)
       (equal rarm-motion-symbol :push-rotate-ccw-z45))
  (assoc-cube :arms)
  (push-rotate-motion :rotate-angle (/ pi 4.0) :rotate-axis :z
                      :move-robot move-robot :use-torso use-torso)
  (dissoc-cube :arms)
  (return-from execute-motion-symbols (list larm-motion-symbol rarm-motion-symbol)))
 ;; push rotate cw z45
 ((and (equal larm-motion-symbol :push-rotate-cw-z45)
       (equal rarm-motion-symbol :push-rotate-cw-z45))
  (assoc-cube :arms)
  (push-rotate-motion :rotate-angle (/ -pi 4.0) :rotate-axis :z
                      :move-robot move-robot :use-torso use-torso)
  (dissoc-cube :arms)
  (return-from execute-motion-symbols (list larm-motion-symbol rarm-motion-symbol)))
 ((and (equal larm-motion-symbol :support-rotate-ccw-x45)
       (equal rarm-motion-symbol :support-rotate-ccw-x45))
  (assoc-cube :arms)
  (let* ((initial-rotate-angle
           (if (and (equal larm-contact-state :top-left-contact)
                    (equal rarm-contact-state :top-right-contact))
             (/ pi 4.0) 0))
         (cube-rpy (float-vector 0 0 initial-rotate-angle))
         (cube-height (get-cube-height :cube-rpy cube-rpy))
         (cube-width (get-cube-width :cube-rpy cube-rpy))
         (cube-depth (get-cube-depth :cube-rpy cube-rpy)))
    (support-rotate-motion :rotate-angle (/ pi 4.0) :rotate-axis :x
                           :initial-rotate-angle initial-rotate-angle
                           :cube-height cube-height
                           :cube-width cube-width
                           :cube-depth cube-depth
                           :move-robot move-robot
                           :use-torso use-torso))
  (dissoc-cube :arms)
  (return-from execute-motion-symbols (list larm-motion-symbol rarm-motion-symbol)))
 ((and (equal larm-motion-symbol :support-rotate-cw-x45)
       (equal rarm-motion-symbol :support-rotate-cw-x45))
  (assoc-cube :arms)
  (let* ((initial-rotate-angle
           (if (and (equal larm-contact-state :top-left-contact)
                    (equal rarm-contact-state :top-right-contact))
             (/ -pi 4.0) 0))
         (cube-rpy (float-vector 0 0 initial-rotate-angle))
         (cube-height (get-cube-height :cube-rpy cube-rpy))
         (cube-width (get-cube-width :cube-rpy cube-rpy))
         (cube-depth (get-cube-depth :cube-rpy cube-rpy)))
    (support-rotate-motion :rotate-angle (/ -pi 4.0) :rotate-axis :x
                           :initial-rotate-angle initial-rotate-angle
                           :cube-height cube-height
                           :cube-width cube-width
                           :cube-depth cube-depth
                           :move-robot move-robot
                           :use-torso use-torso))
  (dissoc-cube :arms)
  (return-from execute-motion-symbols (list larm-motion-symbol rarm-motion-symbol))))
