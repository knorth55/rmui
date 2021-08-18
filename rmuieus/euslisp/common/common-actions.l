(require :common-motions "package://rmuieus/euslisp/common/common-motions.l")


(defun action-init ()
  (motion-init)
  t)


;; dualarm actions

(defun dualarm-hold-up-action (&key (move-robot t) (use-torso t))
  (assoc-cube :arms)
  (hold-lift-up-motion :lift-height 200 :move-robot move-robot :use-torso use-torso)
  (dissoc-cube :arms)
  t)


(defun dualarm-push-hold-up-action (&key (move-robot t) (use-torso t))
  (dualarm-hold-up-action :move-robot move-robot :use-torso use-torso)
  t)


(defun dualarm-support-hold-up-action (&key (move-robot t) (use-torso t))
  (dualarm-hold-up-action :move-robot move-robot :use-torso use-torso)
  t)


(defun dualarm-hold-down-action (&key (move-robot t) (use-torso t))
  (assoc-cube :arms)
  (hold-lift-down-motion :move-robot move-robot :use-torso use-torso)
  (dissoc-cube :arms)
  t)


(defun dualarm-push-hold-down-action (&key (move-robot t) (use-torso t))
  (dualarm-hold-down-action :move-robot move-robot :use-torso use-torso)
  t)


(defun dualarm-support-hold-down-action (&key (move-robot t) (use-torso t))
  (dualarm-hold-down-action :move-robot move-robot :use-torso use-torso)
  t)


(defun dualarm-push-rotate-ccw-x45-action (&key (move-robot t) (use-torso t))
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
  t)


(defun dualarm-push-rotate-cw-x45-action (&key (move-robot t) (use-torso t))
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
  t)


(defun dualarm-push-rotate-ccw-x90-action (&key (move-robot t) (use-torso t))
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
  t)


(defun dualarm-lift-push-rotate-ccw-x90-action (&key (move-robot t) (use-torso t))
  (dualarm-push-rotate-ccw-x90-action :move-robot move-robot :use-torso use-torso)
  t)


(defun dualarm-side-push-rotate-ccw-x90-action (&key (move-robot t) (use-torso t))
  (dualarm-push-rotate-ccw-x90-action :move-robot move-robot :use-torso use-torso)
  t)


(defun dualarm-push-rotate-cw-x90-action (&key (move-robot t) (use-torso t))
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
  t)


(defun dualarm-lift-push-rotate-cw-x90-action (&key (move-robot t) (use-torso t))
  (dualarm-push-rotate-cw-x90-action :move-robot move-robot :use-torso use-torso)
  t)


(defun dualarm-side-push-rotate-cw-x90-action (&key (move-robot t) (use-torso t))
  (dualarm-push-rotate-cw-x90-action :move-robot move-robot :use-torso use-torso)
  t)


(defun dualarm-push-rotate-ccw-z45-action (&key (move-robot t) (use-torso t))
  (assoc-cube :arms)
  (push-rotate-motion :rotate-angle (/ pi 4.0) :rotate-axis :z
                      :move-robot move-robot :use-torso use-torso)
  (dissoc-cube :arms)
  t)


(defun dualarm-push-rotate-cw-z45-action (&key (move-robot t) (use-torso t))
  (assoc-cube :arms)
  (push-rotate-motion :rotate-angle (/ -pi 4.0) :rotate-axis :z
                      :move-robot move-robot :use-torso use-torso)
  (dissoc-cube :arms)
  t)


(defun dualarm-support-rotate-ccw-x45-action (&key (move-robot t) (use-torso t))
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
  t)


(defun dualarm-support-rotate-cw-x45-action (&key (move-robot t) (use-torso t))
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
  t)


;; common arm actions

(defun front-push-action (arm &key (move-robot t) (use-torso t))
  (push-motion arm :move-robot move-robot :use-torso use-torso
               :push-axes (list :x)
               :push-positions (list :center)
               :push-directions (list :front)
               :approach-directions (list (if (eq arm :larm) :left :right))
               :approach-arm arm)
  t)


(defun back-push-action (arm &key (move-robot t) (use-torso t))
  (push-motion arm :move-robot move-robot :use-torso use-torso
               :push-axes (list :x)
               :push-positions (list :center)
               :push-directions (list :back)
               :approach-directions (list (if (eq arm :larm) :left :right))
               :approach-arm arm)
  t)


(defun top-push-action (arm &key (move-robot t) (use-torso t))
  (push-motion arm :move-robot move-robot :use-torso use-torso
               :push-axes (list :z)
               :push-positions (list :center)
               :push-directions (list :top)
               :approach-directions (list (if (eq arm :larm) :left :right))
               :approach-arm arm)
  t)


(defun move-push-action (arm &key (move-robot t) (use-torso t))
  (assoc-cube arm)
  (push-move-motion arm :move-robot move-robot :use-torso use-torso
                    :push-axis :y
                    :move-distance (if (eq arm :larm) -200.0 200.0))
  (dissoc-cube arm)
  t)


(defun push-release-action (arm &key (move-robot t) (use-torso t))
  (push-release-motion arm :move-robot move-robot :use-torso use-torso)
  (reset-cube-manip-motion arm :move-robot move-robot)
  t)


;; larm actions

(defun larm-left-push-action (&key (move-robot t) (use-torso t))
  (push-motion :larm :move-robot move-robot :use-torso use-torso
               :push-axes (list :y)
               :push-positions (list :center)
               :push-directions (list :left)
               :approach-directions (list :back)
               :approach-arm :larm)
  t)


(defun larm-front-push-action (&key (move-robot t) (use-torso t))
  (front-push-action :larm :move-robot move-robot :use-torso use-torso))


(defun larm-back-push-action (&key (move-robot t) (use-torso t))
  (back-push-action :larm :move-robot move-robot :use-torso use-torso))


(defun larm-top-push-action (&key (move-robot t) (use-torso t))
  (top-push-action :larm :move-robot move-robot :use-torso use-torso))


(defun larm-top-left-push-action (&key (move-robot t) (use-torso t))
  (let* ((cube-rpy (float-vector 0 0 (/ pi 4.0)))
         (cube-height (get-cube-height :cube-rpy cube-rpy))
         (cube-width (get-cube-width :cube-rpy cube-rpy))
         (cube-depth (get-cube-depth :cube-rpy cube-rpy)))
    (push-motion :larm :move-robot move-robot :use-torso use-torso
                 :push-axes (list :y)
                 :push-positions (list :center)
                 :push-directions (list :left)
                 :approach-directions (list :back)
                 :cube-height cube-height
                 :cube-width cube-width
                 :cube-depth cube-depth
                 :cube-rpy cube-rpy
                 :approach-arm :larm))
  t)


(defun larm-bottom-left-push-action (&key (move-robot t) (use-torso t))
  (let* ((cube-rpy (float-vector 0 0 (/ -pi 4.0)))
         (cube-height (get-cube-height :cube-rpy cube-rpy))
         (cube-width (get-cube-width :cube-rpy cube-rpy))
         (cube-depth (get-cube-depth :cube-rpy cube-rpy)))
    (push-motion :larm :move-robot move-robot :use-torso use-torso
                 :push-axes (list :y)
                 :push-positions (list :center)
                 :push-directions (list :left)
                 :approach-directions (list :back)
                 :cube-height cube-height
                 :cube-width cube-width
                 :cube-depth cube-depth
                 :cube-rpy cube-rpy
                 :approach-arm :larm))
  t)


(defun larm-front-left-push-action (&key (move-robot t) (use-torso t))
  (let* ((cube-rpy (float-vector (/ -pi 4.0) 0 0))
         (cube-height (get-cube-height :cube-rpy cube-rpy))
         (cube-width (get-cube-width :cube-rpy cube-rpy))
         (cube-depth (get-cube-depth :cube-rpy cube-rpy)))
    (push-motion :larm :move-robot move-robot :use-torso use-torso
                 :push-axes (list :y)
                 :push-positions (list :center)
                 :push-directions (list :left)
                 :approach-directions (list :back)
                 :cube-height cube-height
                 :cube-width cube-width
                 :cube-depth cube-depth
                 :cube-rpy cube-rpy
                 :approach-arm :larm))
  t)


(defun larm-back-left-push-action (&key (move-robot t) (use-torso t))
  (let* ((cube-rpy (float-vector (/ pi 4.0) 0 0))
         (cube-height (get-cube-height :cube-rpy cube-rpy))
         (cube-width (get-cube-width :cube-rpy cube-rpy))
         (cube-depth (get-cube-depth :cube-rpy cube-rpy)))
    (push-motion :larm :move-robot move-robot :use-torso use-torso
                 :push-axes (list :y)
                 :push-positions (list :center)
                 :push-directions (list :left)
                 :approach-directions (list :back)
                 :cube-height cube-height
                 :cube-width cube-width
                 :cube-depth cube-depth
                 :cube-rpy cube-rpy
                 :approach-arm :larm))
  t)


(defun larm-left-move-push-action (&key (move-robot t) (use-torso t))
  (move-push-action :larm :move-robot move-robot :use-torso use-torso))


(defun larm-bottom-left-side-push-action (&key (move-robot t) (use-torso t))
  (push-motion :larm :move-robot move-robot :use-torso use-torso
               :push-axes (list :z)
               :push-positions (list :left)
               :push-directions (list :bottom)
               :approach-directions (list :back)
               :hold-type :tight
               ;; :hold-type 100
               :approach-arm :larm)
  t)


(defun larm-top-right-side-push-action (&key (move-robot t) (use-torso t))
  (push-motion :larm :move-robot move-robot :use-torso use-torso
               :push-axes (list :z)
               :push-positions (list :right)
               :push-directions (list :top)
               :approach-directions (list :back)
               :approach-arm :larm)
  t)


(defun larm-push-release-action (&key (move-robot t) (use-torso t))
  (push-release-action :larm :move-robot move-robot :use-torso use-torso))


;; rarm actions

(defun rarm-right-push-action (&key (move-robot t) (use-torso t))
  (push-motion :rarm :move-robot move-robot :use-torso use-torso
               :push-axes (list :y)
               :push-positions (list :center)
               :push-directions (list :right)
               :approach-directions (list :back)
               :approach-arm :rarm)
  t)


(defun rarm-front-push-action (&key (move-robot t) (use-torso t))
  (front-push-action :rarm :move-robot move-robot :use-torso use-torso))


(defun rarm-back-push-action (&key (move-robot t) (use-torso t))
  (back-push-action :rarm :move-robot move-robot :use-torso use-torso))


(defun rarm-top-push-action (&key (move-robot t) (use-torso t))
  (top-push-action :rarm :move-robot move-robot :use-torso use-torso))


(defun rarm-top-right-push-action (&key (move-robot t) (use-torso t))
  (let* ((cube-rpy (float-vector 0 0 (/ -pi 4.0)))
         (cube-height (get-cube-height :cube-rpy cube-rpy))
         (cube-width (get-cube-width :cube-rpy cube-rpy))
         (cube-depth (get-cube-depth :cube-rpy cube-rpy)))
    (push-motion :rarm :move-robot move-robot :use-torso use-torso
                 :push-axes (list :y)
                 :push-positions (list :center)
                 :push-directions (list :right)
                 :approach-directions (list :back)
                 :cube-height cube-height
                 :cube-width cube-width
                 :cube-depth cube-depth
                 :cube-rpy cube-rpy
                 :approach-arm :rarm))
  t)


(defun rarm-bottom-right-push-action (&key (move-robot t) (use-torso t))
  (let* ((cube-rpy (float-vector 0 0 (/ pi 4.0)))
         (cube-height (get-cube-height :cube-rpy cube-rpy))
         (cube-width (get-cube-width :cube-rpy cube-rpy))
         (cube-depth (get-cube-depth :cube-rpy cube-rpy)))
    (push-motion :rarm :move-robot move-robot :use-torso use-torso
                 :push-axes (list :y)
                 :push-positions (list :center)
                 :push-directions (list :right)
                 :approach-directions (list :back)
                 :cube-height cube-height
                 :cube-width cube-width
                 :cube-depth cube-depth
                 :cube-rpy cube-rpy
                 :approach-arm :rarm))
  t)


(defun rarm-front-right-push-action (&key (move-robot t) (use-torso t))
  (let* ((cube-rpy (float-vector (/ pi 4.0) 0 0))
         (cube-height (get-cube-height :cube-rpy cube-rpy))
         (cube-width (get-cube-width :cube-rpy cube-rpy))
         (cube-depth (get-cube-depth :cube-rpy cube-rpy)))
    (push-motion :rarm :move-robot move-robot :use-torso use-torso
                 :push-axes (list :y)
                 :push-positions (list :center)
                 :push-directions (list :right)
                 :approach-directions (list :back)
                 :cube-height cube-height
                 :cube-width cube-width
                 :cube-depth cube-depth
                 :cube-rpy cube-rpy
                 :approach-arm :rarm))
  t)


(defun rarm-back-right-push-action (&key (move-robot t) (use-torso t))
  (let* ((cube-rpy (float-vector (/ -pi 4.0) 0 0))
         (cube-height (get-cube-height :cube-rpy cube-rpy))
         (cube-width (get-cube-width :cube-rpy cube-rpy))
         (cube-depth (get-cube-depth :cube-rpy cube-rpy)))
    (push-motion :rarm :move-robot move-robot :use-torso use-torso
                 :push-axes (list :y)
                 :push-positions (list :center)
                 :push-directions (list :right)
                 :approach-directions (list :back)
                 :cube-height cube-height
                 :cube-width cube-width
                 :cube-depth cube-depth
                 :cube-rpy cube-rpy
                 :approach-arm :rarm))
  t)


(defun rarm-right-move-push-action (&key (move-robot t) (use-torso t))
  (move-push-action :rarm :move-robot move-robot :use-torso use-torso))


(defun rarm-bottom-right-side-push-action (&key (move-robot t) (use-torso t))
  (push-motion :rarm :move-robot move-robot :use-torso use-torso
               :push-axes (list :z)
               :push-positions (list :right)
               :push-directions (list :bottom)
               :approach-directions (list :back)
               :hold-type :tight
               ;; :hold-type 100
               :approach-arm :rarm)
  t)


(defun rarm-top-left-side-push-action (&key (move-robot t) (use-torso t))
  (push-motion :rarm :move-robot move-robot :use-torso use-torso
               :push-axes (list :z)
               :push-positions (list :left)
               :push-directions (list :top)
               :approach-directions (list :back)
               :approach-arm :rarm)
  t)


(defun rarm-push-release-action (&key (move-robot t) (use-torso t))
  (push-release-action :rarm :move-robot move-robot :use-torso use-torso))


(provide :common-actions "common-actions.l")
