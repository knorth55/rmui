(require :common-motions "package://rmuieus/euslisp/common/common-motions.l")


;; dualarm actions

(defun dualarm-hold-up-action (&key (move-robot t) (use-torso t))
  (assoc-cube :arms)
  (hold-lift-up-motion :lift-height 200 :move-robot move-robot :use-torso use-torso)
  (dissoc-cube :arms))


(defun dualarm-hold-down-action (&key (move-robot t) (use-torso t))
  (assoc-cube :arms)
  (hold-lift-down-motion :move-robot move-robot :use-torso use-torso)
  (dissoc-cube :arms))


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
  (dissoc-cube :arms))


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
  (dissoc-cube :arms))


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
  (dissoc-cube :arms))


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
  (dissoc-cube :arms))


(defun dualarm-push-rotate-ccw-z45-action (&key (move-robot t) (use-torso t))
  (assoc-cube :arms)
  (push-rotate-motion :rotate-angle (/ pi 4.0) :rotate-axis :z
                      :move-robot move-robot :use-torso use-torso)
  (dissoc-cube :arms))


(defun dualarm-push-rotate-cw-z45-action (&key (move-robot t) (use-torso t))
  (assoc-cube :arms)
  (push-rotate-motion :rotate-angle (/ -pi 4.0) :rotate-axis :z
                      :move-robot move-robot :use-torso use-torso)
  (dissoc-cube :arms))


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
  (dissoc-cube :arms))


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
  (dissoc-cube :arms))


;; larm actions

(defun larm-left-push-action (&key (move-robot t) (use-torso t))
  (push-motion :larm :move-robot move-robot :use-torso use-torso
               :push-axes (list :y)
               :push-positions (list :center)
               :push-directions (list :left)
               :approach-directions (list :back)
               :approach-arm :larm))


(defun larm-front-push-action (&key (move-robot t) (use-torso t))
  (push-motion :larm :move-robot move-robot :use-torso use-torso
               :push-axes (list :x)
               :push-positions (list :center)
               :push-directions (list :front)
               :approach-directions (list :left)
               :approach-arm :larm))


(defun larm-back-push-action (&key (move-robot t) (use-torso t))
  (push-motion :larm :move-robot move-robot :use-torso use-torso
               :push-axes (list :x)
               :push-positions (list :center)
               :push-directions (list :back)
               :approach-directions (list :left)
               :approach-arm :larm))


(defun larm-top-push-action (&key (move-robot t) (use-torso t))
  (push-motion :larm :move-robot move-robot :use-torso use-torso
               :push-axes (list :z)
               :push-positions (list :center)
               :push-directions (list :top)
               :approach-directions (list :left)
               :approach-arm :larm))

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
                 :approach-arm :larm)))


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
                 :approach-arm :larm)))


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
                 :approach-arm :larm)))


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
                 :approach-arm :larm)))


(defun larm-left-move-push-action (&key (move-robot t) (use-torso t))
  (assoc-cube :larm)
  (push-move-motion :larm :move-robot move-robot :use-torso use-torso
                    :push-axis :y
                    :move-distance -200.0)
  (dissoc-cube :larm))


(defun larm-bottom-left-side-push-action (&key (move-robot t) (use-torso t))
  (push-motion :larm :move-robot move-robot :use-torso use-torso
               :push-axes (list :z)
               :push-positions (list :left)
               :push-directions (list :bottom)
               :approach-directions (list :back)
               :hold-type :tight
               ;; :hold-type 100
               :approach-arm :larm))


(defun larm-top-right-side-push-action (&key (move-robot t) (use-torso t))
  (push-motion :larm :move-robot move-robot :use-torso use-torso
               :push-axes (list :z)
               :push-positions (list :right)
               :push-directions (list :top)
               :approach-directions (list :back)
               :approach-arm :larm))


(defun larm-push-release-action (&key (move-robot t) (use-torso t))
  (push-release-motion :larm :move-robot move-robot :use-torso use-torso)
  (reset-cube-manip-motion :larm :move-robot move-robot))


;; rarm actions

(defun rarm-right-push-action (&key (move-robot t) (use-torso t))
  (push-motion :rarm :move-robot move-robot :use-torso use-torso
               :push-axes (list :y)
               :push-positions (list :center)
               :push-directions (list :right)
               :approach-directions (list :back)
               :approach-arm :rarm))


(defun rarm-front-push-action (&key (move-robot t) (use-torso t))
  (push-motion :rarm :move-robot move-robot :use-torso use-torso
               :push-axes (list :x)
               :push-positions (list :center)
               :push-directions (list :front)
               :approach-directions (list :right)
               :approach-arm :rarm))


(defun rarm-back-push-action (&key (move-robot t) (use-torso t))
  (push-motion :rarm :move-robot move-robot :use-torso use-torso
               :push-axes (list :x)
               :push-positions (list :center)
               :push-directions (list :back)
               :approach-directions (list :right)
               :approach-arm :rarm))


(defun rarm-top-push-action (&key (move-robot t) (use-torso t))
  (push-motion :rarm :move-robot move-robot :use-torso use-torso
               :push-axes (list :z)
               :push-positions (list :center)
               :push-directions (list :top)
               :approach-directions (list :right)
               :approach-arm :rarm))


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
                 :approach-arm :rarm)))


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
                 :approach-arm :rarm)))


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
                 :approach-arm :rarm)))


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
                 :approach-arm :rarm)))


(defun rarm-right-move-push-action (&key (move-robot t) (use-torso t))
  (assoc-cube :rarm)
  (push-move-motion :rarm
                    :push-axis :y
                    :move-distance 200.0
                    :move-robot move-robot
                    :use-torso use-torso)
  (dissoc-cube :rarm))


(defun rarm-bottom-right-side-push-action (&key (move-robot t) (use-torso t))
  (push-motion :rarm :move-robot move-robot :use-torso use-torso
               :push-axes (list :z)
               :push-positions (list :right)
               :push-directions (list :bottom)
               :approach-directions (list :back)
               :hold-type :tight
               ;; :hold-type 100
               :approach-arm :rarm))


(defun rarm-top-left-side-push-action (&key (move-robot t) (use-torso t))
  (push-motion :rarm :move-robot move-robot :use-torso use-torso
               :push-axes (list :z)
               :push-positions (list :left)
               :push-directions (list :top)
               :approach-directions (list :back)
               :approach-arm :rarm))


(defun rarm-push-release-action (&key (move-robot t) (use-torso t))
  (push-release-motion :rarm :move-robot move-robot :use-torso use-torso)
  (reset-cube-manip-motion :rarm :move-robot move-robot))


(provide :common-actions "common-actions.l")
