(require :common-motions "package://rmuieus/euslisp/common/common-motions.l")


(defun action-init (&key (cube-pos-y 0) &rest args &allow-other-keys)
  (scene-init :cube-pos-y cube-pos-y)
  (motion-init :robot *robot* :ri *ri* :cube *cube*)
  t)


;; common arm actions

(defun front-push-action (arm &key (move-robot t) (use-torso t)
                              (robot *robot*) (ri *ri*) (cube *cube*)
                              &rest args &allow-other-keys)
  (push-motion arm :move-robot move-robot :use-torso use-torso
               :push-axes (list :x)
               :push-positions (list :center)
               :push-directions (list :front)
               :approach-directions (list (if (eq arm :larm) :left :right))
               :approach-arm arm
               :robot robot :ri ri :cube cube)
  t)


(defun back-push-action (arm &key (move-robot t) (use-torso t)
                             (robot *robot*) (ri *ri*) (cube *cube*)
                             &rest args &allow-other-keys)
  (push-motion arm :move-robot move-robot :use-torso use-torso
               :push-axes (list :x)
               :push-positions (list :center)
               :push-directions (list :back)
               :approach-directions (list (if (eq arm :larm) :left :right))
               :approach-arm arm
               :robot robot :ri ri :cube cube)
  t)


(defun top-push-action (arm &key (move-robot t) (use-torso t)
                            (robot *robot*) (ri *ri*) (cube *cube*)
                            &rest args &allow-other-keys)
  (push-motion arm :move-robot move-robot :use-torso use-torso
               :push-axes (list :z)
               :push-positions (list :center)
               :push-directions (list :top)
               :approach-directions (list (if (eq arm :larm) :left :right))
               :approach-arm arm
               :robot robot :ri ri :cube cube)
  t)


(defun move-push-action (arm &key (move-robot t) (use-torso t)
                             (robot *robot*) (ri *ri*) (cube *cube*)
                             &rest args &allow-other-keys)
  (assoc-cube arm :ri ri :cube cube)
  (push-move-motion arm :move-robot move-robot :use-torso use-torso
                    :push-axis :y
                    :move-distance (if (eq arm :larm) -200.0 200.0)
                    :robot robot :ri ri :cube cube)
  (dissoc-cube arm :ri ri :cube cube)
  t)


(defun push-release-action (arm &key (move-robot t) (use-torso t)
                                (robot *robot*) (ri *ri*) (cube *cube*)
                                &rest args &allow-other-keys)
  (push-release-motion arm :move-robot move-robot :use-torso use-torso
                       :robot robot :ri ri :cube cube)
  (reset-cube-manip-motion arm :move-robot move-robot
                           :robot robot :ri ri :cube cube)
  t)


;; dualarm actions

(defun dualarm-hold-up-action (&key (move-robot t) (use-torso t)
                                    (robot *robot*) (ri *ri*) (cube *cube*) 
                                    &rest args &allow-other-keys)
  (assoc-cube :arms :ri ri :cube cube)
  (hold-lift-up-motion :lift-height 200 :move-robot move-robot :use-torso use-torso
                       :robot robot :ri ri :cube cube)
  (dissoc-cube :arms :ri ri :cube cube)
  t)


(defun dualarm-push-hold-up-action (&key (move-robot t) (use-torso t)
                                         (robot *robot*) (ri *ri*) (cube *cube*)
                                         &rest args &allow-other-keys)
  (dualarm-hold-up-action :move-robot move-robot :use-torso use-torso
                          :robot robot :ri ri :cube cube)
  t)


(defun dualarm-support-hold-up-action (&key (move-robot t) (use-torso t)
                                            (robot *robot*) (ri *ri*) (cube *cube*)
                                            &rest args &allow-other-keys)
  (dualarm-hold-up-action :move-robot move-robot :use-torso use-torso
                          :robot robot :ri ri :cube cube)
  t)


(defun dualarm-hold-down-action (&key (move-robot t) (use-torso t)
                                      (robot *robot*) (ri *ri*) (cube *cube*) 
                                      &rest args &allow-other-keys)
  (assoc-cube :arms :ri ri :cube cube)
  (hold-lift-down-motion :move-robot move-robot :use-torso use-torso
                         :robot robot :ri ri :cube cube)
  (dissoc-cube :arms :ri ri :cube cube)
  t)


(defun dualarm-push-hold-down-action (&key (move-robot t) (use-torso t)
                                           (robot *robot*) (ri *ri*) (cube *cube*)
                                           &rest args &allow-other-keys)
  (dualarm-hold-down-action :move-robot move-robot :use-torso use-torso
                            :robot robot :ri ri :cube cube)
  t)


(defun dualarm-support-hold-down-action (&key (move-robot t) (use-torso t)
                                              (robot *robot*) (ri *ri*) (cube *cube*)
                                              &rest args &allow-other-keys)
  (dualarm-hold-down-action :move-robot move-robot :use-torso use-torso
                            :robot robot :ri ri :cube cube)
  t)


(defun dualarm-push-rotate-ccw-x45-action (&key (move-robot t) (use-torso t)
                                                (robot *robot*) (ri *ri*) (cube *cube*)
                                                &rest args &allow-other-keys)
  (assoc-cube :arms :ri ri :cube cube)
  (let* ((initial-rotate-angle
           (if (and (equal larm-contact-state :bottom-left-contact)
                    (equal rarm-contact-state :top-right-contact))
             (/ -pi 4.0) 0))
         (cube-rpy (float-vector 0 0 initial-rotate-angle))
         (cube-height (get-cube-height :cube-rpy cube-rpy :cube cube))
         (cube-width (get-cube-width :cube-rpy cube-rpy :cube cube))
         (cube-depth (get-cube-depth :cube-rpy cube-rpy :cube cube)))
    (push-rotate-motion :rotate-angle (/ pi 4.0) :rotate-axis :x
                        :initial-rotate-angle initial-rotate-angle
                        :cube-height cube-height
                        :cube-width cube-width
                        :cube-depth cube-depth
                        :move-robot move-robot
                        :use-torso use-torso
                        :robot robot :ri ri :cube cube))
  (dissoc-cube :arms :ri ri :cube cube)
  t)


(defun dualarm-push-rotate-cw-x45-action (&key (move-robot t) (use-torso t)
                                               (robot *robot*) (ri *ri*) (cube *cube*)
                                               &rest args &allow-other-keys)
  (assoc-cube :arms :ri ri :cube cube)
  (let* ((initial-rotate-angle
           (if (and (equal larm-contact-state :top-left-contact)
                    (equal rarm-contact-state :bottom-right-contact))
             (/ pi 4.0) 0))
         (cube-rpy (float-vector 0 0 initial-rotate-angle))
         (cube-height (get-cube-height :cube-rpy cube-rpy :cube cube))
         (cube-width (get-cube-width :cube-rpy cube-rpy :cube cube))
         (cube-depth (get-cube-depth :cube-rpy cube-rpy :cube cube)))
    (push-rotate-motion :rotate-angle (/ -pi 4.0) :rotate-axis :x
                        :initial-rotate-angle initial-rotate-angle
                        :cube-height cube-height
                        :cube-width cube-width
                        :cube-depth cube-depth
                        :move-robot move-robot
                        :use-torso use-torso
                        :robot robot :ri ri :cube cube))
  (dissoc-cube :arms :ri ri :cube cube)
  t)


(defun dualarm-push-rotate-ccw-x90-action (&key (move-robot t) (use-torso t)
                                                (robot *robot*) (ri *ri*) (cube *cube*)
                                                &rest args &allow-other-keys)
  (assoc-cube :arms :ri ri :cube cube)
  (let* ((initial-rotate-angle 0)
         (cube-rpy (float-vector 0 0 initial-rotate-angle))
         (cube-height (get-cube-height :cube-rpy cube-rpy :cube cube))
         (cube-width (get-cube-width :cube-rpy cube-rpy :cube cube))
         (cube-depth (get-cube-depth :cube-rpy cube-rpy :cube cube)))
    (push-rotate-motion :rotate-angle (/ pi 2.0) :rotate-axis :x
                        :initial-rotate-angle initial-rotate-angle
                        :cube-height cube-height
                        :cube-width cube-width
                        :cube-depth cube-depth
                        :move-robot move-robot
                        :use-torso use-torso
                        :robot robot :ri ri :cube cube))
  (dissoc-cube :arms :ri ri :cube cube)
  t)


(defun dualarm-lift-push-rotate-ccw-x90-action (&key (move-robot t) (use-torso t)
                                                     (robot *robot*) (ri *ri*) (cube *cube*)
                                                     &rest args &allow-other-keys)
  (dualarm-push-rotate-ccw-x90-action :move-robot move-robot :use-torso use-torso
                                      :robot robot :ri ri :cube cube)
  t)


(defun dualarm-side-push-rotate-ccw-x90-action (&key (move-robot t) (use-torso t)
                                                     (robot *robot*) (ri *ri*) (cube *cube*)
                                                     &rest args &allow-other-keys)
  (dualarm-push-rotate-ccw-x90-action :move-robot move-robot :use-torso use-torso
                                      :robot robot :ri ri :cube cube)
  t)


(defun dualarm-push-rotate-cw-x90-action (&key (move-robot t) (use-torso t)
                                               (robot *robot*) (ri *ri*) (cube *cube*)
                                               &rest args &allow-other-keys)
  (assoc-cube :arms :ri ri :cube cube)
  (let* ((initial-rotate-angle 0)
         (cube-rpy (float-vector 0 0 initial-rotate-angle))
         (cube-height (get-cube-height :cube-rpy cube-rpy :cube cube))
         (cube-width (get-cube-width :cube-rpy cube-rpy :cube cube))
         (cube-depth (get-cube-depth :cube-rpy cube-rpy :cube cube)))
    (push-rotate-motion :rotate-angle (/ -pi 2.0) :rotate-axis :x
                        :initial-rotate-angle initial-rotate-angle
                        :cube-height cube-height
                        :cube-width cube-width
                        :cube-depth cube-depth
                        :move-robot move-robot
                        :use-torso use-torso
                        :robot robot :ri ri :cube cube))
  (dissoc-cube :arms :ri ri :cube cube)
  t)


(defun dualarm-lift-push-rotate-cw-x90-action (&key (move-robot t) (use-torso t)
                                                    (robot *robot*) (ri *ri*)  (cube *cube*)
                                                    &rest args &allow-other-keys)
  (dualarm-push-rotate-cw-x90-action :move-robot move-robot :use-torso use-torso
                                     :robot robot :ri ri :cube cube)
  t)


(defun dualarm-side-push-rotate-cw-x90-action (&key (move-robot t) (use-torso t)
                                                    (robot *robot*) (ri *ri*) (cube *cube*) 
                                                    &rest args &allow-other-keys)
  (dualarm-push-rotate-cw-x90-action :move-robot move-robot :use-torso use-torso
                                     :robot robot :ri ri :cube cube)
  t)


(defun dualarm-push-rotate-ccw-z45-action (&key (move-robot t) (use-torso t)
                                                (robot *robot*) (ri *ri*) (cube *cube*) 
                                                &rest args &allow-other-keys)
  (assoc-cube :arms :ri ri :cube cube)
  (push-rotate-motion :rotate-angle (/ pi 4.0) :rotate-axis :z
                      :move-robot move-robot :use-torso use-torso
                      :robot robot :ri ri :cube cube)
  (dissoc-cube :arms :ri ri :cube cube)
  t)


(defun dualarm-push-rotate-cw-z45-action (&key (move-robot t) (use-torso t)
                                               (robot *robot*) (ri *ri*) (cube *cube*) 
                                               &rest args &allow-other-keys)
  (assoc-cube :arms :ri ri :cube cube)
  (push-rotate-motion :rotate-angle (/ -pi 4.0) :rotate-axis :z
                      :move-robot move-robot :use-torso use-torso
                      :robot robot :ri ri :cube cube)
  (dissoc-cube :arms :ri ri :cube cube)
  t)


(defun dualarm-support-rotate-ccw-x45-action (&key (move-robot t) (use-torso t)
                                                   (robot *robot*) (ri *ri*) (cube *cube*)
                                                   &rest args &allow-other-keys)
  (assoc-cube :arms :ri ri :cube cube)
  (let* ((initial-rotate-angle
           (if (and (equal larm-contact-state :top-left-contact)
                    (equal rarm-contact-state :top-right-contact))
             (/ pi 4.0) 0))
         (cube-rpy (float-vector 0 0 initial-rotate-angle))
         (cube-height (get-cube-height :cube-rpy cube-rpy :cube cube)) 
         (cube-width (get-cube-width :cube-rpy cube-rpy :cube cube))
         (cube-depth (get-cube-depth :cube-rpy cube-rpy :cube cube)))
    (support-rotate-motion :rotate-angle (/ pi 4.0) :rotate-axis :x
                           :initial-rotate-angle initial-rotate-angle
                           :cube-height cube-height
                           :cube-width cube-width
                           :cube-depth cube-depth
                           :move-robot move-robot
                           :use-torso use-torso
                           :robot robot :ri ri :cube cube))
  (dissoc-cube :arms :ri ri :cube cube)
  t)


(defun dualarm-support-rotate-cw-x45-action (&key (move-robot t) (use-torso t)
                                                  (robot *robot*) (ri *ri*) (cube *cube*)
                                                  &rest args &allow-other-keys)
  (assoc-cube :arms :ri ri :cube cube)
  (let* ((initial-rotate-angle
           (if (and (equal larm-contact-state :top-left-contact)
                    (equal rarm-contact-state :top-right-contact))
             (/ -pi 4.0) 0))
         (cube-rpy (float-vector 0 0 initial-rotate-angle))
         (cube-height (get-cube-height :cube-rpy cube-rpy :cube cube))
         (cube-width (get-cube-width :cube-rpy cube-rpy :cube cube))
         (cube-depth (get-cube-depth :cube-rpy cube-rpy :cube cube)))
    (support-rotate-motion :rotate-angle (/ -pi 4.0) :rotate-axis :x
                           :initial-rotate-angle initial-rotate-angle
                           :cube-height cube-height
                           :cube-width cube-width
                           :cube-depth cube-depth
                           :move-robot move-robot
                           :use-torso use-torso
                           :robot robot :ri ri :cube cube))
  (dissoc-cube :arms :ri ri :cube cube)
  t)


(defun dualarm-push-release-action (&key (move-robot t) (use-torso t)
                                         (robot *robot*) (ri *ri*) (cube *cube*)
                                         &rest args &allow-other-keys)
  (push-release-action :larm :move-robot nil :use-torso use-torso
                       :robot robot :ri ri :cube cube)
  (push-release-action :rarm :move-robot move-robot :use-torso use-torso
                       :robot robot :ri ri :cube cube))


;; larm actions

(defun larm-left-push-action (&key (move-robot t) (use-torso t)
                                   (robot *robot*) (ri *ri*) (cube *cube*)
                                   &rest args &allow-other-keys)
  (push-motion :larm :move-robot move-robot :use-torso use-torso
               :push-axes (list :y)
               :push-positions (list :center)
               :push-directions (list :left)
               :approach-directions (list :back)
               :approach-arm :larm
               :robot robot :ri ri :cube cube)
  t)


(defun larm-front-push-action (&key (move-robot t) (use-torso t)
                                    (robot *robot*) (ri *ri*) (cube *cube*)
                                    &rest args &allow-other-keys)
  (front-push-action :larm :move-robot move-robot :use-torso use-torso
                     :robot robot :ri ri :cube cube))


(defun larm-back-push-action (&key (move-robot t) (use-torso t)
                                   (robot *robot*) (ri *ri*) (cube *cube*)
                                   &rest args &allow-other-keys)
  (back-push-action :larm :move-robot move-robot :use-torso use-torso
                    :robot robot :ri ri :cube cube))


(defun larm-top-push-action (&key (move-robot t) (use-torso t)
                                  (robot *robot*) (ri *ri*) (cube *cube*)
                                  &rest args &allow-other-keys)
  (top-push-action :larm :move-robot move-robot :use-torso use-torso
                   :robot robot :ri ri :cube cube))


(defun larm-top-left-push-action (&key (move-robot t) (use-torso t)
                                       (robot *robot*) (ri *ri*) (cube *cube*)
                                       &rest args &allow-other-keys)
  (let* ((cube-rpy (float-vector 0 0 (/ pi 4.0)))
         (cube-height (get-cube-height :cube-rpy cube-rpy :cube cube))
         (cube-width (get-cube-width :cube-rpy cube-rpy :cube cube))
         (cube-depth (get-cube-depth :cube-rpy cube-rpy :cube cube)))
    (push-motion :larm :move-robot move-robot :use-torso use-torso
                 :push-axes (list :y)
                 :push-positions (list :center)
                 :push-directions (list :left)
                 :approach-directions (list :back)
                 :cube-height cube-height
                 :cube-width cube-width
                 :cube-depth cube-depth
                 :cube-rpy cube-rpy
                 :approach-arm :larm
                 :robot robot :ri ri :cube cube))
  t)


(defun larm-bottom-left-push-action (&key (move-robot t) (use-torso t)
                                          (robot *robot*) (ri *ri*) (cube *cube*)
                                          &rest args &allow-other-keys)
  (let* ((cube-rpy (float-vector 0 0 (/ -pi 4.0)))
         (cube-height (get-cube-height :cube-rpy cube-rpy :cube cube))
         (cube-width (get-cube-width :cube-rpy cube-rpy :cube cube))
         (cube-depth (get-cube-depth :cube-rpy cube-rpy :cube cube)))
    (push-motion :larm :move-robot move-robot :use-torso use-torso
                 :push-axes (list :y)
                 :push-positions (list :center)
                 :push-directions (list :left)
                 :approach-directions (list :back)
                 :cube-height cube-height
                 :cube-width cube-width
                 :cube-depth cube-depth
                 :cube-rpy cube-rpy
                 :approach-arm :larm
                 :robot robot :ri ri :cube cube))
  t)


(defun larm-front-left-push-action (&key (move-robot t) (use-torso t)
                                         (robot *robot*) (ri *ri*) (cube *cube*)
                                         &rest args &allow-other-keys)
  (let* ((cube-rpy (float-vector (/ -pi 4.0) 0 0))
         (cube-height (get-cube-height :cube-rpy cube-rpy :cube cube))
         (cube-width (get-cube-width :cube-rpy cube-rpy :cube cube))
         (cube-depth (get-cube-depth :cube-rpy cube-rpy :cube cube)))
    (push-motion :larm :move-robot move-robot :use-torso use-torso
                 :push-axes (list :y)
                 :push-positions (list :center)
                 :push-directions (list :left)
                 :approach-directions (list :back)
                 :cube-height cube-height
                 :cube-width cube-width
                 :cube-depth cube-depth
                 :cube-rpy cube-rpy
                 :approach-arm :larm
                 :robot robot :ri ri :cube cube))
  t)


(defun larm-back-left-push-action (&key (move-robot t) (use-torso t)
                                        (robot *robot*) (ri *ri*) (cube *cube*)
                                        &rest args &allow-other-keys)
  (let* ((cube-rpy (float-vector (/ pi 4.0) 0 0))
         (cube-height (get-cube-height :cube-rpy cube-rpy :cube cube))
         (cube-width (get-cube-width :cube-rpy cube-rpy :cube cube))
         (cube-depth (get-cube-depth :cube-rpy cube-rpy :cube cube)))
    (push-motion :larm :move-robot move-robot :use-torso use-torso
                 :push-axes (list :y)
                 :push-positions (list :center)
                 :push-directions (list :left)
                 :approach-directions (list :back)
                 :cube-height cube-height
                 :cube-width cube-width
                 :cube-depth cube-depth
                 :cube-rpy cube-rpy
                 :approach-arm :larm
                 :robot robot :ri ri :cube cube))
  t)


(defun larm-left-move-push-action (&key (move-robot t) (use-torso t)
                                        (robot *robot*) (ri *ri*) (cube *cube*)
                                        &rest args &allow-other-keys)
  (move-push-action :larm :move-robot move-robot :use-torso use-torso
                    :robot robot :ri ri :cube cube))


(defun larm-bottom-left-side-push-action (&key (move-robot t) (use-torso t)
                                               (robot *robot*) (ri *ri*) (cube *cube*)
                                               &rest args &allow-other-keys)
  (push-motion :larm :move-robot move-robot :use-torso use-torso
               :push-axes (list :z)
               :push-positions (list :left)
               :push-directions (list :bottom)
               :approach-directions (list :back)
               :hold-type :tight
               ;; :hold-type 100
               :approach-arm :larm
               :robot robot :ri ri :cube cube)
  t)


(defun larm-top-right-side-push-action (&key (move-robot t) (use-torso t)
                                             (robot *robot*) (ri *ri*) (cube *cube*)
                                             &rest args &allow-other-keys)
  (push-motion :larm :move-robot move-robot :use-torso use-torso
               :push-axes (list :z)
               :push-positions (list :right)
               :push-directions (list :top)
               :approach-directions (list :back)
               :approach-arm :larm
               :robot robot :ri ri :cube cube)
  t)


(defun larm-push-release-action (&key (move-robot t) (use-torso t)
                                      (robot *robot*) (ri *ri*) (cube *cube*)
                                      &rest args &allow-other-keys)
  (push-release-action :larm :move-robot move-robot :use-torso use-torso
                       :robot robot :ri ri :cube cube))


;; rarm actions

(defun rarm-right-push-action (&key (move-robot t) (use-torso t)
                                    (robot *robot*) (ri *ri*) (cube *cube*)
                                    &rest args &allow-other-keys)
  (push-motion :rarm :move-robot move-robot :use-torso use-torso
               :push-axes (list :y)
               :push-positions (list :center)
               :push-directions (list :right)
               :approach-directions (list :back)
               :approach-arm :rarm
               :robot robot :ri ri :cube cube)
  t)


(defun rarm-front-push-action (&key (move-robot t) (use-torso t)
                                    (robot *robot*) (ri *ri*) (cube *cube*)
                                    &rest args &allow-other-keys)
  (front-push-action :rarm :move-robot move-robot :use-torso use-torso
                     :robot robot :ri ri :cube cube))


(defun rarm-back-push-action (&key (move-robot t) (use-torso t)
                                   (robot *robot*) (ri *ri*) (cube *cube*)
                                   &rest args &allow-other-keys)
  (back-push-action :rarm :move-robot move-robot :use-torso use-torso
                    :robot robot :ri ri :cube cube))


(defun rarm-top-push-action (&key (move-robot t) (use-torso t)
                                  (robot *robot*) (ri *ri*) (cube *cube*)
                                  &rest args &allow-other-keys)
  (top-push-action :rarm :move-robot move-robot :use-torso use-torso
                   :robot robot :ri ri :cube cube))


(defun rarm-top-right-push-action (&key (move-robot t) (use-torso t)
                                        (robot *robot*) (ri *ri*) (cube *cube*)
                                        &rest args &allow-other-keys)
  (let* ((cube-rpy (float-vector 0 0 (/ -pi 4.0)))
         (cube-height (get-cube-height :cube-rpy cube-rpy :cube cube))
         (cube-width (get-cube-width :cube-rpy cube-rpy :cube cube))
         (cube-depth (get-cube-depth :cube-rpy cube-rpy :cube cube)))
    (push-motion :rarm :move-robot move-robot :use-torso use-torso
                 :push-axes (list :y)
                 :push-positions (list :center)
                 :push-directions (list :right)
                 :approach-directions (list :back)
                 :cube-height cube-height
                 :cube-width cube-width
                 :cube-depth cube-depth
                 :cube-rpy cube-rpy
                 :approach-arm :rarm
                 :robot robot :ri ri :cube cube))
  t)


(defun rarm-bottom-right-push-action (&key (move-robot t) (use-torso t)
                                           (robot *robot*) (ri *ri*) (cube *cube*)
                                           &rest args &allow-other-keys)
  (let* ((cube-rpy (float-vector 0 0 (/ pi 4.0)))
         (cube-height (get-cube-height :cube-rpy cube-rpy :cube cube))
         (cube-width (get-cube-width :cube-rpy cube-rpy :cube cube))
         (cube-depth (get-cube-depth :cube-rpy cube-rpy :cube cube)))
    (push-motion :rarm :move-robot move-robot :use-torso use-torso
                 :push-axes (list :y)
                 :push-positions (list :center)
                 :push-directions (list :right)
                 :approach-directions (list :back)
                 :cube-height cube-height
                 :cube-width cube-width
                 :cube-depth cube-depth
                 :cube-rpy cube-rpy
                 :approach-arm :rarm
                 :robot robot :ri ri :cube cube))
  t)


(defun rarm-front-right-push-action (&key (move-robot t) (use-torso t)
                                          (robot *robot*) (ri *ri*) (cube *cube*)
                                          &rest args &allow-other-keys)
  (let* ((cube-rpy (float-vector (/ pi 4.0) 0 0))
         (cube-height (get-cube-height :cube-rpy cube-rpy :cube cube))
         (cube-width (get-cube-width :cube-rpy cube-rpy :cube cube))
         (cube-depth (get-cube-depth :cube-rpy cube-rpy :cube cube)))
    (push-motion :rarm :move-robot move-robot :use-torso use-torso
                 :push-axes (list :y)
                 :push-positions (list :center)
                 :push-directions (list :right)
                 :approach-directions (list :back)
                 :cube-height cube-height
                 :cube-width cube-width
                 :cube-depth cube-depth
                 :cube-rpy cube-rpy
                 :approach-arm :rarm
                 :robot robot :ri ri :cube cube))
  t)


(defun rarm-back-right-push-action (&key (move-robot t) (use-torso t)
                                         (robot *robot*) (ri *ri*) (cube *cube*)
                                         &rest args &allow-other-keys)
  (let* ((cube-rpy (float-vector (/ -pi 4.0) 0 0))
         (cube-height (get-cube-height :cube-rpy cube-rpy :cube cube))
         (cube-width (get-cube-width :cube-rpy cube-rpy :cube cube))
         (cube-depth (get-cube-depth :cube-rpy cube-rpy :cube cube)))
    (push-motion :rarm :move-robot move-robot :use-torso use-torso
                 :push-axes (list :y)
                 :push-positions (list :center)
                 :push-directions (list :right)
                 :approach-directions (list :back)
                 :cube-height cube-height
                 :cube-width cube-width
                 :cube-depth cube-depth
                 :cube-rpy cube-rpy
                 :approach-arm :rarm
                 :robot robot :ri ri :cube cube))
  t)


(defun rarm-right-move-push-action (&key (move-robot t) (use-torso t)
                                         (robot *robot*) (ri *ri*) (cube *cube*) 
                                         &rest args &allow-other-keys)
  (move-push-action :rarm :move-robot move-robot :use-torso use-torso
                    :robot robot :ri ri :cube cube))


(defun rarm-bottom-right-side-push-action (&key (move-robot t) (use-torso t)
                                                (robot *robot*) (ri *ri*) (cube *cube*) 
                                                &rest args &allow-other-keys)
  (push-motion :rarm :move-robot move-robot :use-torso use-torso
               :push-axes (list :z)
               :push-positions (list :right)
               :push-directions (list :bottom)
               :approach-directions (list :back)
               :hold-type :tight
               ;; :hold-type 100
               :approach-arm :rarm
               :robot robot :ri ri :cube cube)
  t)


(defun rarm-top-left-side-push-action (&key (move-robot t) (use-torso t)
                                            (robot *robot*) (ri *ri*) (cube *cube*) 
                                            &rest args &allow-other-keys)
  (push-motion :rarm :move-robot move-robot :use-torso use-torso
               :push-axes (list :z)
               :push-positions (list :left)
               :push-directions (list :top)
               :approach-directions (list :back)
               :approach-arm :rarm
               :robot robot :ri ri :cube cube)
  t)


(defun rarm-push-release-action (&key (move-robot t) (use-torso t)
                                      (robot *robot*) (ri *ri*) (cube *cube*) 
                                      &rest args &allow-other-keys)
  (push-release-action :rarm :move-robot move-robot :use-torso use-torso
                       :robot robot :ri ri :cube cube))


(provide :common-actions "common-actions.l")
