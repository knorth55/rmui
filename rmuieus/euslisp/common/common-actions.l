(require :common-motions "package://rmuieus/euslisp/common/common-motions.l")


(defun action-init (&key (table-pos-y 0) (n-box 1) (box-types :ctb)
                         (move-robot t) &allow-other-keys)
  (scene-init :table-pos-y table-pos-y
              :n-box n-box
              :box-types box-types)
  (motion-init :move-robot move-robot
               :robot *robot* :ri *ri* :cubes *boxes* :vw *irtviewer*)
  t)


(defun pilot-action-init (&key (device-pos-y 0) &allow-other-keys)
  (pilot-scene-init :device-pos-y device-pos-y)
  (motion-init :move-robot nil
               :robot *pilot* :ri nil :cubes *pilot-devices* :vw *pilotviewer*)
  t)


(defun statenet-action-init (&rest args)
  t)


;; common arm actions

(defun front-push-action (arm &rest args)
  (let ((executed nil))
    (setq executed
          (apply #'push-motion
                 (append
                   (list arm
                         :push-axes (list :x)
                         :push-positions (list :center)
                         :push-directions (list :front)
                         :approach-directions
                         (list (if (eq arm :larm) :left :right))
                         :approach-arm arm)
                   args)))
    (if executed t nil)))


(defun back-push-action (arm &rest args)
  (let ((executed nil))
    (setq executed
          (apply #'push-motion
                 (append
                   (list arm
                         :push-axes (list :x)
                         :push-positions (list :center)
                         :push-directions (list :back)
                         :approach-directions
                         (list (if (eq arm :larm) :left :right))
                         :approach-arm arm)
                   args)))
    (if executed t nil)))


(defun top-push-action (arm &rest args)
  (let ((executed nil))
    (setq executed
          (apply #'push-motion
                 (append
                   (list arm
                         :push-axes (list :z)
                         :push-positions (list :center)
                         :push-directions (list :top)
                         :approach-directions
                         (list (if (eq arm :larm) :left :right))
                         :approach-arm arm)
                   args)))
    (if executed t nil)))


(defun move-push-action (arm &rest args &key (ri *ri*) (cube *box0*)
                             &allow-other-keys)
  (let ((executed nil))
    (assoc-cube arm :ri ri :cube cube)
    (setq executed
          (apply #'push-move-motion
                 (append
                   (list arm
                         :push-axis :y
                         :move-distance (if (eq arm :larm) -200.0 200.0))
                         args)))
    (dissoc-cube arm :ri ri :cube cube)
    (if executed t nil)))


(defun push-release-action (&rest args)
  (let ((executed nil))
    (setq executed (apply #'push-release-and-reset-motion args))
    (if executed t nil)))


;; dualarm actions

(defun dualarm-hold-up-action (&rest args &key (ri *ri*) (cube *box0*)
                                     &allow-other-keys)
  (let ((executed nil))
    (assoc-cube :arms :ri ri :cube cube)
    (setq executed
          (apply #'hold-lift-up-motion (append (list :lift-height 200) args)))
    (dissoc-cube :arms :ri ri :cube cube)
    (if executed t nil)))


(defun dualarm-left-right-push-hold-action (&rest args)
  (let ((executed nil))
    (setq executed
          (apply #'push-hold-motion
                 (append
                   (list :push-axis :y
                         :push-position :center
                         :approach-direction :back)
                   args)))
    (if executed t nil)))


(defun dualarm-front-left-back-right-push-hold-action
  (&rest args &key (cube *box0*) &allow-other-keys)
  (let* ((cube-rpy (float-vector (/ -pi 4.0) 0 0))
         (cube-height (get-cube-height :cube-rpy cube-rpy :cube cube))
         (cube-width (get-cube-width :cube-rpy cube-rpy :cube cube))
         (cube-depth (get-cube-depth :cube-rpy cube-rpy :cube cube))
         (executed nil))
    (setq executed
          (apply #'push-hold-motion
                 (append
                   (list :push-axis :y
                         :push-position :center
                         :approach-direction :back
                         :cube-height cube-height
                         :cube-width cube-width
                         :cube-depth cube-depth
                         :cube-rpy cube-rpy)
                   args)))
    (if executed t nil)))


(defun dualarm-back-left-front-right-push-hold-action
  (&rest args &key (cube *box0*) &allow-other-keys)
  (let* ((cube-rpy (float-vector (/ pi 4.0) 0 0))
         (cube-height (get-cube-height :cube-rpy cube-rpy :cube cube))
         (cube-width (get-cube-width :cube-rpy cube-rpy :cube cube))
         (cube-depth (get-cube-depth :cube-rpy cube-rpy :cube cube))
         (executed nil))
    (setq executed
          (apply #'push-hold-motion
                 (append
                   (list :push-axis :y
                         :push-position :center
                         :approach-direction :back
                         :cube-height cube-height
                         :cube-width cube-width
                         :cube-depth cube-depth
                         :cube-rpy cube-rpy)
                   args)))
    (if executed t nil)))


(defun dualarm-hold-release-action (&rest args)
  (apply #'push-release-action (append (list :arms) args)))


(defun dualarm-push-hold-up-action (&rest args)
  (apply #'dualarm-hold-up-action args))


(defun dualarm-support-hold-up-action (&rest args)
  (apply #'dualarm-hold-up-action args))


(defun dualarm-hold-down-action (&rest args &key (ri *ri*) (cube *box0*)
                                       &allow-other-keys)
  (let ((executed nil))
    (assoc-cube :arms :ri ri :cube cube)
    (setq executed
          (apply #'hold-lift-down-motion args))
    (dissoc-cube :arms :ri ri :cube cube)
    (if executed t nil)))


(defun dualarm-push-hold-down-action (&rest args)
  (apply #'dualarm-hold-down-action args))


(defun dualarm-support-hold-down-action (&rest args)
  (apply #'dualarm-hold-down-action args))


(defun dualarm-push-rotate-ccw-x45-action (&rest args &key (ri *ri*) (cube *box0*)
                                                 &allow-other-keys)
  (let* ((initial-rotate-angle
           (if (and (equal larm-contact-state :bottom-left-contact)
                    (equal rarm-contact-state :top-right-contact))
             (/ -pi 4.0) 0))
         (cube-rpy (float-vector 0 0 initial-rotate-angle))
         (cube-height (get-cube-height :cube-rpy cube-rpy :cube cube))
         (cube-width (get-cube-width :cube-rpy cube-rpy :cube cube))
         (cube-depth (get-cube-depth :cube-rpy cube-rpy :cube cube))
         (executed nil))
    (assoc-cube :arms :ri ri :cube cube)
    (setq executed
          (apply #'push-rotate-motion
                 (append
                   (list :rotate-angle (/ pi 4.0)
                         :rotate-axis :x
                         :initial-rotate-angle initial-rotate-angle
                         :cube-height cube-height
                         :cube-width cube-width
                         :cube-depth cube-depth)
                   args)))
    (dissoc-cube :arms :ri ri :cube cube)
    (if executed t nil)))


(defun dualarm-push-rotate-cw-x45-action (&rest args &key (ri *ri*) (cube *box0*)
                                                &allow-other-keys)
  (let* ((initial-rotate-angle
           (if (and (equal larm-contact-state :top-left-contact)
                    (equal rarm-contact-state :bottom-right-contact))
             (/ pi 4.0) 0))
         (cube-rpy (float-vector 0 0 initial-rotate-angle))
         (cube-height (get-cube-height :cube-rpy cube-rpy :cube cube))
         (cube-width (get-cube-width :cube-rpy cube-rpy :cube cube))
         (cube-depth (get-cube-depth :cube-rpy cube-rpy :cube cube))
         (executed nil))
    (assoc-cube :arms :ri ri :cube cube)
    (setq executed
          (apply #'push-rotate-motion
                 (append
                   (list :rotate-angle (/ -pi 4.0)
                         :rotate-axis :x
                         :initial-rotate-angle initial-rotate-angle
                         :cube-height cube-height
                         :cube-width cube-width
                         :cube-depth cube-depth)
                   args)))
    (dissoc-cube :arms :ri ri :cube cube)
    (if executed t nil)))


(defun dualarm-push-rotate-ccw-x90-action (&rest args &key (ri *ri*) (cube *box0*)
                                                 &allow-other-keys)
  (let* ((initial-rotate-angle 0)
         (cube-rpy (float-vector 0 0 initial-rotate-angle))
         (cube-height (get-cube-height :cube-rpy cube-rpy :cube cube))
         (cube-width (get-cube-width :cube-rpy cube-rpy :cube cube))
         (cube-depth (get-cube-depth :cube-rpy cube-rpy :cube cube))
         (executed nil))
    (assoc-cube :arms :ri ri :cube cube)
    (setq executed
          (apply #'push-rotate-motion
                 (append
                   (list :rotate-angle (/ pi 2.0)
                         :rotate-axis :x
                         :initial-rotate-angle initial-rotate-angle
                         :cube-height cube-height
                         :cube-width cube-width
                         :cube-depth cube-depth)
                   args)))
    (dissoc-cube :arms :ri ri :cube cube)
    (if executed t nil)))


(defun dualarm-lift-push-rotate-ccw-x90-action (&rest args)
  (apply #'dualarm-push-rotate-ccw-x90-action args))


(defun dualarm-side-push-rotate-ccw-x90-action (&rest args)
  (apply #'dualarm-push-rotate-ccw-x90-action args))


(defun dualarm-push-rotate-cw-x90-action (&rest args &key (ri *ri*) (cube *box0*)
                                                &allow-other-keys)
  (let* ((initial-rotate-angle 0)
         (cube-rpy (float-vector 0 0 initial-rotate-angle))
         (cube-height (get-cube-height :cube-rpy cube-rpy :cube cube))
         (cube-width (get-cube-width :cube-rpy cube-rpy :cube cube))
         (cube-depth (get-cube-depth :cube-rpy cube-rpy :cube cube))
         (executed nil))
    (assoc-cube :arms :ri ri :cube cube)
    (setq executed
          (apply #'push-rotate-motion
                 (append
                   (list :rotate-angle (/ -pi 2.0)
                         :rotate-axis :x
                         :initial-rotate-angle initial-rotate-angle
                         :cube-height cube-height
                         :cube-width cube-width
                         :cube-depth cube-depth)
                   args)))
    (dissoc-cube :arms :ri ri :cube cube)
    (if executed t nil)))


(defun dualarm-lift-push-rotate-cw-x90-action (&rest args)
  (apply #'dualarm-push-rotate-cw-x90-action args))


(defun dualarm-side-push-rotate-cw-x90-action (&rest args)
  (apply #'dualarm-push-rotate-cw-x90-action args))


(defun dualarm-push-rotate-ccw-z45-action (&rest args &key (ri *ri*) (cube *box0*)
                                                 &allow-other-keys)
  (let ((executed nil))
    (assoc-cube :arms :ri ri :cube cube)
    (setq executed
          (apply #'push-rotate-motion
                 (append (list :rotate-angle (/ pi 4.0) :rotate-axis :z) args)))
    (dissoc-cube :arms :ri ri :cube cube)
    (if executed t nil)))


(defun dualarm-push-rotate-cw-z45-action (&rest args &key (ri *ri*) (cube *box0*)
                                                &allow-other-keys)
  (let ((executed nil))
    (assoc-cube :arms :ri ri :cube cube)
    (setq executed
          (apply #'push-rotate-motion
                 (append (list :rotate-angle (/ -pi 4.0) :rotate-axis :z) args)))
    (dissoc-cube :arms :ri ri :cube cube)
    (if executed t nil)))


(defun dualarm-support-rotate-ccw-x45-action (&rest args &key (ri *ri*) (cube *box0*)
                                                    &allow-other-keys)
  (let* ((initial-rotate-angle
           (if (and (equal larm-contact-state :top-left-contact)
                    (equal rarm-contact-state :top-right-contact))
             (/ pi 4.0) 0))
         (cube-rpy (float-vector 0 0 initial-rotate-angle))
         (cube-height (get-cube-height :cube-rpy cube-rpy :cube cube))
         (cube-width (get-cube-width :cube-rpy cube-rpy :cube cube))
         (cube-depth (get-cube-depth :cube-rpy cube-rpy :cube cube))
         (executed nil))
    (assoc-cube :arms :ri ri :cube cube)
    (setq executed
          (apply #'support-rotate-motion
                 (append
                   (list :rotate-angle (/ pi 4.0)
                         :rotate-axis :x
                         :initial-rotate-angle initial-rotate-angle
                         :cube-height cube-height
                         :cube-width cube-width
                         :cube-depth cube-depth)
                   args)))
    (dissoc-cube :arms :ri ri :cube cube)
    (if executed t nil)))


(defun dualarm-support-rotate-cw-x45-action (&rest args &key (ri *ri*) (cube *box0*)
                                                   &allow-other-keys)
  (let* ((initial-rotate-angle
           (if (and (equal larm-contact-state :top-left-contact)
                    (equal rarm-contact-state :top-right-contact))
             (/ -pi 4.0) 0))
         (cube-rpy (float-vector 0 0 initial-rotate-angle))
         (cube-height (get-cube-height :cube-rpy cube-rpy :cube cube))
         (cube-width (get-cube-width :cube-rpy cube-rpy :cube cube))
         (cube-depth (get-cube-depth :cube-rpy cube-rpy :cube cube))
         (executed nil))
    (assoc-cube :arms :ri ri :cube cube)
    (setq executed
          (apply #'support-rotate-motion
                 (append
                   (list :rotate-angle (/ -pi 4.0)
                         :rotate-axis :x
                         :initial-rotate-angle initial-rotate-angle
                         :cube-height cube-height
                         :cube-width cube-width
                         :cube-depth cube-depth)
                   args)))
    (dissoc-cube :arms :ri ri :cube cube)
    (if executed t nil)))


(defun dualarm-push-hold-pile-up-action (&rest args &key (ri *ri*) (cube *box0*)
                                               &allow-other-keys)
  (let ((executed nil))
    (assoc-cube :arms :ri ri :cube cube)
    (setq executed (apply #'hold-pile-up-motion args))
    (dissoc-cube :arms :ri ri :cube cube)
    (if executed t nil)))


(defun dualarm-front-wall-push-next-to-action (&rest args &key (ri *ri*) (cube *box0*)
                                                     (wall-front-limit *wall-front-limit*)
                                                     &allow-other-keys)
  (let ((executed nil))
    (assoc-cube :larm :ri ri :cube cube)
    (setq executed (apply #'push-front-wall-next-to-motion (append (list :arms) args)))
    (dissoc-cube :larm :ri ri :cube cube)
    (if executed t nil)))


(defun dualarm-front-wall-push-release-action (&rest args &key (ri *ri*) (cube *box0*)
                                                     &allow-other-keys)
  (let ((executed nil))
    (assoc-cube :larm :ri ri :cube cube)
    (setq executed
          (apply #'hold-move-motion
                 (append
                   (list
                     :cube-pos-x
                     (elt (send cube :get :initial-pos) 0)
                     :cube-pos-y
                     (elt (send cube :worldpos) 1)
                     :cube-pos-z
                     (elt (send cube :worldpos) 2))
                   args)))
    (dissoc-cube :larm :ri ri :cube cube)
    (if executed t nil)))


;; larm actions

(defun larm-left-push-action (&rest args)
  (let ((executed nil))
    (setq executed
          (apply #'push-motion
                 (append
                   (list :larm
                         :push-axes (list :y)
                         :push-positions (list :center)
                         :push-directions (list :left)
                         :approach-directions (list :back)
                         :approach-arm :larm)
                   args)))
    (if executed t nil)))


(defun larm-front-push-action (&rest args)
  (apply #'front-push-action (append (list :larm) args)))


(defun larm-back-push-action (&rest args)
  (apply #'back-push-action (append (list :larm) args)))


(defun larm-top-push-action (&rest args)
  (apply #'top-push-action (append (list :larm) args)))


(defun larm-top-left-push-action (&rest args &key (cube *box0*)
                                        &allow-other-keys)
  (let* ((cube-rpy (float-vector 0 0 (/ pi 4.0)))
         (cube-height (get-cube-height :cube-rpy cube-rpy :cube cube))
         (cube-width (get-cube-width :cube-rpy cube-rpy :cube cube))
         (cube-depth (get-cube-depth :cube-rpy cube-rpy :cube cube))
         (executed nil))
    (setq executed
          (apply #'push-motion
                 (append
                   (list :larm
                         :push-axes (list :y)
                         :push-positions (list :center)
                         :push-directions (list :left)
                         :approach-directions (list :back)
                         :cube-height cube-height
                         :cube-width cube-width
                         :cube-depth cube-depth
                         :cube-rpy cube-rpy
                         :approach-arm :larm)
                   args)))
    (if executed t nil)))


(defun larm-bottom-left-push-action (&rest args &key (cube *box0*)
                                           &allow-other-keys)
  (let* ((cube-rpy (float-vector 0 0 (/ -pi 4.0)))
         (cube-height (get-cube-height :cube-rpy cube-rpy :cube cube))
         (cube-width (get-cube-width :cube-rpy cube-rpy :cube cube))
         (cube-depth (get-cube-depth :cube-rpy cube-rpy :cube cube))
         (executed nil))
    (setq executed
          (apply #'push-motion
                 (append
                   (list :larm
                         :push-axes (list :y)
                         :push-positions (list :center)
                         :push-directions (list :left)
                         :approach-directions (list :back)
                         :cube-height cube-height
                         :cube-width cube-width
                         :cube-depth cube-depth
                         :cube-rpy cube-rpy
                         :approach-arm :larm)
                   args)))
    (if executed t nil)))


(defun larm-front-left-push-action (&rest args &key (cube *box0*)
                                          &allow-other-keys)
  (let* ((cube-rpy (float-vector (/ -pi 4.0) 0 0))
         (cube-height (get-cube-height :cube-rpy cube-rpy :cube cube))
         (cube-width (get-cube-width :cube-rpy cube-rpy :cube cube))
         (cube-depth (get-cube-depth :cube-rpy cube-rpy :cube cube))
         (executed nil))
    (setq executed
          (apply #'push-motion
                 (append
                   (list :larm
                         :push-axes (list :y)
                         :push-positions (list :center)
                         :push-directions (list :left)
                         :approach-directions (list :back)
                         :cube-height cube-height
                         :cube-width cube-width
                         :cube-depth cube-depth
                         :cube-rpy cube-rpy
                         :approach-arm :larm)
                   args)))
    (if executed t nil)))


(defun larm-back-left-push-action (&rest args &key (cube *box0*)
                                         &allow-other-keys)
  (let* ((cube-rpy (float-vector (/ pi 4.0) 0 0))
         (cube-height (get-cube-height :cube-rpy cube-rpy :cube cube))
         (cube-width (get-cube-width :cube-rpy cube-rpy :cube cube))
         (cube-depth (get-cube-depth :cube-rpy cube-rpy :cube cube))
         (executed nil))
    (setq executed
          (apply #'push-motion
                 (append
                   (list :larm
                         :push-axes (list :y)
                         :push-positions (list :center)
                         :push-directions (list :left)
                         :approach-directions (list :back)
                         :cube-height cube-height
                         :cube-width cube-width
                         :cube-depth cube-depth
                         :cube-rpy cube-rpy
                         :approach-arm :larm)
                   args)))
    (if executed t nil)))


(defun larm-left-move-push-action (&rest args)
  (apply #'move-push-action (append (list :larm) args)))


(defun larm-bottom-left-side-push-action (&rest args)
  (let ((executed nil))
    (setq executed
          (apply #'push-motion
                 (append
                   (list :larm
                         :push-axes (list :z)
                         :push-positions (list :left)
                         :push-directions (list :bottom)
                         :approach-directions (list :back)
                         :hold-type :tight
                         ;; :hold-type 100
                         :approach-arm :larm)
                   args)))
    (if executed t nil)))


(defun larm-top-right-side-push-action (&rest args)
  (let ((executed nil))
    (setq executed
          (apply #'push-motion
                 (append
                   (list :larm
                         :push-axes (list :z)
                         :push-positions (list :right)
                         :push-directions (list :top)
                         :approach-directions (list :back)
                         :approach-arm :larm)
                   args)))
    (if executed t nil)))


(defun larm-push-release-action (&rest args)
  (apply #'push-release-action (append (list :larm) args)))


(defun larm-right-push-next-to-action (&rest args &key (ri *ri*) (cube *box0*)
                                             &allow-other-keys)
  (let ((executed nil))
    (assoc-cube :larm :ri ri :cube cube)
    (setq executed
          (apply #'push-next-to-motion
                 (append
                   (list :larm
                         :push-axis :y)
                   args)))
    (dissoc-cube :larm :ri ri :cube cube)
    (if executed t nil)))


(defun larm-front-push-next-to-action (&rest args &key (ri *ri*) (cube *box0*)
                                             &allow-other-keys)
  (let ((executed nil))
    (assoc-cube :larm :ri ri :cube cube)
    (setq executed
          (apply #'push-next-to-motion
                 (append
                   (list :larm
                         :push-axis :x)
                   args)))
    (dissoc-cube :larm :ri ri :cube cube)
    (if executed t nil)))


;; rarm actions

(defun rarm-right-push-action (&rest args)
  (let ((executed nil))
    (setq executed
          (apply #'push-motion
                 (append
                   (list :rarm
                         :push-axes (list :y)
                         :push-positions (list :center)
                         :push-directions (list :right)
                         :approach-directions (list :back)
                         :approach-arm :rarm)
                   args)))
    (if executed t nil)))


(defun rarm-front-push-action (&rest args)
  (apply #'front-push-action (append (list :rarm) args)))


(defun rarm-back-push-action (&rest args)
  (apply #'back-push-action (append (list :rarm) args)))


(defun rarm-top-push-action (&rest args)
  (apply #'top-push-action (append (list :rarm) args)))


(defun rarm-top-right-push-action (&rest args &key (cube *box0*)
                                         &allow-other-keys)
  (let* ((cube-rpy (float-vector 0 0 (/ -pi 4.0)))
         (cube-height (get-cube-height :cube-rpy cube-rpy :cube cube))
         (cube-width (get-cube-width :cube-rpy cube-rpy :cube cube))
         (cube-depth (get-cube-depth :cube-rpy cube-rpy :cube cube))
         (executed nil))
    (setq executed
          (apply #'push-motion
                 (append
                   (list :rarm
                         :push-axes (list :y)
                         :push-positions (list :center)
                         :push-directions (list :right)
                         :approach-directions (list :back)
                         :cube-height cube-height
                         :cube-width cube-width
                         :cube-depth cube-depth
                         :cube-rpy cube-rpy
                         :approach-arm :rarm)
                   args)))
    (if executed t nil)))


(defun rarm-bottom-right-push-action (&rest args &key (cube *box0*)
                                            &allow-other-keys)
  (let* ((cube-rpy (float-vector 0 0 (/ pi 4.0)))
         (cube-height (get-cube-height :cube-rpy cube-rpy :cube cube))
         (cube-width (get-cube-width :cube-rpy cube-rpy :cube cube))
         (cube-depth (get-cube-depth :cube-rpy cube-rpy :cube cube))
         (executed nil))
    (setq executed
          (apply #'push-motion
                 (append
                   (list :rarm
                         :push-axes (list :y)
                         :push-positions (list :center)
                         :push-directions (list :right)
                         :approach-directions (list :back)
                         :cube-height cube-height
                         :cube-width cube-width
                         :cube-depth cube-depth
                         :cube-rpy cube-rpy
                         :approach-arm :rarm)
                   args)))
    (if executed t nil)))


(defun rarm-front-right-push-action (&rest args &key (cube *box0*)
                                           &allow-other-keys)
  (let* ((cube-rpy (float-vector (/ pi 4.0) 0 0))
         (cube-height (get-cube-height :cube-rpy cube-rpy :cube cube))
         (cube-width (get-cube-width :cube-rpy cube-rpy :cube cube))
         (cube-depth (get-cube-depth :cube-rpy cube-rpy :cube cube))
         (executed nil))
    (setq executed
          (apply #'push-motion
                 (append
                   (list :rarm
                         :push-axes (list :y)
                         :push-positions (list :center)
                         :push-directions (list :right)
                         :approach-directions (list :back)
                         :cube-height cube-height
                         :cube-width cube-width
                         :cube-depth cube-depth
                         :cube-rpy cube-rpy
                         :approach-arm :rarm)
                   args)))
    (if executed t nil)))


(defun rarm-back-right-push-action (&rest args &key (cube *box0*)
                                          &allow-other-keys)
  (let* ((cube-rpy (float-vector (/ -pi 4.0) 0 0))
         (cube-height (get-cube-height :cube-rpy cube-rpy :cube cube))
         (cube-width (get-cube-width :cube-rpy cube-rpy :cube cube))
         (cube-depth (get-cube-depth :cube-rpy cube-rpy :cube cube))
         (executed nil))
    (setq executed
          (apply #'push-motion
                 (append
                   (list :rarm
                         :push-axes (list :y)
                         :push-positions (list :center)
                         :push-directions (list :right)
                         :approach-directions (list :back)
                         :cube-height cube-height
                         :cube-width cube-width
                         :cube-depth cube-depth
                         :cube-rpy cube-rpy
                         :approach-arm :rarm)
                   args)))
    (if executed t nil)))


(defun rarm-right-move-push-action (&rest args)
  (apply #'move-push-action (append (list :rarm) args)))


(defun rarm-bottom-right-side-push-action (&rest args)
  (let ((executed nil))
    (setq executed
          (apply #'push-motion
                 (append
                   (list :rarm
                         :push-axes (list :z)
                         :push-positions (list :right)
                         :push-directions (list :bottom)
                         :approach-directions (list :back)
                         :hold-type :tight
                         ;; :hold-type 100
                         :approach-arm :rarm)
                   args)))
    (if executed t nil)))


(defun rarm-top-left-side-push-action (&rest args)
  (let ((executed nil))
    (setq executed
          (apply #'push-motion
                 (append
                   (list :rarm
                         :push-axes (list :z)
                         :push-positions (list :left)
                         :push-directions (list :top)
                         :approach-directions (list :back)
                         :approach-arm :rarm)
                   args)))
    (if executed t nil)))


(defun rarm-push-release-action (&rest args)
  (apply #'push-release-action (append (list :rarm) args)))


(defun rarm-left-push-next-to-action (&rest args &key (ri *ri*) (cube *box0*)
                                             &allow-other-keys)
  (let ((executed nil))
    (assoc-cube :rarm :ri ri :cube cube)
    (setq executed
          (apply #'push-next-to-motion
                 (append
                   (list :rarm
                         :push-axis :y)
                   args)))
    (dissoc-cube :rarm :ri ri :cube cube)
    (if executed t nil)))


(defun rarm-back-push-next-to-action (&rest args &key (ri *ri*) (cube *box0*)
                                             &allow-other-keys)
  (let ((executed nil))
    (assoc-cube :rarm :ri ri :cube cube)
    (setq executed
          (apply #'push-next-to-motion
                 (append
                   (list :rarm
                         :push-axis :x)
                   args)))
    (dissoc-cube :rarm :ri ri :cube cube)
    (if executed t nil)))


(defun rarm-left-wall-push-next-to-action (&rest args &key (ri *ri*) (cube *box0*)
                                                 (wall-left-limit *wall-left-limit*)
                                                 &allow-other-keys)
  (let ((executed nil))
    (assoc-cube :rarm :ri ri :cube cube)
    (setq executed
          (apply #'push-left-wall-next-to-motion (append (list :rarm) args)))
    (dissoc-cube :rarm :ri ri :cube cube)
    (if executed t nil)))


(defun rarm-front-wall-push-next-to-action (&rest args &key (ri *ri*) (cube *box0*)
                                                  (wall-front-limit *wall-front-limit*)
                                                  &allow-other-keys)
  (let ((executed nil))
    (assoc-cube :rarm :ri ri :cube cube)
    (setq executed
          (apply #'push-front-wall-next-to-motion (append (list :rarm) args)))
    (dissoc-cube :rarm :ri ri :cube cube)
    (if executed t nil)))


(provide :common-actions "common-actions.l")
