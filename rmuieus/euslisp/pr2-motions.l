(require :pr2-interface "package://pr2eus/pr2-interface.l")


;; init

(defun motion-init ()
  (pr2-init)
  (setq *cube* (make-cube 350 350 265))
  (send *cube* :translate #f(450 0 800))
  (reset-motion)
  (objects (list *pr2* *cube*)))


;; motions
;; push & push-hold

(defun get-push-target-coords
  (arm &key (motion-type :push)
            (push-axis :y)
            (push-position :center)
            (push-direction :left)
            (approach-direction :back))

  (when (not (or (eq arm :larm) (eq arm :rarm)))
    (return-from get-push-target-coords nil))
  (when (not (or (eq push-axis :x) (eq push-axis :y) (eq push-axis :z)))
    (return-from get-push-target-coords nil))
  (when (or (and (eq push-axis :x)
                 (not (and (or (eq push-direction :front)
                               (eq push-direction :back))
                           (or (and (or (and (eq arm :larm)
                                             (eq approach-direction :left))
                                        (and (eq arm :rarm)
                                             (eq approach-direction :right)))
                                    (or (eq push-position :upper)
                                        (eq push-position :center)
                                        (eq push-position :lower)))
                               (and (eq approach-direction :upper)
                                    (or (eq push-position :left)
                                        (eq push-position :center)
                                        (eq push-position :right)))))))
            (and (eq push-axis :y)
                 (not (and (or (eq push-direction :left)
                               (eq push-direction :right))
                           (or (and (eq approach-direction :back)
                                    (or (eq push-position :upper)
                                        (eq push-position :center)
                                        (eq push-position :lower)))
                               (and (eq approach-direction :upper)
                                    (or (eq push-position :front)
                                        (eq push-position :center)
                                        (eq push-position :back))))))))
    (return-from get-push-target-coords nil))
  (cond
    ((eq push-axis :x)
     (let ((target-coords (send (send *cube* :worldcoords) :copy-worldcoords))
           (x-offset (if (eq motion-type :approach) 50 0)))
       (cond
         ((or (and (eq arm :larm) (eq approach-direction :left))
              (and (eq arm :rarm) (eq approach-direction :right)))
          (let ((z-offset (cond ((eq push-position :center) 0.0)
                                ((eq push-position :upper)
                                 (- (/ (z-of-cube *cube*) 2.0) 50))
                                ((eq push-position :lower)
                                 (- 50 (/ (z-of-cube *cube*) 2.0))))))
            (send target-coords :translate
                  (float-vector
                    (* (if (eq push-direction :front) 1.0 -1.0)
                       (+ (/ (x-of-cube *cube*) 2.0) x-offset))
                    (if (eq arm :larm) -100 100)
                    z-offset))
            (send target-coords :rotate (if (eq push-direction :front) -pi/2 pi/2) :y)
            (send target-coords :rotate (if (eq arm :larm) -pi/2 pi/2) :z)))
         ((eq approach-direction :upper)
          (let ((y-offset (cond ((eq push-position :center) 0.0)
                                ((eq push-position :left)
                                 (- (/ (y-of-cube *cube*) 2.0) 50))
                                ((eq push-position :right)
                                 (- 50 (/ (y-of-cube *cube*) 2.0))))))

            (send target-coords :translate
                  (float-vector
                    (* (if (eq push-direction :front) 1.0 -1.0)
                       (+ (/ (x-of-cube *cube*) 2.0) x-offset))
                    y-offset
                    -100))
            (send target-coords :rotate pi/2 :y)
            (when (eq push-direction :front)
              (send target-coords :rotate pi :x))))
         (t (return-from get-push-target-coords nil)))
       target-coords))
    ((eq push-axis :y)
     (let ((target-coords (send (send *cube* :worldcoords) :copy-worldcoords))
           (y-offset (if (eq motion-type :approach) 50 0)))
       (cond
         ((eq approach-direction :back)
          (let ((z-offset (cond ((eq push-position :center) 0.0)
                                ((eq push-position :upper)
                                 (- (/ (z-of-cube *cube*) 2.0) 50))
                                ((eq push-position :lower)
                                 (- 50 (/ (z-of-cube *cube*) 2.0))))))
            (send target-coords :translate
                  (float-vector
                    100
                    (* (if (eq push-direction :left) 1.0 -1.0)
                       (+ (/ (y-of-cube *cube*) 2.0) y-offset))
                    z-offset))
            (send target-coords :rotate (if (eq push-direction :left) pi/2 -pi/2) :x)))
         ((eq approach-direction :upper)
          (let ((x-offset (cond ((eq push-position :center) 0.0)
                                ((eq push-position :front)
                                 (- (/ (x-of-cube *cube*) 2.0) 50))
                                ((eq push-position :back)
                                 (- 50 (/ (x-of-cube *cube*) 2.0))))))
            (send target-coords :translate
                  (float-vector
                    x-offset
                    (* (if (eq push-direction :left) 1.0 -1.0)
                       (+ (/ (y-of-cube *cube*) 2.0) y-offset))
                    -100))
            (send target-coords :rotate (if (eq push-direction :left) pi/2 -pi/2) :x)
            (send target-coords :rotate (if (eq push-direction :left) -pi/2 pi/2) :z)))
         (t (return-from get-push-target-coords nil)))
       target-coords))
    (t (return-from get-push-target-coords nil))))


(defun push-motion-step
  (arm &key (motion-type :push)
            (push-axes (list :y))
            (push-positions (list :center))
            (push-directions (list :left))
            (approach-directions (list :back)))

  (when (not (or (eq arm :larm) (eq arm :rarm) (eq arm :arms)))
    (return-from push-motion-step nil))
  ;; push
  (let ((ik-solved))
    (if (eq arm :arms)
      (let ((larm-target-coords
              (get-push-target-coords
                :larm :motion-type motion-type
                :push-axis (car push-axes)
                :push-position (car push-positions)
                :push-direction (car push-directions)
                :approach-direction (car approach-directions)))
            (rarm-target-coords
              (get-push-target-coords
                :rarm :motion-type motion-type
                :push-axis (cadr push-axes)
                :push-position (cadr push-positions)
                :push-direction (cadr push-directions)
                :approach-direction (cadr approach-directions))))
        (when (or (null larm-target-coords)
                  (null rarm-target-coords))
          (return-from push-motion-step nil))
        (setq ik-solved
              (send *pr2* :inverse-kinematics
                    (list larm-target-coords rarm-target-coords)
                    :move-target (list (send *pr2* :larm :end-coords)
                                       (send *pr2* :rarm :end-coords))
                    :rotation-axis (list t t)
                    :use-torso 0.001))
        (unless ik-solved
          (setq ik-solved
                (send *pr2* :inverse-kinematics
                      (list larm-target-coords rarm-target-coords)
                      :move-target (list (send *pr2* :larm :end-coords)
                                         (send *pr2* :rarm :end-coords))
                      :rotation-axis (list :z :z)
                      :use-torso 0.001))))
      (let ((target-coords
              (get-push-target-coords
                arm :motion-type motion-type
                :push-axis (car push-axes)
                :push-position (car push-positions)
                :push-direction (car push-directions)
                :approach-direction (car approach-directions))))
        (when (null target-coords)
          (return-from push-motion-step nil))
        (setq ik-solved
              (send *pr2* arm :inverse-kinematics target-coords
                    :rotation-axis t
                    :use-torso 0.001))
        (unless ik-solved
          (setq ik-solved
                (send *pr2* arm :inverse-kinematics target-coords
                      :rotation-axis :z
                      :use-torso 0.001)))))
    (when ik-solved
      (send *irtviewer* :draw-objects)
      (send *ri* :angle-vector (send *pr2* :angle-vector) 5000
            (cond ((eq arm :larm) :larm-controller)
                  ((eq arm :rarm) :rarm-controller)
                  (t nil))
            0)
      (send *ri* :angle-vector (send *pr2* :angle-vector) 5000 :torso-controller 0)
      (send *ri* :wait-interpolation))))


(defun push-motion
  (arm &key (push-axes (list :y))
            (push-positions (list :center))
            (push-directions (list :left))
            (approach-directions (list :back)))

  (when (not (or (eq arm :larm) (eq arm :rarm) (eq arm :arms)))
    (return-from push-motion nil))
  (let ((approached))
    (setq approached
      (push-motion-step
        arm :motion-type :approach
        :push-axes push-axes
        :push-positions push-positions
        :push-directions push-directions
        :approach-directions approach-directions))
    (when approached
      (push-motion-step
        arm :motion-type :push
        :push-axes push-axes
        :push-positions push-positions
        :push-directions push-directions
        :approach-directions approach-directions))))


(defun push-hold-motion
  (&key (push-axis :y)
        (push-position :center)
        (approach-direction :back)
        (front-arm :larm)
        (upper-arm :larm))

  (when (not (or (eq front-arm :larm)
                 (eq front-arm :rarm)))
    (ros::ros-error (format nil "front-arm is invalid: ~A" front-arm))
    (return-from push-hold-motion nil))
  (when (not (or (eq upper-arm :larm)
                 (eq upper-arm :rarm)))
    (ros::ros-error (format nil "upper-arm is invalid: ~A" upper-arm))
    (return-from push-hold-motion nil))
  (when (not (or (eq approach-direction :back)
                 (eq approach-direction :upper)
                 (eq approach-direction :side)))
    (ros::ros-error (format nil "approach-direction is invalid: ~A" approach-direction))
    (return-from push-hold-motion nil))
  (let ((push-directions
          (cond ((eq push-axis :x)
                 (if (eq front-arm :larm)
                   (list :front :back) (list :back :front)))
                ((eq push-axis :y) (list :left :right))
                ((eq push-axis :z)
                 (if (eq upper-arm :larm)
                   (list :upper :lower) (list :upper :lower)))))
        (approach-directions
          (cond ((eq approach-direction :side)
                 (list :left :right))
                (t (list approach-direction approach-direction)))))
    (push-motion
      :arms
      :push-axes (list push-axis push-axis)
      :push-positions (list push-position push-position)
      :push-directions push-directions
      :approach-directions approach-directions)))


;; push-release & push-hold-release

(defun get-push-release-target-coords (arm)
  (let ((target-coords (send (send *pr2* arm :end-coords) :copy-worldcoords)))
    (send target-coords :translate (float-vector 0 0 -50))
    target-coords))


(defun push-release-motion (arm)
  ;; release
  (when (not (or (eq arm :larm) (eq arm :rarm) (eq arm :arms)))
    (return-from push-release-motion nil))
  (let ((ik-solved))
    (if (eq arm :arms)
      (let ((larm-target-coords (get-push-release-target-coords :larm))
            (rarm-target-coords (get-push-release-target-coords :rarm)))
        (setq ik-solved
              (send *pr2* :inverse-kinematics
                    (list larm-target-coords rarm-target-coords)
                    :move-target (list (send *pr2* :larm :end-coords)
                                       (send *pr2* :rarm :end-coords))
                    :rotation-axis (list t t)
                    :use-torso 0.001))
        (unless ik-solved
          (setq ik-solved
                (send *pr2* :inverse-kinematics
                      (list larm-target-coords rarm-target-coords)
                      :move-target (list (send *pr2* :larm :end-coords)
                                         (send *pr2* :rarm :end-coords))
                      :rotation-axis (list :z :z)
                      :use-torso 0.001))))
      (let ((target-coords (get-push-release-target-coords arm)))
        (setq ik-solved
              (send *pr2* arm :inverse-kinematics
                    target-coords :rotation-axis t :use-torso 0.001))
        (unless ik-solved
          (setq ik-solved
                (send *pr2* arm :inverse-kinematics
                      target-coords :rotation-axis :z :use-torso 0.001)))))
    (when ik-solved
      (send *irtviewer* :draw-objects)
      (send *ri* :angle-vector (send *pr2* :angle-vector) 5000
            (cond ((eq arm :larm) :larm-controller)
                  ((eq arm :rarm) :rarm-controller)
                  (t nil))
            0)
      (send *ri* :angle-vector (send *pr2* :angle-vector) 5000 :torso-controller 0)
      (send *ri* :wait-interpolation))))


(defun push-hold-release-motion ()
  (push-release-motion :arms))


(defun hold-lift-up-motion ()
  (let ((larm-target-coords (send (send *pr2* :larm :end-coords) :copy-worldcoords))
        (rarm-target-coords (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
        ik-solved)
    ;; lift up
    (send larm-target-coords :translate (float-vector 0 0 100) :world)
    (send rarm-target-coords :translate (float-vector 0 0 100) :world)
    (setq ik-solved
          (send *pr2* :inverse-kinematics
                (list larm-target-coords rarm-target-coords)
                :rotation-axis (list t t)
                :move-target (list (send *pr2* :larm :end-coords)
                                   (send *pr2* :rarm :end-coords))
                :use-torso 0.001))
    (unless ik-solved
      (setq ik-solved
            (send *pr2* :inverse-kinematics
                  (list larm-target-coords rarm-target-coords)
                  :rotation-axis (list :z :z)
                  :move-target (list (send *pr2* :larm :end-coords)
                                     (send *pr2* :rarm :end-coords))
                  :use-torso 0.001)))
    (when ik-solved
      (send *irtviewer* :draw-objects)
      (send *ri* :angle-vector (send *pr2* :angle-vector) 5000 nil 0)
      (send *ri* :wait-interpolation))))


(defun hold-lift-down-motion ()
  (let ((larm-target-coords (send (send *pr2* :larm :end-coords) :copy-worldcoords))
        (rarm-target-coords (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
        ik-solved)
    ;; lift down
    (send larm-target-coords :translate (float-vector 0 0 -100) :world)
    (send rarm-target-coords :translate (float-vector 0 0 -100) :world)
    (setq ik-solved
          (send *pr2* :inverse-kinematics
                (list larm-target-coords rarm-target-coords)
                :rotation-axis (list t t)
                :move-target (list (send *pr2* :larm :end-coords)
                                   (send *pr2* :rarm :end-coords))
                :use-torso 0.001))
    (when ik-solved
      (send *irtviewer* :draw-objects)
      (send *ri* :angle-vector (send *pr2* :angle-vector) 5000 nil 0)
      (send *ri* :wait-interpolation))))


(defun reset-motion ()
  (send *pr2* :reset-manip-pose)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 5000 nil 0)
  (send *ri* :wait-interpolation))


(provide :pr2-motions "pr2-motions.l")
