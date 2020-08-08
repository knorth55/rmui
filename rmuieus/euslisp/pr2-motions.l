(require :pr2-interface "package://pr2eus/pr2-interface.l")


;; init

(defun motion-init ()
  (pr2-init)
  (send *pr2* :reset-manip-pose)
  (setq *cube* (make-cube 350 350 265))
  (send *cube* :translate #f(500 0 1000))
  (objects (list *pr2* *cube*))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 5000 nil 0)
  (send *ri* :wait-interpolation))


;; motions

(defun get-push-y-axis-target-coords (arm &key (push-position :center)
                                               (approach-direction :back)
                                               (approach nil))
  (let ((target-coords (send (send *cube* :worldcoords) :copy-worldcoords))
        (y-offset (if approach 100 0)))
    (cond
      ((eq approach-direction :back)
       (let ((z-offset (cond ((eq push-position :center) 0.0)
                             ((eq push-position :upper)
                              (- (/ (z-of-cube *cube*) 2.0) 50))
                             ((eq push-position :lower)
                              (- 50 (/ (z-of-cube *cube*) 2.0))))))
         (if (eq arm :larm)
           (progn
             (send target-coords :translate
                   (float-vector 100 (+ (/ (y-of-cube *cube*) 2.0) y-offset) z-offset))
             (send target-coords :rotate 1.57 :x))
           (progn
             (send target-coords :translate
                   (float-vector 100 (* -1.0 (+ (/ (y-of-cube *cube*) 2.0) y-offset)) z-offset))
             (send target-coords :rotate -1.57 :x)))))
      ((eq approach-direction :upper)
       (let ((x-offset (cond ((eq push-position :center) 0.0)
                             ((eq push-position :front)
                              (- (/ (x-of-cube *cube*) 2.0) 50))
                             ((eq push-position :back)
                              (- 50 (/ (x-of-cube *cube*) 2.0))))))
         (if (eq arm :larm)
           (progn
             (send target-coords :translate
                   (float-vector x-offset (+ (/ (y-of-cube *cube*) 2.0) y-offset) -100))
             (send target-coords :rotate 1.57 :x)
             (send target-coords :rotate -1.57 :z))
           (progn
             (send target-coords :translate
                   (float-vector x-offset (* -1.0 (+ (/ (y-of-cube *cube*) 2.0) y-offset)) -100))
             (send target-coords :rotate -1.57 :x)
             (send target-coords :rotate 1.57 :z)))))
      (t (return-from get-push-y-axis-target-coords nil)))
    target-coords))


(defun push-y-axis-step (arm &key (push-position :center)
                                  (approach-direction :back)
                                  (approach nil))
  ;; push
  (if (eq arm :arms)
    (let ((larm-target-coords
            (get-push-y-axis-target-coords :larm :approach approach
                                           :push-position push-position
                                           :approach-direction approach-direction))
          (rarm-target-coords
            (get-push-y-axis-target-coords :rarm :approach approach
                                           :push-position push-position
                                           :approach-direction approach-direction)))
      (when (or (null larm-target-coords)
                (null rarm-target-coords))
        (return-from push-y-axis-push nil))
      (send *pr2* :inverse-kinematics
            (list larm-target-coords rarm-target-coords)
            :move-target (list (send *pr2* :larm :end-coords)
                               (send *pr2* :rarm :end-coords))
            :rotation-axis (list t t)
            :use-torso 0.001))
    (let ((target-coords
            (get-push-y-axis-target-coords arm :approach nil
                                           :push-position push-position
                                           :approach-direction approach-direction)))
      (when (null target-coords)
        (return-from push-y-axis-push nil))
      (send *pr2* arm :inverse-kinematics
            target-coords :rotation-axis t :use-torso 0.001)))
  (send *irtviewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 5000 :torso-controller 0)
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 5000
        (cond ((eq arm :larm) :larm-controller)
              ((eq arm :rarm) :rarm-controller)
              (t nil))
        0)
  (send *ri* :wait-interpolation))


(defun push-y-axis-push (arm &key (push-position :center)
                                  (approach-direction :back))
  (push-y-axis-step arm :approach nil
                    :push-position push-position
                    :approach-direction approach-direction))


(defun push-y-axis-approach (arm &key (push-position :center)
                                  (approach-direction :back))
  (push-y-axis-step arm :approach t
                    :push-position push-position
                    :approach-direction approach-direction))



(defun push-y-axis (arm &key (push-position :center)
                             (approach-direction :back))
  (when (not (or (and (eq approach-direction :back)
                      (eq push-position :upper))
                 (and (eq approach-direction :back)
                      (eq push-position :center))
                 (and (eq approach-direction :back)
                      (eq push-position :lower))
                 (and (eq approach-direction :upper)
                      (eq push-position :front))
                 (and (eq approach-direction :upper)
                      (eq push-position :center))
                 (and (eq approach-direction :upper)
                      (eq push-position :back))))
    (return-from push-y-axis nil))
  (when (not (or (eq arm :larm) (eq arm :rarm) (eq arm :arms)))
    (return-from push-y-axis nil))
  (push-y-axis-approach arm
                        :push-position push-position
                        :approach-direction approach-direction)
  (push-y-axis-push arm
                    :push-position push-position
                    :approach-direction approach-direction))


(defun get-push-release-target-coords (arm)
  (let ((target-coords (send (send *pr2* arm :end-coords) :copy-worldcoords)))
    (send target-coords :translate (float-vector 0 0 -100))
    target-coords))

(defun push-release (arm)
  ;; release
  (if (eq arm :arms)
    (let ((larm-target-coords (get-push-release-target-coords :larm))
          (rarm-target-coords (get-push-release-target-coords :rarm)))
      (send *pr2* :inverse-kinematics
            (list larm-target-coords rarm-target-coords)
            :move-target (list (send *pr2* :larm :end-coords)
                               (send *pr2* :rarm :end-coords))
            :rotation-axis (list t t)
            :use-torso 0.001))
    (let ((target-coords (get-push-release-target-coords arm))
      (send *pr2* arm :inverse-kinematics
            target-coords :rotation-axis t :use-torso 0.001))))
  (send *irtviewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 5000 :torso-controller 0)
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 5000
        (cond ((eq arm :larm) :larm-controller)
              ((eq arm :rarm) :rarm-controller)
              (t nil))
        0)
  (send *ri* :wait-interpolation))


(defun dualarm-lift-up ()
  (let ((larm-target-coords (send (send *pr2* :larm :end-coords) :copy-worldcoords))
        (rarm-target-coords (send (send *pr2* :rarm :end-coords) :copy-worldcoords)))
    ;; lift up
    (send larm-target-coords :translate (float-vector 0 0 100) :world)
    (send rarm-target-coords :translate (float-vector 0 0 100) :world)
    (send *pr2* :inverse-kinematics
          (list larm-target-coords rarm-target-coords)
          :rotation-axis (list t t)
          :move-target (list (send *pr2* :larm :end-coords)
                             (send *pr2* :rarm :end-coords))
          :use-torso 0.001)
    (send *irtviewer* :draw-objects)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 5000 :torso-controller 0)
    (send *ri* :wait-interpolation)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 5000 nil 0)
    (send *ri* :wait-interpolation)))


(defun dualarm-lift-down ()
  (let ((larm-target-coords (send (send *pr2* :larm :end-coords) :copy-worldcoords))
        (rarm-target-coords (send (send *pr2* :rarm :end-coords) :copy-worldcoords)))
    ;; lift down
    (send larm-target-coords :translate (float-vector 0 0 -100) :world)
    (send rarm-target-coords :translate (float-vector 0 0 -100) :world)
    (send *pr2* :inverse-kinematics
          (list larm-target-coords rarm-target-coords)
          :rotation-axis (list t t)
          :move-target (list (send *pr2* :larm :end-coords)
                             (send *pr2* :rarm :end-coords))
          :use-torso 0.001)
    (send *irtviewer* :draw-objects)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 5000 :torso-controller 0)
    (send *ri* :wait-interpolation)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 5000 nil 0)
    (send *ri* :wait-interpolation)))


(provide :pr2-motions "pr2-motions.l")
