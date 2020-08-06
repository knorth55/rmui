(require :pr2-interface "package://pr2eus/pr2-interface.l")


;; init

(defun motion-init ()
  (pr2-init)
  (send *pr2* :reset-manip-pose)
  (setq *cube* (make-cube 350 350 265))
  (send *cube* :translate #f(500 0 1000))
  (objects (list *pr2* *cube*)))


;; motions

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
    (return-from push-side nil))
  (when (not (or (eq arm :larm) (eq arm :rarm)))
    (return-from push-side nil))
  (let ((target-coords (send (send *cube* :worldcoords) :copy-worldcoords)))
    (cond
      ((eq approach-direction :back)
       (let ((z-offset (cond ((eq push-position :center) 0.0)
                             ((eq push-position :upper) (- (/ (z-of-cube *cube*) 2.0) 50))
                             ((eq push-position :lower) (- 50 (/ (z-of-cube *cube*) 2.0))))))
         (if (eq arm :larm)
           (progn
             (send target-coords :translate
                   (float-vector 100 (+ (/ (y-of-cube *cube*) 2.0) 100) z-offset))
             (send target-coords :rotate 1.57 :x))
           (progn
             (send target-coords :translate
                   (float-vector 100 (* -1.0 (+ (/ (y-of-cube *cube*) 2.0) 100)) z-offset))
             (send target-coords :rotate -1.57 :x)))))
      ((eq approach-direction :upper)
       (let ((x-offset (cond ((eq push-position :center) 0.0)
                             ((eq push-position :front) (- (/ (x-of-cube *cube*) 2.0) 50))
                             ((eq push-position :back) (- 50 (/ (x-of-cube *cube*) 2.0))))))
         (if (eq arm :larm)
           (progn
             (send target-coords :translate
                   (float-vector x-offset (+ (/ (y-of-cube *cube*) 2.0) 100) -100))
             (send target-coords :rotate 1.57 :x)
             (send target-coords :rotate -1.57 :z))
           (progn
             (send target-coords :translate
                   (float-vector x-offset (* -1.0 (+ (/ (y-of-cube *cube*) 2.0) 100)) -100))
             (send target-coords :rotate -1.57 :x)
             (send target-coords :rotate 1.57 :z)))))
      (t (return-from push-side nil)))
    ;; approach
    (send *pr2* arm :inverse-kinematics
          target-coords :rotation-axis t :use-torso 0.001)
    (send *irtviewer* :draw-objects)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 5000
          (if (eq arm :larm) :larm-controller :rarm-controller) 0)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 5000 :torso-controller 0)
    (send *ri* :wait-interpolation)
    ;; push
    (send target-coords :translate #f(0 0 100))
    (send *pr2* arm :inverse-kinematics
          target-coords :rotation-axis t :use-torso 0.001)
    (send *irtviewer* :draw-objects)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 5000
          (if (eq arm :larm) :larm-controller :rarm-controller) 0)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 5000 :torso-controller 0)
    (send *ri* :wait-interpolation)))


(defun push-release (arm)
  (let ((target-coords (send (send *pr2* arm :end-coords) :copy-worldcoords)))
    ;; release
    (send target-coords :translate (float-vector 0 0 -100))
    (send *pr2* arm :inverse-kinematics
          target-coords :rotation-axis t :use-torso 0.001)
    (send *irtviewer* :draw-objects)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 5000
          (if (eq arm :larm) :larm-controller :rarm-controller) 0)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 5000 :torso-controller 0)
    (send *ri* :wait-interpolation)))


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
    (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
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
    (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
    (send *ri* :wait-interpolation)))


(provide :pr2-motions "pr2-motions.l")
