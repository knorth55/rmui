(load "package://pr2eus/pr2-interface.l")


(defun init ()
  (pr2-init)
  (setq *cube* (make-cube 300 500 300))
  (send *cube* :translate #f(500 0 800))
  (objects (list *pr2* *cube*)))


(defun dualarm-hold ()
  (let ((larm-target-coords (send *cube* :copy-worldcoords))
        (rarm-target-coords (send *cube* :copy-worldcoords)))
    (send larm-target-coords :translate
          (float-vector 100 (+ (/ (y-of-cube *cube*) 2.0) 100) 0))
    (send larm-target-coords :rotate 1.57 :x)
    (send rarm-target-coords :translate
          (float-vector 100 (* -1.0 (+ (/ (y-of-cube *cube*) 2.0) 100)) 0))
    (send rarm-target-coords :rotate -1.57 :x)
    
    (send *pr2* :inverse-kinematics
          (list larm-target-coords rarm-target-coords)
          :rotation-axis (list t t)
          :move-target (list (send *pr2* :larm :end-coords)
                             (send *pr2* :rarm :end-coords))
          :use-torso 0.0001)
    (send *irtviewer* :draw-objects)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
    (send *ri* :wait-interpolation)

    (send larm-target-coords :translate (float-vector 0 0 100))
    (send rarm-target-coords :translate (float-vector 0 0 100))
    (send *pr2* :inverse-kinematics
          (list larm-target-coords rarm-target-coords)
          :rotation-axis (list t t)
          :move-target (list (send *pr2* :larm :end-coords)
                             (send *pr2* :rarm :end-coords))
          :use-torso 0.0001)
    (send *irtviewer* :draw-objects)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
    (send *ri* :wait-interpolation)
    (send (send *pr2* :larm :end-coords :parent) :assoc *cube*)))

(defun dualarm-hold-release ()
  (send (send *pr2* :larm :end-coords :parent) :dissoc *cube*))
