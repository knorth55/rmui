(require :pr2-motions "package://rmuieus/euslisp/pr2-motions.l")


;; init

(defun planner-init ()
  (motion-init))


;; motion planners

(defun dualarm-push-hold-release (&key (move-robot t))
  (let* ((released (push-hold-release-motion :move-robot nil))
         (avs (car released))
         (tms (cadr released)))
    (when released
      (when move-robot
        (send *ri* :angle-vector-sequence avs tms nil 0)
        (send *ri* :wait-interpolation))
      :push-hold-release)))


(defun dualarm-push-hold-up
  (&key (push-axis :y)
        (push-position :center)
        (approach-direction :back)
        (front-arm :larm)
        (upper-arm :larm)
        (move-robot t))
  (let* ((holded (push-hold-motion :push-axis push-axis
                                   :push-position push-position
                                   :approach-direction approach-direction
                                   :front-arm front-arm
                                   :upper-arm upper-arm
                                   :move-robot nil))
         (lifted-up (when holded (hold-lift-up-motion :move-robot nil)))
         (avs (when lifted-up (append (car holded) (car lifted-up))))
         (tms (when lifted-up (append (cadr holded) (cadr lifted-up)))))
    (when lifted-up
      (when move-robot
        (send *ri* :angle-vector-sequence avs tms nil 0)
        (send *ri* :wait-interpolation))
      :push-hold-up)))


(defun dualarm-push-hold-down (&key (move-robot t))
  (let* ((lifted-down (hold-lift-down-motion :move-robot nil))
         (released (when lifted-down (push-hold-release-motion :move-robot nil)))
         (avs (when released (append (car lifted-down) (car released))))
         (tms (when released (append (cadr lifted-down) (cadr released)))))
    (when released
      (when move-robot
        (send *ri* :angle-vector-sequence avs tms nil 0)
        (send *ri* :wait-interpolation))
      :push-hold-down)))


(defun dualarm-push-hold-rotate
  (&key (rotate-angle pi/2)
        (rotate-axis :x)
        (front-arm :larm)
        (move-robot t))
  (let* ((holded (push-hold-motion :push-axis (if (eq rotate-axis :y) :x :y)
                                   :push-position :center
                                   :approach-direction
                                   (cond ((eq rotate-axis :x) :back)
                                         ((eq rotate-axis :y) :side)
                                         ((eq rotate-axis :z) :upper))
                                   :front-arm front-arm
                                   :hold-type :light
                                   :move-robot nil))
         (rotated (when holded (rotate-motion :rotate-angle rotate-angle
                                              :rotate-axis rotate-axis
                                              :front-arm front-arm
                                              :move-robot nil)))
         (released (when rotated (push-hold-release-motion :move-robot nil)))
         (avs (when released (append (car holded) (car rotated) (car released))))
         (tms (when released (append (cadr holded) (cadr rotated) (cadr released)))))
    (when released
      (when move-robot
        (send *ri* :angle-vector-sequence avs tms nil 0)
        (send *ri* :wait-interpolation))
      :push-hold-rotate)))


(provide :pr2-planners "pr2-planners.l")
