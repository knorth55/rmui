(require :pr2-motions "package://rmuieus/euslisp/pr2-motions.l")


;; init

(defun planner-init ()
  (motion-init))


;; motion planners

(defun dualarm-push-hold
  (&key (push-axis :y)
        (push-position :center)
        (approach-direction :back)
        (front-arm :larm)
        (upper-arm :larm))
  (when (push-hold-motion
          :push-axis push-axis
          :push-position push-position
          :approach-direction approach-direction
          :front-arm front-arm :upper-arm upper-arm)
    :push-hold))


(defun dualarm-push-hold-release ()
  (when (push-hold-release-motion) :push-hold-release))


(defun dualarm-push-hold-up
  (&key (push-axis :y)
        (push-position :center)
        (approach-direction :back)
        (front-arm :larm)
        (upper-arm :larm))
  (when (dualarm-push-hold
          :push-axis push-axis
          :push-position push-position
          :approach-direction approach-direction
          :front-arm front-arm :upper-arm upper-arm)
    (when (hold-lift-up-motion) :push-hold-up)))


(defun dualarm-push-hold-down ()
  (when (hold-lift-down-motion)
    (when (dualarm-push-hold-release) :push-hold-down)))


(provide :pr2-planners "pr2-planners.l")
