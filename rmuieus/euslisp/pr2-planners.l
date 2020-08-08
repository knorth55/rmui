(require :pr2-motions "package://rmuieus/euslisp/pr2-motions.l")


;; init

(defun planner-init ()
  (motion-init))


;; motion planners

(defun assoc-cube (arm)
  (when (eq arm :arms) (setq arm :larm))
  (send (send *pr2* arm :end-coords :parent) :assoc *cube*))


(defun dissoc-cube (arm)
  (when (eq arm :arms) (setq arm :larm))
  (send (send *pr2* arm :end-coords :parent) :dissoc *cube*))


(defun dualarm-push-hold
  (&key (push-axis :y) (push-position :center) (approach-direction :back)
        (front-arm :larm) (upper-arm :larm))
  ;; push-hold
  (let ((holded))
    (setq holded
          (push-hold-motion
            :push-axis push-axis
            :push-position push-position
            :approach-direction approach-direction
            :front-arm front-arm :upper-arm upper-arm))
    ;; assoc
    (when holded (assoc-cube :arms))))


(defun dualarm-push-hold-release ()
  ;; disassoc
  (dissoc-cube :arms)
  ;; push-hold-release
  (push-hold-release-motion))


(defun dualarm-push-hold-up
  (&key (push-axis :y) (push-position :center) (approach-direction :back)
        (front-arm :larm) (upper-arm :larm))
  (let ((holded))
    (setq holded
          (dualarm-push-hold
            :push-axis push-axis
            :push-position push-position
            :approach-direction approach-direction
            :front-arm front-arm :upper-arm upper-arm))
    (when holded (hold-lift-up-motion))))


(defun dualarm-push-hold-down ()
  (let ((holded))
    (setq holded (hold-lift-down-motion))
    (when holded (dualarm-push-hold-release))))


(provide :pr2-planners "pr2-planners.l")
