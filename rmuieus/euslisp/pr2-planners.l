(require :pr2-motions "package://rmuieus/euslisp/pr2-motions.l")


;; init

(defun planner-init ()
  (motion-init))


;; motion planners

(defun dualarm-push-hold (&key (push-direction :y)
                               (push-position :center)
                               (approach-direction :back))
  (when (not (or (eq push-direction :x)
                 (eq push-direction :y)))
    (return-from dualarm-push-hold nil))
  ;; push-hold
  (cond
    ((eq push-direction :y)
     (push-y-axis :arms
                  :push-position push-position
                  :approach-direction approach-direction))
    (t nil))
  ;; assoc
  (send (send *pr2* :larm :end-coords :parent) :assoc *cube*))



(defun dualarm-push-hold-release ()
  ;; disassoc
  (send (send *pr2* :larm :end-coords :parent) :dissoc *cube*)

  ;; push-hold-release
  (push-release :arms))


(defun dualarm-push-hold-up (&key (push-direction :y)
                                  (push-position :center)
                                  (approach-direction :back))
  (dualarm-push-hold :push-direction push-direction
                     :push-position push-position
                     :approach-direction approach-direction)
  (dualarm-lift-up))


(defun dualarm-push-hold-down (&key (push-direction :y)
                                    (push-position :center)
                                    (approach-direction :back))
  (dualarm-lift-down)
  (dualarm-push-hold-release))


(provide :pr2-planners "pr2-planners.l")
