(require :state-machine-ros "package://roseus_smach/src/state-machine-ros.l")
(require :state-machine-utils "package://roseus_smach/src/state-machine-utils.l")

(require :statenet-graph "package://rmuieus/euslisp/statenet/statenet-graph.l")
(require :statenet-util "package://rmuieus/euslisp/statenet/statenet-util.l")
(require :common-io "package://rmuieus/euslisp/common/common-io.l")
(require :common-graph "package://rmuieus/euslisp/common/common-graph.l")
(require :common-actions "package://rmuieus/euslisp/common/common-actions.l")


;; state

(defvar *reset-state*
  '((:larm-contact-state :released)
    (:rarm-contact-state :released)
    (:floor-contact-state :bottom-contact)
    (:object-location-state :center)
    (:object-rotation-state (:x0 :y0 :z0))))


(defun add-initial-state (gr)
  (add-action-state-in-graph gr '(action-init) '(:init) *reset-state*))


(defun load-merged-statenet-graph ()
  (let ((gr (merge-statenet-graphs (load-all-statenet-graph))))
    (if (null gr) (setq gr (load-merged-statenet-graph)))
    (dump-merged-statenet-graph gr)
    (add-initial-state gr)
    gr))


(defun solve-statenet-path (gr solver start-state goal-state)
  (send gr :start-state (send gr :search-node-from-pddl-state start-state))
  (send gr :goal-state (send gr :search-node-from-pddl-state goal-state))
  (send solver :solve gr))


(defun statenet-state-func (userdate)
  t)


(defun generate-smach-from-state (gr solver start-state goal-state)
  (let* ((solution
           (solve-statenet-path gr solver start-state goal-state))
         (sm (if solution
               (convert-solution-to-smach
                 solution :state-func #'statenet-state-func))))
    sm))


(provide :common-statenet "common-statenet.l")
