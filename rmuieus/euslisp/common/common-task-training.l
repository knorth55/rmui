#!/usr/bin/env roseus

(require :state-machine-ros "package://roseus_smach/src/state-machine-ros.l")
(require :state-machine-utils "package://roseus_smach/src/state-machine-utils.l")

(require :statenet-graph "package://rmuieus/euslisp/statenet/statenet-graph.l")
(require :common-io "package://rmuieus/euslisp/common/common-io.l")
(require :common-graph "package://rmuieus/euslisp/common/common-graph.l")
(require :common-actions "package://rmuieus/euslisp/common/common-actions.l")


(setq *initial-state*
      (list (list :larm-contact-state :released)
            (list :rarm-contact-state :released)
            (list :floor-contact-state :bottom-contact)
            (list :object-location-state :center)
            (list :object-rotation-state (list :x0 :y0 :z0))))


(defun add-initial-state (gr)
  (add-action-state-in-graph gr '(action-init) '(:init) *initial-state*))


(defun main (&key (pdf nil))
  (setq *statenet-graph* (merge-statenet-graphs (load-all-statenet-graph)))
  (add-initial-state *statenet-graph*)
  (if pdf
    (progn
      (send *statenet-graph* :write-to-pdf "rmui.pdf")
      (piped-fork "xdg-open rmui.pdf")))

  (setq *sm* (convert-graph-to-smach *statenet-graph* "(:init)"))
  (exec-state-machine *sm* nil :hz 1.0)
  )


(provide :common-task-training "common-task-training.l")
