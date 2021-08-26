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


(setq *solver* (instance depth-first-graph-search-solver :init))
(setq *solution* nil)
(setq *sm* nil)


(defun add-initial-state (gr)
  (add-action-state-in-graph gr '(action-init) '(:init) *initial-state*))


(defun load-merged-statenet-graph ()
  (setq *statenet-graph* (merge-statenet-graphs (load-all-statenet-graph)))
  (add-initial-state *statenet-graph*))


(defun solve-statenet-path (start-state goal-state)
  (send *statenet-graph* :start-state
        (send *statenet-graph* :search-node-from-pddl-state start-state))
  (send *statenet-graph* :goal-state
        (send *statenet-graph* :search-node-from-pddl-state goal-state))
  (send *solver* :solve *statenet-graph*))


(defun main (&key (pdf nil))
  (load-merged-statenet-graph)
  (if pdf
    (progn
      (send *statenet-graph* :write-to-pdf "rmui.pdf")
      (piped-fork "xdg-open rmui.pdf")))

  ;; TODO (knorth55): solve solution from user input
  (setq *solution*
        (solve-statenet-path
          ;; start-state
          '(:init)
          ;; goal-state
          '((:larm-contact-state :released)
            (:rarm-contact-state :released)
            (:floor-contact-state :bottom-contact)
            (:object-location-state :center)
            (:object-rotation-state (:x0 :y0 :z90)))))
  (if *solution*
    (progn
      (setq *sm* (convert-solution-to-smach *solution*))
      (exec-state-machine *sm* nil :hz 1.0)))
  )


(provide :common-statenet "common-statenet.l")
