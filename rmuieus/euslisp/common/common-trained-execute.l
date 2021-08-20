#!/usr/bin/env roseus

(require :state-machine-ros "package://roseus_smach/src/state-machine-ros.l")
(require :state-machine-utils "package://roseus_smach/src/state-machine-utils.l")

(require :common-io "package://rmuieus/euslisp/common/common-io.l")
(require :common-graph "package://rmuieus/euslisp/common/common-graph.l")
(require :common-actions "package://rmuieus/euslisp/common/common-actions.l")


(setq *initial-state*
      (list (list :larm-contact-state :released)
            (list :rarm-contact-state :released)
            (list :floor-contact-state :bottom-contact)
            (list :object-location-state :center)
            (list :object-rotation-state (list :x0 :y0 :z0))))


(defun main (&key (pdf nil) (stamp "latest"))
  (load-pddl-graph :stamp stamp)
  (send *pddl-graph* :add-pddl-state-node '(:init) :node-name :pprint)
  (send *pddl-graph* :add-arc-from-to
        (send *pddl-graph* :search-node-from-pddl-state '(:init))
        (send *pddl-graph* :search-node-from-pddl-state *initial-state*)
        "(action-init)")
  ; (setq *pddl-graph* (make-readable-graph *pddl-graph*))
  (if pdf
    (progn
      (send *pddl-graph* :write-to-pdf "rmui.pdf")
      (piped-fork "xdg-open rmui.pdf")))

  (setq *sm* (convert-graph-to-smach *pddl-graph* "(:init)"))
  (exec-state-machine *sm* nil :hz 1.0)
  )


(provide :common-trained-execute "common-trained-execute.l")
