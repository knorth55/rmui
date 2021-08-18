#!/usr/bin/env roseus

(require :state-machine-ros "package://roseus_smach/src/state-machine-ros.l")
(require :state-machine-utils "package://roseus_smach/src/state-machine-utils.l")
(require :pddl2smach "package://roseus_smach/src/pddl2smach.l")

(require :common-actions "package://rmuieus/euslisp/common/common-actions.l")
(require :common-planners "package://rmuieus/euslisp/common/common-planners.l")


(defun main ()
  (load-pddl-graph)
  (send *pddl-graph* :add-pddl-state-node '(:init) :node-name :pprint)
  (send *pddl-graph* :add-arc-from-to
        (send *pddl-graph* :search-node-from-pddl-state '(:init))
        (send *pddl-graph* :search-node-from-pddl-state
              '((:larm-contact-state :released)
                (:rarm-contact-state :released)
                (:floor-contact-state :bottom-contact)
                (:object-location-state :center)))
        "(action-init)")
  ; (setq *pddl-graph* (make-readable-graph *pddl-graph*))
  (send *pddl-graph* :write-to-pdf "rmui.pdf")
  (piped-fork "xdg-open rmui.pdf")

  (setq *sm* (convert-smach *pddl-graph* :use-userdata nil
                            :start-state "(action-init)"
                            :goal-state nil))
  (exec-state-machine *sm* nil :hz 1.0))


(provide :common-trained-execute "common-trained-execute.l")
