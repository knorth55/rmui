#!/usr/bin/env roseus

(require :state-machine-ros "package://roseus_smach/src/state-machine-ros.l")
(require :state-machine-utils "package://roseus_smach/src/state-machine-utils.l")
(require :pddl2smach "package://roseus_smach/src/pddl2smach.l")

(require :common-actions "package://rmuieus/euslisp/common/common-actions.l")
(require :common-planners "package://rmuieus/euslisp/common/common-planners.l")

(defun main ()
  (load-pddl-graph)
  (send *pddl-graph* :write-to-pdf "rmui.pdf")
  (piped-fork "xdg-open rmui.pdf")
  
  (motion-init)
  (exec-state-machine (convert-smach *pddl-graph*) nil :hz 1.0))


(provide :common-trained-execute "common-trained-execute.l")
