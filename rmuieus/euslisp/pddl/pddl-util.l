(require :pddl-result-graph "package://pddl_planner/src/pddl-result-graph.l")
(require :eus-pddl-client "package://pddl_planner/src/eus-pddl-client.l")
(require :state-machine-ros "package://roseus_smach/src/state-machine-ros.l")
(require :state-machine-utils "package://roseus_smach/src/state-machine-utils.l")
(require :pddl2smach "package://roseus_smach/src/pddl2smach.l")

(require :common-actions "package://rmuieus/euslisp/common/common-actions.l")
(require :pddl-domain "package://rmuieus/euslisp/pddl/pddl-domain.l")
(require :pddl-actions "package://rmuieus/euslisp/pddl/pddl-actions.l")


(defun load-pddl-problem (problem)
  (load (format nil "package://rmuieus/euslisp/pddl/pddl-~A-problem.l"
                (string-downcase problem))))


(defun solve-pddl ()
  (dolist (act *actions*) (send *domain* :add :action act))
  (pprint (setq *result* (solve-pddl-planning *domain* *problem* :debug nil)))

  (setq *graph* (make-graph-from-pddl-results (list *result*) :node-name :pprint))
  (setq *graph* (add-failed-nodes *result* (list 'liftup 'contact 'zrotate) :graph *graph*))

  (send *graph* :name (send *domain* :name))

  (setq *graph* (make-readable-graph *graph*))
  (send *graph* :write-to-pdf "rmui.pdf")

  (when (ros::get-param "~display_graph") (piped-fork "xdg-open rmui.pdf")))


(defun execute-smach (&key (move-robot t) (use-torso t))
  (setq *move-robot* move-robot)
  (setq *use-torso* use-torso)

  (motion-init)
  (ros::ros-info "motion-init finished")
  (exec-state-machine (convert-smach *graph*) nil :hz 1.0))


(provide :pddl-util "pddl-util.l")
