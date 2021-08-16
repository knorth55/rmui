(load "package://pddl_planner/src/pddl-result-graph.l")
(load "package://pddl_planner/src/eus-pddl-client.l")
(load "package://roseus_smach/src/state-machine-ros.l")
(load "package://roseus_smach/src/state-machine-utils.l")
(load "package://roseus_smach/src/pddl2smach.l")

(require :common-actions "package://rmuieus/euslisp/common/common-actions.l")
(require :rmui-pddl-domain "package://rmuieus/euslisp/pddl/rmui-pddl-domain.l")
(require :rmui-pddl-actions "package://rmuieus/euslisp/pddl/rmui-pddl-actions.l")

;;
;; problem
;;
(setq *problem* (instance pddl-problem :init :name 'dualarm_liftup :domain 'rmui))


(send *problem* :objects
      '((TABLE . spot)
        (CUBE . object)))


(send *problem* :initial-condition
      '((ON CUBE TABLE)
        (AT TABLE)
        (ZROTATE CUBE Z0)))


(send *problem* :goal-condition
      '((HOLDUP CUBE)
        (ZROTATE CUBE Z180)))

(dolist (act *actions*)
  (send *domain* :add :action act))

;;
;; solve planning
;;

(defun solve-pddl ()
  (pprint (setq *result* (solve-pddl-planning *domain* *problem* :debug nil)))

  (setq *graph* (make-graph-from-pddl-results (list *result*) :node-name :pprint))
  (setq *graph* (add-failed-nodes *result* (list 'liftup 'contact 'zrotate) :graph *graph*))

  (send *graph* :name (send *domain* :name))

  (setq *graph* (make-readable-graph *graph*))
  (send *graph* :write-to-pdf "rmui.pdf")

  (when (ros::get-param "~display_graph")
    (piped-fork "xdg-open rmui.pdf")))

(defun execute-smach (&key (move-robot t) (use-torso t))
  (setq *move-robot* move-robot)
  (setq *use-torso* use-torso)

  (motion-init)
  (ros::ros-info "motion-init finished")
  (exec-state-machine (convert-smach *graph*) nil :hz 1.0))


(provide :solve-dualarm-liftup "solve-dualarm-liftup.l")
