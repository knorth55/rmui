(require :pddl-result-graph "package://pddl_planner/src/pddl-result-graph.l")
(require :eus-pddl-client "package://pddl_planner/src/eus-pddl-client.l")


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
        (ZROTATE CUBE Z90)))


(provide :pddl-dualarm-rotate-liftup-problem "pddl-dualarm-rotate-liftup-problem.l")
