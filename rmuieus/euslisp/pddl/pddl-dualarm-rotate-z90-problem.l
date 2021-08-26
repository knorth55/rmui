(require :pddl-result-graph "package://pddl_planner/src/pddl-result-graph.l")
(require :eus-pddl-client "package://pddl_planner/src/eus-pddl-client.l")


(setq *problem* (instance pddl-problem :init :name 'dualarm_rotate :domain 'rmui))


(send *problem* :objects
      '((TABLE . spot)
        (CUBE . object)))


(send *problem* :initial-condition
      '((ON CUBE TABLE)
        (AT TABLE)
        (ZROTATE CUBE Z0)))


(send *problem* :goal-condition
      '((ON CUBE TABLE)
        (ZROTATE CUBE Z90)))


(provide :pddl-dualarm-rotate-z90-problem "pddl-dualarm-rotate-z90-problem.l")
