(require :pddl-result-graph "package://pddl_planner/src/pddl-result-graph.l")


(defclass statenet-graph
  :super pddl-graph
  :slots ())


(defmethod statenet-graph
  (:init ()
    (send-super :init))
  )


(provide :statenet-graph "statenet-graph.l")
