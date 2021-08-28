(require :pddl-result-graph "package://pddl_planner/src/pddl-result-graph.l")


(defclass statenet-graph
  :super pddl-graph
  :slots ())


(defmethod statenet-graph
  (:init ()
    (send-super :init))
  (:add-arc-from-to
   (from to name &key (both nil))
   (if (find-if #'(lambda (x) (and (eq (cdr x) to)
                                   (string-equal (car x) name)))
                (send from :neighbor-action-alist))
     (ros::ros-warn (format nil "[:add-arc-from-to] same arc found: ~A" name))
     (let ((ar (instance costed-arc :init from to 1)))
       (send ar :name name)
       (send from :neighbors))))
  )


(provide :statenet-graph "statenet-graph.l")
