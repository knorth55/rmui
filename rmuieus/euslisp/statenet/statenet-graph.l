(require :pddl-result-graph "package://pddl_planner/src/pddl-result-graph.l")
(require :common-utils "package://rmuieus/euslisp/common/common-utils.l")


(defclass statenet-graph
  :super pddl-graph
  :slots ())


(defmethod statenet-graph
  (:init ()
    (send-super :init))
  (:add-pddl-state-node
   (pddl-state &key (node-name))
   (let ((nd (send self :search-node-from-pddl-state pddl-state)))
     (unless nd
       (let ((n (instance arced-node :init :name
                          (cond
                           ((eq node-name :simple)
                            (intern (string-upcase (format nil "node_~A" cntr))))
                           ((eq node-name :pprint)
                            (cond
                              ((listp (car pddl-state))
                               (convert-scene-state-to-string pddl-state))
                              (t (format nil "~A" pddl-state))))
                           (t (string-upcase (format nil "~A" pddl-state)))))))
         (incf cntr)
         (setf (get n :pddl-state) pddl-state)
         (send self :add-node n)
         (setq nd n)))
     nd))
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
