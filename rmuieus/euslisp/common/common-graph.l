(require :pddl-result-graph "package://pddl_planner/src/pddl-result-graph.l")
(require :eus-pddl-client "package://pddl_planner/src/eus-pddl-client.l")
(require :state-machine "package://roseus_smach/src/state-machine.l")

(require :statenet-graph "package://rmuieus/euslisp/statenet/statenet-graph.l")


(defun convert-string-to-sym (name)
  (remove '|n| (read-from-string name)))


(defun convert-graph-to-smach (graph start-state)
  (let ((sm (instance state-machine :init))
        goals)
    (dolist (node (send graph :nodes))
      (let* ((state-sym (convert-string-to-sym (send node :name)))
             (name (format nil "~A" state-sym))
             (acts (send node :neighbor-action-alist)))
        (if (null (send sm :node name))
          (send sm :add-node (instance state :init name #'(lambda (x) t))))
        (if (null acts)
          (setq goals (append goals (list (cons name (format nil "goal~A" (length goals)))))))
        (dolist (act acts)
          (let* ((action-name (car act))
                 (action-sym (read-from-string action-name))
                 (action-func
                   `(lambda-closure nil 0 0 (x)
                                    (apply #',(car action-sym) ',(cdr action-sym))))
                 (node-name (format nil "~A" (list action-sym state-sym)))
                 (next-node (cdr act))
                 (next-state-sym (convert-string-to-sym (send next-node :name)))
                 (next-name (format nil "~A" next-state-sym))
                 (next-acts (send next-node :neighbor-action-alist)))
            (if (null (send sm :node node-name))
              (send sm :add-node (instance state :init node-name action-func)))
            (send sm :add-transition name node-name t)
            (if (null (send sm :node next-name))
              (send sm :add-node (instance state :init next-name #'(lambda (x) t))))
            (send sm :add-transition node-name next-name t)
            ))))
    ;; start-state
    (send sm :start-state start-state)
    ;; goal-state
    (send sm :goal-state (mapcar #'cdr goals))
    ;; goals
    (dolist (goal goals)
      (let ((name (car goal))
            (goal-name (cdr goal)))
        (if (null (send sm :node name))
          (send sm :add-node (instance state :init name #'(lambda (x) t))))
        (send sm :add-transition name goal-name t)))
    (ros::ros-info (format nil "convert-smach: smach start-state is ~A."
                           (send (send sm :start-state) :name)))
    (ros::ros-info (format nil "convert-smach: smach goal-state are ~A."
                           (send-all (send sm :goal-state) :name)))
    (send sm :reset-state)
    sm))


(provide :common-graph "common-graph.l")
