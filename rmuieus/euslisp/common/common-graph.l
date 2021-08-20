(require :pddl-result-graph "package://pddl_planner/src/pddl-result-graph.l")
(require :eus-pddl-client "package://pddl_planner/src/eus-pddl-client.l")
(require :state-machine "package://roseus_smach/src/state-machine.l")


(defun convert-graph-to-smach (graph start-state)
  (let ((sm (instance state-machine :init))
        goals)
    (dolist (node (send graph :nodes))
      (let ((name (send node :name))
            (acts (send node :neighbor-action-alist)))
        (if (null acts)
          (setq goals (append goals (list (cons name (format nil "goal~A" (length goals)))))))
        (dolist (act acts)
          (let* ((action-name (car act))
                 (action-sym (read-from-string action-name))
                 (action-func
                   `(lambda-closure nil 0 0 (x)
                                    (apply #',(car action-sym) ',(cdr action-sym))))
                 (node-name (format nil "~A\\n~A" name action-name))
                 (next-node (cdr act))
                 (next-name (send next-node :name))
                 (next-acts (send next-node :neighbor-action-alist)))
            (if (null (send sm :node node-name))
              (send sm :add-node (instance state :init node-name action-func)))
            (if (and (null next-acts)
                     (null (send sm :node next-name)))
              (progn
                (send sm :add-node (instance state :init next-name #'(lambda (x) t)))
                (send sm :add-transition node-name next-name t)))
            (dolist (next-act next-acts)
              (let* ((next-action-name (car next-act))
                     (next-action-sym (read-from-string next-action-name))
                     (next-action-func
                       `(lambda-closure nil 0 0 (x)
                                        (apply #',(car next-action-sym) ',(cdr next-action-sym))))
                     (next-node-name (format nil "~A\\n~A" next-name next-action-name)))
                (if (null (send sm :node next-node-name))
                  (send sm :add-node (instance state :init next-node-name next-action-func)))
                (send sm :add-transition node-name next-node-name t)))))))
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
