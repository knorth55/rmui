(require :pddl-result-graph "package://pddl_planner/src/pddl-result-graph.l")
(require :eus-pddl-client "package://pddl_planner/src/eus-pddl-client.l")
(require :state-machine "package://roseus_smach/src/state-machine.l")

(require :statenet-graph "package://rmuieus/euslisp/statenet/statenet-graph.l")
(require :common-io "package://rmuieus/euslisp/common/common-io.l")


(defun convert-string-to-sym (name)
  (remove '|n| (read-from-string name)))


(defun add-node (sm name func node-type)
  (if (null (send sm :node name))
    (let ((n (instance state :init name func)))
      (setf (get n :node-type) node-type)
      (send sm :add-node n))))


(defun add-state-node (sm name &optional (state-func #'(lambda (x) t)))
  (add-node sm name state-func :state-node))


(defun add-action-node (sm name action-func)
  (add-node sm name action-func :action-node))


(defun convert-graph-to-smach (gr start-state)
  (let ((sm (instance state-machine :init))
        goals)
    (dolist (node (send gr :nodes))
      (let* ((state-sym (convert-string-to-sym (send node :name)))
             (name (format nil "~A" state-sym))
             (acts (send node :neighbor-action-alist)))
        (add-state-node sm name #'(lambda (x) t))
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
                 (next-name (format nil "~A" next-state-sym)))
            (add-action-node sm node-name action-func)
            (send sm :add-transition name node-name t)
            (add-state-node sm next-name #'(lambda (x) t))
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
        (add-state-node sm name #'(lambda (x) t))
        (send sm :add-transition name goal-name t)))
    (ros::ros-info (format nil "convert-smach: smach start-state is ~A."
                           (send (send sm :start-state) :name)))
    (ros::ros-info (format nil "convert-smach: smach goal-state are ~A."
                           (send-all (send sm :goal-state) :name)))
    (send sm :reset-state)
    sm))


(defun convert-solution-to-smach (solution)
  (let ((sm (instance state-machine :init))
        (start-state "(:start)")
        (goal-state "(:goal)"))
    (dolist (solver-node solution)
      (let ((ac (send solver-node :action)))
        (if ac
          (let* ((from-state-sym (convert-string-to-sym (send (send ac :from) :name)))
                 (to-state-sym (convert-string-to-sym (send (send ac :to) :name)))
                 (action-name (send ac :name))
                 (action-sym (read-from-string action-name))
                 (action-func
                   `(lambda-closure nil 0 0 (x)
                                    (apply #',(car action-sym) ',(cdr action-sym))))
                 (node-name (format nil "~A" (list action-sym from-state-sym)))
                 (from-name (format nil "~A" from-state-sym))
                 (to-name (format nil "~A" to-state-sym)))
            (add-state-node sm from-name #'(lambda (x) t))
            (add-action-node sm node-name action-func)
            (send sm :add-transition from-name node-name t)
            (add-state-node sm to-name #'(lambda (x) t))
            (send sm :add-transition node-name to-name t)))))
    ;; start-state
    (add-state-node sm start-state #'(lambda (x) t))
    (send sm :start-state start-state)
    ;; goal-state
    (send sm :goal-state (list goal-state))
    ;; start and goal
    (let* ((first-state-sym (convert-string-to-sym (send (send (car solution) :state) :name)))
           (last-state-sym (convert-string-to-sym (send (send (car (last solution)) :state) :name)))
           (first-name (format nil "~A" first-state-sym))
           (last-name (format nil "~A" last-state-sym)))
      (add-state-node sm first-name #'(lambda (x) t))
      (send sm :add-transition start-state first-name t)
      (add-state-node sm goal-state #'(lambda (x) t))
      (add-state-node sm last-name #'(lambda (x) t))
      (send sm :add-transition last-name goal-state t))
    (ros::ros-info (format nil "convert-smach: smach start-state is ~A."
                           (send (send sm :start-state) :name)))
    (ros::ros-info (format nil "convert-smach: smach goal-state are ~A."
                           (send-all (send sm :goal-state) :name)))
    (send sm :reset-state)
    sm))


(defun add-action-state-in-graph (gr action prev-scene-state scene-state)
  (let ((prev-node (send gr :search-node-from-pddl-state prev-scene-state))
        (node (send gr :search-node-from-pddl-state scene-state)))
    (if (null prev-node)
      (setq prev-node (send gr :add-pddl-state-node prev-scene-state :node-name :pprint)))
    (if (null node)
      (setq node (send gr :add-pddl-state-node scene-state :node-name :pprint)))
    ;; (ros::ros-info (format nil "add-arc-from-to: ~A  - ~A -> ~A" prev-scene-state action scene-state))
    (send gr :add-arc-from-to prev-node node (format nil "~A" action))))


(defun merge-statenet-graphs (graphs)
  (merge-graphs graphs statenet-graph))


(defun merge-pddl-graphs (graphs)
  (merge-graphs graphs pddl-graph))


(defun merge-graphs (graphs graphclass)
  (let ((mgr (instance graphclass :init)))
    (dolist (gr graphs)
      (dolist (node (send gr :nodes))
        (let* ((arcs (send node :arc-list))
               (node-sym (convert-string-to-sym (send node :name)))
               (mnode (send mgr :search-node-from-pddl-state node-sym)))
          (if (null arcs)
            (if (null mnode)
              (send mgr :add-pddl-state-node node-sym :node-name :pprint))
            (dolist (ac arcs)
              (let ((from-sym (convert-string-to-sym (send (send ac :from) :name)))
                    (to-sym (convert-string-to-sym (send (send ac :to) :name)))
                    (action (read-from-string (send ac :name))))
                (if (equal node-sym from-sym)
                  (add-action-state-in-graph mgr action from-sym to-sym)
                  (ros::ros-error (format nil "node and arc :from is not same: ~A != ~A"
                                          node-sym from-sym)))))))))
    mgr))


(provide :common-graph "common-graph.l")
