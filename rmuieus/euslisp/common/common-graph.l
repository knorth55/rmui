(require :pddl-result-graph "package://pddl_planner/src/pddl-result-graph.l")
(require :eus-pddl-client "package://pddl_planner/src/eus-pddl-client.l")
(require :state-machine "package://roseus_smach/src/state-machine.l")

(require :statenet-graph "package://rmuieus/euslisp/statenet/statenet-graph.l")
(require :common-io "package://rmuieus/euslisp/common/common-io.l")


(defmethod state-machine
  (:append-goal-state (gs)
    (if (atom gs) (setq gs (list gs)))
    (setq gs (mapcar #'(lambda(g) (instance state :init g nil)) gs))
    (dolist (n gs) (send self :add-node n))
    (send-super :goal-state (append goal-state gs))))


(defun convert-string-to-sym (name)
  (remove '|n| (read-from-string name)))


(defun add-node (sm name func node-type)
  (let ((node-func
          (if (equal 'lambda-closure (car func))
            func (eval func))))
    (if (null (send sm :node name))
      (let ((n (instance state :init name node-func)))
        (setf (get n :node-type) node-type)
        (send sm :add-node n)))))


(defun add-state-node (sm name &optional (state-func #'(lambda (x) t)))
  (add-node sm name state-func :state-node))


(defun add-action-node (sm name action-func)
  (add-node sm name action-func :action-node))


(defun add-teach-node (sm name teach-func)
  (add-node sm name teach-func :teach-node))


(defun add-abort-nodes (sm aborts state-func)
  (dolist (ab aborts)
    (let ((from-name (car ab))
          (abort-name (cdr ab)))
      (add-state-node sm from-name state-func)
      (send sm :add-transition from-name abort-name nil))))


(defun add-teach-node-trans (sm from-name teach-name state-func teach-func)
   (add-state-node sm from-name state-func)
   (add-teach-node sm teach-name teach-func)
   (send sm :add-transition from-name teach-name :teach)
   (send sm :add-transition teach-name from-name t))


(defun add-smach-transition (sm from-name node-name
                                &key (smach-transition t) (add-t-transition nil))
  (let ((sub-sm (send (send sm :node from-name) :submachine)))
    (if sub-sm
      (let ((goal-state-names
              (mapcar #'(lambda (x) (send x :name)) (send sub-sm :goal-state))))
        (if (null (find smach-transition goal-state-names :test #'equal))
          (progn
            (if (null (equal smach-transition t))
              (send sub-sm :append-goal-state smach-transition))
            (send sub-sm :add-transition "(:next)" smach-transition t)
            )))))
  (send sm :add-transition from-name node-name smach-transition)
  (if add-t-transition (send sm :add-transition from-name node-name t)))

(defun add-action-node-trans (sm from-name node-name to-name state-func action-func
                                 &key (smach-transition t) (add-t-transition nil))
  (add-state-node sm from-name state-func)
  (add-action-node sm node-name action-func)
  (add-smach-transition sm from-name node-name
                        :smach-transition smach-transition
                        :add-t-transition add-t-transition)
  (add-state-node sm to-name state-func)
  (send sm :add-transition node-name to-name t))


(defun add-nodes-from-trans
  (sm trans &key (state-func #'(lambda (x) t))
      (smach-transition t) (add-t-transition nil))
  (let ((from-name (cdr (assoc :from-name trans :test #'equal)))
        (node-name (cdr (assoc :node-name trans :test #'equal)))
        (action-func (cdr (assoc :action-func trans :test #'equal)))
        (to-name (cdr (assoc :to-name trans :test #'equal))))
    (add-action-node-trans sm from-name node-name to-name state-func action-func
                           :smach-transition smach-transition
                           :add-t-transition add-t-transition)))


(defun convert-graph-to-smach
  (gr start-state &key (state-func #'(lambda (x) t)) (teach-func nil))
  (let ((sm (instance state-machine :init))
        (goals nil)
        (aborts nil)
        (abort-index 0))
    (dolist (node (send gr :nodes))
      (let* ((from-state-sym (convert-string-to-sym (send node :name)))
             (from-name (format nil "~A" from-state-sym))
             (acts (send node :neighbor-action-alist)))
        (add-state-node sm from-name state-func)
        (if (null acts)
          (setq goals
                (append goals (list (cons from-name
                                          (format nil "(:goal~A)" (length goals)))))))
        (dolist (act acts)
          (let* ((action-name (car act))
                 (action-sym (read-from-string action-name))
                 (action-func
                   `(lambda-closure nil 0 0 (x)
                                    (apply #',(car action-sym) ',(cdr action-sym))))
                 (node-name (format nil "~A" (list action-sym state-sym)))
                 (to-node (cdr act))
                 (to-state-sym (convert-string-to-sym (send to-node :name)))
                 (to-name (format nil "~A" to-state-sym))
                 (abort-name (format nil "(:abort~A)" abort-index))
                 (teach-name (format nil "~A" (list :teach from-state-sym))))
            (add-action-node-trans sm from-name node-name to-name state-func action-func
                                   :smach-transition t :add-t-transition nil)
            (if teach-func
              (add-teach-node-trans sm from-name teach-name state-func teach-func))
            (setq aborts (append aborts (list (cons from-name abort-name))))
            (setq abort-index (+ abort-index 1))))))
    ;; start-state
    (send sm :start-state start-state)
    ;; goal-state
    (send sm :goal-state (append (mapcar #'cdr goals) (mapcar #'cdr aborts)))
    ;; goals
    (dolist (goal goals)
      (let ((from-name (car goal))
            (goal-name (cdr goal)))
        (add-state-node sm from-name state-func)
        (send sm :add-transition from-name goal-name t)))
    ;; aborts
    (add-abort-nodes sm aborts state-func)
    ;; ros-info
    (ros::ros-info (format nil "convert-graph-to-smach: smach start-state is ~A."
                           (send (send sm :start-state) :name)))
    (ros::ros-info (format nil "convert-graph-to-smach: smach goal-state are ~A."
                           (send-all (send sm :goal-state) :name)))
    (send sm :reset-state)
    sm))


(defun convert-solution-to-smach
  (solution &key (state-func #'(lambda (x) t)) (teach-func nil) (no-loop nil))
  (let ((sm (instance state-machine :init))
        (start-state "(:start)")
        (goal-state "(:goal)")
        (aborts nil)
        (abort-index 0)
        (trans-alist nil))
    (dolist (sn solution)
      (let ((ac (send sn :action)))
        (if ac
          (let* ((from-state-sym (convert-string-to-sym (send (send ac :from) :name)))
                 (to-state-sym (convert-string-to-sym (send (send ac :to) :name)))
                 (action-name (send ac :name))
                 (action-sym (read-from-string action-name))
                 (action-func
                   `(lambda-closure nil 0 0 (userdata)
                                    (apply #',(car action-sym) ',(cdr action-sym))))
                 (node-name (format nil "~A" (list action-sym from-state-sym)))
                 (from-name (format nil "~A" from-state-sym))
                 (to-name (format nil "~A" to-state-sym))
                 (trans (list (cons :from-name from-name)
                              (cons :node-name node-name)
                              (cons :action-sym action-sym)
                              (cons :action-func action-func)
                              (cons :to-name to-name)
                              (cons :to-state-sym to-state-sym))))
            (setq trans-alist
                  (append trans-alist (list (cons from-state-sym nil))))
            (push trans (cdr (assoc from-state-sym trans-alist :test #'equal)))))))
    (if no-loop
      (let ((current-state-sym
              (convert-string-to-sym
                (send (send (car solution) :state) :name)))
            (last-state-sym
              (convert-string-to-sym
                (send (send (car (last solution)) :state) :name))))
        (while (null (equal current-state-sym last-state-sym))
          (let* ((trans-a (assoc current-state-sym trans-alist :test #'equal))
                 (trans (car (cdr trans-a)))
                 (from-state-sym (car trans-a))
                 (from-name (format nil "~A" from-state-sym))
                 (to-state-sym (cdr (assoc :to-state-sym trans :test #'equal)))
                 (abort-name (format nil "(:abort~A)" abort-index))
                 (teach-name (format nil "~A" (list :teach from-state-sym))))
            (add-nodes-from-trans
              sm trans :state-func state-func
              :smach-transition
              (format nil "~A" (cdr (assoc :action-sym trans :test #'equal)))
              :add-t-transition t)
            (if teach-func
              (add-teach-node-trans sm from-name teach-name state-func teach-func))
            (setq aborts (append aborts (list (cons from-name abort-name))))
            (setq abort-index (+ abort-index 1))
            (setq current-state-sym to-state-sym))))
      (dolist (trans-a trans-alist)
        (let* ((trans-list (cdr trans-a))
               (from-state-sym (car trans-a))
               (from-name (format nil "~A" from-state-sym))
               (abort-name (format nil "(:abort~A)" abort-index))
               (teach-name (format nil "~A" (list :teach from-state-sym)))
               (trans-index 0))
          (dolist (trans trans-list)
            (add-nodes-from-trans
              sm trans :state-func state-func
              :smach-transition
              (format nil "~A" (cdr (assoc :action-sym trans :test #'equal)))
              :add-t-transition (equal trans-index 0))
            (setq trans-index (+ trans-index 1)))
          (if teach-func
            (add-teach-node-trans sm from-name teach-name state-func teach-func))
          (setq aborts (append aborts (list (cons from-name abort-name))))
          (setq abort-index (+ abort-index 1)))))
    ;; start-state
    (add-node sm start-state #'(lambda (x) t) nil)
    (send sm :start-state start-state)
    ;; goal-state
    (send sm :goal-state (append (list goal-state) (mapcar #'cdr aborts)))
    ;; start and goal
    (let* ((first-state-sym
             (convert-string-to-sym
               (send (send (car solution) :state) :name)))
           (last-state-sym
             (convert-string-to-sym
               (send (send (car (last solution)) :state) :name)))
           (first-name (format nil "~A" first-state-sym))
           (last-name (format nil "~A" last-state-sym)))
      (add-state-node sm first-name state-func)
      (send sm :add-transition start-state first-name t)
      (add-node sm goal-state #'(lambda (x) t) nil)
      (add-state-node sm last-name state-func)
      (send sm :add-transition last-name goal-state t))
    ;; aborts
    (add-abort-nodes sm aborts state-func)
    ;; ros-info
    (ros::ros-info (format nil "convert-solution-to-smach: smach start-state is ~A."
                           (send (send sm :start-state) :name)))
    (ros::ros-info (format nil "convert-solution-to-smach: smach goal-state are ~A."
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
