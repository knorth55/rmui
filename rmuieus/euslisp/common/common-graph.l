(require :pddl-result-graph "package://pddl_planner/src/pddl-result-graph.l")
(require :eus-pddl-client "package://pddl_planner/src/eus-pddl-client.l")
(require :state-machine "package://roseus_smach/src/state-machine.l")

(require :statenet-graph "package://rmuieus/euslisp/statenet/statenet-graph.l")
(require :smach-state-machine "package://rmuieus/euslisp/smach/smach-state-machine.l")
(require :common-io "package://rmuieus/euslisp/common/common-io.l")
(require :common-utils "package://rmuieus/euslisp/common/common-utils.l")


(defun remove-key-from-action (action-sym key)
  (if (cdr action-sym)
    (append
      (list (car action-sym))
      (remove-key-from-args (cdr action-sym) key))
    action-sym))


(defun override-key-value-in-action (action-sym key value)
  (if (cdr action-sym)
    (append
      (list (car action-sym))
      (override-key-value-in-args (cdr action-sym) key value))
    action-sym))


(defun convert-string-to-sym (name)
  (let ((node-sym (read-from-string name)))
    (remove '|n|
            (mapcar #'(lambda (x) (if (listp x) (remove '|n| x) x))
                    node-sym))))


(defun add-node (sm name func node-type &optional (duration nil))
  (let ((node-func
          (if (equal 'send (car func))
            (eval func) func)))
    (if (null (send sm :node name))
      (let ((n (instance state :init name node-func)))
        (send n :put :node-type node-type)
        (if duration (send n :put :duration duration))
        (send sm :add-node n)
        n))))


(defun add-transition (sm from-name to-name arc-key)
  (let* ((arc-names
           (mapcar #'(lambda (x) (send x :name))
                   (send (send sm :node from-name) :arc-list))))
    (if (null (find arc-key arc-names :test #'equal))
      (send sm :add-transition from-name to-name arc-key))))


(defun add-state-node
  (sm name &optional (state-func `(lambda-closure nil 0 0 (userdata) t))
                     (duration nil) (sntype nil))
  (let ((n (add-node sm name state-func :state-node duration)))
    (if (and sntype n)
      (send n :put :state-node-type sntype))
    n))


(defun add-action-node (sm name action-func &optional (antype nil))
  (let ((n (add-node sm name action-func :action-node)))
    (if (and antype n)
      (send n :put :action-node-type antype))
    n))


(defun add-teach-node (sm name teach-func &optional (tntype nil))
  (let ((n (add-node sm name teach-func :teach-node)))
    (if (and tntype n)
      (send n :put :teach-node-type tntype))
    n))


(defun add-state-abort-nodes (sm aborts)
  (dolist (ab aborts)
    (let ((from-name (car ab))
          (abort-name (cadr ab))
          (state-func (caddr ab)))
      (add-state-node sm from-name state-func)
      (add-transition sm from-name abort-name nil))))


(defun add-action-abort-nodes (sm aborts)
  (dolist (ab aborts)
    (let ((node-name (car ab))
          (abort-name (cadr ab))
          (action-func (caddr ab)))
      (add-action-node sm node-name action-func)
      (add-transition sm node-name abort-name nil))))


(defun add-state-goal-nodes (sm goals)
  (dolist (gl goals)
    (let ((from-name (car gl))
          (goal-name (cadr gl))
          (state-func (caddr gl)))
      (add-state-node sm from-name state-func)
      (add-transition sm from-name goal-name :finish))))


(defun add-teach-goal-nodes (sm goals)
  (dolist (gl goals)
    (let ((teach-name (car gl))
          (goal-name (cadr gl))
          (teach-func (caddr gl)))
      (add-teach-node sm teach-name teach-func)
      (add-transition sm teach-name goal-name :finish))))


(defun add-goal-nodes (sm goals state-func)
  (dolist (gl goals)
    (let ((from-name (car gl))
          (goal-name (cdr gl)))
      (add-state-node sm from-name state-func)
      (add-transition sm from-name goal-name t))))


(defun add-teach-node-trans (sm from-name teach-name state-func teach-func)
  (add-state-node sm from-name state-func)
  (add-teach-node sm teach-name teach-func)
  (add-transition sm from-name teach-name :teach)
  (add-transition sm teach-name from-name t))


(defun add-smach-transition (sm from-name node-name
                                &key (smach-transition t) (add-t-transition nil))
  (let ((sub-sm (send (send sm :node from-name) :submachine)))
    (if sub-sm
      (let ((goal-state-names
              (mapcar #'(lambda (x) (send x :name)) (send sub-sm :goal-state))))
        (if (null (find smach-transition goal-state-names :test #'equal))
          (let ((next-node-name (send sub-sm :get :next-node-name)))
            (if (null (equal smach-transition t))
              (send sub-sm :append-goal-state smach-transition))
            (add-transition sub-sm next-node-name
                            smach-transition smach-transition)
            )))))
  (add-transition sm from-name node-name smach-transition)
  (if add-t-transition (add-transition sm from-name node-name t)))


(defun add-action-node-trans (sm from-name node-name to-name state-func action-func
                                 &key (smach-transition t) (add-t-transition nil)
                                 (from-duration nil) (to-duration nil))
  (add-state-node sm from-name state-func from-duration)
  (add-action-node sm node-name action-func)
  (add-smach-transition sm from-name node-name
                        :smach-transition smach-transition
                        :add-t-transition add-t-transition)
  (add-state-node sm to-name state-func to-duration)
  (add-transition sm node-name to-name t))


(defun add-nodes-from-trans
  (sm trans &key (state-func `(lambda-closure nil 0 0 (userdata) t))
      (smach-transition t) (add-t-transition nil))
  (let ((from-name (cdr (assoc :from-name trans :test #'equal)))
        (from-duration (cdr (assoc :from-duration trans :test #'equal)))
        (node-name (cdr (assoc :node-name trans :test #'equal)))
        (action-func (cdr (assoc :action-func trans :test #'equal)))
        (to-name (cdr (assoc :to-name trans :test #'equal)))
        (to-duration (cdr (assoc :to-duration trans :test #'equal))))
    (add-action-node-trans sm from-name node-name to-name state-func action-func
                           :smach-transition smach-transition
                           :add-t-transition add-t-transition
                           :from-duration from-duration
                           :to-duration to-duration
                           )))


(defun convert-graph-to-smach
  (gr start-state &key (state-func `(lambda-closure nil 0 0 (userdata) t))
                       (teach-func nil))
  (let* ((sm (instance state-machine :init))
         (goal-state "(:goal)")
         (abort-name "(:abort)")
         (goals (list goal-state))
         (state-aborts nil)
         (state-goals nil)
         (action-aborts nil)
         (teach-goals nil))
    (dolist (node (send gr :nodes))
      (let* ((from-state-sym (convert-string-to-sym (send node :name)))
             (from-name (format nil "~A" from-state-sym))
             (acts (send node :neighbor-action-alist)))
        (add-state-node sm from-name state-func)
        (if (null acts)
          (setq goals
                (append goals
                        (list (cons from-name
                                    (format nil ":goal~A" (length goals)))))))
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
                 (teach-name (format nil "~A" (list '(:teach) from-state-sym))))
            (add-action-node-trans sm from-name node-name to-name state-func action-func
                                   :smach-transition t :add-t-transition nil)
            (setq state-aborts
                  (append state-aborts (list (list from-name abort-name state-func))))
            (setq state-goals
                  (append state-goals (list (list from-name goal-state state-func))))
            (setq action-aborts
                  (append action-aborts (list (list node-name abort-name action-func))))
            (if teach-func
              (progn
                (add-teach-node-trans sm from-name teach-name state-func teach-func)
                (setq teach-goals
                      (append teach-goals
                              (list (list teach-name goal-state teach-func))))))
            ))))
    ;; start-state
    (send sm :start-state start-state)
    ;; goal-state
    (send sm :goal-state (append (mapcar #'cdr goals) (list goal-state abort-name)))
    ;; goals
    (add-goal-nodes sm goals state-func)
    ;; aborts
    (add-state-abort-nodes sm state-aborts)
    (add-action-abort-nodes sm action-aborts)
    ;; goals
    (add-state-goal-nodes sm state-goals)
    (add-teach-goal-nodes sm teach-goals)
    ;; ros-info
    (ros::ros-info (format nil "convert-graph-to-smach: smach start-state is ~A."
                           (send (send sm :start-state) :name)))
    (ros::ros-info (format nil "convert-graph-to-smach: smach goal-state are ~A."
                           (send-all (send sm :goal-state) :name)))
    (send sm :reset-state)
    sm))


(defun convert-trans-alist-to-smach
  (trans-alist first-state-sym last-state-sym
    &key (state-func `(lambda-closure nil 0 0 (userdata) t))
         (teach-func nil) (no-loop nil))
  (let ((sm (instance state-machine :init))
        (start-state "(:start)")
        (goal-state "(:goal)")
        (abort-name "(:abort)")
        (state-aborts nil)
        (state-goals nil)
        (action-aborts nil)
        (teach-goals nil))
    (if no-loop
      (let ((current-state-sym (copy-seq first-state-sym)))
        (while (null (equal current-state-sym last-state-sym))
          (let* ((trans-a (assoc current-state-sym trans-alist :test #'equal))
                 (trans (car (cdr trans-a)))
                 (from-state-sym (car trans-a))
                 (from-name (format nil "~A" from-state-sym))
                 (node-name (cdr (assoc :node-name trans :test #'equal)))
                 (action-func (cdr (assoc :action-func trans :test #'equal)))
                 (to-state-sym (cdr (assoc :to-state-sym trans :test #'equal)))
                 (teach-name (format nil "~A" (list '(:teach) from-state-sym))))
            (add-nodes-from-trans
              sm trans :state-func state-func
              :smach-transition
              (format nil "~A" (cdr (assoc :action-sym trans :test #'equal)))
              :add-t-transition t)
            (setq state-aborts
                  (append state-aborts (list (list from-name abort-name state-func))))
            (setq state-goals
                  (append state-goals (list (list from-name goal-state state-func))))
            (setq action-aborts
                  (append action-aborts (list (list node-name abort-name action-func))))
            (if teach-func
              (progn
                (add-teach-node-trans sm from-name teach-name state-func teach-func)
                (setq teach-goals
                      (append teach-goals (list (list teach-name goal-state teach-func))))))
            (setq current-state-sym to-state-sym))))
      (dolist (trans-a trans-alist)
        (let* ((trans-list (cdr trans-a))
               (from-state-sym (car trans-a))
               (from-name (format nil "~A" from-state-sym))
               (teach-name (format nil "~A" (list '(:teach) from-state-sym)))
               (trans-index 0))
          (dolist (trans trans-list)
            (let ((node-name (cdr (assoc :node-name trans :test #'equal)))
                  (action-func (cdr (assoc :action-func trans :test #'equal))))
              (add-nodes-from-trans
                sm trans :state-func state-func
                :smach-transition
                (format nil "~A" (cdr (assoc :action-sym trans :test #'equal)))
                :add-t-transition (equal trans-index 0))
              (setq action-aborts
                    (append action-aborts (list (list node-name abort-name action-func))))
              (setq trans-index (+ trans-index 1))))
          (setq state-aborts
                (append state-aborts (list (list from-name abort-name state-func))))
          (setq state-goals
                (append state-goals (list (list from-name goal-state state-func))))
          (if teach-func
            (progn
              (add-teach-node-trans sm from-name teach-name state-func teach-func)
              (setq teach-goals
                    (append teach-goals (list (list teach-name goal-state teach-func))))))
          )))
    ;; start-state
    (add-node sm start-state
              `(lambda-closure nil 0 0 (userdata) t) nil)
    (send sm :start-state start-state)
    ;; goal-state
    (send sm :goal-state (list goal-state abort-name))
    ;; start and goal
    (let* ((first-name (format nil "~A" first-state-sym))
           (last-name (format nil "~A" last-state-sym)))
      (add-state-node sm first-name state-func)
      (add-transition sm start-state first-name t)
      (add-node sm goal-state
                `(lambda-closure nil 0 0 (userdata) t) nil)
      (add-state-node sm last-name state-func)
      (add-transition sm last-name goal-state t))
    ;; aborts
    (add-state-abort-nodes sm state-aborts)
    (add-action-abort-nodes sm action-aborts)
    ;; goals
    (add-state-goal-nodes sm state-goals)
    (add-teach-goal-nodes sm teach-goals)
    sm))


(defun convert-solution-to-trans-alist
  (solution &key (robot *robot*) (ri *ri*) (cubes *boxes*)
                 (vw *irtviewer*) (use-torso 0.001) (move-robot t))
  (let ((trans-alist nil))
    ;; create trans-alist from solution
    (dolist (sn solution)
      (let ((ac (send sn :action)))
        (if ac
          (let* ((from-state-sym (convert-string-to-sym (send (send ac :from) :name)))
                 (to-state-sym (convert-string-to-sym (send (send ac :to) :name)))
                 (action-name (send ac :name))
                 (action-syms (read-from-string action-name))
                 (cube-index
                   (position-if #'(lambda (x) (and x (listp x))) action-syms))
                 (action-sym
                   (if cube-index
                     (let ((as (copy-seq (elt action-syms cube-index))))
                       (setq as (remove-key-from-action as :move-robot))
                       (setq as (remove-key-from-action as :use-torso))
                       as)
                     (copy-seq action-syms)))
                 (action-func-sym
                   (if cube-index
                     (let ((afs (copy-seq action-sym)))
                       (setq afs (override-key-value-in-action afs :move-robot move-robot))
                       (setq afs (override-key-value-in-action afs :use-torso use-torso))
                       afs)
                     (copy-seq action-sym)))
                 (action-func
                   `(lambda-closure
                      nil 0 0 (userdata)
                      (apply #',(car action-func-sym)
                             ',(append (cdr action-func-sym)
                                       (if cube-index
                                         (list :robot robot
                                               :ri ri
                                               :cube (elt cubes cube-index)
                                               :cubes cubes
                                               :vw vw))))))
                 (action-sym-list
                   (if cube-index
                     (let
                       ((afsl (make-list (length action-syms) :initial-element nil)))
                       (setf (elt afsl cube-index) action-sym)
                       afsl)
                     action-sym))
                 (action-func-sym-list
                   (if cube-index
                     (let
                       ((afsl (make-list (length action-syms) :initial-element nil)))
                       (setf (elt afsl cube-index) action-func-sym)
                       afsl)
                     action-func-sym))
                 (node-name
                   (format nil "~A"
                           (list action-func-sym-list from-state-sym)))
                 (from-name (format nil "~A" from-state-sym))
                 (to-name (format nil "~A" to-state-sym))
                 (trans (list (cons :from-name from-name)
                              (cons :from-state-sym from-state-sym)
                              (cons :from-duration nil)
                              (cons :node-name node-name)
                              (cons :action-sym action-sym-list)
                              (cons :action-func action-func)
                              (cons :to-name to-name)
                              (cons :to-state-sym to-state-sym)
                              (cons :to-duration nil)
                              )))
            (setq trans-alist
                  (append trans-alist (list (cons from-state-sym nil))))
            (push trans (cdr (assoc from-state-sym trans-alist :test #'equal)))))))
    trans-alist))


(defun convert-collabo-path-to-trans-alist
  (collabo-path &key (robot *robot*) (ri *ri*) (cubes *boxes*)
                     (vw *irtviewer*) (use-torso 0.001) (move-robot t))
  (let ((trans-alist nil)
        (collabo-nodes (copy-seq collabo-path)))
    (while (> (length collabo-nodes) 1)
      (let* ((from-node (car collabo-nodes))
             (from-node-type (cadr (assoc :node-type from-node)))
             (from-state-sym (cadr (assoc :state-action from-node)))
             (from-start-stamp (cadr (assoc :start-stamp from-node)))
             (from-end-stamp (cadr (assoc :end-stamp from-node)))
             (from-duration
               (if (and from-start-stamp from-end-stamp)
                 (- from-end-stamp from-start-stamp)))
             (from-name (format nil "~A" from-state-sym))
             (action-node (cadr collabo-nodes))
             (action-node-type (cadr (assoc :node-type action-node)))
             (action-syms (cadr (assoc :state-action action-node)))
             (cube-index
               (position-if #'(lambda (x) (and x (listp x))) action-syms))
             (action-sym
               (if cube-index
                 (let ((as (copy-seq (elt action-syms cube-index))))
                   (setq as (remove-key-from-action as :move-robot))
                   (setq as (remove-key-from-action as :use-torso))
                   as)
                 (copy-seq action-syms)))
             (action-func-sym
               (if cube-index
                 (let ((afs (copy-seq action-sym)))
                   (setq afs (override-key-value-in-action afs :move-robot move-robot))
                   (setq afs (override-key-value-in-action afs :use-torso use-torso))
                   afs)
                 (copy-seq action-sym)))
             (action-func
               `(lambda-closure
                  nil 0 0 (userdata)
                  (apply #',(car action-func-sym)
                         ',(append (cdr action-func-sym)
                                  (if cube-index
                                    (list :robot robot
                                          :ri ri
                                          :cube (elt cubes cube-index)
                                          :cubes cubes :vw vw))))))
             (action-sym-list
               (if cube-index
                 (let
                   ((afsl (make-list (length action-syms) :initial-element nil)))
                   (setf (elt afsl cube-index) action-sym)
                   afsl)
                 action-sym))
             (action-func-sym-list
               (if cube-index
                 (let
                   ((afsl (make-list (length action-syms) :initial-element nil)))
                   (setf (elt afsl cube-index) action-func-sym)
                   afsl)
                 action-func-sym))
             (node-name
               (format nil "~A"
                       (list action-func-sym-list from-state-sym)))
             (to-node (caddr collabo-nodes))
             (to-node-type (cadr (assoc :node-type to-node)))
             (to-state-sym (cadr (assoc :state-action to-node)))
             (to-start-stamp (cadr (assoc :start-stamp to-node)))
             (to-end-stamp (cadr (assoc :end-stamp to-node)))
             (to-duration
               (if (and to-start-stamp to-end-stamp)
                 (- to-end-stamp to-start-stamp)))
             (to-name (format nil "~A" to-state-sym))
             (trans (list (cons :from-name from-name)
                          (cons :from-duration from-duration)
                          (cons :node-name node-name)
                          (cons :action-sym action-sym-list)
                          (cons :action-func action-func)
                          (cons :to-name to-name)
                          (cons :to-state-sym to-state-sym)
                          (cons :to-duration to-duration))))
        (cond
          ((and (equal from-node-type :state-node)
                (equal action-node-type :action-node)
                (equal to-node-type :state-node))
           (setq trans-alist (append trans-alist (list (cons from-state-sym nil))))
           (push trans (cdr (assoc from-state-sym trans-alist :test #'equal))))
          (t
           (ros::ros-error "something is wrong in collabo-path")
           (ros::ros-error "from-node  : ~A" from-node)
           (ros::ros-error "action-node: ~A" action-node)
           (ros::ros-error "to-node    : ~A" to-node)))
        ;; state0, action0, state1, action1... -> state1, action1...
        (setq collabo-nodes (if (> (length collabo-nodes) 2) (subseq collabo-nodes 2)))))
  trans-alist))


(defun convert-solution-to-smach
  (solution &key (state-func `(lambda-closure nil 0 0 (userdata) t))
                 (teach-func nil) (no-loop nil)
                 (robot *robot*) (ri *ri*) (cubes *boxes*)
                 (vw *irtviewer*) (use-torso 0.001) (move-robot t))
  (let* ((first-state-sym
           (convert-string-to-sym (send (send (car solution) :state) :name)))
         (last-state-sym
           (convert-string-to-sym (send (send (car (last solution)) :state) :name)))
         (trans-alist
           (convert-solution-to-trans-alist
             solution
             :robot robot :ri ri :cubes cubes
             :vw vw :use-torso use-torso :move-robot move-robot))
         (sm (convert-trans-alist-to-smach
               trans-alist first-state-sym last-state-sym
               :state-func state-func :teach-func teach-func
               :no-loop no-loop)))
    ;; ros-info
    (ros::ros-info
      (format nil "convert-solution-to-smach: smach start-state is ~A."
              (send (send sm :start-state) :name)))
    (ros::ros-info
      (format nil "convert-solution-to-smach: smach goal-state are ~A."
              (send-all (send sm :goal-state) :name)))
    (send sm :reset-state)
    sm))


(defun convert-collabo-path-to-smach
  (collabo-path &key (state-func `(lambda-closure nil 0 0 (userdata) t))
                     (teach-func nil) (no-loop nil)
                     (robot *robot*) (ri *ri*) (cubes *boxes*)
                     (vw *irtviewer*) (use-torso 0.001) (move-robot t))
  (let* ((collabo-nodes
           (if (equal :task (car (car collabo-path)))
             (cdr collabo-path) collabo-path))
         (first-state-sym
           (cadr (assoc :state-action (car collabo-nodes))))
         (last-state-sym
           (cadr (assoc :state-action (car (last collabo-nodes)))))
         (trans-alist
           (convert-collabo-path-to-trans-alist
             collabo-nodes
             :robot robot :ri ri :cubes cubes
             :vw vw :use-torso use-torso :move-robot move-robot))
         (sm (convert-trans-alist-to-smach
               trans-alist first-state-sym last-state-sym
               :state-func state-func :teach-func teach-func
               :no-loop no-loop)))
    (ros::ros-info (format nil "convert-collabo-path-to-smach: smach start-state is ~A."
                           (send (send sm :start-state) :name)))
    (ros::ros-info (format nil "convert-collabo-path-to-smach: smach goal-state are ~A."
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
