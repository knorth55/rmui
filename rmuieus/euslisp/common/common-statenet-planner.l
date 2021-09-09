(require :common-statenet "package://rmuieus/euslisp/common/common-statenet.l")


(defclass statenet-planner
  :super propertied-object
  :slots (gr
          slvr
          state-func
          teach-func)
  )


(defmethod statenet-planner
  (:init (&key ((:graph g))
               ((:solver s))
               ((:state-func sfunc))
               ((:teach-func tfunc))
               &rest args)
    (setq gr g
          slvr s
          state-func sfunc
          teach-func tfunc)
    self)
  (:solve-statenet-path (start-state goal-state &key (first-action nil))
    (let ((start-node (send gr :search-node-from-pddl-state start-state))
          (goal-node (send gr :search-node-from-pddl-state goal-state)))
      (if first-action
        (let* ((first-action-name (format nil "~A" first-action))
               (acts (send start-node :neighbor-action-alist))
               (second-node
                 (cdr (assoc first-action-name acts :test #'equal)))
               (arc-alist
                 (mapcar #'(lambda (x) (cons (send x :name) x))
                         (mapcar #'car (send start-node :successors))))
               (first-arc
                 (cdr (assoc first-action-name arc-alist :test #'equal)))
               (solution nil))
          (send gr :start-state second-node)
          (send gr :goal-state goal-node)
          (setq solution (send slvr :solve gr))
          (if solution
            (let ((first-node
                    (instance solver-node :init start-node :cost 0)))
              (send (car solution) :parent first-node)
              (send (car solution) :action first-arc)
              (dolist (sn solution)
                (send sn :cost (+ 1 (send sn :cost))))
              (push first-node solution)
              solution)))
        (let ((solution nil))
          (send gr :start-state start-node)
          (send gr :goal-state goal-node)
          (setq solution (send slvr :solve gr))
          solution))))
  (:generate-smach-from-state (start-state goal-state &key (first-action nil) (no-loop nil))
    (let* ((solution
             (send self :solve-statenet-path
                   start-state goal-state :first-action first-action))
           (sm (if solution
                 (convert-solution-to-smach
                   solution
                   :state-func state-func
                   :teach-func teach-func
                   :no-loop no-loop))))
      sm)))


(provide :common-statenet-planner "common-statenet-planner.l")