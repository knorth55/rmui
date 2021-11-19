(require :common-statenet "package://rmuieus/euslisp/common/common-statenet.l")


(defclass statenet-planner
  :super propertied-object
  :slots (gr
          slvr
          state-func
          teach-func
          robot
          ri
          boxes
          robotviewer
          use-torso
          move-robot
          )
  )


(defmethod statenet-planner
  (:init (&key ((:graph g))
               ((:solver s))
               ((:state-func sfunc))
               ((:teach-func tfunc))
               ((:robot rbt) *robot*)
               ((:ri rbti) *ri*)
               ((:boxes bxs) *boxes*)
               ((:robotviewer rbtvw) *irtviewer*)
               ((:use-torso utrso) 0.001)
               ((:move-robot mvrbt) t))
    (setq gr g)
    (setq slvr s)
    (setq state-func sfunc)
    (setq teach-func tfunc)
    (setq robot rbt)
    (setq ri rbti)
    (setq boxes bxs)
    (setq robotviewer rbtvw)
    (setq use-torso utrso)
    (setq move-robot mvrbt)
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
  (:convert-solution-to-state-machine (solution &key (no-loop nil))
    (convert-solution-to-smach
      solution :state-func state-func :teach-func teach-func :no-loop no-loop
      :robot robot :ri ri :cubes boxes
      :vw robotviewer :use-torso use-torso
      :move-robot move-robot))
  (:convert-collabo-path-to-state-machine (collabo-path &key (no-loop))
    (convert-collabo-path-to-smach
      (if (equal :task (car (car collabo-path)))
        (cdr collabo-path) collabo-path)
      :state-func state-func :teach-func teach-func :no-loop no-loop
      :robot robot :ri ri :cubes boxes
      :vw robotviewer :use-torso use-torso
      :move-robot move-robot))
  (:generate-smach-from-state (start-state goal-state &key (first-action nil) (no-loop nil))
    (let* ((solution
             (send self :solve-statenet-path
                   start-state goal-state :first-action first-action))
           (sm (if solution
                 (send self :convert-solution-to-state-machine
                       solution :no-loop no-loop))))
      sm)))


(provide :common-statenet-planner "common-statenet-planner.l")
