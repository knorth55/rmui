#!/usr/bin/env roseus

(load "package://pddl_planner/src/pddl-result-graph.l")
(load "package://pddl_planner/src/eus-pddl-client.l")

;;
;; problem
;;
(setq *problem* (instance pddl-problem :init :name 'dualarm_liftup :domain 'rmui))


(send *problem* :objects
      '((TABLE . spot)
        (CUBE . object)
        ))


(send *problem* :initial-condition
      '((ON CUBE TABLE)
        (AT TABLE)))


(send *problem* :goal-condition
      '((HOLDUP CUBE)))

;;
;; domain
;;
(setq *domain* (instance pddl-domain :init :name 'rmui))
(send *domain* :requirements '(:typing))
(send *domain* :types '(object spot arm side))
(send *domain* :constants '(CUBE - object LARM RARM - arm TABLE - spot LEFT RIGHT - side))
(send *domain* :predicates '((ON ?obj - object ?spot - spot)
                             (AT ?spot - spot)
                             (CONTACT ?obj - object ?arm - arm ?side - side)
                             (HOLDUP ?obj - object)))

;;making action
(setq *actions*
      (list
       (instance pddl-action :init
                 :name "move-to"
                 :parameters '((?FROM ?TO spot))
                 :precondition '((AT ?FROM))
                 :effect '((AT ?TO)
                           (NOT (AT ?FROM))))
       (instance pddl-action :init
                 :name "contact"
                 :parameters '((?OBJECT object) (?ARM arm) (?SPOT spot) (?SIDE side))
                 :precondition '((ON ?OBJECT ?SPOT)
                                 (AT ?SPOT))
                 :effect '((CONTACT ?OBJECT ?ARM ?SIDE)))
       (instance pddl-action :init
                 :name "contact_f"
                 :parameters '((?OBJECT object) (?ARM arm) (?SPOT spot) (?SIDE side))
                 :precondition '((ON ?OBJECT ?SPOT)
                                 (AT ?SPOT))
                 :effect '())
       (instance pddl-action :init
                 :name "release"
                 :parameters '((?OBJECT object) (?ARM arm) (?SPOT spot) (?SIDE side))
                 :precondition '((ON ?OBJECT ?SPOT)
                                 (CONTACT ?OBJECT ?ARM ?SIDE)
                                 (AT ?SPOT))
                 :effect '((NOT (CONTACT ?OBJECT ?ARM ?SIDE))))
       (instance pddl-action :init
                 :name "liftup"
                 :parameters '((?OBJECT object) (?SPOT spot))
                 :precondition '((ON ?OBJECT ?SPOT)
                                 (AT ?SPOT)
                                 (CONTACT ?OBJECT LARM LEFT)
                                 (CONTACT ?OBJECT RARM RIGHT))
                 :effect '((HOLDUP ?OBJECT)
                           (NOT (ON ?OBJECT ?SPOT))))
       (instance pddl-action :init
                 :name "liftup_f"
                 :parameters '((?OBJECT object) (?SPOT spot))
                 :precondition '((ON ?OBJECT ?SPOT)
                                 (AT ?SPOT)
                                 (CONTACT ?OBJECT LARM LEFT)
                                 (CONTACT ?OBJECT RARM RIGHT))
                 :effect '((ON ?OBJECT ?SPOT)
                           (NOT (CONTACT ?OBJECT LARM LEFT))
                           (NOT (CONTACT ?OBJECT RARM RIGHT))))
       (instance pddl-action :init
                 :name "liftdown"
                 :parameters '((?OBJECT object) (?SPOT spot))
                 :precondition '((AT ?SPOT)
                                 (HOLDUP ?OBJECT))
                 :effect '((ON ?OBJECT ?SPOT)
                           (NOT (HOLDUP ?OBJECT))))))


(dolist (act *actions*)
  (send *domain* :add :action act))

;;
;; solve planning
;;
(pprint (setq *result* (solve-pddl-planning *domain* *problem* :debug nil)))

(setq *graph* (make-graph-from-pddl-results (list *result*) :node-name :pprint))
(setq *graph* (add-failed-nodes *result* (list 'liftup 'contact) :graph *graph*))

(send *graph* :name (send *domain* :name))

(setq *graph* (make-readable-graph *graph*))
(send *graph* :write-to-pdf "rmui.pdf")

(when (ros::get-param "~display_graph")
  (piped-fork "xdg-open rmui.pdf"))

(when (string= "__log:=" (subseq (car (last lisp::*eustop-argument*)) 0 7))
  (ros::exit))
