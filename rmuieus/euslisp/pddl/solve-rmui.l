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
        (AT TABLE)
        (ZROTATE CUBE Z0)
        ))


(send *problem* :goal-condition
      '((HOLDUP CUBE)
        (ZROTATE CUBE Z180)
        ))

;;
;; domain
;;
(setq *domain* (instance pddl-domain :init :name 'rmui))
(send *domain* :requirements '(:typing :equality :conditional-effects :quantified-preconditions))
(send *domain* :types '(object spot arm lside rside))
(send *domain* :constants '(CUBE - object
                            LARM RARM - arm
                            TABLE - spot
                            LEFT LEFT-FRONT LEFT-BACK - lside
                            RIGHT RIGHT-FRONT RIGHT-BACK - rside
                            LEFT LEFT-FRONT LEFT-BACK RIGHT RIGHT-FRONT RIGHT-BACK - side
                            Z45 Z-45 - rotate-zangle
                            Z0 Z45 Z90 Z135 Z180 Z-45 Z-90 Z-135 - zangle
                            ))
(send *domain* :predicates '((ON ?obj - object ?spot - spot)
                             (AT ?spot - spot)
                             (CONTACT ?obj - object ?arm - arm ?side - side)
                             (ZROTATE ?obj - object ?zangle - zangle)
                             (HOLDUP ?obj - object)
                             ))

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
                                 (AT ?SPOT)
                                 (NOT (exists (?CSIDE - side)
                                              (CONTACT ?OBJECT ?ARM ?CSIDE))))
                 :effect '((CONTACT ?OBJECT ?ARM ?SIDE)))
       (instance pddl-action :init
                 :name "contact_f"
                 :parameters '((?OBJECT object) (?ARM arm) (?SPOT spot) (?SIDE side))
                 :precondition '((ON ?OBJECT ?SPOT)
                                 (AT ?SPOT)
                                 (NOT (exists (?CSIDE - side)
                                              (CONTACT ?OBJECT ?ARM ?CSIDE))))
                 :effect '())
       (instance pddl-action :init
                 :name "release"
                 :parameters '((?OBJECT object) (?ARM arm) (?SPOT spot) (?SIDE side))
                 :precondition '((ON ?OBJECT ?SPOT)
                                 (CONTACT ?OBJECT ?ARM ?SIDE)
                                 (AT ?SPOT))
                 :effect '((NOT (CONTACT ?OBJECT ?ARM ?SIDE))))
       (instance pddl-action :init
                 :name "zrotate"
                 :parameters '((?OBJECT object) (?SPOT spot) (?LSIDE lside) (?RSIDE rside)
                               (?ZANGLE zangle) (?ROTATE-ZANGLE rotate-zangle))
                 :precondition '((ON ?OBJECT ?SPOT)
                                 (AT ?SPOT)
                                 (CONTACT ?OBJECT LARM ?LSIDE)
                                 (CONTACT ?OBJECT RARM ?RSIDE)
                                 (ZROTATE ?OBJECT ?ZANGLE))
                 :effect '((WHEN (AND (= ?ZANGLE Z0)
                                      (= ?ROTATE-ZANGLE Z45)
                                      (= ?LSIDE LEFT)
                                      (= ?RSIDE RIGHT))
                                 (ZROTATE ?OBJECT Z45))
                           (WHEN (AND (= ?ZANGLE Z45)
                                      (= ?ROTATE-ZANGLE Z45)
                                      (= ?LSIDE LEFT-FRONT)
                                      (= ?RSIDE RIGHT-BACK))
                                 (ZROTATE ?OBJECT Z90))
                           (WHEN (AND (= ?ZANGLE Z90)
                                      (= ?ROTATE-ZANGLE Z45)
                                      (= ?LSIDE LEFT)
                                      (= ?RSIDE RIGHT))
                                 (ZROTATE ?OBJECT Z135))
                           (WHEN (AND (= ?ZANGLE Z135)
                                      (= ?ROTATE-ZANGLE Z45)
                                      (= ?LSIDE LEFT-FRONT)
                                      (= ?RSIDE RIGHT-BACK))
                                 (ZROTATE ?OBJECT Z180))
                           (WHEN (AND (= ?ZANGLE Z180)
                                      (= ?ROTATE-ZANGLE Z45)
                                      (= ?LSIDE LEFT)
                                      (= ?RSIDE RIGHT))
                                 (ZROTATE ?OBJECT Z-135))
                           (WHEN (AND (= ?ZANGLE Z-135)
                                      (= ?ROTATE-ZANGLE Z45)
                                      (= ?LSIDE LEFT-FRONT)
                                      (= ?RSIDE RIGHT-BACK))
                                 (ZROTATE ?OBJECT Z-90))
                           (WHEN (AND (= ?ZANGLE Z-90)
                                      (= ?ROTATE-ZANGLE Z45)
                                      (= ?LSIDE LEFT)
                                      (= ?RSIDE RIGHT))
                                 (ZROTATE ?OBJECT Z-45))
                           (WHEN (AND (= ?ZANGLE Z-45)
                                      (= ?ROTATE-ZANGLE Z45)
                                      (= ?LSIDE LEFT-FRONT)
                                      (= ?RSIDE RIGHT-BACK))
                                 (ZROTATE ?OBJECT Z0))
                           (WHEN (AND (= ?ZANGLE Z0)
                                      (= ?ROTATE-ZANGLE Z-45)
                                      (= ?LSIDE LEFT)
                                      (= ?RSIDE RIGHT))
                                 (ZROTATE ?OBJECT Z-45))
                           (WHEN (AND (= ?ZANGLE Z-45)
                                      (= ?ROTATE-ZANGLE Z-45)
                                      (= ?LSIDE LEFT-BACK)
                                      (= ?RSIDE RIGHT-FRONT))
                                 (ZROTATE ?OBJECT Z-90))
                           (WHEN (AND (= ?ZANGLE Z-90)
                                      (= ?ROTATE-ZANGLE Z-45)
                                      (= ?LSIDE LEFT)
                                      (= ?RSIDE RIGHT))
                                 (ZROTATE ?OBJECT Z-135))
                           (WHEN (AND (= ?ZANGLE Z-135)
                                      (= ?ROTATE-ZANGLE Z-45)
                                      (= ?LSIDE LEFT-BACK)
                                      (= ?RSIDE RIGHT-FRONT))
                                 (ZROTATE ?OBJECT Z180))
                           (WHEN (AND (= ?ZANGLE Z180)
                                      (= ?ROTATE-ZANGLE Z-45)
                                      (= ?LSIDE LEFT)
                                      (= ?RSIDE RIGHT))
                                 (ZROTATE ?OBJECT Z135))
                           (WHEN (AND (= ?ZANGLE Z135)
                                      (= ?ROTATE-ZANGLE Z-45)
                                      (= ?LSIDE LEFT-BACK)
                                      (= ?RSIDE RIGHT-FRONT))
                                 (ZROTATE ?OBJECT Z90))
                           (WHEN (AND (= ?ZANGLE Z90)
                                      (= ?ROTATE-ZANGLE Z-45)
                                      (= ?LSIDE LEFT)
                                      (= ?RSIDE RIGHT))
                                 (ZROTATE ?OBJECT Z45))
                           (WHEN (AND (= ?ZANGLE Z45)
                                      (= ?ROTATE-ZANGLE Z-45)
                                      (= ?LSIDE LEFT-BACK)
                                      (= ?RSIDE RIGHT-FRONT))
                                 (ZROTATE ?OBJECT Z0))
                           (WHEN (AND (= ?ROTATE-ZANGLE Z45)
                                      (= ?LSIDE LEFT)
                                      (= ?RSIDE RIGHT))
                                 (AND (CONTACT ?OBJECT LARM LEFT-BACK)
                                      (CONTACT ?OBJECT RARM RIGHT-FRONT)))
                           (WHEN (AND (= ?ROTATE-ZANGLE Z45)
                                      (= ?LSIDE LEFT-FRONT)
                                      (= ?RSIDE RIGHT-BACK))
                                 (AND (CONTACT ?OBJECT LARM LEFT)
                                      (CONTACT ?OBJECT RARM RIGHT)))
                           (WHEN (AND (= ?ROTATE-ZANGLE Z-45)
                                      (= ?LSIDE LEFT)
                                      (= ?RSIDE RIGHT))
                                 (AND (CONTACT ?OBJECT LARM LEFT-FRONT)
                                      (CONTACT ?OBJECT RARM RIGHT-BACK)))
                           (WHEN (AND (= ?ROTATE-ZANGLE Z-45)
                                      (= ?LSIDE LEFT-BACK)
                                      (= ?RSIDE RIGHT-FRONT))
                                 (AND (CONTACT ?OBJECT LARM LEFT)
                                      (CONTACT ?OBJECT RARM RIGHT)))
                           (NOT (ZROTATE ?OBJECT ?ZANGLE))
                           (NOT (CONTACT ?OBJECT LARM ?LSIDE))
                           (NOT (CONTACT ?OBJECT RARM ?RSIDE))))
       (instance pddl-action :init
                 :name "zrotate_f"
                 :parameters '((?OBJECT object) (?SPOT spot)
                               (?ZANGLE zangle) (?ROTATE-ZANGLE rotate-zangle))
                 :precondition '((ON ?OBJECT ?SPOT)
                                 (AT ?SPOT)
                                 (CONTACT ?OBJECT LARM LEFT)
                                 (CONTACT ?OBJECT RARM RIGHT)
                                 (ZROTATE ?OBJECT ?ZANGLE))
                 :effect '())
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
                                 (CONTACT ?OBJECT LARM LEFT)
                                 (CONTACT ?OBJECT RARM RIGHT)
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
(setq *graph* (add-failed-nodes *result* (list 'liftup 'contact 'zrotate) :graph *graph*))

(send *graph* :name (send *domain* :name))

(setq *graph* (make-readable-graph *graph*))
(send *graph* :write-to-pdf "rmui.pdf")

(when (ros::get-param "~display_graph")
  (piped-fork "xdg-open rmui.pdf"))

(when (string= "__log:=" (subseq (car (last lisp::*eustop-argument*)) 0 7))
  (ros::exit))
