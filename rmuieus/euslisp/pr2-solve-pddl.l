#!/usr/bin/env roseus

(require :pddl-common "package://rmuieus/euslisp/pddl/pddl-common.l")

(require :pr2-scenes "package://rmuieus/euslisp/pr2/pr2-scenes.l")
(require :pr2-motions "package://rmuieus/euslisp/pr2/pr2-motions.l")


(defun main (problem)
  (ros::roseus "pr2_solve_pddl")
  (load-pddl-problem problem)
  (solve-pddl)
  (execute-smach :move-robot t :use-torso t)
  (when (string= "__log:=" (subseq (car (last lisp::*eustop-argument*)) 0 7))
    (ros::exit)))
