#!/usr/bin/env roseus

(require :solve-dualarm-liftup "package://rmuieus/euslisp/pddl/solve-dualarm-liftup.l")

(require :pr2-scenes "package://rmuieus/euslisp/pr2/pr2-scenes.l")
(require :pr2-motions "package://rmuieus/euslisp/pr2/pr2-motions.l")


(ros::roseus "pr2_solve_dualarm_liftup")
(solve-pddl)
(execute-smach :move-robot t :use-torso t)

(when (string= "__log:=" (subseq (car (last lisp::*eustop-argument*)) 0 7))
  (ros::exit))
