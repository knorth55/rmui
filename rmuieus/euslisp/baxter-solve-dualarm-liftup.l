#!/usr/bin/env roseus

(require :solve-dualarm-liftup "package://rmuieus/euslisp/pddl/solve-dualarm-liftup.l")

(require :baxter-scenes "package://rmuieus/euslisp/baxter/baxter-scenes.l")
(require :baxter-motions "package://rmuieus/euslisp/baxter/baxter-motions.l")


(ros::roseus "baxter_solve_dualarm_liftup")
(solve-pddl)
(execute-smach :move-robot t :use-torso t)

(when (string= "__log:=" (subseq (car (last lisp::*eustop-argument*)) 0 7))
  (ros::exit))
