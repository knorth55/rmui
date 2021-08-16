#!/usr/bin/env roseus

(require :rmui-pddl-common "package://rmuieus/euslisp/pddl/rmui-pddl-common.l")

(require :baxter-scenes "package://rmuieus/euslisp/baxter/baxter-scenes.l")
(require :baxter-motions "package://rmuieus/euslisp/baxter/baxter-motions.l")


(defun main (problem)
  (ros::roseus "baxter_solve_pddl")
  (load-pddl-problem problem)
  (solve-pddl)
  (execute-smach :move-robot t :use-torso t)
  (when (string= "__log:=" (subseq (car (last lisp::*eustop-argument*)) 0 7))
    (ros::exit)))
