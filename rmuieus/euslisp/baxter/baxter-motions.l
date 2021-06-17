#!/usr/bin/env roseus

(require :common-motions "package://rmuieus/euslisp/common/common-motions.l")
(require :baxter-scenes "package://rmuieus/euslisp/baxter/baxter-scenes.l")
(require :baxter-interface "package://baxtereus/baxter-interface.l")


(defmethod baxter-robot
  (:reset-cube-manip-pose ()
     (send self :reset-pose)))


(defun start-grasp (&optional (arm :arms) (wait nil) &rest args)
  (send* *ri* :start-grasp arm :wait wait args))


(defun stop-grasp (&optional (arm :arms) (wait nil) &rest args)
  (send* *ri* :stop-grasp arm :wait wait args))


(provide :baxter-motions "baxter-motions.l")
