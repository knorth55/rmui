#!/usr/bin/env roseus

(require :common-motions "package://rmuieus/euslisp/common/common-motions.l")
(require :pr2-scenes "package://rmuieus/euslisp/pr2/pr2-scenes.l")
(require :pr2-interface "package://pr2eus/pr2-interface.l")


(defmethod pr2-robot
  (:reset-cube-manip-pose ()
     (send self :angle-vector
           (float-vector 300.0  ;; torso
                         ;; 75 20 85 -70 -60 -5 15 ;; larm
                         ;; -75 20 -85 -70 60 -5 -15 ;; rarm
                         ;; 70 -7.5 60 -90 -15 -5 -10  ;; larm
                         ;; -70 -7.5 -60 -90 15 -5 10  ;; rarm
                         70 30 105 -95 -30 -25 -20  ;; larm
                         -70 30 -105 -95 30 -25 20  ;; rarm
                         0.0 50.0  ;; head
                         ))))


(provide :pr2-motions "pr2-motions.l")
