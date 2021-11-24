#!/usr/bin/env roseus

(require :baxter-interface "package://baxtereus/baxter-interface.l")
(require :collision-object-publisher "package://pr2eus_moveit/euslisp/collision-object-publisher.l")
(require :common-scenes "package://rmuieus/euslisp/common/common-scenes.l")


(defvar *table-z* -900)
(defvar *box-pos-x* 500)
(defvar *contact-offset* (float-vector -150 0 50))


;; functions

(defun robot-scene-init (&key (table-objects) (table-pos))
  (setq *baxter* (instance baxter-robot-safe :init))
  (setq *robot* *baxter*)
  (setq *ri* (instance baxter-interface :init :objects table-objects))
  (let* ((larm-contact-coords
           (send (send *robot* :larm-end-coords) :copy-worldcoords))
         (rarm-contact-coords
           (send (send *robot* :rarm-end-coords) :copy-worldcoords)))
    (send larm-contact-coords :rotate pi/2 :x)
    (send rarm-contact-coords :rotate -pi/2 :x)
    (robot-scene-init-base
      :table-pos table-pos
      :larm-ee-frame-id "left_wrist"
      :rarm-ee-frame-id "right_wrist"
      :larm-contact-coords larm-contact-coords
      :rarm-contact-coords rarm-contact-coords
      :base-frame-id "base")))


(provide :baxter-scenes "baxter-scenes.l")
