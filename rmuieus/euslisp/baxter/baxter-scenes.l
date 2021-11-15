#!/usr/bin/env roseus

(require :baxter-interface "package://baxtereus/baxter-interface.l")
(require :collision-object-publisher "package://pr2eus_moveit/euslisp/collision-object-publisher.l")
(require :common-scenes "package://rmuieus/euslisp/common/common-scenes.l")


(defvar *table-z* -900)
(defvar *table-height* 700)
(defvar *box-pos-x* 600)
(defvar *contact-offset* (float-vector -150 0 50))


;; functions

(defun robot-scene-init (&key (table-objects) (table-pos))
  (setq *baxter* (instance baxter-robot-safe :init))
  (setq *robot* *baxter*)
  (setq *ri* (instance baxter-interface :init :objects table-objects))
  (robot-scene-init-base :table-objects table-objects
                         :table-pos table-pos
                         :larm-ee-frame-id "left_wrist"
                         :rarm-ee-frame-id "right_wrist"
                         :base-frame-id "base"))


(provide :baxter-scenes "baxter-scenes.l")
