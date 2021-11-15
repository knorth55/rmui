#!/usr/bin/env roseus

(require :pr2-interface "package://pr2eus/pr2-interface.l")
(require :common-scenes "package://rmuieus/euslisp/common/common-scenes.l")


(defvar *table-z* 0)
(defvar *table-height* 700)
(defvar *box-pos-x* 550)
(defvar *contact-offset* (float-vector -80 0 0))


;; functions

(defun robot-scene-init (table-objects table-pos)
  ;; robot-init
  (setq *pr2* (instance pr2-robot :init))
  (setq *robot* *pr2*)
  (setq *ri* (instance pr2-interface :init :objects table-objects))
  (robot-scene-init-base :table-pos table-pos
                         :larm-ee-frame-id "l_wrist_roll_link"
                         :rarm-ee-frame-id "r_wrist_roll_link"
                         :base-frame-id "base_footprint"))


(provide :pr2-scenes "pr2-scenes.l")
