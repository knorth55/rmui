#!/usr/bin/env roseus

(require :pr2-scenes "package://rmuieus/euslisp/pr2/pr2-scenes.l")
(require :pr2-motions "package://rmuieus/euslisp/pr2/pr2-motions.l")
(require :human-scenes "package://rmuieus/euslisp/human/human-scenes.l")
(require :human-motions "package://rmuieus/euslisp/human/human-motions.l")
(require :common-rmui-planning "package://rmuieus/euslisp/common/common-rmui-planning.l")
(require :rmui-utils "package://rmuieus/euslisp/rmui/rmui-utils.l")


;; parameters

(defvar *prx-threshold* 800)
(defvar *prx-position-threshold* 0.5)
;; (defvar *motion-symbol-length* 10)
(defvar *motion-symbol-length* 5)
(defvar *use-torso* 0.001)


(defun main (&key (rmui-name "rmui0"))
  (ros::roseus "pr2_rmui_planning")
  (rmui-planning-main :rmui-name rmui-name))
