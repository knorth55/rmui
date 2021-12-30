#!/usr/bin/env roseus

(require :pr2-scenes "package://rmuieus/euslisp/pr2/pr2-scenes.l")
(require :pr2-motions "package://rmuieus/euslisp/pr2/pr2-motions.l")
(require :human-scenes "package://rmuieus/euslisp/human/human-scenes.l")
(require :human-motions "package://rmuieus/euslisp/human/human-motions.l")
(require :common-rmui-server "package://rmuieus/euslisp/common/common-rmui-server.l")


;; parameters

(defvar *prx-threshold* 500)
(defvar *prx-position-threshold* 0.5)
(defvar *motion-symbol-length* 15)
; (defvar *motion-symbol-length* 5)
(defvar *use-torso* 0.001)


(defun main (&key (scene-name "one_ctb") (wall-p nil))
  (ros::roseus "pr2_rmui_planning")
  (rmui-server-main :scene-name scene-name :wall-p wall-p))
