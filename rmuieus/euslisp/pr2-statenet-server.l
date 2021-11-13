#!/usr/bin/env roseus

(require :pr2-scenes "package://rmuieus/euslisp/pr2/pr2-scenes.l")
(require :pr2-motions "package://rmuieus/euslisp/pr2/pr2-motions.l")
(require :common-statenet-server "package://rmuieus/euslisp/common/common-statenet-server.l")
(require :rmui-utils "package://rmuieus/euslisp/rmui/rmui-utils.l")


;; 60 seconds for duration threshold
(defvar *duration-threshold* 60)
(defvar *interrupt-timeout* 10)
(defvar *prx-threshold* 800)
(defvar *prx-position-threshold* 0.5)


(defun main (&key (pdf nil) (rmui-names (list "rmui0")))
  (ros::roseus "pr2_statenet_server")
  (statenet-server-main :pdf pdf :rmui-names rmui-names))
