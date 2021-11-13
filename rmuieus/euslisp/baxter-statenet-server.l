#!/usr/bin/env roseus

(require :baxter-scenes "package://rmuieus/euslisp/baxter/baxter-scenes.l")
(require :baxter-motions "package://rmuieus/euslisp/baxter/baxter-motions.l")
(require :common-statenet-server "package://rmuieus/euslisp/common/common-statenet-server.l")
(require :rmui-utils "package://rmuieus/euslisp/rmui/rmui-utils.l")


;; 60 seconds for duration threshold
(defvar *duration-threshold* 60)
(defvar *interrupt-timeout* 10)
(defvar *prx-threshold* 800)
(defvar *prx-position-threshold* 0.5)


(defun main (&key (pdf nil) (rmui-names (list "rmui0")))
  (ros::roseus "baxter_statenet_server")
  (statenet-server-main :pdf pdf :rmui-names rmui-names))
