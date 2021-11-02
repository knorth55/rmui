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
(defvar *world-frame-id* "world")


(defun main (&key (pdf nil) (rmui-name "rmui0"))
  (ros::roseus "baxter_statenet_server")
  (set-rmui-parameters rmui-name)
  (setq *server*
        (instance statenet-action-server :init
                  :device-namespace *device-namespace*
                  :device-groupname *device-groupname*
                  :duration-threshold *duration-threshold*
                  :interrupt-timeout *interrupt-timeout*
                  :prx-threshold *prx-threshold*
                  :prx-position-threshold *prx-position-threshold*
                  :device-frame-id *device-frame-id*
                  :world-frame-id *world-frame-id*
                  :show-pdf pdf))
  (ros::rate 10)
  (while (ros::ok) (send *server* :spin-once)))
