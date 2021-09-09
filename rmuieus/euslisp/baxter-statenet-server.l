#!/usr/bin/env roseus

(require :baxter-scenes "package://rmuieus/euslisp/baxter/baxter-scenes.l")
(require :baxter-motions "package://rmuieus/euslisp/baxter/baxter-motions.l")
(require :common-statenet-server "package://rmuieus/euslisp/common/common-statenet-server.l")


(defvar *device-namespace* "/rmui0/rmui_node")
(defvar *device-groupname* "rmui_device")
(defvar *prx-threshold* 800)
(defvar *prx-position-threshold* 0.5)
(defvar *device-frame-id* "rmui_link")
(defvar *world-frame-id* "world")


(defun main (&key (pdf nil))
  (ros::roseus "baxter_statenet_server")
  (setq *server*
        (instance statenet-action-server :init
                  :device-namespace *device-namespace*
                  :device-groupname *device-groupname*
                  :prx-threshold *prx-threshold*
                  :prx-position-threshold *prx-position-threshold*
                  :device-frame-id *device-frame-id*
                  :world-frame-id *world-frame-id*))
  (if pdf (send *server* :show-pdf))
  (ros::rate 10)
  (while (ros::ok)
    (send *server* :spin-once)))
