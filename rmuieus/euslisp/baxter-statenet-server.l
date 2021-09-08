#!/usr/bin/env roseus

(require :baxter-scenes "package://rmuieus/euslisp/baxter/baxter-scenes.l")
(require :baxter-motions "package://rmuieus/euslisp/baxter/baxter-motions.l")
(require :common-statenet-server "package://rmuieus/euslisp/common/common-statenet-server.l")


(defun main (&key (pdf nil))
  (ros::roseus "baxter_statenet_server")
  (setq *server* (instance statenet-action-server :init))
  (if pdf (send *server* :show-pdf))
  (ros::rate 10)
  (while (ros::ok)
    (send *server* :spin-once)))
