#!/usr/bin/env roseus

(require :pr2-scenes "package://rmuieus/euslisp/pr2/pr2-scenes.l")
(require :pr2-motions "package://rmuieus/euslisp/pr2/pr2-motions.l")
(require :common-statenet-server "package://rmuieus/euslisp/common/common-statenet-server.l")


(defun main (&key (pdf nil))
  (ros::roseus "pr2_statenet_server")
  (setq *server* (instance statenet-action-server :init))
  (if pdf (send *server* :show-pdf))
  (ros::rate 10)
  (while (ros::ok)
    (send *server* :spin-once)))
