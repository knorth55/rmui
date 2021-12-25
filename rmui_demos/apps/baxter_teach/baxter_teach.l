#!/usr/bin/env roseus

(load "package://rmuieus/euslisp/baxter-execute-statenet.l")


(ros::roseus "baxter_teach")
(setq *res* (main "teach"))
(unix::exit 0)
