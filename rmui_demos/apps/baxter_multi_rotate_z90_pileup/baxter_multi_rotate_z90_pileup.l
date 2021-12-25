#!/usr/bin/env roseus

(load "package://rmuieus/euslisp/baxter-execute-statenet.l")


(ros::roseus "baxter_multi_rotate_z90_pileup")
(setq *res* (main "dualarm-multi-rotate-z90-pileup"))
(unix::exit 0)
