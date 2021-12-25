#!/usr/bin/env roseus

(load "package://rmuieus/euslisp/baxter-execute-statenet.l")

(ros::roseus "baxter_multi_pileup")
(setq *res* (main "dualarm-multi-pileup"))
(unix::exit 0)
