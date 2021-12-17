#!/usr/bin/env roseus

(load "package://rmuieus/euslisp/baxter-execute-statenet.l")


(setq *res* (main "dualarm-multi-pileup"))
(unix::exit 0)
