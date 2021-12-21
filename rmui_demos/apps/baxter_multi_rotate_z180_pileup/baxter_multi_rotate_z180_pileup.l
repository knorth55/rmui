#!/usr/bin/env roseus

(load "package://rmuieus/euslisp/baxter-execute-statenet.l")


(setq *res* (main "dualarm-multi-rotate-z180-pileup"))
(unix::exit 0)
