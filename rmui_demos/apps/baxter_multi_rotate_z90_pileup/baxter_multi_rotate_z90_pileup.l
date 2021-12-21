#!/usr/bin/env roseus

(load "package://rmuieus/euslisp/baxter-execute-statenet.l")


(setq *res* (main "dualarm-multi-rotate-z90-pileup"))
(unix::exit 0)
