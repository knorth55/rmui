#!/usr/bin/env roseus

(load "package://rmuieus/euslisp/baxter-execute-statenet.l")


(setq *res* (main "teach"))
(unix::exit 0)