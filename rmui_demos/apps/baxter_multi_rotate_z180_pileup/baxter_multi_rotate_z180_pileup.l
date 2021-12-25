#!/usr/bin/env roseus

(load "package://rmuieus/euslisp/baxter-execute-statenet.l")


(ros::roseus "baxter_multi_rotate_z180_pileup")
(setq *res* (main "dualarm-multi-rotate-z180-pileup"))
(if (and *res* (send *res* :success))
  (progn
    (ros::ros-info "Succeeded")
    (unix::exit 0))
  (progn
    (ros::ros-error "Failed")
    (unix::exit 1)))
