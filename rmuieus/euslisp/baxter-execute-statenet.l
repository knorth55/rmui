#!/usr/bin/env roseus

(require :statenet-util "package://rmuieus/euslisp/statenet/statenet-util.l")
(require :common-statenet-client "package://rmuieus/euslisp/common/common-statenet-client.l")

(ros::roseus-add-msgs "rmui_msgs")


(defun main (state)
  (ros::roseus "baxter_execute_statenet")
  (load-statenet-state state)
  (call-execute-statenet *start-state* *goal-state*))
