#!/usr/bin/env roseus

(require :common-statenet-client "package://rmuieus/euslisp/common/common-statenet-client.l")

(ros::roseus-add-msgs "rmui_msgs")


(defun main (state &key (n-box nil) (server-name "/statenet_server"))
  (ros::roseus "pr2_execute_statenet")
  (statenet-action-client-main state :n-box n-box
                               :server-name server-name))
