(require :common-statenet "package://rmuieus/euslisp/common/common-statenet.l")

(ros::roseus-add-msgs "rmui_msgs")


(defun call-execute-statenet (start-state goal-state &key (server-name "/statenet_server"))
  (let ((client
          (instance ros::simple-action-client :init
                    (format nil "~A/execute" server-name)
                    rmui_msgs::ExecuteStatenetAction))
        (goal (instance rmui_msgs::ExecuteStatenetActionGoal :init)))
    (send goal :goal :start_state (format nil "~A" start-state))
    (send goal :goal :goal_state (format nil "~A" goal-state))
    (send client :wait-for-server)
    (send client :send-goal goal)
    (send client :wait-for-result)
    (send client :get-result)))


(defun cancel-execute-statenet (&key (server-name "/statenet_server"))
  (let ((client
          (instance ros::simple-action-client :init
                    (format nil "~A/execute" server-name)
                    rmui_msgs::ExecuteStatenetAction)))
    (send client :wait-for-server)
    (send client :cancel-all-goals)))


(provide :common-statenet-client "common-statenet-client.l")
