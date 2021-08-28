(require :common-statenet "package://rmuieus/euslisp/common/common-statenet.l")

(ros::roseus-add-msgs "rmui_msgs")


(defun get-current-state (&key (server-name "/statenet_server"))
  (let* ((msg (one-shot-subscribe (format nil "~A/current_state" server-name)
                                  rmui_msgs::StatenetStateStamped))
         (current-state (if msg (read-from-string (send msg :state :state)))))
    current-state))


(defun execute-reset-statenet ()
  (let ((current-state (get-current-state)))
    (call-execute-statenet current-state *reset-state*)))


(defun call-execute-statenet (start-state goal-state &key (server-name "/statenet_server"))
  (let ((client
          (instance ros::simple-action-client :init
                    (format nil "~A/execute" server-name)
                    rmui_msgs::ExecuteStatenetAction))
        (current-state (get-current-state))
        (goal (instance rmui_msgs::ExecuteStatenetActionGoal :init)))
    (if (null (equal start-state current-state))
      (progn
        (ros::ros-warn "current-state and start-state are not same")
        (ros::ros-warn (format nil "current-state: ~A" current-state))
        (ros::ros-warn (format nil "start-state  : ~A" start-state))))
    (send goal :goal :start_state :state (format nil "~A" start-state))
    (send goal :goal :goal_state :state (format nil "~A" goal-state))
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
