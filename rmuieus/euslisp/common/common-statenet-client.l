(require :common-statenet "package://rmuieus/euslisp/common/common-statenet.l")

(ros::roseus-add-msgs "rmui_msgs")


(defclass statenet-action-client
  :super ros::simple-action-client
  :slots (server-name
          current-state
          reset-state)
  )

(defmethod statenet-action-client
  (:init (&key ((:server-name sn) "/statenet_server")
               ((:spec sp) rmui_msgs::ExecuteStatenetAction)
               ((:reset-state rs) *reset-state*)
               &rest args)
    (setq server-name sn)
    (setq reset-state rs)
    (send-super* :init (format nil "~A/execute" server-name) sp args)
    (ros::subscribe (format nil "~A/current_state" server-name)
                    rmui_msgs::StatenetStateStamped
                    #'send self :current-state-cb 50)
    self)
  (:current-state () current-state)
  (:current-state-cb (msg)
    (setq current-state (read-from-string (send msg :state :state))))
  (:execute-reset-statenet ()
    (let ((current-state (get-current-state)))
      (send self :call-execute-statenet current-state reset-state)))
  (:call-execute-statenet (start-state goal-state)
    (let ((goal (instance rmui_msgs::ExecuteStatenetActionGoal :init)))
      (if (null (equal start-state current-state))
        (progn
          (ros::ros-warn "current-state and start-state are not same")
          (ros::ros-warn (format nil "current-state: ~A" current-state))
          (ros::ros-warn (format nil "start-state  : ~A" start-state))))
      (send goal :goal :start_state :state (format nil "~A" start-state))
      (send goal :goal :goal_state :state (format nil "~A" goal-state))
      (send self :wait-for-server)
      (send self :send-goal goal)
      (send self :wait-for-result)
      (send self :get-result)))
  (:cancel-execute-statenet ()
    (send self :wait-for-server)
    (send self :cancel-all-goals)))


(provide :common-statenet-client "common-statenet-client.l")
