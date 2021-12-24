(require :common-statenet "package://rmuieus/euslisp/common/common-statenet.l")

(ros::roseus-add-msgs "std_msgs")
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
      (send goal :goal :allow_update_start_state t)
      (send self :wait-for-server)
      (send self :send-goal goal)
      (send self :wait-for-result)
      (send self :get-result)))
  (:cancel-execute-statenet ()
    (send self :wait-for-server)
    (send self :cancel-all-goals)))


(defun statenet-action-client-main (state &key (n-box nil) (server-name "/statenet_server"))
  (load-statenet-state state)
  (ros::rate 10)
  (if (not n-box)
    (while (null n-box)
      (let* ((n-box-topic-name
               (format nil "~A/n_box" server-name))
             (n-box-msg
               (one-shot-subscribe
                 n-box-topic-name std_msgs::Int8 :timeout 10000)))
        (if n-box-msg
          (setq n-box (send n-box-msg :data))
          (ros::ros-error
            (format nil "Waiting for ~A subscription" n-box-topic-name)))
        (ros::sleep))))
  (set-statenet-reset-state n-box)
  (setq *client*
        (instance statenet-action-client :init
                  :server-name server-name
                  :reset-state *reset-state*))
  (send *client* :call-execute-statenet *start-state* *goal-state*))


(provide :common-statenet-client "common-statenet-client.l")
