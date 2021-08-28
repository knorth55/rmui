(require :state-machine-ros "package://roseus_smach/src/state-machine-ros.l")
(require :state-machine-utils "package://roseus_smach/src/state-machine-utils.l")

(require :statenet-graph "package://rmuieus/euslisp/statenet/statenet-graph.l")
(require :common-io "package://rmuieus/euslisp/common/common-io.l")
(require :common-graph "package://rmuieus/euslisp/common/common-graph.l")
(require :common-actions "package://rmuieus/euslisp/common/common-actions.l")

(ros::roseus-add-msgs "rmui_msgs")


;; solver and smach

(setq *solver-class* depth-first-graph-search-solver)
(setq *solver* (instance *solver-class* :init))
(setq *sm* nil)


;; state

(setq *initial-state* '(:init))
(setq *current-state* '(:init))
(setq *reset-state*
      '((:larm-contact-state :released)
        (:rarm-contact-state :released)
        (:floor-contact-state :bottom-contact)
        (:object-location-state :center)
        (:object-rotation-state (:x0 :y0 :z0))))


;; functions

(defun add-initial-state (gr)
  (add-action-state-in-graph gr '(action-init) '(:init) *reset-state*))


(defun load-merged-statenet-graph ()
  (setq *statenet-graph* (merge-statenet-graphs (load-all-statenet-graph)))
  (if (null *statenet-graph*) (setq *statenet-graph* (load-merged-statenet-graph)))
  (dump-merged-statenet-graph *statenet-graph*)
  (add-initial-state *statenet-graph*))


(defun solve-statenet-path (start-state goal-state)
  (send *statenet-graph* :start-state
        (send *statenet-graph* :search-node-from-pddl-state start-state))
  (send *statenet-graph* :goal-state
        (send *statenet-graph* :search-node-from-pddl-state goal-state))
  (send *solver* :solve *statenet-graph*))


(defun generate-smach-from-state (start-state goal-state)
  (let ((solution (solve-statenet-path start-state goal-state)))
    (if solution (setq *sm* (convert-solution-to-smach solution)))))


(defun execute-statenet-cb (server goal)
  (let ((start-state
          (if (> (length (send goal :goal :start_state)) 0)
            (read-from-string (send goal :goal :start_state))
            *current-state*))
        (goal-state (read-from-string (send goal :goal :goal_state)))
        (result (instance rmui_msgs::ExecuteStatenetActionResult :init)))
    (ros::ros-info (format nil "start-state: ~A" start-state))
    (ros::ros-info (format nil "goal-state: ~A" goal-state))
    ;; check current-state and start-state
    (if (null (equal *current-state* start-state))
      (progn
        (ros::ros-warn
          (format nil "start-state and current-state are different: ~A != ~A"
                  start-state *current-state*))
        (if (null (send goal :goal :force_update_start_state))
          (progn
            (ros::ros-warn
              "Aborting execution because start-state and current-state are different.")
            (send result :result :success nil)
            (send server :set-aborted result)
            (return-from execute-statenet-cb nil)))))
    ;; check start-state and goal-state
    (if (equal start-state goal-state)
      (progn
        (ros::ros-warn
          (format nil "start-state and goal-state are same: ~A = ~A"
                  start-state goal-state))
        (ros::ros-warn "Skipping execution because start-state and goal-state are same")
        (send result :result :success t)
        (send server :set-succeeded result)
        (return-from execute-statenet-cb t)))
    ;; generate smach machine
    (generate-smach-from-state start-state goal-state)
    (if (null *sm*)
      (progn
        (send result :result :success nil)
        (send server :set-aborted result)
        (return-from execute-statenet-cb nil)))

    ;; execute state machine
    ;; modified from (exec-state-machine *sm* nil :hz 1.0)
    (let* ((root-name "SM_ROOT")
           (insp (instance state-machine-inspector :init *sm* :root-name root-name))
           (mydata nil)
           (hz 1.0))
      (unix::sleep 2)
      (send *sm* :reset-state)
      (send insp :publish-structure) ;; publish once and latch
      (apply #'send *sm* :arg-keys
             (union (send *sm* :arg-keys) (mapcar #'car mydata)))
      (while (ros::ok)
        (ros::spin-once)
        (if (and (boundp '*ri*) *ri*) (send *ri* :spin-once))
        (send insp :publish-status mydata)
        (when (send *sm* :goal-reached) (return))
        (send *sm* :execute mydata :step -1)
        (unix::usleep (round (/ 1e6 hz))))
      (send *sm* :active-state))

    ;; finish state-machine
    (setq *current-state* goal-state)
    (ros::ros-info (format nil "current-state: ~A" *current-state*))
    (send result :result :success t)
    (send server :set-succeeded result)
    t))


(defun main (&key (pdf nil))
  (load-merged-statenet-graph)
  (if pdf
    (progn
      (send *statenet-graph* :write-to-pdf "rmui.pdf")
      (piped-fork "xdg-open rmui.pdf")))
  (setq *server* (instance ros::simple-action-server :init
                           "~execute" rmui_msgs::ExecuteStatenetAction
                           :execute-cb 'execute-statenet-cb))
  (ros::rate 10)
  (do-until-key
    (send *server* :worker)
    (send *server* :spin-once)
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (if (boundp '*ri*) (send *ri* :draw-objects))
    (ros::sleep)))


(defun call-execute-statenet (start-state goal-state &key (server-name "/demo_rmui"))
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


(provide :common-statenet "common-statenet.l")
