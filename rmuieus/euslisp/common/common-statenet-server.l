(require :common-statenet "package://rmuieus/euslisp/common/common-statenet.l")
(require :common-statenet-planner
         "package://rmuieus/euslisp/common/common-statenet-planner.l")
(require :common-rmui-planner "package://rmuieus/euslisp/common/common-rmui-planner.l")
(require :common-graph "package://rmuieus/euslisp/common/common-graph.l")

(ros::roseus-add-msgs "rmui_msgs")


(defvar *solver-class* depth-first-graph-search-solver)


(defclass statenet-action-server
  :super ros::simple-action-server
  :slots (namespace
          groupname
          ;; statenet
          slvr
          sm
          insp
          gr
          stplanner
          ;; rmui
          device
          device-namespace
          device-groupname
          rmplanner
          ;; state
          initial-state
          current-state
          reset-state
          ))


(defmethod statenet-action-server
  (:init (&key ((:namespace ns) "~execute")
               ((:groupname gpn))
               ((:spec sp) rmui_msgs::ExecuteStatenetAction)
               ((:reset-state rs) *reset-state*)
               ((:device-namespace devns))
               ((:device-groupname devgpn))
               (prx-threshold) (prx-position-threshold)
               (device-frame-id) (world-frame-id)
               &rest args)
    (send-super* :init ns sp :groupname gpn args)
    (setq namespace ns)
    (setq groupname gpn)
    (setq device-namespace devns)
    (setq device-groupname devgpn)
    (setq reset-state rs)

    ;; instances
    (setq gr (load-merged-statenet-graph-with-reset reset-state))
    (setq slvr (instance *solver-class* :init))
    (setq stplanner
          (instance statenet-planner :init
                    :graph gr :solver slvr
                    :state-func
                    `(lambda-closure nil 0 0 (userdata)
                                     (send ,self :statenet-state-func userdata))
                    :teach-func
                    `(lambda-closure nil 0 0 (userdata)
                                     (send ,self :statenet-teach-func userdata))))
    (setq device
          (instance rmui-device :init
                    :namespace device-namespace
                    :prx-threshold prx-threshold
                    :prx-position-threshold prx-position-threshold
                    :device-frame-id device-frame-id
                    :world-frame-id world-frame-id
                    :groupname device-groupname))
    (setq rmplanner (instance rmui-planner :init :namespace device-namespace))

    ;; state
    (setq initial-state '(:init))
    (setq current-state '(:init))
    (ros::advertise "~current_state" rmui_msgs::StatenetStateStamped 1 t)
    (send self :init-timer)
    (ros::ros-info "init finished")
    self)
  (:graph () gr)
  (:solver () slvr)
  (:statenet-planner () stplanner)
  (:execute-cb ()
   (if (send self :goal)
     (send self :execute-statenet-cb (send self :goal)))
   (send-super :execute-cb))
  (:init-timer ()
    (ros::create-timer 0.1 #'send self :current-state-publish-timer-cb)
    (ros::create-timer 0.1 #'send self :visualize-update-cb)
    (ros::create-timer 0.1
                       `(lambda-closure nil 0 0 (event)
                                        (send ,self :execute-cb))))
  (:show-pdf ()
    (send gr :write-to-pdf "rmui.pdf")
    (piped-fork "xdg-open rmui.pdf"))
  (:initialize-state-machine (userdata &key (root-name "SM_ROOT"))
    (send sm :reset-state)
    (setq insp (instance state-machine-inspector :init sm :root-name root-name))
    (send insp :publish-structure) ;; publish once and latch
    (apply #'send sm :arg-keys (union (send sm :arg-keys) (mapcar #'car userdata))))
  (:set-active-state (&key ((:action a) nil) ((:state s) nil))
    (if s
      (let ((node-name (format nil "~A" (if a (list a s) s))))
        (send sm :active-state (send sm :node node-name)))))
  (:set-start-state (&key ((:action a) nil) ((:state s) nil))
    (if s
      (let ((node-name (format nil "~A" (if a (list a s) s))))
        (send sm :start-state (send sm :node node-name)))))
  (:plan-state-machine (start-state goal-state &key (first-action nil) (no-loop nil))
    (send stplanner :generate-smach-from-state
          start-state goal-state
          :first-action first-action
          :no-loop no-loop))
  (:replan-in-statenet-state-func (userdata start-state goal-state first-action)
    (ros::ros-warn "replanning...")
    (let ((replan-sm
            (send self :plan-state-machine start-state goal-state
                  :first-action first-action :no-loop nil)))
      (if replan-sm
        (progn
          (ros::ros-info "replanning succeeded")
          (setq sm replan-sm)
          (send self :set-start-state :action first-action
                :state start-state)
          (send self :initialize-state-machine userdata)
          (return-from :replan-in-statenet-state-func t))
        (progn
          (ros::ros-error "replanning failed...")
          (ros::ros-error "cannot find a valid path to the goal-state")
          (return-from :replan-in-statenet-state-func nil)))))
  ;; TODO (knorth55): implement teaching
  (:statenet-teach-func (userdata)
   (let* ((current-sm-node (send sm :active-state))
          (current-sm-state (read-from-string (send current-sm-node :name)))
          (before-state (cadr current-sm-state)))
      (ros::ros-warn (format nil "teaching before state: ~A" before-state))
      (ros::ros-error "teaching is not implemented.")
      (return-from :statenet-teach-func t)))
  (:statenet-state-func (userdata)
    (let* ((current-sm-node (send sm :active-state))
           (current-sm-state (read-from-string (send current-sm-node :name)))
           (goal-state (cdr (assoc :goal-state userdata))))
      (tagbody retry-ask
        (let ((user-decision (ask-user-decision sm gr)))
          (cond
            ;;pass
            ((equal user-decision :pass)
             (return-from :statenet-state-func t))
            ;; abort
            ((equal user-decision :abort)
             (return-from :statenet-state-func nil))
            ;; teach
            ((equal user-decision :teach)
             (return-from :statenet-state-func :teach))
            ;; replan
            ((and (listp user-decision)
                  (equal (car user-decision) :replan))
             (if (send self :replan-in-statenet-state-func
                       userdata current-sm-state goal-state (cdr user-decision))
               ;; replan, so this transition does not have any effect
               (return-from :statenet-state-func t)
               (go retry-ask)))
            ;; next
            ((and (listp user-decision)
                  (equal (car user-decision) :next))
             (return-from :statenet-state-func
                          (format nil "~A" (cdr user-decision))))
            ;; something is wrong
            ((equal user-decision nil)
             (ros::ros-error "something is wrong.")
             (return-from :statenet-state-func nil))
            (t (go retry-ask)))))))
  (:execute-statenet-cb (goal)
    (let ((start-state
            (if (> (length (send goal :goal :start_state :state)) 0)
              (read-from-string (send goal :goal :start_state :state))
              current-state))
          (goal-state (read-from-string (send goal :goal :goal_state :state)))
          (result (instance rmui_msgs::ExecuteStatenetActionResult :init))
          (feedback (instance rmui_msgs::ExecuteStatenetActionFeedback :init))
          (start-stamp (ros::time-now)))
      (ros::ros-info (format nil "start-state: ~A" start-state))
      (ros::ros-info (format nil "goal-state: ~A" goal-state))
      ;; check current-state and start-state
      (if (null (equal current-state start-state))
        (progn
          (ros::ros-warn
            (format nil "start-state and current-state are different: ~A != ~A"
                    start-state current-state))
          (if (null (send goal :goal :force_update_start_state))
            (progn
              (ros::ros-warn
                "Aborting execution because start-state and current-state are different.")
              (send result :result :stamp (ros::time- (ros::time-now) start-stamp))
              (send result :result :success nil)
              (send result :result :final_state :state (format nil "~A" current-state))
              (send self :set-aborted result)
              (return-from :execute-statenet-cb nil)))))
      ;; check start-state and goal-state
      (if (equal start-state goal-state)
        (progn
          (ros::ros-warn
            (format nil "start-state and goal-state are same: ~A = ~A"
                    start-state goal-state))
          (ros::ros-warn "Skipping execution because start-state and goal-state are same")
          (send result :result :stamp (ros::time- (ros::time-now) start-stamp))
          (send result :result :success t)
          (send result :result :final_state :state (format nil "~A" current-state))
          (send self :set-succeeded result)
          (return-from :execute-statenet-cb t)))
      ;; generate smach machine
      (setq sm (send self :plan-state-machine start-state goal-state
                     :first-action nil :no-loop t))
      (if (null sm)
        (progn
          (send result :result :stamp (ros::time- (ros::time-now) start-stamp))
          (send result :result :success nil)
          (send result :result :final_state :state (format nil "~A" current-state))
          (send self :set-aborted result)
          (return-from :execute-statenet-cb nil)))

      ;; execute state machine
      ;; modified from (exec-state-machine *sm* nil :hz 1.0)
      (let* ((userdata (list (cons :goal-state goal-state)))
             (hz 1.0))
        (unix::usleep (round (* 1e6 0.5)))
        (send self :initialize-state-machine userdata)
        (ros::rate hz)
        (while (ros::ok)
          (send self :spin-once)
          (if (and (boundp '*ri*) *ri*) (send *ri* :spin-once))
          ;; check if sm is not nil for replanning
          (if (null sm)
            (progn
              (send result :result :stamp
                    (ros::time- (ros::time-now) start-stamp))
              (send result :result :success nil)
              (send result :result :final_state :state
                    (format nil "~A" current-state))
              (send self :set-aborted result)
              (return-from :execute-statenet-cb nil)))
          (let* ((active-state (send sm :active-state))
                 (node-type (get active-state :node-type))
                 (node-name (send active-state :name))
                 (node-sym (read-from-string node-name)))
            ;; check node type
            (ros::ros-info (format nil "active-state: ~A" node-name))
            (ros::ros-info (format nil "node-type   : ~A" node-type))
            (cond
              ((equal node-type :state-node)
               (setq current-state node-sym)
               (ros::ros-info (format nil "current-state: ~A" current-state)))
              ((equal node-type :action-node)
               (ros::ros-info (format nil "action-function: ~A" (car node-sym))))
              ((equal node-type :teach-node)
               (ros::ros-info (format nil "teach-state: ~A" node-sym)))
              (t nil))
            (send insp :publish-status userdata)
            (when (send sm :goal-reached) (return))
            ;; cancel
            (if (send self :is-preempt-requested)
              (progn
                (send result :result :stamp (ros::time- (ros::time-now) start-stamp))
                (send result :result :success nil)
                (send result :result :final_state :state (format nil "~A" current-state))
                (send self :set-preempted result)
                (return-from :execute-statenet-cb nil)))
            ;; execute
            (send sm :execute userdata :step -1)
            ;; feedback
            (send feedback :feedback :stamp
                  (ros::time- (ros::time-now) start-stamp))
            (send self :publish-feedback feedback)
            ;; sleep
            (ros::sleep))))

      ;; finish state-machine
      (ros::ros-info (format nil "current-state: ~A" current-state))
      (let* ((final-state (send (send sm :active-state) :name))
             (abortp (equal (read-from-string (subseq final-state 1 7)) :abort))
             (goalp (equal (read-from-string (subseq final-state 1 6)) :goal)))
        (send result :result :stamp (ros::time- (ros::time-now) start-stamp))
        (send result :result :success (and goalp (null abortp)))
        (send result :result :final_state :state (format nil "~A" current-state))
        (send self :set-succeeded result))
      t))
  (:current-state-publish-timer-cb (event) (send self :publish-current-state))
  (:publish-current-state ()
    (let ((msg (instance rmui_msgs::StatenetStateStamped :init)))
      (send msg :header :stamp (ros::time-now))
      (send msg :state :state (format nil "~A" current-state))
      (ros::publish "~current_state" msg)))
  (:visualize-update-cb (event)
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (if (boundp '*ri*) (send *ri* :draw-objects)))
  (:spin-once ()
    (if device-groupname (ros::spin-once device-groupname))
    (send-super :spin-once))
  )


(provide :common-statenet-server "common-statenet-server.l")
