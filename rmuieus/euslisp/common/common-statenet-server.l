(require :common-statenet "package://rmuieus/euslisp/common/common-statenet.l")

(ros::roseus-add-msgs "rmui_msgs")


(defvar *solver-class* depth-first-graph-search-solver)


(defclass statenet-action-server
  :super ros::simple-action-server
  :slots (solver
          sm
          insp
          gr
          initial-state
          current-state))


(defmethod statenet-action-server
  (:init (&key ((:namespace ns) "~execute")
               ((:spec sp) rmui_msgs::ExecuteStatenetAction)
               &rest args)
    (send-super* :init ns sp args)
    (setq gr (load-merged-statenet-graph))
    (setq solver (instance *solver-class* :init))
    ;; state
    (setq initial-state '(:init))
    (setq current-state '(:init))
    (ros::advertise "~current_state" rmui_msgs::StatenetStateStamped 1 t)
    (ros::ros-info "init finished")
    self)
  (:execute-cb ()
   (if (send self :goal)
     (send self :execute-statenet-cb (send self :goal)))
   (send-super :execute-cb))
  (:init-timer ()
    (ros::create-timer 0.1 #'send self :current-state-publish-timer-cb)
    (ros::create-timer 0.1 #'send self :visualize-update-cb))
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
  (:replan-state-machine (current-state goal-state first-action)
    (let ((replan-sm
            (generate-smach-from-state
                gr solver
                current-state goal-state
                ;; segmentation fault
                ;: :state-func #'statenet-state-func
                :state-func
                `(lambda-closure nil 0 0 (userdata)
                                 (send ,self :statenet-state-func userdata))
                :first-action first-action
                :no-loop nil)))
      replan-sm))
  (:statenet-state-func (userdata)
    (let* ((current-sm-node (send sm :active-state))
           (current-state (read-from-string (send current-sm-node :name)))
           (goal-state (cdr (assoc :goal-state userdata))))
      (tagbody retry-ask
        (let ((user-decision (ask-user-decision :sm sm :gr gr)))
          (cond
            ((equal user-decision :pass)
             (return-from :statenet-state-func t))
            ((equal user-decision :abort)
             (return-from :statenet-state-func nil))
            ;; TODO (knorth55): implement teaching
            ((equal user-decision :teach)
             (ros::ros-error "teaching is not implemented. aborting.")
             (return-from :statenet-state-func nil))
            ((and (listp user-decision)
                  (equal (car user-decision) :replan))
             (let ((replan-sm))
               (ros::ros-warn "replanning...")
               (setq replan-sm
                     (send self :replan-state-machine
                           current-state goal-state (cdr user-decision)))
               (if replan-sm
                 (progn
                   (ros::ros-info "replanning succeeded")
                   (setq sm replan-sm)
                   (send self :set-start-state :action (cdr user-decision)
                         :state current-state)
                   (send self :initialize-state-machine userdata)
                   ;; replan, so this transition does not have any effect
                   (return-from :statenet-state-func t))
                 (progn
                   (ros::ros-error "replanning failed...")
                   (go retry-ask)))))
            ((and (listp user-decision)
                  (equal (car user-decision) :next))
             (return-from :statenet-state-func
                          (format nil "~A" (cdr user-decision))))
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
        (setq sm (generate-smach-from-state
                   gr solver start-state goal-state
                   ;; segmentation fault
                   ;: :state-func #'statenet-state-func
                   :state-func
                   `(lambda-closure nil 0 0 (userdata)
                                    (send ,self :statenet-state-func userdata))
                   ; :state-func #'send self :statenet-state-func
                   :no-loop t))
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
            (ros::spin-once)
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
    (if (boundp '*ri*) (send *ri* :draw-objects))))


(provide :common-statenet-server "common-statenet-server.l")
