(require :common-statenet "package://rmuieus/euslisp/common/common-statenet.l")

(ros::roseus-add-msgs "rmui_msgs")


;; solver and smach

(setq *solver-class* depth-first-graph-search-solver)
(setq *solver* (instance *solver-class* :init))
(setq *sm* nil)
(setq *insp* nil)


;; state

(setq *initial-state* '(:init))
(setq *current-state* '(:init))


;; functions


(defun initialize-state-machine (userdata &key (root-name "SM_ROOT"))
  (send *sm* :reset-state)
  (setq *insp* (instance state-machine-inspector :init *sm* :root-name root-name))
  (send *insp* :publish-structure) ;; publish once and latch
  (apply #'send *sm* :arg-keys (union (send *sm* :arg-keys) (mapcar #'car userdata))))


(defun set-active-state (&key ((:action a) nil) ((:state s) nil))
  (if s
    (let ((node-name (format nil "~A" (if a (list a s) s))))
      (send *sm* :active-state (send *sm* :node node-name)))))


(defun set-start-state (&key ((:action a) nil) ((:state s) nil))
  (if s
    (let ((node-name (format nil "~A" (if a (list a s) s))))
      (send *sm* :start-state (send *sm* :node node-name)))))


(defun replan-state-machine (current-state goal-state first-action)
  (let ((sm (generate-smach-from-state
              *statenet-graph* *solver*
              current-state goal-state
              ;; segmentation fault
              ;: :state-func #'statenet-state-func
              :state-func #'(lambda (userdata) (statenet-state-func userdata))
              :first-action first-action)))
    sm))


(defun statenet-state-func (userdata)
  (let* ((current-sm-node (send *sm* :active-state))
         (current-state (read-from-string (send current-sm-node :name)))
         (goal-state (cdr (assoc :goal-state userdata))))
    (tagbody retry
      (let ((user-decision (ask-user-decision :sm *sm* :gr *statenet-graph*)))
        (cond
          ((equal user-decision :next)
           (return-from statenet-state-func t))
          ((equal user-decision :abort)
           (if (ask-user-abortion)
             (return-from statenet-state-func nil)
             (go retry)))
          ((and (listp user-decision)
                (equal (car user-decision) :replan))
           (let ((sm))
             (ros::ros-warn "replanning...")
             (setq sm (replan-state-machine
                        current-state goal-state (cdr user-decision)))
             (if sm
               (progn
                 (ros::ros-info "replanning succeeded")
                 (setq *sm* sm)
                 (set-start-state :action (cdr user-decision)
                                  :state current-state)
                 (initialize-state-machine userdata)
                 (return-from statenet-state-func t))
               (progn
                 (ros::ros-error "replanning failed...")
                 (go retry)))))
          ;; TODO (knorth55): for :next, replanning, it is not good.
          ((and (listp user-decision)
                (equal (car user-decision) :next))
           (let ((sm))
             (ros::ros-warn "replanning...")
             (setq sm (replan-state-machine
                        current-state goal-state (cdr user-decision)))
             (if sm
               (progn
                 (ros::ros-info "replanning succeeded")
                 (setq *sm* sm)
                 (set-start-state :action (cdr user-decision)
                                  :state current-state)
                 (initialize-state-machine userdata)
                 (return-from statenet-state-func t))
               (progn
                 (ros::ros-error "replanning failed...")
                 (go retry)))))
          ((equal user-decision nil)
           (ros::ros-error "something is wrong.")
           (return-from statenet-state-func nil))
          (t (go retry)))))))


(defun execute-statenet-cb (server goal)
  (let ((start-state
          (if (> (length (send goal :goal :start_state :state)) 0)
            (read-from-string (send goal :goal :start_state :state))
            *current-state*))
        (goal-state (read-from-string (send goal :goal :goal_state :state)))
        (result (instance rmui_msgs::ExecuteStatenetActionResult :init))
        (feedback (instance rmui_msgs::ExecuteStatenetActionFeedback :init))
        (start-stamp (ros::time-now)))
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
            (send result :result :stamp (ros::time- (ros::time-now) start-stamp))
            (send result :result :success nil)
            (send result :result :final_state :state (format nil "~A" *current-state*))
            (send server :set-aborted result)
            (return-from execute-statenet-cb nil)))))
    ;; check start-state and goal-state
    (if (equal start-state goal-state)
      (progn
        (ros::ros-warn
          (format nil "start-state and goal-state are same: ~A = ~A"
                  start-state goal-state))
        (ros::ros-warn "Skipping execution because start-state and goal-state are same")
        (send result :result :stamp (ros::time- (ros::time-now) start-stamp))
        (send result :result :success t)
        (send result :result :final_state :state (format nil "~A" *current-state*))
        (send server :set-succeeded result)
        (return-from execute-statenet-cb t)))
    ;; generate smach machine
    (setq *sm* (generate-smach-from-state
                 *statenet-graph* *solver* start-state goal-state
                 ;; segmentation fault
                 ;: :state-func #'statenet-state-func
                 :state-func #'(lambda (userdata) (statenet-state-func userdata))))
    (if (null *sm*)
      (progn
        (send result :result :stamp (ros::time- (ros::time-now) start-stamp))
        (send result :result :success nil)
        (send result :result :final_state :state (format nil "~A" *current-state*))
        (send server :set-aborted result)
        (return-from execute-statenet-cb nil)))

    ;; execute state machine
    ;; modified from (exec-state-machine *sm* nil :hz 1.0)
    (let* ((userdata (list (cons :goal-state goal-state)))
           (hz 1.0))
      (unix::usleep (round (* 1e6 0.5)))
      (initialize-state-machine userdata)
      (ros::rate hz)
      (while (ros::ok)
        (ros::spin-once)
        (if (and (boundp '*ri*) *ri*) (send *ri* :spin-once))
        ;; check if sm is not nil for replanning
        (if (null *sm*)
          (progn
            (send result :result :stamp
                  (ros::time- (ros::time-now) start-stamp))
            (send result :result :success nil)
            (send result :result :final_state :state
                  (format nil "~A" *current-state*))
            (send server :set-aborted result)
            (return-from execute-statenet-cb nil)))
        (let* ((active-state (send *sm* :active-state))
               (node-type (get active-state :node-type))
               (node-name (send active-state :name))
               (node-sym (read-from-string node-name)))
          ;; check node type
          (ros::ros-info (format nil "active-state: ~A" node-name))
          (ros::ros-info (format nil "node-type   : ~A" node-type))
          (cond
            ((equal node-type :state-node)
             (setq *current-state* node-sym)
             (ros::ros-info (format nil "current-state: ~A" *current-state*)))
            ((equal node-type :action-node)
             (ros::ros-info (format nil "action-function: ~A" (car node-sym))))
            (t nil))
          (send *insp* :publish-status userdata)
          (when (send *sm* :goal-reached) (return))
          ;; cancel
          (if (send server :is-preempt-requested)
            (progn
              (send result :result :stamp (ros::time- (ros::time-now) start-stamp))
              (send result :result :success nil)
              (send result :result :final_state :state (format nil "~A" *current-state*))
              (send server :set-preempted result)
              (return-from execute-statenet-cb nil)))
          ;; execute
          (send *sm* :execute userdata :step -1)
          ;; feedback
          (send feedback :feedback :stamp
                (ros::time- (ros::time-now) start-stamp))
          (send server :publish-feedback feedback)
          ;; sleep
          (ros::sleep))))

    ;; finish state-machine
    (ros::ros-info (format nil "current-state: ~A" *current-state*))
    (let* ((final-state (send (send *sm* :active-state) :name))
           (abortp (equal (read-from-string (subseq final-state 1 7)) :abort))
           (goalp (equal (read-from-string (subseq final-state 1 6)) :goal)))
      (send result :result :stamp (ros::time- (ros::time-now) start-stamp))
      (send result :result :success (and goalp (null abortp)))
      (send result :result :final_state :state (format nil "~A" *current-state*))
      (send server :set-succeeded result))
    t))


(defun current-state-publish-timer-cb (event)
  (publish-current-state))


(defun publish-current-state ()
  (let ((msg (instance rmui_msgs::StatenetStateStamped :init)))
    (send msg :header :stamp (ros::time-now))
    (send msg :state :state (format nil "~A" *current-state*))
    (ros::publish "~current_state" msg)))


(defun visualize-update-cb (event)
  (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
  (if (boundp '*ri*) (send *ri* :draw-objects)))


(defun main (&key (pdf nil))
  (ros::roseus "statenet_server")
  (setq *statenet-graph* (load-merged-statenet-graph))
  (if pdf
    (progn
      (send *statenet-graph* :write-to-pdf "rmui.pdf")
      (piped-fork "xdg-open rmui.pdf")))
  (setq *server* (instance ros::simple-action-server :init
                           "~execute" rmui_msgs::ExecuteStatenetAction
                           :execute-cb 'execute-statenet-cb))
  (ros::advertise "~current_state" rmui_msgs::StatenetStateStamped 1 t)
  (ros::create-timer 0.1 #'current-state-publish-timer-cb)
  (ros::create-timer 0.1 #'(lambda (event) (send *server* :worker)))
  (ros::create-timer 0.1 #'visualize-update-cb)

  (ros::rate 10)
  (ros::spin))


(provide :common-statenet-server "common-statenet-server.l")
