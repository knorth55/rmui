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
          show-pdf
          object-statenet-dir
          collabo-statenet-dir
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
          object-state-estimator
          robot-state-estimator
          ; rmplanner
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
               ((:show-pdf shpdf) nil)
               ((:object-statenet-dir objnet-dir)
                *training-object-statenet-path*)
               ((:collabo-statenet-dir clbnet-dir)
                *training-collabo-statenet-path*)
               (prx-threshold) (prx-position-threshold)
               (device-frame-id) (world-frame-id)
               &rest args)
    (send-super* :init ns sp :groupname gpn args)
    (setq namespace ns)
    (setq groupname gpn)
    (setq device-namespace devns)
    (setq device-groupname devgpn)
    (setq reset-state rs)
    (setq show-pdf shpdf)
    (setq object-statenet-dir objnet-dir)
    (setq collabo-statenet-dir clbnet-dir)

    ;; instances
    (setq gr
          (load-merged-statenet-graph-with-reset
            reset-state :data-dir object-statenet-dir))
    (setq slvr (instance *solver-class* :init))
    (setq stplanner
          (instance statenet-planner :init
                    :graph gr :solver slvr
                    :state-func
                    `(send ,self :get-statenet-state-submachine)
                    :teach-func
                    `(send ,self :get-statenet-teach-submachine)
                    ))
    (setq device
          (instance rmui-device :init
                    :namespace device-namespace
                    :prx-threshold prx-threshold
                    :prx-position-threshold prx-position-threshold
                    :device-frame-id device-frame-id
                    :world-frame-id world-frame-id
                    :groupname device-groupname))
    ; (setq rmplanner (instance rmui-planner :init :namespace device-namespace))

    ;; state
    (setq initial-state '(:init))
    (setq current-state '(:init))
    (ros::advertise "~current_state" rmui_msgs::StatenetStateStamped 1 t)
    (send self :init-timer)
    (ros::ros-info "init finished")
    (if show-pdf (send self :show-statenet-pdf))
    self)
  (:graph () gr)
  (:solver () slvr)
  (:statenet-planner () stplanner)
  (:state-machine () sm)
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
  (:show-statenet-pdf ()
    (send gr :write-to-pdf "rmui_statenet.pdf")
    (piped-fork "xdg-open rmui_statenet.pdf"))
  (:show-smach-pdf ()
    (send sm :write-to-pdf "rmui_smach.pdf")
    (piped-fork "xdg-open rmui_smach.pdf"))
  (:initialize-state-machine (userdata &key (root-name "SM_ROOT"))
    (send sm :reset-state)
    (setq insp (instance state-machine-inspector :init sm :root-name root-name))
    (apply #'send sm :arg-keys (union (send sm :arg-keys) (mapcar #'car userdata)))
    (send insp :publish-structure) ;; publish once and latch
    (if show-pdf (send self :show-smach-pdf)))
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
  (:next-in-statenet-state (userdata)
    (let* ((first-action (cdr (cdr (assoc :user-decision userdata)))))
      (return-from :next-in-statenet-state (format nil "~A" first-action))))
  (:replan-in-statenet-state (userdata)
    (ros::ros-warn "replanning...")
    (let* ((start-state (cdr (assoc :replan-start-state userdata)))
           (goal-state (cdr (assoc :goal-state userdata)))
           (first-action (cdr (cdr (assoc :user-decision userdata))))
           (replan-sm
             (send self :plan-state-machine start-state goal-state
                   :first-action first-action :no-loop nil)))
      (if replan-sm
        (progn
          (ros::ros-info "replanning succeeded")
          (setq sm replan-sm)
          (send self :set-start-state :action first-action
                :state start-state)
          (send self :initialize-state-machine userdata)
          (return-from :replan-in-statenet-state t))
        (progn
          (ros::ros-error "replanning failed...")
          (ros::ros-error "cannot find a valid path to the goal-state")
          (return-from :replan-in-statenet-state nil)))))
  (:ask-user-decision (userdata)
    (let* ((current-sm-node (send sm :active-state))
           (current-sm-state (read-from-string (send current-sm-node :name)))
           (goal-state (cdr (assoc :goal-state userdata))))
        (let ((user-decision (ask-user-decision sm gr)))
          (setf (cdr (assoc :user-decision userdata)) user-decision)
          (cond
            ;; replan
            ((and (consp user-decision)
                  (equal (car user-decision) :replan))
             (setf (cdr (assoc :replan-start-state userdata)) current-sm-state))
            ;; teach
            ((and (consp user-decision)
                  (equal (car user-decision) :teach))
             (setf (cdr (assoc :teach-start-state userdata)) current-sm-state))
            ;; something is wrong
            ((equal user-decision nil)
             (ros::ros-error "something is wrong."))
            (t nil))
          (return-from :ask-user-decision (car user-decision)))))
  (:get-statenet-state-submachine ()
    (let ((sub-sm (instance state-machine :init))
          (ask-node-name "(:ask)")
          (replan-node-name "(:replan)")
          (pass-node-name "(:pass)")
          (abort-node-name "(:abort)")
          (next-node-name "(:next)"))
      (add-state-node sub-sm ask-node-name
                      `(lambda-closure nil 0 0 (userdata)
                                       (send ,self :ask-user-decision userdata)))
      (add-state-node sub-sm replan-node-name
                      `(lambda-closure nil 0 0 (userdata)
                                       (send ,self :replan-in-statenet-state userdata)))
      (add-state-node sub-sm pass-node-name
                      `(lambda-closure nil 0 0 (userdata) t))
      (add-state-node sub-sm abort-node-name
                      `(lambda-closure nil 0 0 (userdata) t))
      (add-state-node sub-sm next-node-name
                      `(lambda-closure nil 0 0 (userdata)
                                       (send ,self :next-in-statenet-state userdata)))
      (send sub-sm :start-state ask-node-name)
      (send sub-sm :goal-state '(t nil :teach :replan))
      (add-transition sub-sm ask-node-name replan-node-name :replan)
      (add-transition sub-sm ask-node-name pass-node-name :pass)
      (add-transition sub-sm ask-node-name abort-node-name :abort)
      (add-transition sub-sm ask-node-name next-node-name :next)
      (add-transition sub-sm ask-node-name :teach :teach)
      (add-transition sub-sm ask-node-name ask-node-name nil)
      (add-transition sub-sm replan-node-name :replan t)
      (add-transition sub-sm replan-node-name ask-node-name nil)
      (add-transition sub-sm pass-node-name t t)
      (add-transition sub-sm abort-node-name nil t)
      ; (add-transition sub-sm next-node-name t t)
      sub-sm))
  (:wait-ready-for-teach (userdata)
    (let* ((start-state (cdr (assoc :teach-start-state userdata)))
           (user-input-state nil))
      ; (ros::rate 0.1)
      ; (while  (not (equal start-state user-input-state))
      ;   (let* ((update-p (send device :update-device-orientation))
      ;          (contact-positions
      ;            (if update-p (send device :get-contact-positions)))
      ;          ;; TODO: implement object-state-estimator
      ;          ; (object-state
      ;          ;   (send object-state-estimator :get-object-state contact-positions))
      ;          )
      ;     ;; TODO: implement robot-stte-estimator
      ;     ; (user-input-state
      ;     ;   (send robot-state-estimator :get-robot-state object-state))
      ;     (ros::ros-info "teach start state: ~A" start-state)
      ;     (ros::ros-info "user input state : ~A" user-input-state)
      ;     (if device-groupname
      ;       (ros::spin-once device-groupname))
      ;     (ros::sleep)))
      ; (ros::ros-info "user input state: ~A" user-input-state)
      t))
  ;; TODO (knorth55): implement teaching
  (:check-teach-scene-state (userdata)
    t)
  ;; TODO (knorth55): implement teaching
  (:confirm-user-teach-action (userdata)
    t)
  ;; TODO (knorth55): implement teaching
  (:search-teach-action (userdata)
    t)
  ;; TODO (knorth55): implement teaching
  (:update-teach-action (userdata)
    t)
  ;; TODO (knorth55): implement teaching
  (:ask-user-finish-teach (userdata)
    t)
  ;; TODO (knorth55): implement teaching
  (:replan-in-statenet-teach (userdata)
    t)
  (:get-statenet-teach-submachine ()
    (let ((sub-sm (instance state-machine :init))
          (wait-ready-node-name "(:wait-ready-for-teach)")
          (check-node-name "(:check-teach-scene-state)")
          (confirm-node-name "(:confirm-user-teach-action)")
          (search-node-name "(:search-teach-action)")
          (update-node-name "(:update-teach-action)")
          (ask-finish-node-name "(:ask-finish)")
          (replan-node-name "(:replan-in-statenet-teach)"))
      (add-teach-node sub-sm wait-ready-node-name
                      `(lambda-closure nil 0 0 (userdata)
                                       (send ,self :wait-ready-for-teach userdata)))
      (add-teach-node sub-sm check-node-name
                      `(lambda-closure nil 0 0 (userdata)
                                       (send ,self :check-teach-scene-state userdata)))
      (add-teach-node sub-sm confirm-node-name
                      `(lambda-closure nil 0 0 (userdata)
                                       (send ,self :confirm-user-teach-action userdata)))
      (add-teach-node sub-sm search-node-name
                      `(lambda-closure nil 0 0 (userdata)
                                       (send ,self :search-teach-action userdata)))
      (add-teach-node sub-sm update-node-name
                      `(lambda-closure nil 0 0 (userdata)
                                       (send ,self :update-teach-action userdata)))
      (add-teach-node sub-sm ask-finish-node-name
                      `(lambda-closure nil 0 0 (userdata)
                                       (send ,self :ask-user-finish-teach userdata)))
      (add-teach-node sub-sm replan-node-name
                      `(lambda-closure nil 0 0 (userdata)
                                       (send ,self :replan-in-statenet-teach userdata)))
      (send sub-sm :start-state wait-ready-node-name)
      (send sub-sm :goal-state '(t))
      (add-transition sub-sm wait-ready-node-name check-node-name t)
      (add-transition sub-sm wait-ready-node-name t nil)
      (add-transition sub-sm check-node-name confirm-node-name t)
      (add-transition sub-sm check-node-name check-node-name nil)
      (add-transition sub-sm confirm-node-name search-node-name t)
      (add-transition sub-sm confirm-node-name check-node-name nil)
      (add-transition sub-sm search-node-name update-node-name t)
      (add-transition sub-sm search-node-name check-node-name nil)
      (add-transition sub-sm update-node-name ask-finish-node-name t)
      (add-transition sub-sm ask-finish-node-name replan-node-name t)
      (add-transition sub-sm ask-finish-node-name check-node-name nil)
      (add-transition sub-sm replan-node-name t t)
      sub-sm))
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
      (let* ((userdata (list (cons :start-state start-state)
                             (cons :goal-state goal-state)
                             (cons :user-decision nil)
                             (cons :replan-start-state nil)))
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
            (send insp :publish-structure)
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
    (if device-groupname
      (ros::spin-once device-groupname))
    (send-super :spin-once))
  )


(provide :common-statenet-server "common-statenet-server.l")
