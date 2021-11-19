(require :rmui-device "package://rmuieus/euslisp/rmui/rmui-device.l")
(require :rmui-utils "package://rmuieus/euslisp/rmui/rmui-utils.l")
(require :common-statenet "package://rmuieus/euslisp/common/common-statenet.l")
(require :common-statenet-planner
         "package://rmuieus/euslisp/common/common-statenet-planner.l")
(require :common-statenet-interactor
         "package://rmuieus/euslisp/common/common-statenet-interactor.l")
(require :common-rmui-planner "package://rmuieus/euslisp/common/common-rmui-planner.l")
(require :common-graph "package://rmuieus/euslisp/common/common-graph.l")

(ros::roseus-add-msgs "rmui_msgs")


(defvar *solver-class* depth-first-graph-search-solver)


(defclass statenet-action-server
  :super ros::simple-action-server
  :slots (namespace
          server-namespace
          server-groupname
          groupname
          show-pdf
          object-statenet-dir
          collabo-statenet-dir
          dump-statenet-dir
          duration-threshold
          interrupt-timeout
          ri
          robotviewer
          use-torso
          move-robot
          ;; statenet
          slvr
          sm
          insp
          gr
          stplanner
          stinteractor
          ;; rmui
          devices
          device-namespaces
          device-groupnames
          object-state-estimator
          robot-state-estimator
          ;; rmplanner
          ;; state
          initial-state
          current-state
          reset-state
          skip-interaction
          ))


(defmethod statenet-action-server
  (:init (&key ((:namespace ns) "~execute")
               ((:server-namespace srvns) "/statenet_server")
               ((:server-groupname srvgpn) "statenet_server")
               ((:groupname gpn))
               ((:spec sp) rmui_msgs::ExecuteStatenetAction)
               ((:reset-state rs) *reset-state*)
               ((:device-namespaces devnss))
               ((:device-groupnames devgpns))
               ((:show-pdf shpdf) nil)
               ((:object-statenet-dir objnet-dir) *training-object-statenet-path*)
               ((:collabo-statenet-dir clbnet-dir) *training-collabo-statenet-path*)
               ((:duration-threshold drthld))
               ((:interrupt-timeout itrtmt))
               (prx-threshold)
               (prx-position-threshold)
               (device-frame-ids)
               (device-world-frame-ids)
               (solver-class *solver-class*)
               ((:robot rbt) *robot*)
               ((:ri rbti) *ri*)
               ((:boxes bxs) *boxes*)
               ((:robotviewer rbtvw) *irtviewer*)
               ((:use-torso utrso) 0.001)
               ((:move-robot mvrbt) t)
               ((:skip-interaction skip-p) nil))
    (send-super :init ns sp :groupname gpn)
    (setq namespace ns)
    (setq server-namespace srvns)
    (setq server-groupname srvgpn)
    (setq groupname gpn)
    (setq reset-state rs)
    (setq device-namespaces devnss)
    (setq device-groupnames devgpns)
    (setq show-pdf shpdf)
    (setq duration-threshold drthld)
    (setq interrupt-timeout itrtmt)
    (setq object-statenet-dir objnet-dir)
    (setq collabo-statenet-dir clbnet-dir)
    (setq dump-statenet-dir nil)

    (setq ri rbti)
    (setq robotviewer rbtvw)
    (setq skip-interaction skip-p)

    ;; instances
    (setq gr
          (load-merged-statenet-graph-with-reset
            reset-state :data-dir object-statenet-dir))
    (setq slvr (instance solver-class :init))
    (setq stplanner
          (instance statenet-planner :init
                    :graph gr :solver slvr
                    :state-func
                    `(send ,self :get-statenet-state-submachine)
                    :teach-func
                    `(send ,self :get-statenet-teach-submachine)
                    :robot rbt
                    :ri rbti
                    :boxes bxs
                    :robotviewer rbtvw
                    :use-torso utrso
                    :move-robot mvrbt
                    ))
    (setq stinteractor
          (instance statenet-interactor :init
                    :namespace server-namespace
                    :groupname server-groupname
                    :mode :ros))

    (setq devices nil)
    (dotimes (i (length device-namespaces))
      (let ((device-namespace (elt device-namespaces i))
            (device-groupname (elt device-groupnames i))
            (device-frame-id (elt device-frame-ids i))
            (device-world-frame-id (elt device-world-frame-ids i)))
        (setq devices
              (append devices
                      (list (instance rmui-device :init
                                      :namespace device-namespace
                                      :prx-threshold prx-threshold
                                      :prx-position-threshold prx-position-threshold
                                      :device-frame-id device-frame-id
                                      :device-world-frame-id device-world-frame-id
                                      :groupname device-groupname))))))
    ; (setq rmplanner (instance rmui-planner :init :namespace device-namespace))

    (ros::set-param (format nil "~A/n_box" server-namespace) (length bxs))
    ;; state
    (setq initial-state '(:init))
    (setq current-state '(:init))
    (ros::advertise "~current_state" rmui_msgs::StatenetStateStamped 1 t)
    (send self :init-timer)
    (ros::ros-info "init finished")
    (send self :dump-collabo-statenet)
    (if show-pdf (send self :show-statenet-pdf))
    self)
  (:graph () gr)
  (:solver () slvr)
  (:statenet-planner () stplanner)
  (:state-machine () sm)
  (:dump-collabo-statenet ()
    (if (null dump-statenet-dir)
      (setq dump-statenet-dir
            (create-dump-dir collabo-statenet-dir)))
    (dump-statenet-graph gr :data-dir collabo-statenet-dir))
  (:dump-executed-collabo-path (executed-collabo-path execute-stamp)
    (if (null dump-statenet-dir)
      (setq dump-statenet-dir
            (create-dump-dir collabo-statenet-dir)))
    (if executed-collabo-path
      (dump-executed-collabo-path
        executed-collabo-path
        :data-dir collabo-statenet-dir
        :execute-stamp execute-stamp)))
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
  (:convert-collabo-path-to-state-machine (collabo-path)
    (send stplanner :convert-collabo-path-to-state-machine collabo-path))
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
          (send self :dump-collabo-statenet)
          (return-from :replan-in-statenet-state t))
        (progn
          (ros::ros-error "replanning failed...")
          (ros::ros-error "cannot find a valid path to the goal-state")
          (return-from :replan-in-statenet-state nil)))))
  (:interrupt-wait (userdata)
    ;; skip interaction
    (if skip-interaction (return-from :interrupt-wait t))
    (let* ((current-sm-node (send sm :active-state))
           (duration (get current-sm-node :duration))
           (interrupt-p nil))
      ;; first time node, move to ask user input
      (if (null duration) (return-from :interrupt-wait t))
      ;; duration is long, move to ask user input
      (if (> duration duration-threshold) (return-from :interrupt-wait t))
      (ros::ros-info "Duration: ~A" duration)
      (setq interrupt-p
            (send stinteractor :ask-user-interrupt :timeout interrupt-timeout))
      (return-from :interrupt-wait interrupt-p)))
  (:ask-user-decision (userdata)
    ;; skip interaction
    (if skip-interaction (return-from :ask-user-decision :pass))
    (let* ((current-sm-node (send sm :active-state))
           (current-sm-state (read-from-string (send current-sm-node :name)))
           (goal-state (cdr (assoc :goal-state userdata))))
        (let ((user-decision
                (send stinteractor :ask-user-decision sm gr :timeout nil)))
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
          (interrupt-wait-node-name "(interrupt-wait)")
          (ask-node-name "(ask)")
          (replan-node-name "(replan)")
          (pass-node-name "(pass)")
          (abort-node-name "(abort)")
          (next-node-name "(next)"))
      (add-state-node sub-sm interrupt-wait-node-name
                      `(lambda-closure nil 0 0 (userdata)
                                       (send ,self :interrupt-wait userdata)))
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
      (send sub-sm :start-state interrupt-wait-node-name)
      (send sub-sm :goal-state '(t nil :teach :replan))
      (add-transition sub-sm interrupt-wait-node-name ask-node-name t)
      (add-transition sub-sm interrupt-wait-node-name pass-node-name nil)
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
  ;; TODO (knorth55): support multiple devices
  (:wait-ready-for-teach (userdata)
    (let* ((start-state (cdr (assoc :teach-start-state userdata)))
           (user-input-state nil))
      ; (ros::rate 10)
      ; (while  (not (equal start-state user-input-state))
      ;   (let* ((update-p (send device :update-device-orientation))
      ;          (contact-positions
      ;            (if update-p (send device :get-contact-positions)))
      ;          ;; TODO (knorth55): implement object-state-estimator
      ;          ; (object-state
      ;          ;   (send object-state-estimator :get-object-state contact-positions))
      ;          )
      ;     ;; TODO (knorth55): implement robot-stte-estimator
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
          (wait-ready-node-name "(wait-ready-for-teach)")
          (check-node-name "(check-teach-scene-state)")
          (confirm-node-name "(confirm-user-teach-action)")
          (search-node-name "(search-teach-action)")
          (update-node-name "(update-teach-action)")
          (ask-finish-node-name "(ask-finish)")
          (replan-node-name "(replan-in-statenet-teach)"))
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
  (:simplify-trained-collabo-path (collabo-path)
    (let ((simple-collabo-path (list (car collabo-path)))
          (prev-node-type)
          (prev-state-action nil)
          (start-stamp-list nil)
          (end-stamp-list nil)
          (current-n)
          (last-n (car (last collabo-path))))
      (dolist (n (cdr collabo-path))
        (let ((node-type (cadr (assoc :node-type n)))
              (state-action (cadr (assoc :state-action n)))
              (start-stamp (cadr (assoc :start-stamp n)))
              (end-stamp (cadr (assoc :end-stamp n))))
          (cond
            ;; initial
            ((and (null prev-node-type)
                  (null prev-state-action))
             (setq start-stamp-list (list start-stamp))
             (setq end-stamp-list (list end-stamp))
             (setq current-n
                   (mapcar
                     #'(lambda (x) (if (listp x) (copy-seq x) x)) n)))
            ;; continue
            ((and (null (equal n last-n))
                  (equal prev-node-type node-type)
                  (equal prev-state-action state-action))
             (setq start-stamp-list
                   (append start-stamp-list (list start-stamp)))
             (setq end-stamp-list
                   (append end-stamp-list (list end-stamp))))
            ;; last
            ((equal n last-n)
             (setq start-stamp-list
                   (append start-stamp-list (list start-stamp)))
             (setq end-stamp-list
                   (append end-stamp-list (list end-stamp)))
             (setf (cadr (assoc :start-stamp current-n))
                   (reduce #'min start-stamp-list))
             (setf (cadr (assoc :end-stamp current-n))
                   (reduce #'max end-stamp-list))
             (setq simple-collabo-path
                   (append simple-collabo-path (list current-n)))
             (setq start-stamp-list (list start-stamp))
             (setq end-stamp-list (list end-stamp))
             (setq current-n
                   (mapcar
                     #'(lambda (x) (if (listp x) (copy-seq x) x)) n)))
            ;; diff
            (t
             (setf (cadr (assoc :start-stamp current-n))
                   (reduce #'min start-stamp-list))
             (setf (cadr (assoc :end-stamp current-n))
                   (reduce #'max end-stamp-list))
             (setq simple-collabo-path
                   (append simple-collabo-path (list current-n)))
             (setq start-stamp-list (list start-stamp))
             (setq end-stamp-list (list end-stamp))
             (setq current-n
                   (mapcar
                     #'(lambda (x) (if (listp x) (copy-seq x) x)) n)))
            )
          (setq prev-node-type node-type)
          (setq prev-state-action state-action)))
      simple-collabo-path))
  (:simplify-trained-collabo-paths (collabo-paths)
    (mapcar #'(lambda (collabo-path)
                (send self :simplify-trained-collabo-path
                      collabo-path))
            collabo-paths))
  (:load-trained-collabo-paths (start-state goal-state)
    (let ((trained-collabo-paths
            (send self :simplify-trained-collabo-paths
                  (load-all-executed-collabo-path
                    :data-dir collabo-statenet-dir
                    :sort-predicate #'string>))))
      (if (equal start-state '(:init))
        ;; start-state is (:init)
        (remove
          nil
          (mapcar
            #'(lambda (collabo-path)
                (let* ((task-data (cdr (assoc :task collabo-path)))
                       (successp (cadr (assoc :result task-data)))
                       (sstate (cadr (assoc :start-state task-data)))
                       (gstate (cadr (assoc :goal-state task-data))))
                  (if (and (equal sstate start-state)
                           (equal gstate goal-state)
                           successp)
                    collabo-path)))
            trained-collabo-paths))
        (remove
          nil
          (mapcar
            #'(lambda (collabo-path)
                (let* ((task-data (cdr (assoc :task collabo-path)))
                       (successp (cadr (assoc :result task-data)))
                       (sstate (cadr (assoc :start-state task-data)))
                       (gstate (cadr (assoc :goal-state task-data))))
                  (if (and (equal sstate '(:init))
                           (> (length collabo-path) 3))
                    ;; if sstate is (:init) and state-state is not (:init)
                    (let ((task-info (car collabo-path))
                          (sn (elt collabo-path 3)))
                      (if (and (equal (cadr (assoc :state-action sn)) start-state)
                               (equal gstate goal-state)
                               successp)
                        (progn
                          (setf (cadr (assoc :start-state task-info))
                                (cadr (assoc :state-action sn)))
                          (setf (cadr (assoc :start-stamp task-info))
                                (cadr (assoc :start-stamp sn)))
                          (append (list task-info) (subseq collabo-path 3)))))
                    ;; if sstate is not (:init) and state-state is not (:init)
                    (if (and (equal sstate start-state)
                             (equal gstate goal-state)
                             successp)
                      collabo-path))))
            trained-collabo-paths)))))
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
          (if (send goal :goal :allow_update_start_state)
            (progn
              (ros::ros-warn "allow_update_start_state is set." )
              (ros::ros-warn "Updating start-state from ~A to ~A."
                             start-state current-state)
              (setq start-state (copy-seq current-state)))
            (progn
              (ros::ros-error
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
      ;; execute state machine
      ;; modified from (exec-state-machine *sm* nil :hz 1.0)
      (let* ((userdata (list (cons :start-state start-state)
                             (cons :goal-state goal-state)
                             (cons :user-decision nil)
                             (cons :replan-start-state nil)))
             (hz 10.0)
             (execute-stamp (get-iso-stamp))
             (trained-executed-collabo-paths
               (send self :load-trained-collabo-paths start-state goal-state))
             (executed-collabo-path
               (list
                 (list
                   :task
                   (list :start-state start-state)
                   (list :goal-state goal-state)
                   (list :result nil)
                   (list :start-stamp
                         (send (ros::time-
                                 (ros::time-now)
                                 start-stamp)
                               :to-sec))
                   (list :end-stamp nil)
                   (list :first-time
                         (if trained-executed-collabo-paths t nil))))))
        ;; TODO: support multi cube
        ; (if trained-executed-collabo-paths
        (if nil
          (progn
            (ros::ros-warn "This is a trained task.")
            (ros::ros-warn "Loading the trained path.")
            ;; generate sm from latest successful path
            (setq sm (send self :convert-collabo-path-to-state-machine
                           (car trained-executed-collabo-paths)))
            )
          ;; first time for this task, so planning.
          ;; generate smach machine
          (progn
            (ros::ros-warn "This is a new task.")
            (ros::ros-warn "Planning the path")
            (setq sm (send self :plan-state-machine start-state goal-state
                           :first-action nil :no-loop t))
            ))

        ;; check if sm is nil
        (if (null sm)
          (progn
            (send result :result :stamp (ros::time- (ros::time-now) start-stamp))
            (send result :result :success nil)
            (send result :result :final_state :state (format nil "~A" current-state))
            (send self :set-aborted result)
            (return-from :execute-statenet-cb nil)))

        (unix::usleep (round (* 1e6 0.5)))
        (send self :initialize-state-machine userdata)
        (while (ros::ok)
          (ros::rate hz)
          (send self :spin-once)
          (if ri (send ri :spin-once))
          ;; check if sm is not nil for replanning
          (if (null sm)
            (progn
              (send self :dump-collabo-statenet)
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
                (send self :dump-collabo-statenet)
                (send result :result :stamp (ros::time- (ros::time-now) start-stamp))
                (send result :result :success nil)
                (send result :result :final_state :state (format nil "~A" current-state))
                (send self :set-preempted result)
                (return-from :execute-statenet-cb nil)))
            (let ((execute-start-stamp nil)
                  (execute-end-stamp nil))

              ;; execute
              (setq execute-start-stamp (ros::time-now))
              (send sm :execute userdata :step -1)
              (setq execute-end-stamp (ros::time-now))

              ;; append executed state actions
              (let ((last-executed-state-action (car (last executed-collabo-path)))
                    (relative-execute-start-sec
                      (send (ros::time- execute-start-stamp start-stamp) :to-sec))
                    (relative-execute-end-sec
                      (send (ros::time- execute-end-stamp start-stamp) :to-sec)))
                (cond
                  ((equal node-type :state-node)
                   (setq executed-collabo-path
                         (append
                           executed-collabo-path
                           (list
                             (list
                               :state
                               (list :node-type node-type)
                               (list :state-action node-sym)
                               (list :start-stamp relative-execute-start-sec)
                               (list :end-stamp relative-execute-end-sec)))))
                   (send self :dump-executed-collabo-path
                         executed-collabo-path execute-stamp)
                   (ros::ros-info (format nil "current-state: ~A" current-state)))
                  ((equal node-type :action-node)
                   (setq executed-collabo-path
                         (append
                           executed-collabo-path
                           (list
                             (list
                               :action
                               (list :node-type node-type)
                               (list :state-action (car node-sym))
                               (list :start-stamp relative-execute-start-sec)
                               (list :end-stamp relative-execute-end-sec)))))
                   (send self :dump-executed-collabo-path
                         executed-collabo-path execute-stamp)
                   (ros::ros-info (format nil "action-function: ~A" (car node-sym))))
                  ((equal node-type :teach-node)
                   (ros::ros-info (format nil "teach-state: ~A" node-sym)))
                  (t nil))))
            ;; feedback
            (send feedback :feedback :stamp
                  (ros::time- (ros::time-now) start-stamp))
            (send self :publish-feedback feedback)
            (send insp :publish-structure)
            ;; sleep
            (ros::sleep)))
        ;; finish state-machine
        (ros::ros-info (format nil "current-state: ~A" current-state))
        (let* ((final-state (send (send sm :active-state) :name))
               (abortp (equal (read-from-string (subseq final-state 0 6)) :abort))
               (goalp (equal (read-from-string (subseq final-state 0 5)) :goal))
               (successp (and goalp (null abortp)))
               (end-stamp (ros::time- (ros::time-now) start-stamp)))
          (setf (cadr (assoc :result (cdr (assoc :task executed-collabo-path))))
                successp)
          (setf (cadr (assoc :end-stamp (cdr (assoc :task executed-collabo-path))))
                (send end-stamp :to-sec))
          (send self :dump-collabo-statenet)
          (send self :dump-executed-collabo-path
                executed-collabo-path execute-stamp)
          (send result :result :stamp end-stamp)
          (send result :result :success successp)
          (send result :result :final_state :state (format nil "~A" current-state))
          (send self :set-succeeded result)))
      t))
  (:current-state-publish-timer-cb (event) (send self :publish-current-state))
  (:publish-current-state ()
    (let ((msg (instance rmui_msgs::StatenetStateStamped :init)))
      (send msg :header :stamp (ros::time-now))
      (send msg :state :state (format nil "~A" current-state))
      (ros::publish "~current_state" msg)))
  (:visualize-update-cb (event)
    (if robotviewer (send robotviewer :draw-objects))
    (if ri (send ri :draw-objects)))
  (:spin-once ()
    (if device-groupnames
      (dolist (device-groupname device-groupnames)
        (ros::spin-once device-groupname)))
    (if server-groupname
      (ros::spin-once server-groupname))
    (send-super :spin-once))
  )


(defun statenet-server-main (&key (pdf nil) (rmui-names (list "rmui0"))
                                  (box-types :ctb) (skip-interaction nil))
  (if (not (listp rmui-names))
    (setq rmui-names (list rmui-names)))
  (set-rmui-parameters rmui-names)
  (scene-init :table-pos-y 0
              :n-box (length rmui-names)
              :box-types box-types)
  (motion-init :move-robot t
               :robot *robot*
               :ri *ri*
               :cubes *boxes*
               :vw *irtviewer*)

  ;; TODO (knorth55): change initial state from argument
  (set-statenet-reset-state (length rmui-names))
  (setq *server*
        (instance statenet-action-server :init
                  :reset-state *reset-state*
                  :device-namespaces *device-namespaces*
                  :device-groupnames *device-groupnames*
                  :duration-threshold *duration-threshold*
                  :interrupt-timeout *interrupt-timeout*
                  :prx-threshold *prx-threshold*
                  :prx-position-threshold *prx-position-threshold*
                  :device-frame-ids *device-frame-ids*
                  :device-world-frame-ids *device-world-frame-ids*
                  :solver-class *solver-class*
                  :robot *robot*
                  :ri *ri*
                  :boxes *boxes*
                  :robotviewer *irtviewer*
                  :use-torso *use-torso*
                  :move-robot t
                  :show-pdf pdf
                  :skip-interaction skip-interaction))
  (ros::rate 10)
  (while (ros::ok) (send *server* :spin-once)))


(provide :common-statenet-server "common-statenet-server.l")
