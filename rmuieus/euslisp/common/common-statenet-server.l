(require :rmui-device "package://rmuieus/euslisp/rmui/rmui-device.l")
(require :rmui-utils "package://rmuieus/euslisp/rmui/rmui-utils.l")
(require :common-graph "package://rmuieus/euslisp/common/common-graph.l")
(require :common-collabo-path "package://rmuieus/euslisp/common/common-collabo-path.l")
(require :common-statenet "package://rmuieus/euslisp/common/common-statenet.l")
(require :common-statenet-planner
         "package://rmuieus/euslisp/common/common-statenet-planner.l")
(require :common-statenet-interactor
         "package://rmuieus/euslisp/common/common-statenet-interactor.l")
(require :common-rmui-executor "package://rmuieus/euslisp/common/common-rmui-executor.l")
(require :common-rmui-planner "package://rmuieus/euslisp/common/common-rmui-planner.l")
(require :common-rmui-updater "package://rmuieus/euslisp/common/common-rmui-updater.l")

(ros::roseus-add-msgs "std_msgs")
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
          pilotviewer
          skip-interaction
          ;; statenet
          slvr
          sm
          insp
          statenet-gr
          stplanner
          stinteractor
          ;; rmui
          devices
          device-namespaces
          device-groupnames
          devplanner
          devexecutor
          devupdater
          ;; state
          initial-state
          current-state
          reset-state
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
               ((:motion-symbol-length msym-length) 5)
               ((:robot rbt) *robot*)
               ((:ri rbti) *ri*)
               ((:boxes bxs) *boxes*)
               ((:robotviewer rbtvw) *irtviewer*)
               ((:table-z tblz) *table-z*)
               ((:table-height tblh) *table-height*)
               ((:wall-front-limit wfl) *wall-front-limit*)
               ((:wall-left-limit wll) *wall-left-limit*)
               ((:pilot plt) *pilot*)
               ((:pilot-devices pltdevs) *pilot-devices*)
               ((:pilotviewer pltvw) *pilotviewer*)
               ((:pilot-table-z plttblz) *pilot-table-z*)
               ((:pilot-table-height plttblh) *pilot-table-height*)
               ((:pilot-wall-front-limit pwfl) *pilot-wall-front-limit*)
               ((:pilot-wall-left-limit pwll) *pilot-wall-left-limit*)
               ((:use-torso utrso) 0.001)
               ((:move-robot mvrbt) t)
               ((:skip-interaction skip-intr-p) nil))
    (let ((device-topic-namespace))
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
      (setq pilotviewer pltvw)
      (setq skip-interaction skip-intr-p)

      (setq device-topic-namespace
            (if (> (length device-namespaces) 1)
                "multi_rmui" (car device-namespaces)))

      ;; instances
      (setq statenet-gr
            (load-merged-statenet-graph-with-reset
              reset-state
              :data-dirs (list object-statenet-dir collabo-statenet-dir)))
      (setq slvr (instance solver-class :init))
      (setq stplanner
            (instance statenet-planner :init
                      :solver slvr
                      :state-func
                      (if skip-intr-p
                        `(lambda-closure nil 0 0 (userdata) t)
                        `(send ,self :get-statenet-state-submachine)
                        )
                      :teach-func
                      (if skip-intr-p
                        nil
                        `(send ,self :get-statenet-teach-submachine))
                      :robot rbt
                      :ri rbti
                      :boxes bxs
                      :robotviewer rbtvw
                      :use-torso utrso
                      :move-robot mvrbt
                      ))
      (setq stinteractor
            (instance statenet-interactor :init
                      :mode :ros-topic
                      :namespace server-namespace
                      :groupname server-groupname
                      :ri rbti
                      :use-speak-inform t
                      :use-speech-recognition t
                      :soundplay-topic-name "remotesound"
                      :speech-server-name "/speech_recognition"))

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
      (setq devplanner (instance rmui-planner :init
                                 :namespace device-topic-namespace))
      (setq devexecutor
            (instance rmui-executor :init
                      :namespace device-topic-namespace
                      :motion-symbol-length msym-length
                      :robot rbt
                      :ri rbti
                      :boxes bxs
                      :robotviewer rbtvw
                      :table-z tblz
                      :table-height tblh
                      :wall-front-limit wfl
                      :wall-left-limit wll
                      :pilot plt
                      :pilot-devices pltdevs
                      :pilotviewer pltvw
                      :pilot-table-z plttblz
                      :pilot-table-height plttblh
                      :pilot-wall-front-limit pwfl
                      :pilot-wall-left-limit pwll
                      :use-torso utrso
                      :move-robot mvrbt
                      ))
      (setq devupdater (instance rmui-updater :init
                                 :namespace device-topic-namespace
                                 :boxes bxs))

      (ros::advertise "~n_box" std_msgs::Int8 1 t)
      (unix::usleep (round 1e5))
      (ros::publish "~n_box" (instance std_msgs::Int8 :data (length bxs)))

      ;; state
      (setq initial-state '(:init))
      (setq current-state '(:init))
      (ros::advertise "~current_state" rmui_msgs::StatenetStateStamped 1 t)
      (send self :init-timer)
      (ros::ros-info "init finished")
      (send self :dump-collabo-statenet)
      (if show-pdf (send self :show-statenet-pdf)))
    self)
  (:statenet-graph () statenet-gr)
  (:solver () slvr)
  (:statenet-planner () stplanner)
  (:state-machine () sm)
  (:add-action-state-in-statenet-graph (action prev-scene-states scene-states)
    (if statenet-gr
      (add-action-state-in-graph statenet-gr action prev-scene-states scene-states)))
  (:dump-collabo-statenet ()
    (if (null dump-statenet-dir)
      (setq dump-statenet-dir
            (create-dump-dir collabo-statenet-dir)))
    (dump-statenet-graph statenet-gr :data-dir collabo-statenet-dir))
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
    (if statenet-gr
      (progn
        (send statenet-gr :write-to-pdf "rmui_statenet.pdf")
        (piped-fork "xdg-open rmui_statenet.pdf"))
      (ros::ros-error "statenet is not loaded.")))
  (:show-smach-pdf ()
    (send sm :current-draw-mode)
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
  (:plan-state-machine-with-trained-paths
    (start-state goal-state &key (first-action nil) (no-loop nil))
    (let* ((trained-data
            (send self :load-trained-collabo-paths start-state goal-state))
           (trained-executed-collabo-paths (car trained-data))
           (trained-dump-dirnames (cdr trained-data))
           (trained-index 0)
           (tmp-sm nil))
      (if trained-executed-collabo-paths
        (progn
          (ros::ros-warn "This is a trained task.")
          (ros::ros-warn "Loading the trained path.")
          (ros::ros-warn "Trained data: ~A"
                         (elt trained-dump-dirnames trained-index))
          ;; generate sm from latest successful path
          (setq tmp-sm
                (send self :convert-collabo-path-to-state-machine
                      (elt trained-executed-collabo-paths trained-index)
                      :no-loop nil)))
        ;; first time for this task, so planning.
        ;; generate smach machine
        (progn
          (ros::ros-warn "This is a new task.")
          (ros::ros-warn "Planning the path")
          (setq tmp-sm
                (send self :plan-state-machine start-state goal-state
                      :first-action first-action :no-loop no-loop))))
      (if tmp-sm (list tmp-sm trained-executed-collabo-paths))))
  (:plan-state-machine (start-state goal-state &key (first-action nil) (no-loop nil))
    (if statenet-gr
      (send stplanner :generate-smach-from-state
            statenet-gr start-state goal-state
            :first-action first-action
            :no-loop no-loop)))
  (:convert-collabo-path-to-state-machine (collabo-path &key (no-loop nil))
    (send stplanner :convert-collabo-path-to-state-machine collabo-path :no-loop no-loop))
  (:next-in-state (userdata)
    (let* ((first-action (cdr (cdr (assoc :user-decision userdata)))))
      (return-from :next-in-state (format nil "~A" first-action))))
  (:replan-in-state (userdata)
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
          (return-from :replan-in-state t))
        (progn
          (ros::ros-error "replanning failed...")
          (ros::ros-error "cannot find a valid path to the goal-state")
          (return-from :replan-in-state nil)))))
  (:interrupt-wait (userdata)
    (let* ((current-sm-node (send sm :active-state))
           (duration (send current-sm-node :get :duration))
           (node-sym (read-from-string (send current-sm-node :name)))
           (state-node-type
             (send current-sm-node :get :state-node-type))
           (interrupt-p nil))
      ;; skip interruption at the first state node
      (if (and (equal state-node-type :first-state-node)
               (equal node-sym '(:init)))
        (return-from :interrupt-wait nil))
      ;; skip interruption at the last state node
      (if (equal state-node-type :last-state-node)
        (return-from :interrupt-wait nil))
      ;; first time node, move to ask user input
      (if (and duration-threshold (null duration))
        (return-from :interrupt-wait t))
      ;; duration is long, move to ask user input
      (if (and duration-threshold duration
               (> duration duration-threshold))
        (progn
         (ros::ros-info "Duration: ~A" duration)
         (return-from :interrupt-wait t)))
      (send stinteractor :reset-interactor)
      (setq interrupt-p
            (send stinteractor :ask-user-interrupt :timeout interrupt-timeout :speak-wait t))
      (return-from :interrupt-wait interrupt-p)))
  (:ask-user-state-decision (userdata)
    (send stinteractor :reset-interactor)
    (let* ((current-sm-node (send sm :active-state))
           (current-sm-state (read-from-string (send current-sm-node :name)))
           (goal-state (cdr (assoc :goal-state userdata))))
        (let ((user-decision
                (send stinteractor :ask-user-state-decision sm statenet-gr :timeout nil)))
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
          (return-from :ask-user-state-decision (car user-decision)))))
  (:get-statenet-state-submachine ()
    (let ((sub-sm (instance state-machine :init))
          (interrupt-wait-node-name "(interrupt-wait)")
          (ask-node-name "(ask-user-state-decision)")
          (replan-node-name "(replan-in-state)")
          (next-node-name "(next-in-state)"))
      (add-state-node sub-sm interrupt-wait-node-name
                      `(lambda-closure nil 0 0 (userdata)
                                       (send ,self :interrupt-wait userdata)))
      (add-state-node sub-sm ask-node-name
                      `(lambda-closure nil 0 0 (userdata)
                                       (send ,self :ask-user-state-decision userdata)))
      (add-state-node sub-sm replan-node-name
                      `(lambda-closure nil 0 0 (userdata)
                                       (send ,self :replan-in-state userdata)))
      (add-state-node sub-sm next-node-name
                      `(lambda-closure nil 0 0 (userdata)
                                       (send ,self :next-in-state userdata)))
      (send sub-sm :start-state interrupt-wait-node-name)
      (send sub-sm :goal-state '(t nil :finish :teach :replan))
      (add-transition sub-sm interrupt-wait-node-name ask-node-name t)
      (add-transition sub-sm interrupt-wait-node-name t nil)
      (add-transition sub-sm ask-node-name replan-node-name :replan)
      (add-transition sub-sm ask-node-name t :pass)
      (add-transition sub-sm ask-node-name nil :abort)
      (add-transition sub-sm ask-node-name next-node-name :next)
      (add-transition sub-sm ask-node-name :teach :teach)
      (add-transition sub-sm ask-node-name :finish :finish)
      (add-transition sub-sm ask-node-name ask-node-name nil)
      (add-transition sub-sm replan-node-name :replan t)
      (add-transition sub-sm replan-node-name ask-node-name nil)
      (send sub-sm :put :next-node-name next-node-name)
      ; (add-transition sub-sm next-node-name t t)
      sub-sm))
  (:ask-user-ready-device (userdata)
    (send stinteractor :reset-interactor)
    (let ((ready-device-p (send stinteractor :ask-user-ready-device)))
      (if ready-device-p
        (ros::ros-info "Start teaching")
        (ros::ros-warn "Abort teaching, go back to state"))
      ready-device-p))
  (:initialize-device (userdata)
    (let ((teach-start-state (cdr (assoc :teach-start-state userdata))))
      (ros::ros-info "Initializing teach device")
      (send devexecutor :reset-executor)
      (send devupdater :reset-updater
            :initial-scene-states (copy-tree teach-start-state))
      (setf (cdr (assoc :teach-state userdata))
            (copy-tree teach-start-state))
      (setf (cdr (assoc :teach-motion-symbols userdata)) nil)
      (setf (cdr (assoc :teach-executed-action userdata)) nil)
      t))
  (:wait-device-user-input (userdata)
    (let ((from-stamp (ros::time-now))
          (prev-scene-states (cdr (assoc :teach-state userdata))))
      (send stinteractor :reset-interactor)
      (send stinteractor :ros-speak-info
            "Waiting for device input or user interruption.")
      (send stinteractor :start-speech-recognition)
      (ros::rate 10)
      (while (ros::ok)
        (ros::sleep)
        (send insp :publish-structure)
        (if device-groupnames
          (dolist (device-groupname device-groupnames)
            (ros::spin-once device-groupname)))
        (if server-groupname
          (ros::spin-once server-groupname))
        (if (send stinteractor :ros-wait-user-interrupt
                  nil :timeout 0.05 :from-stamp from-stamp
                  :toggle-server nil)
          (progn
            (send stinteractor :stop-speech-recognition)
            (return-from :wait-device-user-input nil)))
        (let* ((update-p
                 (eval (append '(and)
                                (send-all devices :update-device-orientation))))
               (device-states
                 (if update-p (send-all devices :get-device-state)))
               (contact-positions
                 (if device-states
                   (mapcar
                     #'(lambda (device-state)
                         (cadr (assoc :contact-positions device-state)))
                   device-states)))
               (motion-symbols
                 (if contact-positions
                   (send devplanner :get-motion-symbols
                         contact-positions
                         :scene-states prev-scene-states)))
               (update-motion-symbol-queues
                 (if motion-symbols
                   (send devexecutor :update-motion-symbol-queues motion-symbols)))
               (checked-motion-symbols
                 (if update-motion-symbol-queues
                   (send devexecutor :check-motion-symbol-queues motion-symbols))))
          (ros::ros-info
            (format nil "motion-symbols         : ~A" motion-symbols))
          (ros::ros-info
            (format nil "checked-motion-symbols : ~A" checked-motion-symbols))
          (if (not (every #'null checked-motion-symbols))
            (progn
              (setf (cdr (assoc :teach-motion-symbols userdata)) checked-motion-symbols)
              (send stinteractor :stop-speech-recognition)
              (return-from :wait-device-user-input t)))))))
  (:execute-device-user-input (userdata)
    (let* ((prev-scene-states (cdr (assoc :teach-state userdata)))
           (checked-motion-symbols (cdr (assoc :teach-motion-symbols userdata)))
           (executed-symbols-and-actions
             (send devexecutor :execute-checked-motion-symbols
                   checked-motion-symbols
                   :scene-states prev-scene-states))
           (update-scene-p
             (if executed-symbols-and-actions
               (send devupdater :update-scene-states executed-symbols-and-actions))))
      (if robotviewer (send robotviewer :draw-objects))
      (if pilotviewer (send pilotviewer :draw-objects))
      (if ri (send ri :draw-objects))
      ;; when motion is executed
      (return-from :execute-device-user-input (if update-scene-p t nil))))
  (:update-from-teach-execution (userdata)
    (let* ((prev-scene-states (cdr (assoc :teach-state userdata)))
           (scene-states (send devupdater :last-scene-states))
           (executed-action (send devupdater :last-executed-action)))
      (ros::ros-warn
        (format nil "executed-action : ~A" executed-action))
      (send self :add-action-state-in-statenet-graph
            executed-action prev-scene-states scene-states)
      (send self :dump-collabo-statenet)
      (setf (cdr (assoc :teach-state userdata)) scene-states)
      (setf (cdr (assoc :teach-executed-action userdata)) executed-action)
      t))
  (:ask-user-teach-decision (userdata)
    (send stinteractor :reset-interactor)
    (let ((user-decision (send stinteractor :ask-user-teach-decision)))
      (if user-decision
        (progn
          (ros::ros-warn "Finishing teaching")
          (setf (cdr (assoc :teach-motion-symbols userdata)) nil)
          (setf (cdr (assoc :teach-executed-action userdata)) nil)
          (if (equal user-decision :finish)
            (ros::ros-warn "Finishing the task")))
        (ros::ros-warn "Continue teaching"))
      user-decision))
  (:replan-in-teach (userdata)
    (ros::ros-warn "replanning...")
    (let* ((start-state (cdr (assoc :teach-state userdata)))
           (goal-state (cdr (assoc :goal-state userdata)))
           (replan-sm
             (car (send self :plan-state-machine-with-trained-paths
                        start-state goal-state :first-action nil :no-loop t))))
      (if replan-sm
        (progn
          (ros::ros-info "replanning succeeded")
          (setq sm replan-sm)
          (send self :set-start-state :state start-state)
          (send self :initialize-state-machine userdata)
          (send self :dump-collabo-statenet)
          (return-from :replan-in-teach t))
        (progn
          (ros::ros-error "replanning failed...")
          (ros::ros-error "cannot find a valid path to the goal-state")
          (setf (cdr (assoc :teach-start-state userdata))
                (copy-tree (cdr (assoc :teach-state userdata))))
          (ros::ros-error "go back to teach state")
          (return-from :replan-in-teach nil)))))
  (:reset-device (userdata)
    (setf (cdr (assoc :teach-state userdata)) nil)
    (setf (cdr (assoc :teach-motion-symbols userdata)) nil)
    (setf (cdr (assoc :teach-executed-action userdata)) nil)
    t)
  (:get-statenet-teach-submachine ()
    (let ((sub-sm (instance state-machine :init))
          (ask-ready-node-name "(ask-user-ready-device)")
          (init-device-node-name "(initialize-device)")
          (wait-device-node-name "(wait-device-user-input)")
          (execute-node-name "(execute-device-user-input)")
          (update-teach-node-name "(update-from-teach-execution)")
          (ask-finish-node-name "(ask-user-teach-decision)")
          (replan-node-name "(replan-in-teach)")
          (reset-device-node-name "(reset-device)"))
      (add-teach-node sub-sm ask-ready-node-name
                      `(lambda-closure nil 0 0 (userdata)
                                       (send ,self :ask-user-ready-device userdata)))
      (add-teach-node sub-sm init-device-node-name
                      `(lambda-closure nil 0 0 (userdata)
                                       (send ,self :initialize-device userdata)))
      (add-teach-node sub-sm wait-device-node-name
                      `(lambda-closure nil 0 0 (userdata)
                                       (send ,self :wait-device-user-input userdata)))
      (add-teach-node sub-sm execute-node-name
                      `(lambda-closure nil 0 0 (userdata)
                                       (send ,self :execute-device-user-input userdata)))
      (add-teach-node sub-sm update-teach-node-name
                      `(lambda-closure nil 0 0 (userdata)
                                       (send ,self :update-from-teach-execution userdata))
                      :update-teach-node)
      (add-teach-node sub-sm ask-finish-node-name
                      `(lambda-closure nil 0 0 (userdata)
                                       (send ,self :ask-user-teach-decision userdata)))
      (add-teach-node sub-sm replan-node-name
                      `(lambda-closure nil 0 0 (userdata)
                                       (send ,self :replan-in-teach userdata)))
      (add-teach-node sub-sm reset-device-node-name
                      `(lambda-closure nil 0 0 (userdata)
                                       (send ,self :reset-device userdata)))
      (send sub-sm :start-state ask-ready-node-name)
      (send sub-sm :goal-state '(t nil :finish))
      (add-transition sub-sm ask-ready-node-name init-device-node-name t)
      (add-transition sub-sm ask-ready-node-name reset-device-node-name nil)
      (add-transition sub-sm init-device-node-name wait-device-node-name t)
      (add-transition sub-sm wait-device-node-name execute-node-name t)
      (add-transition sub-sm wait-device-node-name ask-finish-node-name nil)
      (add-transition sub-sm execute-node-name update-teach-node-name t)
      (add-transition sub-sm execute-node-name ask-finish-node-name nil)
      (add-transition sub-sm update-teach-node-name ask-finish-node-name t)
      (add-transition sub-sm ask-finish-node-name replan-node-name t)
      (add-transition sub-sm ask-finish-node-name replan-node-name :replan)
      (add-transition sub-sm ask-finish-node-name :finish :finish)
      (add-transition sub-sm ask-finish-node-name wait-device-node-name nil)
      (add-transition sub-sm replan-node-name reset-device-node-name t)
      (add-transition sub-sm replan-node-name ask-ready-node-name nil)
      (add-transition sub-sm reset-device-node-name t t)
      sub-sm))
  (:simplify-trained-collabo-paths (collabo-paths)
    (mapcar #'(lambda (collabo-path)
                (simplify-collabo-path collabo-path))
            collabo-paths))
  (:load-trained-collabo-paths (start-state goal-state)
    (let ((trained-collabo-paths
            (send self :simplify-trained-collabo-paths
                  (load-all-executed-collabo-path
                    :data-dir collabo-statenet-dir
                    :sort-predicate #'string>)))
          (trained-dump-dirnames
            (get-all-executed-collabo-path-dump-dirnames
              :data-dir collabo-statenet-dir
              :sort-predicate #'string>)))
      (setq trained-collabo-paths
            (if (equal start-state '(:init))
              ;; start-state is (:init)
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
                trained-collabo-paths)
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
                trained-collabo-paths)))
      (dotimes (i (length trained-dump-dirnames))
        (if (not (elt trained-collabo-paths i))
          (setf (elt trained-dump-dirnames i) nil)))
      (setq trained-dump-dirnames (remove nil trained-dump-dirnames))
      (setq trained-collabo-paths (remove nil trained-collabo-paths))
      (cons trained-collabo-paths trained-dump-dirnames)))
  (:append-executed-collabo-path
    (executed-collabo-path &key (node-type) (state-action) (start-stamp) (end-stamp))
    (append
      executed-collabo-path
      (list
        (list
          (cond
            ((equal node-type :action-node) :action)
            ((equal node-type :state-node) :state)
            (t node-type))
          (list :node-type node-type)
          (list :state-action (copy-tree state-action))
          (list :start-stamp start-stamp)
          (list :end-stamp end-stamp)))))
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
              (setq start-state (copy-tree current-state)))
            (progn
              (send stinteractor :ros-speak-error "I aborted the task execution.")
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
          (send stinteractor :ros-speak-warn "I skipped the task execution.")
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
      (let* ((userdata
               (list
                 (cons :start-state start-state)
                 (cons :goal-state goal-state)
                 (cons :user-decision nil)
                 (cons :replan-start-state nil)
                 (cons :teach-start-state nil)
                 (cons :teach-state nil)
                 (cons :teach-motion-symbols nil)
                 (cons :teach-executed-action nil)))
             (hz 10.0)
             (planning-result
               (if (null (equal goal-state '(:teach)))
                 (send self :plan-state-machine-with-trained-paths
                       start-state goal-state :first-action nil :no-loop t)))
             (trained-executed-collabo-paths (cadr planning-result))
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
                         (if trained-executed-collabo-paths t nil)))))
             (execute-stamp (get-iso-stamp)))
        (setq sm (car planning-result))
        ;; check if sm is nil
        (if (null sm)
          (if skip-interaction
            (progn
              (send stinteractor :ros-speak-error "I aborted the task execution.")
              (send result :result :stamp (ros::time- (ros::time-now) start-stamp))
              (send result :result :success nil)
              (send result :result :final_state :state (format nil "~A" current-state))
              (send self :set-aborted result)
              (return-from :execute-statenet-cb nil))
            (progn
              (ros::ros-warn "Planning failed... Start teaching...")
              (setf (cdr (assoc :teach-start-state userdata))
                    (if (equal start-state '(:init))
                      reset-state start-state))
              (setq sm
                    (send stplanner :generate-teach-smach-from-state
                          start-state :reset-state reset-state)))))
        ;; start smach state machine
        (unix::usleep (round (* 1e6 0.5)))
        (send self :initialize-state-machine userdata)
        (while (ros::ok)
          (ros::rate hz)
          (send self :spin-once)
          (if ri (send ri :spin-once))
          ;; check if sm is not nil for replanning
          (if (null sm)
            (progn
              (send stinteractor :ros-speak-error "I aborted the task execution.")
              (send self :dump-collabo-statenet)
              (send result :result :stamp
                    (ros::time- (ros::time-now) start-stamp))
              (send result :result :success nil)
              (send result :result :final_state :state
                    (format nil "~A" current-state))
              (send self :set-aborted result)
              (return-from :execute-statenet-cb nil)))
          (let* ((top-active-state (send sm :active-state))
                 (bottom-active-state (car (send sm :bottom-active-state)))
                 (top-node-type
                   (send top-active-state :get :node-type))
                 (bottom-node-type
                   (send bottom-active-state :get :node-type))
                 (node-type
                   (prog1
                     top-node-type
                     (if (null (equal top-node-type bottom-node-type))
                       (ros::ros-warn
                         (format nil "node-type are different in submachine: ~A != ~A"
                                 top-node-type bottom-node-type)))))
                 (teach-node-type
                   (if (equal node-type :teach-node)
                     (send bottom-active-state :get :teach-node-type)))
                 (node-name (send top-active-state :name))
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
               (setq current-state
                     (if (cdr (assoc :teach-state userdata))
                       (cdr (assoc :teach-state userdata))
                       (cdr (assoc :teach-start-state userdata))))
               (ros::ros-info (format nil "current-state: ~A" current-state)))
              (t nil))
            (send insp :publish-status userdata)
            (when (send sm :goal-reached) (return))
            ;; cancel
            (if (send self :is-preempt-requested)
              (progn
                (send stinteractor :ros-speak-warn "I stopped the task execution.")
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
              (let ((relative-execute-start-sec
                      (send (ros::time- execute-start-stamp start-stamp) :to-sec))
                    (relative-execute-end-sec
                      (send (ros::time- execute-end-stamp start-stamp) :to-sec)))
                (if (or (equal node-type :state-node)
                        (equal node-type :action-node)
                        (equal node-type :teach-node))
                  (let ((ntype nil)
                        (saction nil))
                    (cond
                      ((equal node-type :state-node)
                       (setq ntype :state-node)
                       (setq saction current-state))
                      ((equal node-type :action-node)
                       (setq ntype :action-node)
                       (setq saction (car node-sym)))
                      ((and (equal node-type :teach-node)
                            (equal teach-node-type :update-teach-node))
                       (setq ntype :action-node)
                       (setq saction (cdr (assoc :teach-executed-action userdata))))
                      ((equal node-type :teach-node)
                       (setq ntype :state-node)
                       (setq saction current-state))
                      (t nil))
                    (if (and ntype saction)
                      (progn
                        (setq executed-collabo-path
                              (send self :append-executed-collabo-path
                                    executed-collabo-path
                                    :node-type ntype
                                    :state-action saction
                                    :start-stamp relative-execute-start-sec
                                    :end-stamp relative-execute-end-sec))
                        (send self :dump-executed-collabo-path
                              executed-collabo-path execute-stamp)
                        ))))))
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
               (abortp (equal (read-from-string (subseq final-state 0 8)) '(:abort)))
               (goalp (equal (read-from-string (subseq final-state 0 7)) '(:goal)))
               (successp (and goalp (null abortp)))
               (end-stamp (ros::time- (ros::time-now) start-stamp))
               (executed-goal-state
                 (cadr (assoc :goal-state (cdr (assoc :task executed-collabo-path))))))
          (setf (cadr (assoc :result (cdr (assoc :task executed-collabo-path))))
                successp)
          (setf (cadr (assoc :end-stamp (cdr (assoc :task executed-collabo-path))))
                (send end-stamp :to-sec))
          (if (null (equal executed-goal-state current-state))
            (progn
              (ros::ros-warn "Goal state updated.")
              (setf (cadr (assoc :goal-state (cdr (assoc :task executed-collabo-path))))
                    current-state)))
          (if successp
            (send stinteractor :ros-speak-info "I completed the task execution.")
            (send stinteractor :ros-speak-info "I failed the task execution."))
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
                                  (box-types :ctb) (wall-p nil) (align-axis :y)
                                  (skip-interaction nil))
  (if (not (listp rmui-names))
    (setq rmui-names (list rmui-names)))
  (set-rmui-parameters rmui-names)
  (scene-init :table-pos-y 0
              :n-box (length rmui-names)
              :box-types box-types
              :wall-p wall-p
              :align-axis align-axis)
  (motion-init :move-robot t
               :robot *robot*
               :ri *ri*
               :cubes *boxes*
               :vw *irtviewer*)
  (pilot-scene-init :table-pos-y 0
                    :n-device (length rmui-names)
                    :wall-p wall-p
                    :align-axis align-axis)
  (motion-init :move-robot nil
               :robot *pilot*
               :ri nil
               :cubes *pilot-devices*
               :vw *pilotviewer*)

  ;; TODO (knorth55): change initial state from argument
  (set-statenet-reset-state (length rmui-names))
  (setq *server*
        (instance statenet-action-server :init
                  :reset-state *reset-state*
                  :device-namespaces *device-namespaces*
                  :device-groupnames *device-groupnames*
                  :object-statenet-dir *training-object-statenet-path*
                  :collabo-statenet-dir *training-collabo-statenet-path*
                  :duration-threshold *duration-threshold*
                  :interrupt-timeout *interrupt-timeout*
                  :prx-threshold *prx-threshold*
                  :prx-position-threshold *prx-position-threshold*
                  :device-frame-ids *device-frame-ids*
                  :device-world-frame-ids *device-world-frame-ids*
                  :solver-class *solver-class*
                  :motion-symbol-length *motion-symbol-length*
                  :robot *robot*
                  :ri *ri*
                  :boxes *boxes*
                  :robotviewer *irtviewer*
                  :table-z *table-z*
                  :table-height *table-height*
                  :pilot *pilot*
                  :pilot-devices *pilot-devices*
                  :pilotviewer *pilotviewer*
                  :pilot-table-z *pilot-table-z*
                  :pilot-table-height *pilot-table-height*
                  :use-torso *use-torso*
                  :move-robot t
                  :show-pdf pdf
                  :skip-interaction skip-interaction))
  (ros::rate 10)
  (while (ros::ok) (send *server* :spin-once)))


(provide :common-statenet-server "common-statenet-server.l")
