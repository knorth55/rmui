(require :common-statenet "package://rmuieus/euslisp/common/common-statenet.l")
(require :common-utils "package://rmuieus/euslisp/common/common-utils.l")

(ros::roseus-add-msgs "speech_recognition_msgs")
(ros::roseus-add-srvs "speech_recognition_msgs")
(ros::roseus-add-msgs "std_msgs")


(defclass statenet-interactor
  :super propertied-object
  :slots (mode
          namespace
          groupname
          ri
          use-speak-inform
          use-speech-recognition
          speech-server-name
          soundplay-topic-name
          ask-sentence-topic-name
          response
          response-stamp)
  )

(defmethod statenet-interactor
  (:init (&key ((:mode md) :ros-topic) ((:namespace ns) nil) ((:groupname gpn))
               ((:ri rbti) *ri*)
               ((:use-speak-inform use-speak) t)
               ((:use-speech-recognition use-speech) t)
               ((:soundplay-topic-name sptname) "remotesound")
               ((:speech-server-name ssrvn) "/speech_recognition"))
    (setq mode md)
    (setq namespace ns)
    (setq groupname gpn)
    (setq ri rbti)
    (setq use-speak-inform use-speak)
    (setq use-speech-recognition use-speech)
    (setq soundplay-topic-name sptname)
    (setq speech-server-name ssrvn)

    (setq ask-sentence-topic-name
          (if namespace (format nil "~A/user_ask_sentence" namespace) "/user_ask_sentence"))
    (ros::advertise ask-sentence-topic-name std_msgs::String 1)
    (if groupname (ros::create-nodehandle groupname))
    (if groupname
      (ros::subscribe
        (if namespace (format nil "~A/user_response" namespace) "/user_response")
        std_msgs::String #'send self :string-cb
        :groupname groupname)
      (ros::subscribe
        (if namespace (format nil "~A/user_response" namespace) "/user_response")
        std_msgs::String #'send self :string-cb))
    self)
  (:reset-interactor ()
    (setq response nil)
    (setq response-stamp nil)
    t)
  (:string-cb (msg)
    (if msg
      (progn
        (setq response (send msg :data))
        (setq response-stamp (ros::time-now)))))
  (:start-speech-recognition ()
    (let ((start-service-name
           (format nil "~A/start" speech-server-name)))
      (start-speech-recognition start-service-name)))
  (:stop-speech-recognition ()
    (let ((stop-service-name
           (format nil "~A/stop" speech-server-name)))
      (stop-speech-recognition stop-service-name)))
  (:get-user-response (&key (timeout nil) (from-stamp nil) (toggle-server t))
    (cond
      ;; timeout is not valid for read-line
      ((equal mode :read-line)
       (read-line t))
      ((equal mode :ros-topic)
       (let ((start-stamp (ros::time-now)))
         (if (and use-speech-recognition toggle-server)
           (send self :start-speech-recognition))
         (setq from-stamp
               (if from-stamp from-stamp (ros::time-now)))
         (ros::rate 50)
         (while (or (null response)
                    (null response-stamp)
                    (ros::time> from-stamp response-stamp))
           (if (and timeout
                    (> (send
                         (ros::time- (ros::time-now) start-stamp)
                         :to-nsec)
                       (* timeout 1e9)))
             (progn
               (if toggle-server
                 (send self :stop-speech-recognition))
               (return-from :get-user-response :timeout)))
           (ros::spin-once groupname))
         (if (and use-speech-recognition toggle-server)
           (send self :stop-speech-recognition))
         (return-from :get-user-response (copy-seq response))))
      ;; timeout is not supported in service
      ((equal mode :ros-service)
       (tagbody retry-speech-service-call
         (let ((speech-req
                 (instance speech_recognition_msgs::SpeechRecognitionRequest :init))
               (speech-res nil))
           (ros::ros-info "Waiting for user speech input")
           (send speech-req :quiet t)
           (setq speech-res
                 (ros::service-call speech-server-name speech-req))
           (setq response
                 (if speech-res (car (send speech-res :result :transcript))))
           (setq response-stamp
                 (if response (ros::time-now)))
           (if response
             (return-from :get-user-response (copy-seq response))
             (go retry-speech-service-call)))))
      (t nil)))
  (:ros-speak-base (sentence &key (wait nil))
    (let ((ask-string-msg
            (instance std_msgs::String :init :data sentence)))
      (ros::publish ask-sentence-topic-name ask-string-msg)
      (send ri :speak sentence
            :topic-name soundplay-topic-name :wait wait)))
  (:ros-speak-info (sentence &key (wait nil))
    (ros::ros-info sentence)
    (if use-speak-inform
      (send self :ros-speak-base sentence :wait wait)))
  (:ros-speak-warn (sentence &key (wait nil))
    (ros::ros-warn sentence)
    (if use-speak-inform
      (send self :ros-speak-base sentence :wait wait)))
  (:ros-speak-error (sentence &key (wait nil))
    (ros::ros-error sentence)
    (if use-speak-inform
      (send self :ros-speak-base sentence :wait wait)))
  (:ros-wait-user-interrupt
    (ask-sentence &key (timeout nil) (from-stamp nil) (toggle-server t) (speak-wait nil))
    (let ((user-response))
      (if ask-sentence
        (send self :ros-speak-info ask-sentence :wait speak-wait))
      ; (finish-output t)
      (send self :reset-interactor)
      (setq user-response
            (send self :get-user-response
                  :timeout timeout :from-stamp from-stamp
                  :toggle-server toggle-server))
      (return-from :ros-wait-user-interrupt
                   (null (equal user-response :timeout)))))
  (:ros-yes-or-no-p
    (ask-sentence &key (use-default t) (default t)
                       (timeout nil) (from-stamp nil) (toggle-server t) (speak-wait nil))
    (let ((user-response))
      (tagbody retry-yes-or-no
        (if ask-sentence
          (send self :ros-speak-info ask-sentence :wait speak-wait))
        ; (finish-output t)
        (send self :reset-interactor)
        (setq user-response
              (string-upcase
                (send self :get-user-response
                      :timeout timeout :from-stamp from-stamp
                      :toggle-server toggle-server)))
        (cond
          ((or (string= user-response "YES")
               (string= user-response "Y"))
           (return-from :ros-yes-or-no-p t))
          ((or (string= user-response "NO")
               (string= user-response "N"))
           (return-from :ros-yes-or-no-p nil))
          ((or (string= user-response "")
               (equal user-response :timeout))
           (if use-default
             (return-from :ros-yes-or-no-p default)
             (progn
               (send self :ros-speak-error "Please answer with yes or no.")
               (go retry-yes-or-no))))
          (t
           (send self :ros-speak-error "Please answer with yes or no.")
           (go retry-yes-or-no))))))
  (:ros-index-choose
    (ask-sentence min-index max-index &key (use-default t) (default-index 0)
                                           (timeout nil) (from-stamp nil)
                                           (toggle-server t) (speak-wait nil))
    (let ((user-response))
      (tagbody retry-index-choose
        (if ask-sentence
          (send self :ros-speak-info ask-sentence :wait speak-wait))
        ; (finish-output t)
        (send self :reset-interactor)
        (setq user-response
              (string-upcase
                (send self :get-user-response
                      :timeout timeout :from-stamp from-stamp
                      :toggle-server toggle-server)))
        (cond
          ((or (string= user-response "TEACHING")
               (string= user-response "TEACH")
               (string= user-response "NO")
               (string= user-response "N"))
            (return-from :ros-index-choose -1))
          ((or (string= user-response "")
               (string= user-response "YES")
               (string= user-response "Y")
               (string= user-response "NEXT"))
            (return-from :ros-index-choose default-index))
          ((equal user-response :timeout)
            (if use-default
              (return-from :ros-index-choose default-index)
              (progn
                (send self :ros-speak-error
                      (format nil "Please answer with a valid number from ~A to ~A."
                              min-index max-index))
                (go retry-index-choose))))
          (t
            (let ((resp-num (if user-response (read-from-string user-response))))
              (if (and (integerp resp-num)
                       (>= resp-num min-index)
                       (<= resp-num max-index))
                (return-from :ros-index-choose resp-num)
                (progn
                  (send self :ros-speak-error
                        (format nil "Please answer with a valid number from ~A to ~A, or yes."
                                min-index max-index))
                  (go retry-index-choose)))))))))
  (:ask-user-index-choose
    (actions &key (use-default t) (default-index 0) (timeout nil) (speak-wait nil))
    (let ((info-sentence
            (format nil "Default action: (~A) ~A"
                    default-index (elt actions default-index)))
          (ask-sentence
            (format nil "Which action do you choose? Default is ~A or yes." default-index)))
      (if use-default (ros::ros-info info-sentence))
      (send self :ros-index-choose ask-sentence
            -1 (- (length actions) 1)
            :use-default use-default
            :default-index default-index
            :timeout timeout
            :speak-wait speak-wait)))
  (:ask-user-interrupt (&key (timeout nil) (speak-wait nil))
    (let ((ask-sentence
            (format nil "Waiting for user interuption in ~A seconds." timeout)))
      (send self :ros-wait-user-interrupt ask-sentence
            :timeout timeout :speak-wait speak-wait)))
  (:ask-user-start-teach (&key (timeout nil) (speak-wait nil))
    (let ((ask-sentence
            (format nil "Do you want to start teaching new actions?")))
      (send self :ros-yes-or-no-p ask-sentence
            :use-default nil :timeout timeout
            :speak-wait speak-wait)))
  (:ask-user-next-action (next-action &key (timeout nil) (speak-wait nil))
    (let ((ask-sentence
            (format nil "Do you want to start next action ~A?" next-action)))
      (send self :ros-yes-or-no-p ask-sentence
            :use-default t :default t :timeout timeout
            :speak-wait speak-wait)))
  (:ask-user-abortion (&key (timeout nil) (speak-wait nil))
    (let ((ask-sentence
            (format nil "Are you sure to abort the execution?")))
      (send self :ros-yes-or-no-p ask-sentence
            :use-default nil :timeout timeout
            :speak-wait speak-wait)))
  (:ask-user-ready-device (&key (timeout nil) (speak-wait nil))
    (let ((ask-sentence
            (format nil "Are you ready for teaching?")))
      (send self :ros-yes-or-no-p ask-sentence
            :use-default nil :timeout timeout
            :speak-wait speak-wait)))
  (:ask-user-finish-teach (&key (timeout nil) (speak-wait nil))
    (let ((ask-sentence
            (format nil "Do you want to finish teaching new actions?")))
      (send self :ros-yes-or-no-p ask-sentence
            :use-default nil :timeout timeout
            :speak-wait speak-wait)))
  (:ask-user-continue-teach (&key (timeout nil) (speak-wait nil))
    (let ((ask-sentence
            (format nil "Do you want to continue teaching new actions?")))
      (send self :ros-yes-or-no-p ask-sentence
            :use-default nil :timeout timeout
            :speak-wait speak-wait)))
  (:ask-user-finish-task (&key (timeout nil) (speak-wait nil))
    (let ((ask-sentence
            (format nil "Do you want to finish the task?")))
      (send self :ros-yes-or-no-p ask-sentence
            :use-default nil :timeout timeout
            :speak-wait speak-wait)))
  (:ask-user-continue-task (&key (timeout nil) (speak-wait nil))
    (let ((ask-sentence
            (format nil "Do you want to continue the task?")))
      (send self :ros-yes-or-no-p ask-sentence
            :use-default nil :timeout timeout
            :speak-wait speak-wait)))
  (:ask-user-state-decision (sm gr &key (timeout nil))
    ;; ask teaching first
    (if (send self :ask-user-start-teach)
      (return-from :ask-user-state-decision (cons :teach nil)))
    (let* ((current-sm-node (send sm :active-state))
           (current-state (read-from-string (send current-sm-node :name)))
           (current-gr-node (send gr :search-node-from-pddl-state current-state))
           (gr-acts (send current-gr-node :neighbor-action-alist))
           (next-sm-actions
             (remove nil
               (mapcar #'(lambda (trans)
                           (if (equal (send trans :name) t)
                             (let* ((to-name-sym
                                      (read-from-string (send (send trans :to) :name)))
                                    (action (if (listp to-name-sym) (car to-name-sym))))
                               (if (and action (listp action)) action nil))))
                       (send current-sm-node :arc-list)))))
      (if (and next-sm-actions gr-acts)
        (let* ((next-gr-actions
                 (mapcar
                   #'(lambda (act) (read-from-string (car act)))
                   gr-acts))
               (next-gr-states
                 (mapcar
                   #'(lambda (act)
                       (convert-string-to-sym (send (cdr act) :name)))
                   gr-acts))
               (next-sm-action-syms
                 (mapcar
                   #'(lambda (next-sm-action)
                       (mapcar
                         #'(lambda (as)
                             (if (and as (listp as))
                               (progn
                                 (setq as (remove-key-from-action as :move-robot))
                                 (setq as (remove-key-from-action as :use-torso))
                                 as)
                               as))
                        next-sm-action))
                   next-sm-actions)))
          (tagbody retry-ask-user-state-decision
            (cond
              ;; if next actions are multiples
              ;; ask which action to execute -> ask if teach or not
              ((> (length next-gr-actions) 1)
               (ros::ros-info (format nil "current-state: ~A" current-state))
               (ros::ros-info "Action candidates:")
               (let ((next-gr-index 0)
                     (default-index 0)
                     (chosen-index 0))
                 (while (> (length next-gr-actions) next-gr-index)
                   (let ((next-gr-action (elt next-gr-actions next-gr-index))
                         (next-gr-state (elt next-gr-states next-gr-index)))
                     (ros::ros-info
                       (format nil "(~A) next-action: ~A"
                               next-gr-index next-gr-action))
                     (ros::ros-info
                       (format nil "(~A) next-state: ~A"
                               next-gr-index next-gr-state))
                     (if (find next-gr-action next-sm-action-syms :test #'equal)
                       (setq default-index next-gr-index))
                     (setq next-gr-index (+ next-gr-index 1))))
                 ;; -1 index for teaching
                 (ros::ros-info "(-1) teach new action or finish the task")
                 (setq chosen-index
                       (send self :ask-user-index-choose
                             next-gr-actions :use-default t
                             :default-index default-index
                             :timeout timeout))
                 (if (>= chosen-index 0)
                   (if (equal default-index chosen-index)
                     (return-from :ask-user-state-decision
                                  (cons :next (elt next-gr-actions chosen-index)))
                     (return-from :ask-user-state-decision
                                  (cons :replan (elt next-gr-actions chosen-index))))
                   (cond
                     ((send self :ask-user-start-teach)
                      (return-from :ask-user-state-decision (cons :teach nil)))
                     ((send self :ask-user-finish-task)
                      (send self :ros-speak-warn "Finishing the task.")
                      (return-from :ask-user-state-decision (cons :finish nil)))
                     ((send self :ask-user-abortion)
                      (return-from :ask-user-state-decision (cons :abort nil)))
                     (t (go retry-ask-user-state-decision))
                     ))))
              ;; if next action is only one
              ;; ask if execute or not -> ask if teach or not -> ask if abort or not
              ((= (length next-gr-actions) 1)
               (if (find (car next-gr-actions) next-sm-action-syms :test #'equal)
                 (let* ((next-action (car next-gr-actions))
                        (next-state (car next-gr-states)))
                   (ros::ros-info (format nil "current-state: ~A" current-state))
                   (ros::ros-info (format nil "next-action  : ~A" next-action))
                   (ros::ros-info (format nil "next-state   : ~A" next-state))
                   (cond
                     ((send self :ask-user-next-action next-action :timeout timeout)
                      (return-from :ask-user-state-decision (cons :next next-action)))
                     ((send self :ask-user-start-teach)
                      (return-from :ask-user-state-decision (cons :teach nil)))
                     ((send self :ask-user-finish-task)
                      (send self :ros-speak-warn "Finishing the task.")
                      (return-from :ask-user-state-decision (cons :finish nil)))
                     ((send self :ask-user-abortion)
                      (return-from :ask-user-state-decision (cons :abort nil)))
                     (t (go retry-ask-user-state-decision))
                     ))))
              (t nil))))
        (cons :pass nil))))
  (:ask-user-teach-decision (&key (timeout nil))
    (let* ((continue-teach-p
             (send self :ask-user-continue-teach))
           (continue-task-p
             (if (null continue-teach-p)
               (send self :ask-user-continue-task))))
      (if continue-teach-p
        (return-from :ask-user-teach-decision nil)
        (if continue-task-p
          (progn
            (send self :ros-speak-warn "Continue the task.")
            (return-from :ask-user-teach-decision :replan))
          (progn
            (send self :ros-speak-warn "Finishing the task.")
            (return-from :ask-user-teach-decision :finish))
          ))))
  )


(provide :common-statenet-interactor "common-statenet-interactor.l")
