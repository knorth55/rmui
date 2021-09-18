(require :common-statenet "package://rmuieus/euslisp/common/common-statenet.l")

(ros::roseus-add-msgs "std_msgs")


(defclass statenet-interactor
  :super propertied-object
  :slots (namespace
          groupname
          mode
          response)
  )

(defmethod statenet-interactor
  (:init (&key ((:mode md) :ros) ((:namespace ns) nil) ((:groupname gpn)))
    (setq mode md)
    (setq namespace ns)
    (setq groupname gpn)
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
  (:string-cb (msg)
    (setq response (if msg (send msg :data))))
  (:get-user-response (&key (mode :ros) (timeout nil) (namespace nil))
    (cond
      ;; timeout is not valid for read-line
      ((equal mode :read-line)
       (read-line t))
      ((equal mode :ros)
       (let ((start-stamp (ros::time-now)))
         (ros::rate 10)
         (while (null response)
           (if (and timeout
                    (> (send
                         (ros::time-
                           (ros::time-now)
                           (start-stamp))
                         :to-nsec)
                       (* timeout 1e9)))
             (return))
           (ros::spin-once groupname))
         (let ((tmp-response (copy-seq response)))
           (setq response nil)
           tmp-response)))
      (t nil)))
  (:ros-yes-or-no-p (ask-sentence &key (use-default t) (default t) (timeout nil))
    (let ((response))
      (tagbody retry-yes-or-no
        (ros::ros-info ask-sentence)
        (finish-output t)
        (setq response
              (string-upcase
                (send self :get-user-response :timeout timeout)))
        (cond
          ((or (string= response "YES")
               (string= response "Y"))
           (return-from :ros-yes-or-no-p t))
          ((or (string= response "NO")
               (string= response "N"))
           (return-from :ros-yes-or-no-p nil))
          ((string= response "")
           (if use-default
             (return-from :ros-yes-or-no-p default)
             (progn
               (ros::ros-error "Please answer with yes or no.")
               (go retry-yes-or-no))))
          (t (ros::ros-error "Please answer with yes or no.")
             (go retry-yes-or-no))))))
  (:ros-index-choose
    (ask-sentence min-index max-index
      &key (use-default t) (default-index 0) (timeout nil))
    (let ((response))
      (tagbody retry-index-choose
        (ros::ros-info ask-sentence)
        (finish-output t)
        (setq response
              (send self :get-user-response :timeout timeout))
        (if (string= response "")
          (if use-default
            (return-from :ros-index-choose default-index)
            (progn
              (ros::ros-error
                (format nil "Please answer with a valid number: ~A to ~A"
                        min-index max-index))
              (go retry-index-choose)))
          (let ((resp-num (read-from-string response)))
            (if (and (integerp resp-num)
                     (>= resp-num min-index)
                     (<= resp-num max-index))
              (return-from :ros-index-choose resp-num)
              (progn
                (ros::ros-error
                  (format nil "Please answer with a valid number: ~A to ~A"
                          min-index max-index))
                (go retry-index-choose))))))))
  (:ask-user-index-choose
    (actions &key (use-default t) (default-index 0) (timeout nil))
    (let ((info-sentence
            (format nil "Default action: (~A) ~A"
                           default-index (elt actions default-index)))
          (ask-sentence
            (format nil "Which action do you choose ? (Default: ~A)" default-index)))
      (if use-default (ros::ros-info info-sentence))
      (send self :ros-index-choose ask-sentence
            -1 (- (length actions) 1)
            :use-default use-default
            :default-index default-index
            :timeout timeout)))
  (:ask-user-teaching (&key (timeout nil))
    (let ((ask-sentence
            (format nil "Do you want to teach a new action? : ")))
      (send self :ros-yes-or-no-p ask-sentence
            :use-default nil :timeout timeout)))
  (:ask-user-next-action (next-action &key (timeout nil))
    (let ((ask-sentence
            (format nil "Do you want to start next action ~A ? : " next-action)))
      (send self :ros-yes-or-no-p ask-sentence
            :use-default t :default t :timeout timeout)))
  (:ask-user-abortion (&key (timeout nil))
    (let ((ask-sentence
            (format nil "Are you sure to abort the execution ? : ")))
      (send self :ros-yes-or-no-p ask-sentence
            :use-default nil :timeout timeout)))
  (:ask-user-decision (sm gr &key (timeout nil))
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
                 (mapcar #'(lambda (act) (read-from-string (car act)))
                         gr-acts))
               (next-gr-states
                 (mapcar #'(lambda (act)
                             (convert-string-to-sym (send (cdr act) :name)))
                         gr-acts)))
          (tagbody retry-ask-user-decision
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
                     (if (find next-gr-action next-sm-actions :test #'equal)
                       (setq default-index next-gr-index))
                     (setq next-gr-index (+ next-gr-index 1))))
                 ;; -1 index for teaching
                 (ros::ros-info "(-1) teach new action")
                 (setq chosen-index
                       (send self :ask-user-index-choose
                             next-gr-actions :use-default t
                             :default-index default-index
                             :timeout timeout))
                 (if (>= chosen-index 0)
                   (if (equal default-index chosen-index)
                     (return-from :ask-user-decision
                                  (cons :next (elt next-gr-actions chosen-index)))
                     (return-from :ask-user-decision
                                  (cons :replan (elt next-gr-actions chosen-index))))
                   (if (send self :ask-user-teaching :timeout timeout)
                     (return-from :ask-user-decision (cons :teach nil))
                     (go retry-ask-user-decision)))))
              ;; if next action is only one
              ;; ask if execute or not -> ask if teach or not -> ask if abort or not
              ((= (length next-gr-actions) 1)
               (if (find (car next-gr-actions) next-sm-actions :test #'equal)
                 (let* ((next-action (car next-gr-actions))
                        (next-state (car next-gr-states)))
                   (ros::ros-info (format nil "current-state: ~A" current-state))
                   (ros::ros-info (format nil "next-action  : ~A" next-action))
                   (ros::ros-info (format nil "next-state   : ~A" next-state))
                   (if (send self :ask-user-next-action next-action :timeout timeout)
                     (return-from :ask-user-decision (cons :next next-action))
                     (if (send self :ask-user-teaching :timeout timeout)
                       (return-from :ask-user-decision :teach)
                       (if (send self :ask-user-abortion :timeout timeout)
                         (return-from :ask-user-decision (cons :abort nil))
                         (go retry-ask-user-decision)))))))
              (t nil))))
        (cons :pass nil)))))


(provide :common-statenet-interactor "common-statenet-interactor.l")
