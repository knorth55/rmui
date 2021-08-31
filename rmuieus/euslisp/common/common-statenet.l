(require :state-machine-ros "package://roseus_smach/src/state-machine-ros.l")
(require :state-machine-utils "package://roseus_smach/src/state-machine-utils.l")

(require :statenet-graph "package://rmuieus/euslisp/statenet/statenet-graph.l")
(require :statenet-util "package://rmuieus/euslisp/statenet/statenet-util.l")
(require :common-io "package://rmuieus/euslisp/common/common-io.l")
(require :common-graph "package://rmuieus/euslisp/common/common-graph.l")
(require :common-actions "package://rmuieus/euslisp/common/common-actions.l")


;; state

(defvar *reset-state*
  '((:larm-contact-state :released)
    (:rarm-contact-state :released)
    (:floor-contact-state :bottom-contact)
    (:object-location-state :center)
    (:object-rotation-state (:x0 :y0 :z0))))


(defun add-initial-state (gr)
  (add-action-state-in-graph gr '(action-init) '(:init) *reset-state*))


(defun load-merged-statenet-graph ()
  (let ((gr (merge-statenet-graphs (load-all-statenet-graph))))
    (if (null gr) (setq gr (load-merged-statenet-graph)))
    (dump-merged-statenet-graph gr)
    (add-initial-state gr)
    gr))


(defun solve-statenet-path (gr solver start-state goal-state)
  (send gr :start-state (send gr :search-node-from-pddl-state start-state))
  (send gr :goal-state (send gr :search-node-from-pddl-state goal-state))
  (send solver :solve gr))


(defun ros-yes-or-no-p (ask-sentence &optional (default t))
  (let ((response))
    (tagbody retry
      (ros::ros-info ask-sentence)
      (finish-output t)
      (setq response (string-upcase (read-line t)))
      (cond
        ((string= response "YES") (return-from ros-yes-or-no-p t))
        ((string= response "NO") (return-from ros-yes-or-no-p nil))
        ((string= response "") (return-from ros-yes-or-no-p default))
        (t (go retry))))))


(defun ros-index-choose (ask-sentence max-index &optional (default-index 0))
  (let ((response))
    (tagbody retry
      (ros::ros-info ask-sentence)
      (finish-output t)
      (setq response (read-line t))
      (cond
        ((string= response "")
         (return-from ros-index-choose default-index))
        ((and (integerp (read-from-string response))
              (>= (read-from-string response) 0)
              (< (read-from-string response) max-index))
         (return-from ros-index-choose
                      (read-from-string response)))
        (t (go retry))))))


(defun ask-user-decision (&key (gr *statenet-graph*) (sm *sm*))
  (let* ((current-sm-node (send sm :active-state))
         (current-state (read-from-string (send current-sm-node :name)))
         (current-gr-node (send gr :search-node-from-pddl-state current-state))
         (gr-acts (send current-gr-node :neighbor-action-alist))
         (next-sm-actions
           (remove nil
             (mapcar #'(lambda (trans)
                         (let ((action
                                 (car (read-from-string
                                        (send (send trans :to) :name)))))
                           (if (listp action) action nil)))
                     (send current-sm-node :arc-list)))))
    (if (and next-sm-actions gr-acts)
      (let* ((next-gr-actions
               (mapcar #'(lambda (act) (read-from-string (car act)))
                       gr-acts))
             (next-gr-states
               (mapcar #'(lambda (act)
                           (convert-string-to-sym (send (cdr act) :name)))
                       gr-acts)))
        (cond
          ((> (length next-gr-actions) 1)
           (ros::ros-info (format nil "current-state: ~A" current-state))
           (ros::ros-info "Action candidates:")
           (let ((next-gr-index 0)
                 (default-index 0)
                 (chosen-index 0))
             (while (> (length next-gr-actions) next-gr-index)
               (let ((next-gr-action (elt next-gr-actions next-gr-index))
                     (next-gr-state (elt next-gr-states next-gr-index)))
                 (ros::ros-info "(~A) next-action: ~A" next-gr-index next-gr-action)
                 (ros::ros-info "(~A) next-state: ~A" next-gr-index next-gr-state)
                 (if (find next-gr-action next-sm-actions :test #'equal)
                   (setq default-index next-gr-index))
                 (setq next-gr-index (+ next-gr-index 1))))
             (ros::ros-info "Default action: (~A) ~A"
                            default-index (elt next-gr-actions default-index))
             (setq chosen-index
                   (ros-index-choose
                     (format nil "Which action do you choose ? (Default: ~A)"
                             default-index)
                     (length next-gr-actions) default-index))
             ;; TODO (knorth55): implement replan
             (if (equal default-index chosen-index) :next :replan)))
          ((= (length next-gr-actions) 1)
           (if (find (car next-gr-actions) next-sm-actions :test #'equal)
             (let* ((next-action (car next-gr-actions))
                    (next-state (car next-gr-states))
                    (ask-sentence
                      (format nil "Do you start next action ~A ? : " next-action)))
               (ros::ros-info (format nil "current-state: ~A" current-state))
               (ros::ros-info (format nil "next-action  : ~A" next-action))
               (ros::ros-info (format nil "next-state   : ~A" next-state))
               (if (ros-yes-or-no-p ask-sentence) :next :abort))))
          (t nil)))
      :next)))


(defun ask-user-abortion ()
  (let ((ask-sentence
          (format nil "Are you sure to abort the execution ? : ")))
    (yes-or-no-p ask-sentence)))


(defun statenet-state-func (userdate)
  (tagbody retry
    (let ((user-decision (ask-user-decision)))
      (cond
        ((equal user-decision :next)
         (return-from statenet-state-func t))
        ((equal user-decision :abort)
         (if (ask-user-abortion)
           (return-from statenet-state-func nil)
           (go retry)))
        ;; TODO (knorth55): implement replan
        ((equal user-decision :replan)
         (ros::ros-error "replanning is not implmented yet")
         (go retry))
        ((equal user-decision nil)
         (ros::ros-error "something is wrong.")
         (return-from statenet-state-func nil))
        (t (go retry))))))


(defun generate-smach-from-state (gr solver start-state goal-state)
  (let* ((solution
           (solve-statenet-path gr solver start-state goal-state))
         (sm (if solution
               (convert-solution-to-smach
                 solution :state-func #'statenet-state-func))))
    sm))


(provide :common-statenet "common-statenet.l")
