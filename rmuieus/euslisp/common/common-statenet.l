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


(defun solve-statenet-path (gr slvr start-state goal-state &key (first-action nil))
  (let ((start-node (send gr :search-node-from-pddl-state start-state))
        (goal-node (send gr :search-node-from-pddl-state goal-state)))
    (if first-action
      (let* ((first-action-name (format nil "~A" first-action))
             (acts (send start-node :neighbor-action-alist))
             (second-node
               (cdr (assoc first-action-name acts :test #'equal)))
             (arc-alist
               (mapcar #'(lambda (x) (cons (send x :name) x))
                       (mapcar #'car (send start-node :successors))))
             (first-arc
               (cdr (assoc first-action-name arc-alist :test #'equal)))
             (solution nil))
        (send gr :start-state second-node)
        (send gr :goal-state goal-node)
        (setq solution (send slvr :solve gr))
        (if solution
          (let ((first-node
                  (instance solver-node :init start-node :cost 0)))
            (send (car solution) :parent first-node)
            (send (car solution) :action first-arc)
            (dolist (sn solution)
              (send sn :cost (+ 1 (send sn :cost))))
            (push first-node solution)
            solution)))
      (let ((solution nil))
        (send gr :start-state start-node)
        (send gr :goal-state goal-node)
        (setq solution (send slvr :solve gr))
        solution))))


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
                         (if (equal (send trans :name) t)
                           (let ((action
                                   (car (read-from-string
                                          (send (send trans :to) :name)))))
                             (if (listp action) action nil))))
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
             (if (equal default-index chosen-index)
               (cons :next (elt next-gr-actions chosen-index))
               (cons :replan (elt next-gr-actions chosen-index)))))
          ((= (length next-gr-actions) 1)
           (if (find (car next-gr-actions) next-sm-actions :test #'equal)
             (let* ((next-action (car next-gr-actions))
                    (next-state (car next-gr-states))
                    (ask-sentence
                      (format nil "Do you start next action ~A ? : " next-action)))
               (ros::ros-info (format nil "current-state: ~A" current-state))
               (ros::ros-info (format nil "next-action  : ~A" next-action))
               (ros::ros-info (format nil "next-state   : ~A" next-state))
               (if (ros-yes-or-no-p ask-sentence) (cons :next next-action) :abort))))
          (t nil)))
      :pass)))


(defun ask-user-abortion ()
  (let ((ask-sentence
          (format nil "Are you sure to abort the execution ? : ")))
    (yes-or-no-p ask-sentence)))


(defun generate-smach-from-state (gr slvr start-state goal-state
                                     &key (state-func #'(lambda (x) t))
                                     (first-action nil) (no-loop nil))
  (let* ((solution
           (solve-statenet-path gr slvr start-state goal-state
                                :first-action first-action))
         (sm (if solution
               (convert-solution-to-smach solution
                                          :state-func state-func
                                          :no-loop no-loop))))
    sm))


(provide :common-statenet "common-statenet.l")
