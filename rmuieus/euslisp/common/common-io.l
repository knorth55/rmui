(require :pddl-result-graph "package://pddl_planner/src/pddl-result-graph.l")
(require :eus-pddl-client "package://pddl_planner/src/eus-pddl-client.l")

(defvar *training-data-path* (ros::resolve-ros-path "package://rmuieus/training_data"))


(defun create-dump-dir (&key (stamp (get-iso-stamp)))
  (let ((dump-dir (format nil "~A/~A" *training-data-path* stamp))
        (latest-dir (format nil "~A/latest" *training-data-path*)))
    (unix::system (format nil "mkdir -p ~A" dump-dir))
    (unix::system (format nil "rm ~A" latest-dir))
    (unix::system (format nil "ln -sf ~A ~A" dump-dir latest-dir))
    dump-dir))


(defun dump-executed-actions (&key (stamp "latest") (filename "executed-actions.l"))
  (let ((filepath (format nil "~A/~A/~A" *training-data-path* stamp filename)))
    (dump-structure filepath *executed-actions*)))


(defun load-executed-actions (&key (stamp "latest") (filename "executed-actions.l"))
  (let ((filepath (format nil "~A/~A/~A" *training-data-path* stamp filename)))
    (with-open-file (executed-actions-f filepath)
      (setq *executed-actions* (read executed-actions-f)))))


(defun dump-scene-states (&key (stamp "latest") (filename "scene-states.l"))
  (let ((filepath (format nil "~A/~A/~A" *training-data-path* stamp filename)))
    (dump-structure filepath *scene-states*)))


(defun load-scene-states (&key (stamp "latest") (filename "scene-states.l"))
  (let ((filepath (format nil "~A/~A/~A" *training-data-path* stamp filename)))
    (with-open-file (scene-states-f filepath)
      (setq *scene-states* (read scene-states-f)))))


(defun dump-pddl-graph (&key (stamp "latest") (filename "pddl-graph.l"))
  (let ((filepath (format nil "~A/~A/~A" *training-data-path* stamp filename)))
    (dump-structure filepath *pddl-graph*)))


(defun load-pddl-graph (&key (stamp "latest") (filename "pddl-graph.l"))
  (let ((filepath (format nil "~A/~A/~A" *training-data-path* stamp filename)))
    (with-open-file (pddl-graph-f filepath)
      (setq *pddl-graph* (read pddl-graph-f)))))


(provide :common-io "common-io.l")
