(require :pddl-result-graph "package://pddl_planner/src/pddl-result-graph.l")
(require :eus-pddl-client "package://pddl_planner/src/eus-pddl-client.l")

(require :statenet-graph "package://rmuieus/euslisp/statenet/statenet-graph.l")

(defvar *training-data-path* (ros::resolve-ros-path "package://rmuieus/training_data"))


(defun link-latest-dump-dir (dump-dir)
  (let ((latest-dir (format nil "~A/latest" *training-data-path*)))
    (unix::system (format nil "rm ~A" latest-dir))
    (unix::system (format nil "ln -sf ~A ~A" dump-dir latest-dir)))
  dump-dir)


(defun create-dump-dir (&key (stamp (get-iso-stamp)))
  (let ((dump-dir (format nil "~A/~A" *training-data-path* stamp)))
    (unix::system (format nil "mkdir -p ~A" dump-dir))
    (link-latest-dump-dir dump-dir)
    dump-dir))


(defun get-dump-dir-list ()
  (let* ((dump-dirnames
           (remove-if-not #'(lambda (x) (equal (subseq x 0 2) "20"))
                          (directory *training-data-path*)))
         (dump-dirs
           (remove-if-not #'directory-p
                          (mapcar #'(lambda (x) (format nil "~A/~A" *training-data-path* x))
                                  dump-dirnames))))
    dump-dirs))


(defun dump-executed-actions (&key (stamp "latest") (filename "executed-actions.l"))
  (let ((filepath (format nil "~A/~A/~A" *training-data-path* stamp filename)))
    (dump-structure filepath *executed-actions*)))


(defun load-executed-actions (&key (stamp "latest") (filename "executed-actions.l"))
  (let ((filepath (format nil "~A/~A/~A" *training-data-path* stamp filename)))
    (with-open-file (executed-actions-f filepath)
      (read executed-actions-f))))


(defun dump-scene-states (&key (stamp "latest") (filename "scene-states.l"))
  (let ((filepath (format nil "~A/~A/~A" *training-data-path* stamp filename)))
    (dump-structure filepath *scene-states*)))


(defun load-scene-states (&key (stamp "latest") (filename "scene-states.l"))
  (let ((filepath (format nil "~A/~A/~A" *training-data-path* stamp filename)))
    (with-open-file (scene-states-f filepath)
      (read scene-states-f))))


(defun dump-pddl-graph (&key (stamp "latest") (filename "pddl-graph.l"))
  (let ((filepath (format nil "~A/~A/~A" *training-data-path* stamp filename)))
    (dump-structure filepath *pddl-graph*)))


(defun load-pddl-graph (&key (stamp "latest") (filename "pddl-graph.l"))
  (let ((filepath (format nil "~A/~A/~A" *training-data-path* stamp filename)))
    (with-open-file (pddl-graph-f filepath)
      (read pddl-graph-f))))


(defun dump-statenet-graph (&key (stamp "latest") (filename "statenet-graph.l"))
  (let ((filepath (format nil "~A/~A/~A" *training-data-path* stamp filename)))
    (dump-structure filepath *statenet-graph*)))


(defun load-statenet-graph (&key (stamp "latest") (filename "statenet-graph.l"))
  (let ((filepath (format nil "~A/~A/~A" *training-data-path* stamp filename)))
    (with-open-file (statenet-graph-f filepath)
      (read statenet-graph-f))))


(provide :common-io "common-io.l")
