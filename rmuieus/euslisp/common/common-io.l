(require :pddl-result-graph "package://pddl_planner/src/pddl-result-graph.l")
(require :eus-pddl-client "package://pddl_planner/src/eus-pddl-client.l")

(require :statenet-graph "package://rmuieus/euslisp/statenet/statenet-graph.l")

(defvar *training-data-path* (ros::resolve-ros-path "package://rmuieus/training_data"))


(defun get-iso-stamp ()
  (let* ((localtime (unix::localtime))
         (iso-stamp (format nil "~A-~0,2d-~0,2dT~0,2d:~0,2d:~0,2d"
                       (+ (elt localtime 5) 1900)
                       (+ (elt localtime 4) 1)
                       (elt localtime 3)
                       (elt localtime 2)
                       (elt localtime 1)
                       (elt localtime 0))))
    iso-stamp))


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


(defun get-dump-dirnames ()
  (let* ((dump-dirnames
           (remove-if-not #'(lambda (x) (equal (subseq x 0 2) "20"))
                          (directory *training-data-path*))))
    (remove-if-not
      #'(lambda (x) (directory-p (format nil "~A/~A" *training-data-path* x)))
      dump-dirnames)))


(defun get-dump-dirpaths ()
  (let* ((dump-dirnames (get-dump-dirnames)))
    (mapcar #'(lambda (x) (format nil "~A/~A" *training-data-path* x)) dump-dirnames)))


(defun dump-executed-actions (executed-actions &key (path nil) (stamp "latest") (filename "executed-actions.l"))
  (let ((filepath (if path path (format nil "~A/~A/~A" *training-data-path* stamp filename))))
    (dump-structure filepath executed-actions)))


(defun load-executed-actions (&key (path nil) (stamp "latest") (filename "executed-actions.l"))
  (let ((filepath (if path path (format nil "~A/~A/~A" *training-data-path* stamp filename))))
    (with-open-file (executed-actions-f filepath)
      (read executed-actions-f))))


(defun load-all-executed-actions ()
  (let ((executed-actions-list nil))
    (dolist (dump-dirname (get-dump-dirnames))
      (setq executed-actions-list
            (append executed-actions-list
                    (list (load-executed-actions :stamp dump-dirname)))))
    executed-actions-list))


(defun dump-scene-states (scene-states &key (path nil) (stamp "latest") (filename "scene-states.l"))
  (let ((filepath (if path path (format nil "~A/~A/~A" *training-data-path* stamp filename))))
    (dump-structure filepath scene-states)))


(defun load-scene-states (&key (path nil) (stamp "latest") (filename "scene-states.l"))
  (let ((filepath (if path path (format nil "~A/~A/~A" *training-data-path* stamp filename))))
    (with-open-file (scene-states-f filepath)
      (read scene-states-f))))


(defun load-all-scene-states ()
  (let ((scene-states-list nil))
    (dolist (dump-dirname (get-dump-dirnames))
      (setq scene-states-list
            (append scene-states-list
                    (list (load-scene-states :stamp dump-dirname)))))
    scene-states-list))


(defun dump-pddl-graph (pddl-graph &key (path nil) (stamp "latest") (filename "pddl-graph.l"))
  (let ((filepath (if path path (format nil "~A/~A/~A" *training-data-path* stamp filename))))
    (dump-structure filepath pddl-graph)))


(defun load-pddl-graph (&key (path nil) (stamp "latest") (filename "pddl-graph.l"))
  (let ((filepath (if path path (format nil "~A/~A/~A" *training-data-path* stamp filename))))
    (with-open-file (pddl-graph-f filepath)
      (read pddl-graph-f))))


(defun load-all-pddl-graph ()
  (let ((pddl-graph-list nil))
    (dolist (dump-dirname (get-dump-dirnames))
      (setq pddl-graph-list
            (append pddl-graph-list
                    (list (load-pddl-graph :stamp dump-dirname)))))
    pddl-graph-list))


(defun dump-statenet-graph (statenet-graph &key (path nil) (stamp "latest") (filename "statenet-graph.l"))
  (let ((filepath (if path path (format nil "~A/~A/~A" *training-data-path* stamp filename))))
    (dump-structure filepath statenet-graph)))


(defun load-statenet-graph (&key (path nil) (stamp "latest") (filename "statenet-graph.l"))
  (let ((filepath (if path path (format nil "~A/~A/~A" *training-data-path* stamp filename))))
    (with-open-file (statenet-graph-f filepath)
      (read statenet-graph-f))))


(defun load-all-statenet-graph ()
  (let ((statenet-graph-list nil))
    (dolist (dump-dirname (get-dump-dirnames))
      (setq statenet-graph-list
            (append statenet-graph-list
                    (list (load-statenet-graph :stamp dump-dirname)))))
    statenet-graph-list))


(defun dump-merged-statenet-graph (merged-statenet-graph &key (path nil) (filename "merged-statenet-graph.l"))
  (let ((filepath (if path path (format nil "~A/~A" *training-data-path* filename))))
    (dump-structure filepath merged-statenet-graph)))


(defun load-merged-statenet-graph (&key (path nil) (filename "merged-statenet-graph.l"))
  (let ((filepath (if path path (format nil "~A/~A" *training-data-path* filename))))
    (with-open-file (merged-statenet-graph-f filepath)
      (read merged-statenet-graph-f))))


(provide :common-io "common-io.l")
