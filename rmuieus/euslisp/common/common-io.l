(require :pddl-result-graph "package://pddl_planner/src/pddl-result-graph.l")
(require :eus-pddl-client "package://pddl_planner/src/eus-pddl-client.l")

(require :statenet-graph "package://rmuieus/euslisp/statenet/statenet-graph.l")


(defvar *training-data-path*
  (ros::resolve-ros-path "package://rmuieus/training_data"))
(defvar *training-object-statenet-path*
  (format nil "~A/object_statenet" *training-data-path*))
(defvar *training-collabo-statenet-path*
  (format nil "~A/collabo_statenet" *training-data-path*))


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


(defun link-latest-dump-dir (data-dir dump-dir)
  (let ((latest-dir (format nil "~A/latest" data-dir)))
    (if (probe-file latest-dir)
      (unix::system (format nil "rm ~A" latest-dir)))
    (unix::system (format nil "ln -sf ~A ~A" dump-dir latest-dir)))
  dump-dir)


(defun create-dump-dir (data-dir &key (stamp (get-iso-stamp)))
  (let ((dump-dir (format nil "~A/~A" data-dir stamp)))
    (unix::system (format nil "mkdir -p ~A" dump-dir))
    (link-latest-dump-dir data-dir dump-dir)
    dump-dir))


(defun get-dump-dirnames (data-dir)
  (let* ((dump-dirnames
           (remove-if-not
             #'(lambda (x) (equal (subseq x 0 2) "20"))
             (directory data-dir))))
    (remove-if-not
      #'(lambda (x)
          (directory-p (format nil "~A/~A" data-dir x)))
      dump-dirnames)))


(defun get-dump-dirpaths (data-dir)
  (let* ((dump-dirnames (get-dump-dirnames data-dir)))
    (mapcar #'(lambda (x) (format nil "~A/~A" data-dir x)) dump-dirnames)))


(defun dump-executed-actions
  (executed-actions
    &key (data-dir *training-object-statenet-path*)
    (path nil) (stamp "latest") (filename "executed-actions.l"))
  (let ((filepath (if path path (format nil "~A/~A/~A" data-dir stamp filename))))
    (dump-structure filepath executed-actions)))


(defun load-executed-actions
  (&key (data-dir *training-object-statenet-path*)
        (path nil) (stamp "latest") (filename "executed-actions.l"))
  (let ((filepath (if path path (format nil "~A/~A/~A" data-dir stamp filename))))
    (with-open-file (executed-actions-f filepath)
      (read executed-actions-f))))


(defun load-all-executed-actions (&key (data-dir *training-object-statenet-path*))
  (let ((executed-actions-list nil))
    (dolist (dump-dirname (get-dump-dirnames data-dir))
      (setq executed-actions-list
            (append executed-actions-list
                    (list (load-executed-actions :data-dir data-dir :stamp dump-dirname)))))
    executed-actions-list))


(defun dump-scene-states
  (scene-states
    &key (data-dir *training-object-statenet-path*)
    (path nil) (stamp "latest") (filename "scene-states.l"))
  (let ((filepath (if path path (format nil "~A/~A/~A" data-dir stamp filename))))
    (dump-structure filepath scene-states)))


(defun load-scene-states
  (&key (data-dir *training-object-statenet-path*)
        (path nil) (stamp "latest") (filename "scene-states.l"))
  (let ((filepath (if path path (format nil "~A/~A/~A" data-dir stamp filename))))
    (with-open-file (scene-states-f filepath)
      (read scene-states-f))))


(defun load-all-scene-states (&key (data-dir *training-object-statenet-path*))
  (let ((scene-states-list nil))
    (dolist (dump-dirname (get-dump-dirnames data-dir))
      (setq scene-states-list
            (append scene-states-list
                    (list (load-scene-states :data-dir data-dir :stamp dump-dirname)))))
    scene-states-list))


(defun dump-pddl-graph
  (pddl-graph
    &key (data-dir *training-object-statenet-path*)
    (path nil) (stamp "latest") (filename "pddl-graph.l"))
  (let ((filepath (if path path (format nil "~A/~A/~A" data-dir stamp filename))))
    (dump-structure filepath pddl-graph)))


(defun load-pddl-graph
  (&key (data-dir *training-object-statenet-path*)
        (path nil) (stamp "latest") (filename "pddl-graph.l"))
  (let ((filepath (if path path (format nil "~A/~A/~A" data-dir stamp filename))))
    (with-open-file (pddl-graph-f filepath)
      (read pddl-graph-f))))


(defun load-all-pddl-graph (&key (data-dir *training-object-statenet-path*))
  (let ((pddl-graph-list nil))
    (dolist (dump-dirname (get-dump-dirnames data-dir))
      (setq pddl-graph-list
            (append pddl-graph-list
                    (list (load-pddl-graph :data-dir data-dir :stamp dump-dirname)))))
    pddl-graph-list))


(defun dump-statenet-graph
  (statenet-graph
    &key (data-dir *training-object-statenet-path*)
    (path nil) (stamp "latest") (filename "statenet-graph.l"))
  (let ((filepath (if path path (format nil "~A/~A/~A" data-dir stamp filename))))
    (dump-structure filepath statenet-graph)))


(defun load-statenet-graph
  (&key (data-dir *training-object-statenet-path*)
        (path nil) (stamp "latest") (filename "statenet-graph.l"))
  (let ((filepath (if path path (format nil "~A/~A/~A" data-dir stamp filename))))
    (with-open-file (statenet-graph-f filepath)
      (read statenet-graph-f))))


(defun load-all-statenet-graph (&key (data-dir *training-object-statenet-path*))
  (let ((statenet-graph-list nil))
    (dolist (dump-dirname (get-dump-dirnames data-dir))
      (setq statenet-graph-list
            (append statenet-graph-list
                    (list (load-statenet-graph :data-dir data-dir :stamp dump-dirname)))))
    statenet-graph-list))


(defun dump-merged-statenet-graph
  (merged-statenet-graph
    &key (data-dir *training-object-statenet-path*)
    (path nil) (filename "merged-statenet-graph.l"))
  (let ((filepath (if path path (format nil "~A/~A" data-dir filename))))
    (dump-structure filepath merged-statenet-graph)))


(defun load-merged-statenet-graph
  (&key (data-dir *training-object-statenet-path*)
        (path nil) (filename "merged-statenet-graph.l"))
  (let ((filepath (if path path (format nil "~A/~A" data-dir filename))))
    (with-open-file (merged-statenet-graph-f filepath)
      (read merged-statenet-graph-f))))


(defun dump-executed-collabo-path
  (executed-collabo-path
    &key (data-dir *training-collabo-statenet-path*)
    (path nil) (stamp "latest")
    (execute-stamp (get-iso-stamp))
    (filename "executed-collabo-path.l"))
  (let* ((currentpath
           (format nil "~A/~A/~A" data-dir stamp execute-stamp))
         (filepath
           (if path path (format nil "~A/~A" currentpath filename))))
    (if (null (probe-file currentpath))
      (unix::system (format nil "mkdir -p ~A" currentpath)))
    (dump-structure filepath executed-collabo-path)))


(defun load-executed-collabo-path
  (&key (data-dir *training-collabo-statenet-path*)
        (path nil) (stamp "latest") (filename "executed-collabo-path.l"))
  (let ((executed-collabo-path nil)
        (dumppath (if path path (format nil "~A/~A" data-dir stamp))))
    (dolist (dump-dirname (get-dump-dirnames dumppath))
      (let ((filepath (format nil "~A/~A/~A" dumppath dump-dirname filename)))
        (with-open-file (executed-collabo-path-f filepath)
          (setq executed-collabo-path
                (append executed-collabo-path
                        (list (read executed-collabo-path-f) ))))))
    executed-collabo-path))


(defun load-all-executed-collabo-path
  (&key (data-dir *training-collabo-statenet-path*))
  (let ((executed-collabo-path nil))
    (dolist (dump-dirname (get-dump-dirnames data-dir))
      (setq executed-collabo-path
            (append executed-collabo-path
                    (load-executed-collabo-path :data-dir data-dir
                                                 :stamp dump-dirname))))
    executed-collabo-path))


(provide :common-io "common-io.l")
