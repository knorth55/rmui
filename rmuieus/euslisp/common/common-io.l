(require :pddl-result-graph "package://pddl_planner/src/pddl-result-graph.l")
(require :eus-pddl-client "package://pddl_planner/src/eus-pddl-client.l")

(require :statenet-graph "package://rmuieus/euslisp/statenet/statenet-graph.l")


(defvar *training-data-path*
  (ros::resolve-ros-path "package://rmuieus/training_data"))
(defvar *training-object-statenet-path*
  (format nil "~A/object_statenet" *training-data-path*))
(defvar *training-collabo-statenet-path*
  (format nil "~A/collabo_statenet" *training-data-path*))
(defvar *scene-path*
  (ros::resolve-ros-path "package://rmuieus/euslisp/scene"))


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


(defun get-dump-dirnames (data-dir &key (sort-predicate #'string<))
  (let* ((dump-dirnames
           (remove-if-not
             #'(lambda (x) (equal (subseq x 0 2) "20"))
             (directory data-dir))))
    (setq dump-dirnames
          (remove-if-not
            #'(lambda (x)
                (directory-p (format nil "~A/~A" data-dir x)))
            dump-dirnames))
    (setq dump-dirnames (sort dump-dirnames sort-predicate))
    dump-dirnames))


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


(defun load-all-executed-actions (&key (data-dir *training-object-statenet-path*)
                                       (sort-predicate #'string<))
  (let ((executed-actions-list nil)
        (dump-dirnames
          (get-dump-dirnames data-dir :sort-predicate sort-predicate)))
    (dolist (dump-dirname dump-dirnames)
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


(defun load-all-scene-states (&key (data-dir *training-object-statenet-path*)
                                   (sort-predicate #'string<))
  (let ((scene-states-list nil)
        (dump-dirnames (get-dump-dirnames data-dir :sort-predicate sort-predicate)))
    (dolist (dump-dirname dump-dirnames)
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


(defun load-all-pddl-graph (&key (data-dir *training-object-statenet-path*)
                                 (sort-predicate #'string<))
  (let ((pddl-graph-list nil)
        (dump-dirnames
          (get-dump-dirnames data-dir :sort-predicate sort-predicate)))
    (dolist (dump-dirname dump-dirnames)
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


(defun load-all-statenet-graph (&key (data-dir *training-object-statenet-path*)
                                     (sort-predicate #'string<))
  (let ((statenet-graph-list nil)
        (dump-dirnames
          (get-dump-dirnames data-dir :sort-predicate sort-predicate)))
    (dolist (dump-dirname dump-dirnames)
      (setq statenet-graph-list
            (append statenet-graph-list
                    (list (load-statenet-graph :data-dir data-dir :stamp dump-dirname)))))
    statenet-graph-list))


(defun dump-merged-statenet-graph
  (merged-statenet-graph
    &key (data-dir *training-object-statenet-path*)
    (path nil) (filename "merged-statenet-graph.l"))
  (let ((filepath (if path path (format nil "~A/~A" data-dir filename))))
    (if (null (probe-file data-dir))
      (unix::system (format nil "mkdir -p ~A" data-dir)))
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


(defun get-all-executed-collabo-path-dump-dirnames
  (&key (data-dir *training-collabo-statenet-path*)
        (sort-predicate #'string<))
  (let ((all-dump-dirnames)
        (dump-dirnames
          (get-dump-dirnames data-dir :sort-predicate sort-predicate)))
    (dolist (dump-dirname dump-dirnames)
      (setq all-dump-dirnames
            (append all-dump-dirnames
                    (get-executed-collabo-path-dump-dirnames
                      :data-dir data-dir
                      :stamp dump-dirname
                      :sort-predicate sort-predicate))))
    all-dump-dirnames))


(defun get-executed-collabo-path-dump-dirnames
  (&key (data-dir *training-collabo-statenet-path*)
        (path nil) (stamp "latest") (filename "executed-collabo-path.l")
        (sort-predicate #'string<))
  (let* ((dumppath (if path path (format nil "~A/~A" data-dir stamp)))
         (dump-dirnames
           (get-dump-dirnames dumppath :sort-predicate sort-predicate)))
    dump-dirnames))


(defun load-executed-collabo-path
  (&key (data-dir *training-collabo-statenet-path*)
        (path nil) (stamp "latest") (filename "executed-collabo-path.l")
        (sort-predicate #'string<))
  (let* ((executed-collabo-path nil)
         (dumppath (if path path (format nil "~A/~A" data-dir stamp)))
         (dump-dirnames
           (get-dump-dirnames dumppath :sort-predicate sort-predicate)))
    (dolist (dump-dirname dump-dirnames)
      (let ((filepath (format nil "~A/~A/~A" dumppath dump-dirname filename)))
        (with-open-file (executed-collabo-path-f filepath)
          (setq executed-collabo-path
                (append executed-collabo-path
                        (list (read executed-collabo-path-f)))))))
    executed-collabo-path))


(defun load-all-executed-collabo-path
  (&key (data-dir *training-collabo-statenet-path*)
        (sort-predicate #'string<))
  (let ((executed-collabo-path nil)
        (dump-dirnames
          (get-dump-dirnames data-dir :sort-predicate sort-predicate)))
    (dolist (dump-dirname dump-dirnames)
      (setq executed-collabo-path
            (append executed-collabo-path
                    (load-executed-collabo-path
                      :data-dir data-dir
                      :stamp dump-dirname
                      :sort-predicate sort-predicate))))
    executed-collabo-path))


(defun load-scene-alist
  (scene-name &key (data-dir *scene-path*))
  (let ((filepath (format nil "~A/~A.l" data-dir scene-name)))
    (with-open-file (scene-f filepath)
      (read config-f))))


(provide :common-io "common-io.l")
