(require :state-machine-ros "package://roseus_smach/src/state-machine-ros.l")
(require :state-machine-utils "package://roseus_smach/src/state-machine-utils.l")

(require :statenet-graph "package://rmuieus/euslisp/statenet/statenet-graph.l")
(require :statenet-util "package://rmuieus/euslisp/statenet/statenet-util.l")
(require :common-io "package://rmuieus/euslisp/common/common-io.l")
(require :common-graph "package://rmuieus/euslisp/common/common-graph.l")
(require :common-actions "package://rmuieus/euslisp/common/common-actions.l")
(require :common-scenes "package://rmuieus/euslisp/common/common-scenes.l")


(defun set-statenet-reset-state (&optional (n-box 1))
  (defvar *reset-state* (get-initial-scene-states n-box)))


(defun add-initial-state (gr reset-state)
  (add-action-state-in-graph gr '(statenet-action-init) '(:init) reset-state))


(defun load-merged-statenet-graph-with-reset
  (reset-state &key (data-dirs (list *training-data-path*)))
  (let
    ((gr
       (merge-statenet-graphs
         (apply #'append
                (mapcar
                  #'(lambda (data-dir)
                      (load-all-statenet-graph :data-dir data-dir))
                  data-dirs)))))
    (if (null gr)
      (setq gr
            (merge-statenet-graphs
              (mapcar
                #'(lambda (data-dir)
                    (load-merged-statenet-graph :data-dir data-dir))
                data-dirs))))
    (dolist (data-dir data-dirs)
      (dump-merged-statenet-graph gr :data-dir data-dir))
    (add-initial-state gr reset-state)
    gr))


(provide :common-statenet "common-statenet.l")
