(require :common-graph "package://rmuieus/euslisp/common/common-graph.l")


(defun simplify-collabo-path (collabo-path)
  (let ((simple-collabo-path (list (car collabo-path)))
        (prev-node-type)
        (prev-state-action nil)
        (start-stamp-list nil)
        (end-stamp-list nil)
        (current-n)
        (last-n (car (last collabo-path))))
    (dolist (n (cdr collabo-path))
      (let ((node-type (cadr (assoc :node-type n)))
            (state-action (cadr (assoc :state-action n)))
            (start-stamp (cadr (assoc :start-stamp n)))
            (end-stamp (cadr (assoc :end-stamp n))))
        (cond
          ;; initial
          ((and (null prev-node-type)
                (null prev-state-action))
           (setq start-stamp-list (list start-stamp))
           (setq end-stamp-list (list end-stamp))
           (setq current-n
                 (mapcar
                   #'(lambda (x) (if (listp x) (copy-tree x) x)) n)))
          ;; continue
          ((and (null (equal n last-n))
                (equal prev-node-type node-type)
                (equal prev-state-action state-action))
           (setq start-stamp-list
                 (append start-stamp-list (list start-stamp)))
           (setq end-stamp-list
                 (append end-stamp-list (list end-stamp))))
          ;; last and continue
          ((and (equal n last-n)
                (equal prev-node-type node-type)
                (equal prev-state-action state-action))
           (setq start-stamp-list
                 (append start-stamp-list (list start-stamp)))
           (setq end-stamp-list
                 (append end-stamp-list (list end-stamp)))
           (setf (cadr (assoc :start-stamp current-n))
                 (reduce #'min start-stamp-list))
           (setf (cadr (assoc :end-stamp current-n))
                 (reduce #'max end-stamp-list))
           (setq simple-collabo-path
                 (append simple-collabo-path (list current-n))))
          ;; last and diff
          ((and (equal n last-n)
                (not (and (equal prev-node-type node-type)
                          (equal prev-state-action state-action))))
           (setf (cadr (assoc :start-stamp current-n))
                 (reduce #'min start-stamp-list))
           (setf (cadr (assoc :end-stamp current-n))
                 (reduce #'max end-stamp-list))
           (setq last-n
                 (mapcar
                   #'(lambda (x) (if (listp x) (copy-tree x) x)) n))
           (setf (cadr (assoc :start-stamp last-n)) start-stamp)
           (setf (cadr (assoc :end-stamp last-n)) end-stamp)
           (setq simple-collabo-path
                 (append simple-collabo-path (list current-n last-n))))
          ;; diff
          (t
           (setf (cadr (assoc :start-stamp current-n))
                 (reduce #'min start-stamp-list))
           (setf (cadr (assoc :end-stamp current-n))
                 (reduce #'max end-stamp-list))
           (setq simple-collabo-path
                 (append simple-collabo-path (list current-n)))
           (setq start-stamp-list (list start-stamp))
           (setq end-stamp-list (list end-stamp))
           (setq current-n
                 (mapcar
                   #'(lambda (x) (if (listp x) (copy-tree x) x)) n)))
          )
        (setq prev-node-type node-type)
        (setq prev-state-action state-action)))
    simple-collabo-path))


(provide :common-collabo-path "common-collabo-path.l")
